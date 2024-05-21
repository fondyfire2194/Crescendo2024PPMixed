// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.ShootingData;
import monologue.Logged;

public class ShootWhileMoving extends Command implements Logged {
  private static InterpolatingDoubleTreeMap angleToleranceMap = new InterpolatingDoubleTreeMap();

  static {
    angleToleranceMap.put(1.36, 30.0353);
    angleToleranceMap.put(1.88, 25.0);
    angleToleranceMap.put(3.0, 15.0);
    angleToleranceMap.put(4.6, 10.0);
  }

  private final ArmSubsystem m_arm;
  private final TransferSubsystem m_transfer;
  private final ShooterSubsystem m_shooter;
  private final SwerveSubsystem m_swerve;
  private final ShootingData m_sd;

  private Pose2d speakerPose;

  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
  private LinearFilter accelXFilter = LinearFilter.movingAverage(2);
  private LinearFilter accelYFilter = LinearFilter.movingAverage(2);

  private final double setpointDebounceTime = 0.20;
  private final double accCompFactor = .5;

  private Debouncer setpointDebouncer = new Debouncer(setpointDebounceTime);

  /** Creates a new ShootWhileMoving. */
  public ShootWhileMoving(

      ArmSubsystem arm,
      TransferSubsystem transfer,
      ShooterSubsystem shooter,
      SwerveSubsystem swerve,
      ShootingData sd) {

    m_arm = arm;
    m_transfer = transfer;
    m_shooter = shooter;
    m_swerve = swerve;
    m_sd = sd;

    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transfer.shootmoving = true;

    speakerPose = AllianceUtil.getSpeakerPose();

    SmartDashboard.putNumberArray("OdometrySpeaker",
        new double[] { speakerPose.getX(), speakerPose.getY(), speakerPose.getRotation().getDegrees() });

    previousSpeeds = m_swerve.getChassisSpeeds();

    setpointDebouncer.calculate(false);

    accelXFilter.reset();
    accelYFilter.reset();
  }

  /*
   * Step 1 Calculate the distance to the real speaker and get the shot time from
   * the table; Shot time * shot distance is used as a substitute for shot
   * velocity
   * Then do the following in a loop up to 5 times
   * Step 2 Create a virtual speaker from the real speaker pose and the robot
   * velocity and acceleration.
   * Step 3 Calculate the distance to the virtual speaker and lookup the shot time
   * for it
   * * align the robot to the virtual target, get the shooter speed and arm angle
   * to the virtual distance and angle lookup tables
   * Step 4 Calculate the difference between real and virtual shot times,
   * Step 5 If the absolute value of the time difference is less than a constant
   * (.01 secs?)
   * then
   * 
   * Make the shot if the robot, shooter and angle are in range
   * 
   * else start over at step 1
   * 
   */

  @Override
  public void execute() {

    Translation2d robotPose = m_swerve.getPose().getTranslation();
    ChassisSpeeds fieldSpeeds = m_swerve.getFieldRelativeSpeeds();

    ChassisSpeeds fieldAcceleration = fieldSpeeds.minus(previousSpeeds);

    double fieldAccelX = accelXFilter.calculate(fieldAcceleration.vxMetersPerSecond);
    double fieldAccelY = accelYFilter.calculate(fieldAcceleration.vyMetersPerSecond);

    SmartDashboard.putNumber("AutoShootMoving/Acceleration X", fieldAccelX);
    SmartDashboard.putNumber("AutoShootMoving/Acceleration Y", fieldAccelY);

    double xdistance = speakerPose.getX() - robotPose.getX();
    double ydistance = speakerPose.getY() - robotPose.getY();
    double distance = Math.sqrt(xdistance * xdistance + ydistance * ydistance);

    double shotTime = Constants.shotTimeMap.get(distance);
    double armAngle = m_sd.armAngleMap.get(distance);
    double rpm = m_sd.shooterRPMMap.get(distance);

    SmartDashboard.putNumber("AutoShootMoving/distancetospeaker", distance);
    SmartDashboard.putNumber("AutoShootMoving/shottimefrommap", shotTime);
    SmartDashboard.putNumber("AutoShootMoving/anglefrommap", armAngle);
    SmartDashboard.putNumber("AutoShootMoving/rpmfrommap", rpm);

    Translation2d virtualGoalLocation = new Translation2d();

    // calculate the virtual goal distance from the robot velocity and the look up
    // the shot time
    // shoot once the actual speaker shot time is very close to the virtual speake
    // shot time

    for (int i = 0; i < 5; i++) {
      double xDistanceTraveledInShotTime = (shotTime * fieldSpeeds.vxMetersPerSecond + fieldAccelX * accCompFactor);

      double virtualGoalX = speakerPose.getX() - xDistanceTraveledInShotTime;

      SmartDashboard.putNumber("AutoShootMoving/VirtShottime", shotTime);

      double yDistanceTraveledInShotTime = (shotTime * fieldSpeeds.vyMetersPerSecond + fieldAccelY * accCompFactor);
      double virtualGoalY = speakerPose.getY()
          - yDistanceTraveledInShotTime;
      SmartDashboard.putNumber("AutoShootMoving/xTravelInShotTime", xDistanceTraveledInShotTime);
      SmartDashboard.putNumber("AutoShootMoving/yTravelInShotTime", yDistanceTraveledInShotTime);
      virtualGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

      double virtualDistance = robotPose.getDistance(virtualGoalLocation);

      double virtualShotTime = Constants.shotTimeMap.get(virtualDistance);

      double virtualArmAngle = m_sd.armAngleMap.get(virtualDistance);

      double virtualRPM = m_sd.shooterRPMMap.get(virtualDistance);

      m_swerve.virtualPose = new Pose2d(virtualGoalLocation, speakerPose.getRotation());

      m_swerve.setTargetPose(m_swerve.virtualPose);

      SmartDashboard.putNumber("AutoShootMoving/virtualDistance", virtualDistance);
      SmartDashboard.putNumber("AutoShootMoving/virtualShotTimefromMap", virtualShotTime);
      SmartDashboard.putNumber("AutoShootMoving/virtualAnglefromMap", virtualArmAngle);
      SmartDashboard.putNumber("AutoShootMoving/virtualRPMfromMap", virtualRPM);

      SmartDashboard.putNumber("AutoShootMoving/ShotTimeDifference", virtualShotTime - shotTime);

      if (Math.abs(virtualShotTime - shotTime) <= 0.01) {
        distance = virtualDistance;
        shotTime = virtualShotTime;
        armAngle = virtualArmAngle;
        rpm = virtualRPM;
        break;
      }
      SmartDashboard.putNumber("AutoShoot/Iterations", i);
    }

    Pose2d virtGoal = new Pose2d(virtualGoalLocation, new Rotation2d());

    SmartDashboard.putNumberArray("AutoShoot/OdometryVG",
        new double[] { virtGoal.getX(), virtGoal.getY(), virtGoal.getRotation().getDegrees() });

    m_arm.setGoal(Units.degreesToRadians(armAngle));

    SmartDashboard.putNumber("AutoShoot/Desired Angle", armAngle);

    // Shooter speed
    m_shooter.topCommandRPM = m_sd.shooterRPMMap.get(distance);

    Rotation2d desiredAngle = robotPose.minus(virtualGoalLocation).getAngle().plus(Rotation2d.fromRadians(Math.PI));

    if (AllianceUtil.isRedAlliance()) {
      desiredAngle = desiredAngle.plus(Rotation2d.fromRadians(Math.PI));
    }

    Rotation2d robotAngle = m_swerve.getHeading();

    double armAngleError = armAngle - m_arm.getAngleDegrees();

    SmartDashboard.putNumber("AutoShootMoving/Arm Angle Error", armAngleError);

    Rotation2d driveAngleError = robotAngle.minus(desiredAngle);

    SmartDashboard.putNumber("AutoShootMoving/Drive Angle Error", driveAngleError.getDegrees());

    if (setpointDebouncer.calculate(
        Math.abs(armAngleError) < ArmConstants.autoShootAngleTolerance
            && m_shooter.bothAtSpeed(.1))
        && Math.abs(driveAngleError.getDegrees()) <= angleToleranceMap.get(distance)) {
      m_transfer.OKShootMoving = true;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.stopMotor();
    m_shooter.stopMotors();
    m_transfer.shootmoving = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
