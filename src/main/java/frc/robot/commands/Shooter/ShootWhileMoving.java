// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import monologue.Logged;

import java.util.function.DoubleSupplier;

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
  private LimelightVision m_llv;

  public PIDController m_alignTargetPID = new PIDController(0.03, 0, 0);
  private Pose2d speakerPose;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private DoubleSupplier m_rotationSup;
  private double rotationVal;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();

  private LinearFilter accelXFilter = LinearFilter.movingAverage(2);
  private LinearFilter accelYFilter = LinearFilter.movingAverage(2);

  private final double setpointDebounceTime = 0.20;
  private final double feedTime = 0.100;

  private Debouncer setpointDebouncer = new Debouncer(setpointDebounceTime);

  /** Creates a new ShootWhileMoving. */
  public ShootWhileMoving(

      ArmSubsystem arm,
      TransferSubsystem transfer,
      ShooterSubsystem shooter,
      SwerveSubsystem swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotSup,
      LimelightVision llv) {
    m_arm = arm;
    m_transfer = transfer;
    m_shooter = shooter;
    m_swerve = swerve;
    m_translationSup = translationSup;
    m_strafeSup = strafeSup;
    m_rotationSup = rotSup;

    m_llv = llv;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_alignTargetPID.enableContinuousInput(-180, 180);
    m_alignTargetPID.setTolerance(0.2);

    speakerPose = AllianceUtil.getSpeakerPose();
    SmartDashboard.putNumberArray("OdometrySpeaker",
        new double[] { speakerPose.getX(), speakerPose.getY(), speakerPose.getRotation().getDegrees() });

    previousSpeeds = m_swerve.getSpeeds();

    setpointDebouncer.calculate(false);

    accelXFilter.reset();
    accelYFilter.reset();
  }

  /*
   * Step 1 Calculate the distance to the real speaker and get the shot time from
   * the table;
   * Then do the following in a loop up to 5 times
   * Step 2 Create a virtual speaker from the real speaker pose and the robot
   * velocity and acceleration.
   * Step 3 Calculate the distance to the virtual speaker and lookup the shot time
   * for it
   * * align the robot to the virtual target, get the shooter speed and arm angle
   * to the virtual
   * distance and angle lookup tables
   * Step 4 Calculate the difference between real and virtual shot times,
   * Step 5 If the absolute value of the time difference is less than a constant
   * (.01 secs?)
   * then
   * 
   * Make the shot if the robot, shooter and angle are in range
   * 
   * else start over at step 1
   * 
   * 
   * 
   * 
   * 
   * 
   */

  @Override
  public void execute() {

    /* Get Values, Deadband */
    double translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    rotationVal = rotationLimiter.calculate(
        MathUtil.applyDeadband(m_rotationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));

    Translation2d robotPose = m_swerve.getPose().getTranslation();
    ChassisSpeeds fieldSpeeds = m_swerve.getSpeeds();

    ChassisSpeeds fieldAcceleration = fieldSpeeds.minus(previousSpeeds).div(0.020);

    double fieldAccelX = accelXFilter.calculate(fieldAcceleration.vxMetersPerSecond);
    double fieldAccelY = accelYFilter.calculate(fieldAcceleration.vyMetersPerSecond);

    SmartDashboard.putNumber("AutoShoot/Acceleration X", fieldAccelX);
    SmartDashboard.putNumber("AutoShoot/Acceleration Y", fieldAccelY);

    // double distance =
    // m_llv.getDistanceFromSpeakerTag(CameraConstants.frontLeftCamera);
    double distance = speakerPose.getX() - robotPose.getX();
    double shotTime = Constants.shotTimeMap.get(distance);

    double armAngle = Constants.armAngleMap.get(distance);

    SmartDashboard.putNumber("AutoShoot/distancetospeaker", distance);
    SmartDashboard.putNumber("AutoShoot/shottimefrommap", shotTime);
    SmartDashboard.putNumber("AutoShoot/anglefrommap", armAngle);

    Translation2d virtualGoalLocation = new Translation2d();

    for (int i = 0; i < 5; i++) {

      double virtualGoalX = speakerPose.getX()
          - shotTime * (fieldSpeeds.vxMetersPerSecond + fieldAccelX * feedTime * 0.5);
      double virtualGoalY = speakerPose.getY()
          - shotTime * (fieldSpeeds.vyMetersPerSecond + fieldAccelY * feedTime * 0.5);
      
      virtualGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

      double virtualDistance = robotPose.minus(virtualGoalLocation).getNorm();

      double virtualShotTime = Constants.shotTimeMap.get(virtualDistance);

      double virtualArmAngle = Constants.armAngleMap.get(virtualDistance);

      SmartDashboard.putNumber("AutoShoot/VirtualDistance", virtualDistance);
      SmartDashboard.putNumber("AutoShoot/VirtualShotTimefromMap", virtualShotTime);
      SmartDashboard.putNumber("AutoShoot/VirtualAnglefromMap", virtualArmAngle);
      SmartDashboard.putNumber("AutoShoot/ShotTimeDifference", virtualShotTime - shotTime);

      if (Math.abs(virtualShotTime - shotTime) <= 0.01) {
        shotTime = virtualShotTime;
        armAngle = virtualArmAngle;
        distance = virtualDistance;
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
    m_shooter.topCommandRPM = Constants.shooterRPMMap.get(distance);

    Rotation2d desiredAngle = robotPose.minus(virtualGoalLocation).getAngle().plus(Rotation2d.fromRadians(Math.PI));

    if (AllianceUtil.isRedAlliance()) {
      desiredAngle = desiredAngle.plus(Rotation2d.fromRadians(Math.PI));
    }

    Rotation2d robotAngle = m_swerve.getHeading();

    double armAngleError = armAngle - m_arm.getAngleDegrees();

    SmartDashboard.putNumber("AutoShoot/Arm Angle Error", armAngleError);

    Rotation2d driveAngleError = robotAngle.minus(desiredAngle);

    SmartDashboard.putNumber("AutoShoot/Drive Angle Error", driveAngleError.getDegrees());

    if (setpointDebouncer.calculate(
        Math.abs(armAngleError) < ArmConstants.autoShootAngleTolerance
            && m_shooter.bothAtSpeed(.1))
        && Math.abs(driveAngleError.getDegrees()) <= angleToleranceMap.get(distance)) {
      m_transfer.transferToShooter();

    }
    previousSpeeds = fieldSpeeds;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      rotationVal = m_alignTargetPID.calculate(driveAngleError.getDegrees(), 0);
    } else {
      rotationVal = m_alignTargetPID.calculate(driveAngleError.getDegrees(), 180);
    }

    m_swerve.drive(
        translationVal *= Constants.SwerveConstants.kmaxSpeed,
        -(strafeVal *= Constants.SwerveConstants.kmaxSpeed),
        rotationVal *= Constants.SwerveConstants.kmaxAngularVelocity,
        true,
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.stopMotor();
    m_shooter.stopMotors();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
