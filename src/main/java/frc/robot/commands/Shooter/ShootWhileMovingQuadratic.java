// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.ShootingData;

public class ShootWhileMovingQuadratic extends Command {
  /** Creates a new VectorSWM. */
  private final ArmSubsystem m_arm;
  private final TransferSubsystem m_transfer;
  private final ShooterSubsystem m_shooter;
  private final SwerveSubsystem m_swerve;
  private final ShootingData m_sd;
  double noteVelocity;

  public Pose2d speakerPose;
  Translation2d speakerTranslation;
  Pose2d robotPose;

  CommandXboxController test = new CommandXboxController(0);
  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
  private LinearFilter accelXFilter = LinearFilter.movingAverage(2);
  private LinearFilter accelYFilter = LinearFilter.movingAverage(2);

  private final double setpointDebounceTime = 0.20;
  private final double accCompFactor = .5;

  private Debouncer setpointDebouncer = new Debouncer(setpointDebounceTime);

  public ShootWhileMovingQuadratic(ArmSubsystem arm,
      TransferSubsystem transfer,
      ShooterSubsystem shooter,
      SwerveSubsystem swerve,
      ShootingData sd) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_transfer = transfer;
    m_shooter = shooter;
    m_swerve = swerve;
    m_sd = sd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_transfer.shootmoving = true;
    speakerPose = AllianceUtil.getSpeakerPose();
    speakerTranslation = speakerPose.getTranslation();
    robotPose = m_swerve.getPose();
    previousSpeeds = m_swerve.getChassisSpeeds();
    setpointDebouncer.calculate(false);

    accelXFilter.reset();
    accelYFilter.reset();

    SmartDashboard.putNumberArray("SWM/OdometrySpeaker",
        new double[] { speakerPose.getX(), speakerPose.getY(), speakerPose.getRotation().getDegrees() });

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    robotPose = m_swerve.getPose();

    ChassisSpeeds fieldSpeeds = m_swerve.getFieldRelativeSpeeds();

    ChassisSpeeds fieldAcceleration = fieldSpeeds.minus(previousSpeeds);

    Translation2d robotVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    double fieldAccelX = accelXFilter.calculate(fieldAcceleration.vxMetersPerSecond);
    double fieldAccelY = accelYFilter.calculate(fieldAcceleration.vyMetersPerSecond);

    SmartDashboard.putNumber("AutoShootMoving/Acceleration X", fieldAccelX);
    SmartDashboard.putNumber("AutoShootMoving/Acceleration Y", fieldAccelY);

    double stationaryDistance = speakerTranslation.getDistance(robotPose.getTranslation());

    double shotTime = Constants.shotTimeMap.get(stationaryDistance);

    noteVelocity = stationaryDistance / shotTime;

    SmartDashboard.putNumber("SWM/noteVel", noteVelocity);

    // https://stackoverflow.com/questions/2248876/2d-game-fire-at-a-moving-target-by-predicting-intersection-of-projectile-and-u
    // a := sqr(target.velocityX) + sqr(target.velocityY) - sqr(projectile_speed)
    double a = Math.pow(robotVelocity.getX(), 2) + Math.pow(robotVelocity.getY(), 2) - Math.pow(noteVelocity, 2);
    // b := 2 * (target.velocityX * (target.startX - cannon.X) + target.velocityY *
    // (target.startY - cannon.Y))
    double b = 2 * (robotVelocity.getX()
        * (speakerTranslation.getX() - robotPose.getTranslation().getX())
        + robotVelocity.getY() * (speakerTranslation.getY() - robotPose.getTranslation().getY()));

    // c := sqr(target.startX - cannon.X) + sqr(target.startY - cannon.Y) *
    // (target.startY - cannon.Y))
    double c = Math.pow((speakerTranslation.getX() - robotPose.getTranslation().getX()), 2)
        * Math.pow((speakerTranslation.getY() - robotPose.getTranslation().getY()), 2);

    double disc = Math.pow(b, 2) - 4 * a * c;

    SmartDashboard.putNumber("SWM/a", a);
    SmartDashboard.putNumber("SWM/b", b);
    SmartDashboard.putNumber("SWM/c", c);
    SmartDashboard.putNumber("SWM/disc", disc);

    double t1 = (-b + Math.sqrt(disc)) / (2 * a);

    SmartDashboard.putNumber("SWM/t1", t1);

    double t2 = (-b - Math.sqrt(disc)) / (2 * a);
    SmartDashboard.putNumber("SWM/t2", t2);
    double t = t2;

    Translation2d aim = new Translation2d((speakerTranslation.getX() - t * robotVelocity.getX()),
        (speakerTranslation.getY() - t * robotVelocity.getY()));
    previousSpeeds = m_swerve.getChassisSpeeds();

    m_swerve.virtualPose = new Pose2d(aim, new Rotation2d());

    double distance = aim.getDistance(robotPose.getTranslation());

    SmartDashboard.putNumber("SWM/distance", distance);

    double shotTime1 = Constants.shotTimeMap.get(distance);
    double armAngle = m_sd.armAngleMap.get(distance);
    double rpm = m_sd.shooterRPMMap.get(distance);

    m_arm.setGoal(Units.degreesToRadians(armAngle));

    SmartDashboard.putNumber("SWM/Desired Angle", armAngle);

    SmartDashboard.putNumber("SWM/Desired RPM", rpm);

      // Shooter speed
    m_shooter.startShooter(rpm);

    previousSpeeds = fieldSpeeds;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
