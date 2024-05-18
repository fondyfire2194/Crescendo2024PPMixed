// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

public class AlignTargetOdometry extends Command {
  /** Creates a new AlignToTagSet */

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private boolean speaker;
  private boolean virtualTarget;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private final SwerveSubsystem m_swerve;
  public PIDController m_alignTargetPID = new PIDController(0.03, 0, 0);

  private double rotationVal;
 

  public AlignTargetOdometry(
      SwerveSubsystem swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotSup,
      boolean speaker) {
    m_swerve = swerve;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotSup;
    this.speaker = speaker;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_alignTargetPID.enableContinuousInput(-180, 180);
    m_alignTargetPID.setTolerance(0.2);

    m_swerve.targetPose = AllianceUtil.getSpeakerPose();
    if (!speaker) {
     m_swerve. targetPose = AllianceUtil.getLobPose();
      m_alignTargetPID.setTolerance(1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    rotationVal = rotationLimiter.calculate(
        MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));

    // get horizontal angle

    Pose2d robotPose = m_swerve.getPose();
    double XDiff = m_swerve.targetPose.getX() - robotPose.getX();
    double YDiff = m_swerve.targetPose.getY() - robotPose.getY();
    double angleRad = Math.atan2(YDiff, XDiff);
    double currentAngleToSpeaker = Units.radiansToDegrees(angleRad);
    // SmartDashboard.putNumber("CAS", currentAngleToSpeaker);
    // SmartDashboard.putNumber("CAROB", robotPose.getRotation().getDegrees());


    rotationVal = m_alignTargetPID.calculate(robotPose.getRotation().getDegrees(), currentAngleToSpeaker);
  
      m_swerve.drive(
        translationVal *= Constants.SwerveConstants.kmaxSpeed,
        -(strafeVal *= Constants.SwerveConstants.kmaxSpeed),
        rotationVal *= Constants.SwerveConstants.kmaxAngularVelocity,
        true,
        true,
        false);

    m_swerve.alignedToTarget = m_alignTargetPID.atSetpoint();
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
