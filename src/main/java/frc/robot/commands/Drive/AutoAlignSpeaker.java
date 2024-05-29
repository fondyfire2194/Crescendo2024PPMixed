// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

public class AutoAlignSpeaker extends Command {

  private final SwerveSubsystem m_swerve;
  private final boolean m_endAtTargets;
  public PIDController m_alignTargetPID = new PIDController(0.03, 0, 0);

  private double rotationVal;

  public AutoAlignSpeaker(
      SwerveSubsystem swerve, boolean endAtTargets) {

    m_swerve = swerve;
    m_endAtTargets = endAtTargets;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_alignTargetPID.enableContinuousInput(-180, 180);
    m_alignTargetPID.setTolerance(0.2);
    m_swerve.targetPose = AllianceUtil.getSpeakerPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get horizontal angle

    Pose2d robotPose = m_swerve.getPose();
    double XDiff = m_swerve.targetPose.getX() - robotPose.getX();
    double YDiff = m_swerve.targetPose.getY() - robotPose.getY();
    double angleRad = Math.atan2(YDiff, XDiff);
    double currentAngleToSpeaker = Units.radiansToDegrees(angleRad);

    rotationVal = m_alignTargetPID.calculate(robotPose.getRotation().getDegrees(), currentAngleToSpeaker);

    m_swerve.alignedToTarget = m_alignTargetPID.atSetpoint();

    m_swerve.drive(
        0, 0,
        rotationVal *= Constants.SwerveConstants.kmaxAngularVelocity,
        true,
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_endAtTargets && m_swerve.alignedToTarget;
  }
}
