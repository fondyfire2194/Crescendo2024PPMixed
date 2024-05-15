// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;

public class CheckAlignedToSpeaker extends Command {

  private final SwerveSubsystem m_swerve;
  private final double m_alignTolDeg;

  public CheckAlignedToSpeaker(
      SwerveSubsystem swerve,
      double alignTolDeg) {

    m_swerve = swerve;
    m_alignTolDeg = alignTolDeg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

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
    double robotAngle = robotPose.getRotation().getDegrees();

    m_swerve.alignedToTarget = Math.abs(currentAngleToSpeaker - robotAngle) < m_alignTolDeg;

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
