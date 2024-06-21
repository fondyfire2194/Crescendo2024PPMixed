// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Factories.CommandFactory;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPose extends Command {
  /** Creates a new DriveToPose. */
  Command pathfindingCommand;
  SwerveSubsystem m_swerve;
  CommandFactory m_cf;

  public DriveToPose(SwerveSubsystem swerve, Pose2d desiredPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;

    pathfindingCommand = swerve.driveToPose(desiredPose);
  }

  public DriveToPose(SwerveSubsystem swerve, CommandFactory cf) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_cf = cf;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathfindingCommand =

        Commands.parallel(
            m_swerve.driveToPose(m_swerve.getPathfindPose()),
            m_cf.doIntake(10));
    pathfindingCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.remainingdistance = .1;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }
}
