// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class PickupUsingPathfind extends SequentialCommandGroup {

        public PickupUsingPathfind(
                        CommandFactory cf,
                        PathPlannerPath path,
                        PathPlannerPath path1,
                        IntakeSubsystem intake,
                        SwerveSubsystem swerve) {

                addCommands(
                                new RunPPath(swerve,
                                                path),
                                new WaitCommand(.1),
                                Commands.parallel(
                                                new RunPPath(swerve, path1),
                                                cf.doIntake()));
        }
}
