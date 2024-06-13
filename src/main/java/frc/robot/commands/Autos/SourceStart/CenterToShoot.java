// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SourceStart;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CenterToShoot extends SequentialCommandGroup {

        public CenterToShoot(
                        CommandFactory cf,
                        PathPlannerPath path,
                        SwerveSubsystem swerve,
                        boolean source) {

                addCommands(
                                Commands.parallel(
                                                new RunPPath(swerve,
                                                                path),
                                                Commands.either(
                                                                cf.positionArmRunShooterSpecialCase(
                                                                                Constants.sourceShootAngle,
                                                                                Constants.sourceShootSpeed),
                                                                cf.positionArmRunShooterSpecialCase(
                                                                                Constants.ampShootAngle,
                                                                                Constants.ampShootSpeed),
                                                                () -> source)),

                                new AutoAlignSpeaker(swerve, true),

                                cf.transferNoteToShooterCommand());

        }
}
