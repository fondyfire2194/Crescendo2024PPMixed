// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class CenterToShoot extends SequentialCommandGroup {

        public CenterToShoot(
                        CommandFactory cf,
                        PathPlannerPath path,
                        SwerveSubsystem swerve,
                        boolean pathfind,
                        boolean amp) {
                Pose2d shootpose = amp ? FieldConstants.ampShootBlue : FieldConstants.sourceShootBlue;
                addCommands(
                                Commands.sequence(
                                                Commands.parallel(
                                                                Commands.deadline(
                                                                                Commands.either(
                                                                                                cf.autopathfind(AllianceUtil
                                                                                                                .getAlliancePose(
                                                                                                                                shootpose),
                                                                                                                0),
                                                                                                new RunPPath(swerve,
                                                                                                                path),
                                                                                                () -> pathfind)),
                                                                cf.positionArmRunShooterSpecialCase(25, 3100, 15)),
                                                Commands.parallel(
                                                                cf.positionArmRunShooterByDistance(false, true),
                                                                new AutoAlignSpeaker(swerve, .2, true)),
                                                cf.transferNoteToShooterCommand()));
        }
}
