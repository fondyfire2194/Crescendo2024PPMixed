// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoSourceShootCenterPathfind extends SequentialCommandGroup {

        public AutoSourceShootCenterPathfind(
                        CommandFactory cf,
                        PathFactory pf,
                        SwerveSubsystem swerve,
                        boolean innerNoteFirst) {

                addCommands(
                                Commands.sequence(
                                                // shoot first note

                                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer
                                                                .getFPGATimestamp()),

                                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed),
                                                cf.transferNoteToShooterCommand(),
                                                // move to center note , pick up if there and move to shoot position
                                                // then shoot

                                                Commands.either(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                sourcepaths.SourceToNearCenter4
                                                                                                                .name())),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                sourcepaths.SourceToNearCenter5
                                                                                                                .name())),
                                                                () -> innerNoteFirst),
                                                new WaitCommand(1),
                                                Commands.parallel(
                                                                Commands.either(
                                                                                new RunPPath(swerve,
                                                                                                pf.pathMaps.get(
                                                                                                                sourcepaths.NearCenter4toCenter4
                                                                                                                                .name())),
                                                                                new RunPPath(swerve,
                                                                                                pf.pathMaps.get(
                                                                                                                sourcepaths.NearCenter5toCenter5
                                                                                                                                .name())),
                                                                                () -> innerNoteFirst),

                                                                cf.doIntake()),

                                                Commands.parallel(
                                                                Commands.runOnce(() -> swerve.autostep = 1),
                                                                Commands.runOnce(
                                                                                () -> cf.innerNoteFirst = innerNoteFirst))));

        }

}
