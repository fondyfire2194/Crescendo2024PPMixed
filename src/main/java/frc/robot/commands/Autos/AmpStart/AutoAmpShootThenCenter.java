// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AmpStart;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AutoAmpShootThenCenter extends SequentialCommandGroup {

        public AutoAmpShootThenCenter(
                        CommandFactory cf,
                        PathFactory pf,
                        SwerveSubsystem swerve,
                        boolean innerNoteFirst) {

                addCommands(

                                // shoot first note
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),

                                // Commands.runOnce(() -> swerve.currentPlannerPath = path),
                                // Commands.runOnce(() -> swerve.currentPlannerPath = path),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),

                                cf.setStartPosebyAlliance(FieldConstants.ampStartPose),

                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),
                                cf.transferNoteToShooterCommand(),
                                // move to center note , pick up if there and move to shoot position then shoot

                                Commands.parallel(
                                                Commands.either(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                amppaths.AmpToCenter2
                                                                                                                .name())),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                amppaths.AmpToCenter1
                                                                                                                .name())),
                                                                () -> innerNoteFirst),
                                                Commands.sequence(
                                                                Commands.waitSeconds(1),
                                                                cf.doIntake())),
                                Commands.parallel(
                                                Commands.runOnce(() -> swerve.autostep = 1),
                                                Commands.runOnce(() -> cf.innerNoteFirst = innerNoteFirst)));

        }

}
