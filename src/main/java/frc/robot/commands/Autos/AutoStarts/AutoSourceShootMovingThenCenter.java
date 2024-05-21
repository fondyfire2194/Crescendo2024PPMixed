// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Drive.CheckAlignedToSpeaker;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AutoSourceShootMovingThenCenter extends SequentialCommandGroup {

        public AutoSourceShootMovingThenCenter(
                        CommandFactory cf,
                        PathFactory pf,
                        SwerveSubsystem swerve,
                        boolean innerNoteFirst) {

                addCommands(

                                // path auto shoots on the fly
                                // move to center note , pick up if there and move to shoot position then shoot
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),

                                // path auto shoots on the fly
                                // move to center note , pick up if there and move to shoot position then shoot
                
                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose),
                                cf.setArmShooterValues(Constants.autoShootArmAngle, Constants.autoShootRPM),
                              //  cf.setAutoShootMoving(true),
                                Commands.deadline(
                                                Commands.either(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                sourcepaths.SourceToCenter4
                                                                                                                .name())),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                sourcepaths.SourceShootToCenter5
                                                                                                                .name())),
                                                                () -> innerNoteFirst),

                                                new CheckAlignedToSpeaker(swerve, 2)),
                                Commands.sequence(
                                                new WaitCommand(2),
                                             //   cf.setAutoShootMoving(false),
                                                cf.doIntake()),
                                                Commands.parallel(
                                                        Commands.runOnce(() -> swerve.autostep = 1),
                                                        Commands.runOnce(() -> cf.innerNoteFirst = innerNoteFirst)));
        }

}
