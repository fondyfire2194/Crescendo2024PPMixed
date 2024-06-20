// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.AutoFactory;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Autos.Autos.CenterToShoot;
import frc.robot.commands.Autos.Autos.PickupUsingVision;
import frc.robot.commands.Autos.Autos.TryForAnotherNote;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AutoSourceCompleteVis extends SequentialCommandGroup {

        public AutoSourceCompleteVis(
                        CommandFactory cf,
                        PathFactory pf,
                        AutoFactory af,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        double switchoverdistance,
                        boolean innerNoteFirst) {

                addCommands(
                                // shoot first note
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> swerve.ampActive = false),
                                Commands.runOnce(() -> swerve.sourceActive = true),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),

                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose),

                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed, 20)),

                                Commands.parallel(
                                                new PickupUsingVision(cf,
                                                                pf.pathMaps.get(sourcepaths.SourceToCenter4.name()),
                                                                pf.pathMaps.get(sourcepaths.SourceToCenter5.name()),
                                                                4,
                                                                5,
                                                                transfer, intake, swerve,
                                                                switchoverdistance,
                                                                innerNoteFirst),
                                                Commands.sequence(
                                                                cf.transferNoteToShooterCommand(),
                                                                cf.stopShooter(),
                                                                Commands.waitSeconds(2),
                                                                cf.doIntake(10))),
                                Commands.either(
                                                Commands.either(
                                                                new CenterToShoot(cf, pf.pathMaps.get(
                                                                                sourcepaths.Center4ToSourceShoot
                                                                                                .name()),
                                                                                swerve),
                                                                new CenterToShoot(cf, pf.pathMaps.get(
                                                                                sourcepaths.Center5ToSourceShoot
                                                                                                .name()),
                                                                                swerve),
                                                                () -> innerNoteFirst),

                                                getAnotherNote(swerve, transfer, intake, cf),
                                                () -> transfer.noteAtIntake()),

                                Commands.parallel(
                                                new PickupUsingVision(
                                                                cf,
                                                                pf.pathMaps.get(sourcepaths.SourceShootToCenter5
                                                                                .name()),
                                                                pf.pathMaps.get(sourcepaths.SourceShootToCenter4
                                                                                .name()),
                                                                5,
                                                                4,
                                                                transfer, intake, swerve,
                                                                switchoverdistance,
                                                                innerNoteFirst),

                                                cf.doIntake(10)),

                                Commands.either(
                                                Commands.either(
                                                                new CenterToShoot(cf, pf.pathMaps
                                                                                .get(sourcepaths.Center5ToSourceShoot
                                                                                                .name()),
                                                                                swerve),
                                                                new CenterToShoot(cf, pf.pathMaps
                                                                                .get(sourcepaths.Center4ToSourceShoot
                                                                                                .name()),
                                                                                swerve),
                                                                () -> innerNoteFirst),

                                                getAnotherNote(swerve, transfer, intake, cf),

                                                () -> transfer.noteAtIntake())

                );

        }

        Command getAnotherNote(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf) {

                return Commands.sequence(
                                new RotateToAngle(swerve, -90),
                                Commands.deadline(
                                                new TryForAnotherNote(swerve, transfer, intake,
                                                                CameraConstants.rearCamera.camname),
                                                cf.doIntake(10)),
                                Commands.waitSeconds(.25),
                                Commands.either(
                                                Commands.sequence(
                                                                cf.autopathfind(AllianceUtil
                                                                                .getSourceClearStagePose()),
                                                                Commands.waitSeconds(.25),
                                                                cf.autopathfind(AllianceUtil
                                                                                .getSourceShootPose()),
                                                                Commands.parallel(
                                                                                cf.positionArmRunShooterByDistance(
                                                                                                false, true),
                                                                                new AutoAlignSpeaker(swerve, 1, true)),
                                                                cf.transferNoteToShooterCommand(),
                                                                Commands.runOnce(() -> this.cancel())),
                                                Commands.runOnce(() -> this.cancel()),
                                                () -> transfer.noteAtIntake()));

        }

        public Command pickupCenter4_5(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake, double switchoverdistance,
                        boolean innerNoteFirst) {
                return Commands.parallel(
                                new PickupUsingVision(cf,
                                                pf.pathMaps.get(sourcepaths.SourceToCenter4.name()),
                                                pf.pathMaps.get(sourcepaths.SourceToCenter5.name()),
                                                4,
                                                5,
                                                transfer, intake, swerve,
                                                switchoverdistance,
                                                innerNoteFirst),
                                Commands.sequence(
                                                cf.transferNoteToShooterCommand(),
                                                cf.stopShooter(),
                                                Commands.waitSeconds(2),
                                                cf.doIntake(10)));
        }

        public Command moveShootCenter4_5(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        boolean innerNoteFirst) {
                return Commands.either(
                                new CenterToShoot(cf, pf.pathMaps.get(
                                                sourcepaths.Center4ToSourceShoot
                                                                .name()),
                                                swerve),
                                new CenterToShoot(cf, pf.pathMaps.get(
                                                sourcepaths.Center5ToSourceShoot
                                                                .name()),
                                                swerve),
                                () -> innerNoteFirst);
        }

        public Command tryOtherNote(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve, boolean innerNoteFirst) {
                return (Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(sourcepaths.Center4ToCenter5.name())),
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(sourcepaths.Center5ToCenter4.name())),
                                                () -> innerNoteFirst),
                                cf.doIntake(2)));

        }
}
