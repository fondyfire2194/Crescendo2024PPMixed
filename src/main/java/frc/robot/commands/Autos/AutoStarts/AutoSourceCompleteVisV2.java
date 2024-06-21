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
import frc.robot.Constants.SwerveConstants;
import frc.robot.Factories.AutoFactory;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Autos.Autos.CenterToShoot;
import frc.robot.commands.Autos.Autos.PickupUsingVision;
import frc.robot.commands.Autos.Autos.PickupUsingVisionNear;
import frc.robot.commands.Autos.Autos.TryForAnotherNote;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AutoSourceCompleteVisV2 extends SequentialCommandGroup {

        public AutoSourceCompleteVisV2(
                        CommandFactory cf,
                        PathFactory pf,
                        AutoFactory af,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        LimelightVision llv,
                        double switchoverdistance,
                        boolean innerNoteFirst,
                        boolean useNear) {

                addCommands(
                                // setup
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> swerve.ampActive = false),
                                Commands.runOnce(() -> swerve.sourceActive = true),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),

                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose),
                                // shoot first note
                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed, 20)),
                                cf.transferNoteToShooterCommand(),

                                // pick up first note either 4 or 5 using full path or near path the rear camera
                                Commands.either(
                                                pickupCenter4_5Near(cf, pf, swerve, transfer, intake,
                                                                llv, innerNoteFirst),
                                                pickupCenter4_5(cf, pf, swerve, transfer, intake, switchoverdistance,
                                                                innerNoteFirst),
                                                () -> useNear),
                                // if a note was picked up then go shoot it. If not do U move to try the
                                // adjacent note
                                Commands.either(
                                                moveShootCenter4_5(cf, pf, swerve, innerNoteFirst),
                                                tryOtherNote(pf, cf, swerve, transfer, innerNoteFirst),
                                                () -> transfer.noteAtIntake() && !transfer.skipFirstNoteInSim),

                                // This command selector has 2 conditions where it doesn't have a note and one
                                // where it does
                                // The having shot condition means the robot is away from center field so
                                // pickupaftershoot will happen
                                // any other condition means Commands.none will happen. Either way the next
                                // command has to take care of it

                                Commands.either(
                                                Commands.either(
                                                                pickUpNoteAfterShootNear(pf, cf, swerve, transfer,
                                                                                intake,
                                                                                llv, innerNoteFirst),
                                                                pickUpNoteAfterShoot(pf, cf, swerve, transfer,
                                                                                intake, switchoverdistance,
                                                                                innerNoteFirst),
                                                                () -> useNear),

                                                Commands.none(),
                                                () -> !transfer.noteAtIntake()
                                                                && (!AllianceUtil.isRedAlliance()
                                                                                && swerve.getX() < (FieldConstants.FIELD_LENGTH
                                                                                                / 2 - 2)
                                                                                || AllianceUtil.isRedAlliance()
                                                                                                && swerve.getX() > (FieldConstants.FIELD_LENGTH
                                                                                                                / 2
                                                                                                                + 2))),
                                // This command is decided by whether a note is present. If it is shot it if not
                                // turn and move up center line to find note
                                Commands.either(
                                                moveShootCenter4_5(cf, pf, swerve, !innerNoteFirst),
                                                getAnotherNote(swerve, transfer, intake, cf, pf),
                                                () -> transfer.noteAtIntake() && !transfer.skipSecondNoteInSim),
                                // Same go shoot decision as earlier except turn 90 and search on center line
                                // for notes
                                Commands.either(
                                                new RunPPath(swerve, pf.pathMaps.get(sourcepaths.SourceShootToCenter4
                                                                .name())),
                                                Commands.none(),
                                                () -> (!AllianceUtil.isRedAlliance()
                                                                && swerve.getX() < (FieldConstants.FIELD_LENGTH
                                                                                / 2 - 2)
                                                                || AllianceUtil.isRedAlliance()
                                                                                && swerve.getX() > (FieldConstants.FIELD_LENGTH
                                                                                                / 2
                                                                                                + 2))),

                                getAnotherNote(swerve, transfer, intake, cf, pf));

        }

        Command getAnotherNote(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf, PathFactory pf) {

                return Commands.sequence(
                                Commands.runOnce(() -> transfer.simnoteatintake = false),
                                new RotateToAngle(swerve, -90),
                                Commands.deadline(
                                                new TryForAnotherNote(swerve, transfer, intake,
                                                                CameraConstants.rearCamera.camname),
                                                cf.doIntake(10)),
                                Commands.waitSeconds(.25),
                                Commands.either(
                                                Commands.sequence(
                                                                cf.autopathfind(AllianceUtil
                                                                                .getSourceClearStagePose(),
                                                                                SwerveConstants.pfConstraints),
                                                                Commands.waitSeconds(.25),
                                                                cf.autopathfind(AllianceUtil
                                                                                .getSourceShootPose(),
                                                                                SwerveConstants.pfConstraints),
                                                                Commands.parallel(
                                                                                cf.positionArmRunShooterByDistance(
                                                                                                false, true),
                                                                                new AutoAlignSpeaker(swerve, 1, true)),
                                                                cf.transferNoteToShooterCommand(),
                                                                new RunPPath(swerve, pf.pathMaps
                                                                                .get(sourcepaths.SourceShootToCenter4
                                                                                                .name())),
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
                                                cf.stopShooter(),
                                                Commands.waitSeconds(2),
                                                cf.doIntake(300)));
        }

        public Command pickupCenter4_5Near(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake, LimelightVision llv,
                        boolean innerNoteFirst) {
                return Commands.parallel(
                                new PickupUsingVisionNear(cf,
                                                pf.pathMaps.get(sourcepaths.SourceToNearCenter4.name()),
                                                pf.pathMaps.get(sourcepaths.SourceToNearCenter5.name()),
                                                4,
                                                5,
                                                transfer, intake, swerve,
                                                llv, innerNoteFirst),
                                Commands.sequence(
                                                cf.stopShooter(),
                                                Commands.waitSeconds(2),
                                                cf.doIntake(300)));
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

        public Command tryOtherNote(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, boolean innerNoteFirst) {
                return (Commands.sequence(
                                Commands.parallel(
                                                Commands.either(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(sourcepaths.Center4ToCenter5
                                                                                                .name())),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(sourcepaths.Center5ToCenter4
                                                                                                .name())),
                                                                () -> innerNoteFirst),
                                                cf.doIntake(2))));

        }

        public Command pickUpNoteAfterShoot(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake, double switchoverdistance,
                        boolean innerNoteFirst) {

                return Commands.parallel(
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

                                cf.doIntake(2));

        }

        public Command pickUpNoteAfterShootNear(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake, LimelightVision llv,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                new PickupUsingVisionNear(
                                                cf,
                                                pf.pathMaps.get(sourcepaths.SourceShootToNearCenter5
                                                                .name()),
                                                pf.pathMaps.get(sourcepaths.SourceShootToNearCenter4
                                                                .name()),
                                                5,
                                                4,
                                                transfer, intake, swerve,
                                                llv, innerNoteFirst),

                                cf.doIntake(2));

        }
}
