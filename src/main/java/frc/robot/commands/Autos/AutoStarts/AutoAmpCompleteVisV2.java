// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import com.pathplanner.lib.commands.PathfindLTV;

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
import frc.robot.Factories.PathFactory.amppaths;
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
public class AutoAmpCompleteVisV2 extends SequentialCommandGroup {

        public AutoAmpCompleteVisV2(
                        CommandFactory cf,
                        PathFactory pf,
                        AutoFactory af,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        LimelightVision llv,
                        double switchoverdistance,
                        boolean innerNoteFirst,
                        boolean pathfind) {

                addCommands( // note
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> swerve.ampActive = true),
                                Commands.runOnce(() -> swerve.sourceActive = false),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),
                                Commands.runOnce(() -> swerve.pickupTargetX = FieldConstants.FIELD_LENGTH / 2),
                                cf.setStartPosebyAlliance(FieldConstants.ampStartPose),

                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed, 20)),

                                Commands.parallel(
                                                cf.transferNoteToShooterCommand(),

                                                // pick up first note either 4 or 5 using full path or near path the
                                                // rear camera
                                                Commands.either(
                                                                pickupCenter1_2PF(cf, innerNoteFirst),
                                                                pickupCenter1_2(cf, pf, swerve, transfer, intake,
                                                                                switchoverdistance,
                                                                                innerNoteFirst),
                                                                () -> pathfind)),
                                Commands.either(
                                                moveShootCenter1_2(cf, pf, swerve, innerNoteFirst, pathfind),
                                                tryOtherNote(pf, cf, swerve, transfer, innerNoteFirst),
                                                () -> transfer.noteAtIntake() && !transfer.skipFirstNoteInSim),

                                Commands.either(
                                                Commands.either(
                                                                pickupNoteAfterShootPF(cf, innerNoteFirst),
                                                                pickUpNoteAfterShoot(pf, cf, swerve, transfer,
                                                                                intake, switchoverdistance,
                                                                                innerNoteFirst),
                                                                () -> pathfind),
                                                Commands.none(),
                                                () -> !transfer.noteAtIntake()
                                                                && (!AllianceUtil.isRedAlliance()
                                                                                && swerve.getX() < (FieldConstants.FIELD_LENGTH
                                                                                                / 2 - 2)
                                                                                || AllianceUtil.isRedAlliance()
                                                                                                && swerve.getX() > (FieldConstants.FIELD_LENGTH
                                                                                                                / 2
                                                                                                                + 2))),

                                Commands.either(
                                                moveShootCenter1_2(cf, pf, swerve, !innerNoteFirst, pathfind),
                                                getAnotherNote(swerve, transfer, intake, cf, pf),
                                                () -> transfer.noteAtIntake() && !transfer.skipSecondNoteInSim),

                                Commands.either(
                                                new RunPPath(swerve, pf.pathMaps.get(amppaths.AmpShootToCenter2
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
                                new RotateToAngle(swerve, 90),
                                Commands.deadline(
                                                new TryForAnotherNote(swerve, transfer, intake,
                                                                CameraConstants.rearCamera.camname),
                                                cf.doIntake(10)),
                                Commands.waitSeconds(.25),
                                Commands.either(
                                                Commands.sequence(
                                                                cf.autopathfind(AllianceUtil
                                                                                .getAmpClearStagePose(),
                                                                                SwerveConstants.pfConstraints, 0, 0),
                                                                Commands.waitSeconds(.25),
                                                                cf.autopathfind(AllianceUtil
                                                                                .getAmpShootPose(),
                                                                                SwerveConstants.pfConstraints, 0, 0),
                                                                Commands.parallel(
                                                                                cf.positionArmRunShooterByDistance(
                                                                                                false, true),
                                                                                new AutoAlignSpeaker(swerve, 1, true)),
                                                                cf.transferNoteToShooterCommand(),
                                                                new RunPPath(swerve, pf.pathMaps
                                                                                .get(amppaths.AmpShootToCenter2
                                                                                                .name())),
                                                                Commands.runOnce(() -> this.cancel())),
                                                Commands.runOnce(() -> this.cancel()),
                                                () -> transfer.noteAtIntake()));

        }

        public Command pickupCenter1_2(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake, double switchoverdistance,
                        boolean innerNoteFirst) {
                return Commands.parallel(
                                new PickupUsingVision(
                                                cf,
                                                pf.pathMaps.get(amppaths.AmpToCenter1.name()),
                                                pf.pathMaps.get(amppaths.AmpToCenter2.name()),
                                                1,
                                                2,
                                                transfer, intake, swerve,
                                                switchoverdistance,
                                                innerNoteFirst),
                                Commands.sequence(
                                                cf.stopShooter(),
                                                Commands.waitSeconds(2),
                                                cf.doIntake(300)));
        }

        public Command pickupCenter1_2PF(CommandFactory cf, boolean innerNoteFirst) {
                return Commands.sequence(
                                cf.autopathfind(AllianceUtil.getAlliancePose(FieldConstants.ampNoteGappose), 0),
                                Commands.either(
                                                cf.autopathfind(AllianceUtil.flipFieldAngle(
                                                                FieldConstants.centerNotesPickup[1]), 2),
                                                cf.autopathfind(AllianceUtil.flipFieldAngle(
                                                                FieldConstants.centerNotesPickup[2]), 2),
                                                () -> innerNoteFirst),
                                cf.doIntakeDelayed(2, 3));
        }

        public Command moveShootCenter1_2(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        boolean innerNoteFirst, boolean pathfind) {
                return Commands.either(
                                new CenterToShoot(cf, pf.pathMaps.get(
                                                amppaths.Center1ToAmpShoot
                                                                .name()),
                                                swerve, pathfind, true),
                                new CenterToShoot(cf, pf.pathMaps.get(amppaths.Center2ToAmpShoot
                                                .name()),
                                                swerve, pathfind, true),
                                () -> innerNoteFirst);
        }

        public Command tryOtherNote(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, boolean innerNoteFirst) {
                return (Commands.sequence(
                                Commands.parallel(
                                                Commands.either(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(amppaths.Center1ToCenter2
                                                                                                .name())),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(amppaths.Center2toCenter1
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
                                                pf.pathMaps.get(amppaths.AmpShootToCenter2
                                                                .name()),
                                                pf.pathMaps.get(amppaths.AmpShootToCenter1
                                                                .name()),
                                                2,
                                                1,
                                                transfer, intake, swerve,
                                                switchoverdistance,
                                                innerNoteFirst),

                                cf.doIntake(2));

        }

        public Command pickupNoteAfterShootPF(CommandFactory cf, boolean innerNoteFirst) {
                return Commands.sequence(
                                Commands.either(
                                                cf.autopathfind(AllianceUtil.flipFieldAngle(
                                                                FieldConstants.centerNotesPickup[2]), 2),
                                                cf.autopathfind(AllianceUtil.flipFieldAngle(
                                                                FieldConstants.centerNotesPickup[1]), 2),
                                                () -> innerNoteFirst),
                                cf.doIntakeDelayed(1, 3));
        }

}
