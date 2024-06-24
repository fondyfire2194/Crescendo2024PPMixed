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
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.commands.Autos.Autos.CenterToShoot;
import frc.robot.commands.Autos.Autos.LookForAnotherNote;
import frc.robot.commands.Drive.AutoAlignNote;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Drive.DriveToPickupNote;
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
                        boolean innerNoteFirst) {

                addCommands( // note
                                Commands.runOnce(() -> transfer.simnoteatintake = false),
                                Commands.runOnce(() -> intake.resetIsIntakingSim()),
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
                                                pickupCenter1_2(cf, pf, swerve, transfer, intake, innerNoteFirst)),

                                // if note in intake go shoot it or try the adjacent one if not
                                Commands.either(
                                                moveShootCenter1_2(cf, pf, swerve, innerNoteFirst),
                                                getAnotherNote(swerve, transfer, intake, cf, pf),
                                                () -> transfer.noteAtIntake()),

                                pickUpNoteAfterShoot(pf, cf, swerve, transfer, intake,
                                                innerNoteFirst),

                                Commands.either(
                                                moveShootCenter1_2(cf, pf, swerve, !innerNoteFirst),
                                                getAnotherNote(swerve, transfer, intake, cf, pf),
                                                () -> transfer.noteAtIntake()));

        }

        public Command pickupCenter1_2(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake,
                        boolean innerNoteFirst) {

                return Commands.sequence(
                                Commands.either(Commands.sequence(
                                                Commands.runOnce(() -> swerve.targetNote = 2),
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.AmpToNearCenter2.name()))),
                                                Commands.sequence(
                                                                Commands.runOnce(() -> swerve.targetNote = 1),
                                                                new RunPPath(swerve, pf.pathMaps.get(
                                                                                amppaths.AmpToNearCenter1.name()))),
                                                () -> innerNoteFirst),
                                new AutoAlignNote(swerve, 5, true),
                                Commands.parallel(
                                                new DriveToPickupNote(swerve, transfer, intake),
                                                cf.doIntake(5)));
        }

        public Command moveShootCenter1_2(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        boolean innerNoteFirst) {
                return Commands.either(
                                new CenterToShoot(cf, pf.pathMaps.get(amppaths.Center2ToAmpShoot
                                                .name()),
                                                swerve),
                                new CenterToShoot(cf, pf.pathMaps.get(amppaths.Center1ToAmpShoot
                                                .name()),
                                                swerve),
                                () -> innerNoteFirst);
        }

        public Command pickUpNoteAfterShoot(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake, boolean innerNoteFirst) {

                return Commands.sequence(
                                Commands.either(Commands.sequence(
                                                Commands.runOnce(() -> swerve.targetNote = 1),
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.AmpShootToNearCenter1
                                                                                .name()))),
                                                Commands.sequence(
                                                                Commands.runOnce(() -> swerve.targetNote = 2),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(amppaths.AmpShootToNearCenter2
                                                                                                .name()))),

                                                () -> innerNoteFirst),

                                new AutoAlignNote(swerve, 5, true),

                                Commands.parallel(
                                                new DriveToPickupNote(swerve, transfer, intake),
                                                cf.doIntake(5)));

        }

        public Command tryOtherNote(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, boolean innerNoteFirst) {
                return Commands.sequence(
                                Commands.parallel(
                                                Commands.either(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(amppaths.Center2ToCenter1
                                                                                                .name())),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(amppaths.Center1ToCenter2
                                                                                                .name())),
                                                                () -> innerNoteFirst),
                                                cf.doIntake(2)));

        }

        Command getAnotherNote(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf, PathFactory pf) {

                return Commands.sequence(
                                Commands.runOnce(() -> transfer.simnoteatintake = false),
                                new RotateToAngle(swerve, 90),
                                Commands.deadline(
                                                new LookForAnotherNote(swerve, transfer, intake),
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
}