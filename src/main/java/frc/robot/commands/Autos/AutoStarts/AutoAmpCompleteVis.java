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
import frc.robot.commands.Autos.Autos.PickupUsingVision;
import frc.robot.commands.Autos.Autos.TryForAnotherNote;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LLPipelines;

/** Add your docs here. */
public class AutoAmpCompleteVis extends SequentialCommandGroup {

        public AutoAmpCompleteVis(
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

                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),
                                Commands.runOnce(() -> swerve.sourceActive = false),
                                Commands.runOnce(() -> swerve.ampActive = true),
                                cf.setStartPosebyAlliance(FieldConstants.ampStartPose),

                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed, 10)),

                                // move to center note , pick up if there and move to shoot position then shoot
                                Commands.parallel(
                                                new PickupUsingVision(cf,
                                                                pf.pathMaps.get(amppaths.AmpToCenter2.name()),
                                                                pf.pathMaps.get(amppaths.AmpToCenter1.name()),
                                                                2,
                                                                1,
                                                                transfer, intake, swerve,
                                                                switchoverdistance,
                                                                innerNoteFirst),
                                                Commands.sequence(
                                                                cf.transferNoteToShooterCommand(),
                                                                cf.stopShooter(),
                                                                Commands.waitSeconds(2),
                                                                cf.doIntake(2))),
                                Commands.either(

                                                Commands.either(
                                                                new CenterToShoot(cf, pf.pathMaps.get(
                                                                                amppaths.Center2ToAmpShoot
                                                                                                .name()),
                                                                                swerve),
                                                                new CenterToShoot(cf, pf.pathMaps.get(
                                                                                amppaths.Center1ToAmpShoot
                                                                                                .name()),
                                                                                swerve),
                                                                () -> innerNoteFirst),

                                                getAnotherNote(swerve, transfer, intake, cf),

                                                () -> transfer.noteAtIntake()),
                                Commands.parallel(
                                                new PickupUsingVision(
                                                                cf,
                                                                pf.pathMaps.get(amppaths.AmpShootToCenter1.name()),
                                                                pf.pathMaps.get(amppaths.AmpShootToCenter2.name()),
                                                                1,
                                                                2,
                                                                transfer, intake, swerve,
                                                                switchoverdistance,
                                                                innerNoteFirst),

                                                cf.doIntake(5)),

                                Commands.either(

                                                Commands.either(
                                                                new CenterToShoot(cf, pf.pathMaps.get(
                                                                                amppaths.Center1ToAmpShoot
                                                                                                .name()),
                                                                                swerve),
                                                                new CenterToShoot(cf, pf.pathMaps.get(
                                                                                amppaths.Center2ToAmpShoot
                                                                                                .name()),
                                                                                swerve),
                                                                () -> innerNoteFirst),

                                                getAnotherNote(swerve, transfer, intake, cf),
                                                () -> transfer.noteAtIntake()));
        }

        Command getAnotherNote(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf) {
                return Commands.sequence(
                                new RotateToAngle(swerve, 90),
                                Commands.deadline(
                                                new TryForAnotherNote(swerve, transfer, intake,
                                                                CameraConstants.rearCamera.camname),
                                                cf.doIntake(10)),
                                Commands.waitSeconds(.25),
                                Commands.either(
                                                Commands.sequence(
                                                                cf.autopathfind(AllianceUtil
                                                                                .getAmpClearStagePose(), SwerveConstants.pfConstraints),
                                                                Commands.waitSeconds(.25),
                                                                cf.autopathfind(AllianceUtil
                                                                                .getAmpShootPose(), SwerveConstants.pfConstraints),
                                                                Commands.parallel(
                                                                                cf.positionArmRunShooterByDistance(
                                                                                                false, true),
                                                                                new AutoAlignSpeaker(swerve, 1, true)),
                                                                cf.transferNoteToShooterCommand(),
                                                                Commands.runOnce(() -> this.cancel())),
                                                Commands.runOnce(() -> this.cancel()),
                                                () -> transfer.noteAtIntake()));
        }

}
