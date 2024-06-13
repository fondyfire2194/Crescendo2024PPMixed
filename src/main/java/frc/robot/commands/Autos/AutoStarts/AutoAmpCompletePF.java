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
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Autos.Autos.CenterToShoot;
import frc.robot.commands.Autos.Autos.PickupUsingPathfind;
import frc.robot.commands.Autos.Autos.TryForAnotherNote;
import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AutoAmpCompletePF extends SequentialCommandGroup {

        public AutoAmpCompletePF(
                        CommandFactory cf,
                        PathFactory pf,
                        AutoFactory af,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        boolean innerNoteFirst) {

                addCommands(

                                // shoot first note
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> swerve.ampActive = false),
                                Commands.runOnce(() -> swerve.sourceActive = true),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),

                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose),

                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),
                                cf.transferNoteToShooterCommand(),
                                cf.armToIntake(),
                                // move to center note , pick up if there and move to shoot position then shoot

                                new PickupUsingPathfind(cf,
                                                pf.pathMaps.get(amppaths.AmpToNearCenter2.name()),
                                                FieldConstants.centerNote2PickupBlue,
                                                pf.pathMaps.get(amppaths.AmpToNearCenter2.name()),
                                                FieldConstants.centerNote1PickupBlue,
                                                intake, swerve, innerNoteFirst),

                                Commands.either(
                                                Commands.either(
                                                                new CenterToShoot(cf, pf.pathMaps.get(
                                                                                amppaths.Center2ToAmpShoot
                                                                                                .name()),
                                                                                swerve, false),
                                                                new CenterToShoot(cf, pf.pathMaps.get(
                                                                                amppaths.Center1ToAmpShoot
                                                                                                .name()),
                                                                                swerve, false),
                                                                () -> innerNoteFirst),
                                                getAnotherNote(swerve, transfer, intake, cf),
                                                () -> transfer.noteAtIntake()),

                                new PickupUsingPathfind(cf,
                                                pf.pathMaps.get(amppaths.AmpShootToCenter2.name()),
                                                FieldConstants.centerNote2PickupBlue,
                                                pf.pathMaps.get(amppaths.AmpShootToCenter1.name()),
                                                FieldConstants.centerNote1PickupBlue,
                                                intake, swerve, innerNoteFirst),

                                Commands.either(
                                                Commands.either(
                                                                new CenterToShoot(cf, pf.pathMaps
                                                                                .get(amppaths.AmpShootToCenter1
                                                                                                .name()),
                                                                                swerve, false),
                                                                new CenterToShoot(cf, pf.pathMaps
                                                                                .get(amppaths.AmpShootToCenter2
                                                                                                .name()),
                                                                                swerve, true),
                                                                () -> innerNoteFirst),

                                                getAnotherNote(swerve, transfer, intake, cf),

                                                () -> transfer.noteAtIntake()));

        }

        Command getAnotherNote(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf) {

                return Commands.sequence(
                                new RotateToAngle(swerve, 90),
                                intake.startIntakeCommand(),
                                Commands.deadline(
                                                new TryForAnotherNote(swerve, transfer, intake,
                                                                CameraConstants.rearCamera.camname),
                                                new TransferIntakeToSensor(transfer, intake, 6)),

                                Commands.either(
                                                Commands.sequence(
                                                                cf.autopickup(AllianceUtil
                                                                                .getSourceClearStagePose()),
                                                                cf.autopickup(AllianceUtil
                                                                                .getSourceShootPose()),
                                                                Commands.runOnce(() -> this.cancel())),
                                                Commands.runOnce(() -> this.cancel()),
                                                () -> transfer.noteAtIntake()));

        }
}
