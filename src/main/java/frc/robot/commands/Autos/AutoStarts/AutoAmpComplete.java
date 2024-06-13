// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Factories.AutoFactory;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.commands.Autos.Autos.CenterToShoot;
import frc.robot.commands.Autos.Autos.PickupUsingVision;
import frc.robot.commands.Autos.Autos.TryForAnotherNote;
import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AutoAmpComplete extends SequentialCommandGroup {

        public AutoAmpComplete(
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

                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),

                                cf.setStartPosebyAlliance(FieldConstants.ampStartPose),

                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle - 5,
                                                Constants.subwfrShooterSpeed),
                                cf.transferNoteToShooterCommand(),
                                cf.armToIntake(),
                                // move to center note , pick up if there and move to shoot position then shoot

                                new PickupUsingVision(cf,
                                                pf.pathMaps.get(amppaths.AmpShootToCenter2.name()),
                                                pf.pathMaps.get(amppaths.AmpShootToCenter1.name()),
                                                transfer, intake, swerve,innerNoteFirst,
                                                -1, 1, -1, 1),

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

                                new PickupUsingVision(
                                                cf,
                                                pf.pathMaps.get(amppaths.AmpShootToCenter1.name()),
                                                pf.pathMaps.get(amppaths.AmpShootToCenter2.name()),
                                                transfer, intake, swerve,innerNoteFirst,
                                                -1, 1, -1, 1),

                                Commands.either(

                                                Commands.either(
                                                                new CenterToShoot(cf, pf.pathMaps.get(
                                                                                amppaths.Center1ToAmpShoot
                                                                                                .name()),
                                                                                swerve, false),
                                                                new CenterToShoot(cf, pf.pathMaps.get(
                                                                                amppaths.Center2ToAmpShoot
                                                                                                .name()),
                                                                                swerve, false),
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
                                                                                .getAmpClearStagePose()),
                                                                cf.autopickup(AllianceUtil
                                                                                .getAmpShootPose())),
                                                Commands.none(),
                                                () -> transfer.noteAtIntake()));

        }

}
