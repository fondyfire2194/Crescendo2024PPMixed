// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.AutoFactory;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Autos.SourceStart.SourceVisionPickup;
import frc.robot.commands.Drive.CheckOKSwitchToDrive;
import frc.robot.commands.Drive.DriveToPickupNote;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AutoSourceShootSelect extends SequentialCommandGroup {

        Supplier<Object> i = () -> "";

        public AutoSourceShootSelect(
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

                                // Commands.runOnce(() -> swerve.currentPlannerPath = path),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),
                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose),

                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle - 5,
                                                Constants.subwfrShooterSpeed),
                                cf.transferNoteToShooterCommand(),
                                cf.armToIntake(),
                                // move to center note , pick up if there and move to shoot position then shoot
                                // visionMethod(swerve, pf, cf, intake,
                                // transfer, innerNoteFirst),
                                new SourceVisionPickup(cf, pf.pathMaps.get(sourcepaths.SourceToCenter4.name()),
                                                pf.pathMaps.get(sourcepaths.SourceToCenter5.name()), 
                                                transfer, intake,
                                                swerve, innerNoteFirst),
                                Commands.parallel(
                                                Commands.runOnce(() -> swerve.autostep = 1),
                                                Commands.runOnce(() -> cf.innerNoteFirst = innerNoteFirst))

                );

        }

        Command pathfindMethod(SwerveSubsystem swerve, PathFactory pf, CommandFactory cf, IntakeSubsystem intake,
                        TransferSubsystem transfer, boolean innerNoteFirst) {

                return Commands.sequence(

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
                                                                                                sourcepaths.NearCenter4ToCenter4
                                                                                                                .name())),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                sourcepaths.NearCenter5ToCenter5
                                                                                                                .name())),
                                                                () -> innerNoteFirst),

                                                cf.doIntake()));

        }

        Command visionMethod(SwerveSubsystem swerve, PathFactory pf, CommandFactory cf, IntakeSubsystem intake,
                        TransferSubsystem transfer, boolean innerNoteFirst) {

                return Commands.sequence(

                                Commands.race(
                                                new CheckOKSwitchToDrive(swerve, 2),
                                                Commands.either(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                sourcepaths.SourceToCenter4
                                                                                                                .name())),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                sourcepaths.SourceShootToCenter5
                                                                                                                .name())),
                                                                () -> innerNoteFirst)),
                                Commands.either(
                                                Commands.parallel(
                                                                new DriveToPickupNote(swerve, transfer, intake,
                                                                                CameraConstants.rearCamera.camname),
                                                                cf.doIntake()),
                                                Commands.none(),
                                                () -> swerve.noteSeen));

        }

}
