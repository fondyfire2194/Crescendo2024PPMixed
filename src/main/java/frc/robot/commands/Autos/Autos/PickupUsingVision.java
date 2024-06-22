// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.CheckOKSwitchToDrive;
import frc.robot.commands.Drive.DriveToPickupNote;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LLPipelines.pipelines;

/** Add your docs here. */
public class PickupUsingVision extends SequentialCommandGroup {

        public PickupUsingVision(
                        CommandFactory cf,
                        PathPlannerPath path,
                        PathPlannerPath path1,
                        int note1,
                        int note2,
                        TransferSubsystem transfer,
                        IntakeSubsystem intake,
                        SwerveSubsystem swerve,
                        double switchoverdistance,
                        boolean innerNoteFirst) {

                addCommands(
                                Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(
                                                CameraConstants.rearCamera.camname, pipelines.NOTEDET1.ordinal())),
                                Commands.either(
                                                Commands.runOnce(() -> swerve
                                                                .setTargetPose(FieldConstants.centerNotes[note1])),
                                                Commands.runOnce(() -> swerve
                                                                .setTargetPose(FieldConstants.centerNotes[note2])),
                                                () -> innerNoteFirst),

                                Commands.race(
                                                new CheckOKSwitchToDrive(swerve, cf, switchoverdistance),
                                                Commands.either(
                                                                new RunPPath(swerve, path),
                                                                new RunPPath(swerve, path1),
                                                                () -> innerNoteFirst)),
                                Commands.either(
                                                Commands.either(
                                                                new DriveToPickupNote(swerve, transfer,
                                                                                intake, note1,
                                                                                CameraConstants.rearCamera.camname),
                                                                new DriveToPickupNote(swerve, transfer,
                                                                                intake, note2,
                                                                                CameraConstants.rearCamera.camname),
                                                                () -> innerNoteFirst),
                                                Commands.none(),
                                                () -> swerve.noteSeen));

        }

}
