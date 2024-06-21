// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Factories.CommandFactory;

import frc.robot.commands.Drive.PathFindToPickupNote;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.LLPipelines.pipelines;

/** Add your docs here. */
public class PickupUsingVisionNear extends SequentialCommandGroup {

        public PickupUsingVisionNear(
                        CommandFactory cf,
                        PathPlannerPath path,
                        PathPlannerPath path1,
                        int note1,
                        int note2,
                        TransferSubsystem transfer,
                        IntakeSubsystem intake,
                        SwerveSubsystem swerve,
                        LimelightVision llv,
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
                                Commands.either(
                                                new RunPPath(swerve, path),
                                                new RunPPath(swerve, path1),
                                                () -> innerNoteFirst),
                                Commands.either(
                                                new PathFindToPickupNote(swerve, transfer,
                                                                intake, llv, note1,
                                                                CameraConstants.rearCamera.camname),
                                                new PathFindToPickupNote(swerve, transfer,
                                                                intake, llv, note2,
                                                                CameraConstants.rearCamera.camname),
                                                () -> innerNoteFirst),
                                                new PickupUsingPathfind(cf, path, null, path1, null, intake, swerve, innerNoteFirst)
        }
}