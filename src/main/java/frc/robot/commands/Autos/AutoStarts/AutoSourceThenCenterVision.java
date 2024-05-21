// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Drive.CheckOKSwitchToDrive;
import frc.robot.commands.Drive.DriveToPickupNote;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class AutoSourceThenCenterVision extends SequentialCommandGroup {

        private int targetNoteNumber = 4;

        public AutoSourceThenCenterVision(
                        CommandFactory cf,
                        PathFactory pf,
                        SwerveSubsystem swerve,
                        LimelightVision llv,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        boolean innerNoteFirst) {

                addCommands(
                                // shoot first note
                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose),
                                new ParallelRaceGroup(
                                                new CheckOKSwitchToDrive(swerve, llv, 1.1),
                                                Commands.either(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                sourcepaths.SourceToCenter4
                                                                                                                .name())),
                                                                Commands.parallel(
                                                                                new RunPPath(swerve,
                                                                                                pf.pathMaps.get(
                                                                                                                sourcepaths.SourceShootToCenter5
                                                                                                                                .name())),
                                                                                Commands.runOnce(
                                                                                                () -> targetNoteNumber = 5)),
                                                                () -> innerNoteFirst)),

                                new DriveToPickupNote(swerve, transfer, intake,
                                                CameraConstants.rearCamera.camname, llv,
                                                targetNoteNumber),
                                Commands.parallel(
                                                Commands.runOnce(() -> swerve.autostep = 1),
                                                Commands.runOnce(() -> cf.innerNoteFirst = innerNoteFirst)));
        }

}
