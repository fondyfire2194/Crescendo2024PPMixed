// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.CheckOKSwitchToDrive;
import frc.robot.commands.Drive.DriveToPickupNote;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class PickupUsingVision extends SequentialCommandGroup {

        public PickupUsingVision(
                        CommandFactory cf,
                        PathPlannerPath path,
                        PathPlannerPath path1,
                        TransferSubsystem transfer,
                        IntakeSubsystem intake,
                        SwerveSubsystem swerve,
                        boolean innerNoteFirst,
                        double xcropmin,
                        double xcropmax,
                        double ycropmin,
                        double ycropmax,
                        double xcropmin1,
                        double xcropmax1,
                        double ycropmin1,
                        double ycropmax1) {

                addCommands(
                                Commands.either(Commands.runOnce(
                                                () -> LimelightHelpers.setCropWindow(CameraConstants.rearCamera.camname,
                                                                xcropmin, xcropmax, ycropmin, ycropmax)),
                                                Commands.runOnce(
                                                                () -> LimelightHelpers.setCropWindow(
                                                                                CameraConstants.rearCamera.camname,
                                                                                xcropmin1, 1, ycropmin1,
                                                                                ycropmax1)),
                                                () -> innerNoteFirst),
                                Commands.race(
                                                new CheckOKSwitchToDrive(swerve, 2),
                                                // cf.doIntake(),
                                                Commands.either(
                                                                new RunPPath(swerve, path),
                                                                new RunPPath(swerve, path1),
                                                                () -> innerNoteFirst)),
                                Commands.either(
                                                Commands.parallel(
                                                                new DriveToPickupNote(swerve, transfer,
                                                                                intake,
                                                                                CameraConstants.rearCamera.camname),
                                                                cf.doIntake()),
                                                Commands.none(),
                                                () -> swerve.noteSeen));

        }

}
