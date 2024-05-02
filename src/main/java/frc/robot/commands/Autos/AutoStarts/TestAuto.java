// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CameraConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.CheckOKSwitchToDrive;
import frc.robot.commands.Drive.DriveToPickupNote;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class TestAuto extends SequentialCommandGroup {

        public PathPlannerPath getPath(String pathname) {
                return PathPlannerPath.fromPathFile(pathname);
        }

        PathConstraints pathConstraints = new PathConstraints(
                        3.0, 4.0,
                        Units.degreesToRadians(360),
                        Units.degreesToRadians(540));

        public Command getPathToPose(Pose2d pose, PathConstraints constraints) {
                return AutoBuilder.pathfindToPose(pose, constraints, 0, 2);
        }

        public TestAuto(
                        CommandFactory cf,
                        PathPlannerPath path,
                        SwerveSubsystem swerve,
                        LimelightVision llv,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer) {

                addCommands(
                                // shoot first note

                                cf.setStartPosebyAlliance(path),
                                new ParallelRaceGroup(
                                                new CheckOKSwitchToDrive(swerve, llv, 1.1),
                                                new RunPPath(swerve,
                                                                path,
                                                                false)),

                                Commands.sequence(
                                                Commands.runOnce(
                                                                () -> SmartDashboard.putNumber("EndP1", swerve.getX())),

                                                new DriveToPickupNote(swerve, transfer, intake,
                                                                CameraConstants.rearCamera.camname, llv,
                                                                4)));

        }

}
