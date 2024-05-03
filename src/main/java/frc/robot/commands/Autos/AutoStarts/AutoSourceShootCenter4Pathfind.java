// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoSourceShootCenter4Pathfind extends SequentialCommandGroup {

        // public PathPlannerPath getPath(String pathname) {
        //         return PathPlannerPath.fromPathFile(pathname);
        // }

        // PathConstraints pathConstraints = new PathConstraints(
        //                 3.0, 4.0,
        //                 Units.degreesToRadians(360),
        //                 Units.degreesToRadians(540));

        // public Command getPathToPose(Pose2d pose, PathConstraints constraints) {
        //         return AutoBuilder.pathfindToPose(pose, constraints, 0, 2);
        // }

        public AutoSourceShootCenter4Pathfind(
                        CommandFactory cf,
                        PathFactory pf,
                        PathPlannerPath path,
                        SwerveSubsystem swerve) {

                addCommands(

                                // shoot first note
                                Commands.runOnce(() -> swerve.currentPlannerPath = path),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),

                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose),
                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),
                                cf.transferNoteToShooterCommand(),
                                // move to center note , pick up if there and move to shoot position then shoot

                                new SequentialCommandGroup(
                                                Commands.runOnce(() -> swerve.toLocation = 4),
                                                new RunPPath(swerve,
                                                                path),
                                                new WaitCommand(1),
                                                new ParallelCommandGroup(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(sourcepaths.NearCenter4toCenter4
                                                                                                .name())),
                                                                Commands.waitSeconds(.1)),
                                                cf.doIntake()),

                                Commands.runOnce(() -> swerve.atLocation = 4));

        }

}
