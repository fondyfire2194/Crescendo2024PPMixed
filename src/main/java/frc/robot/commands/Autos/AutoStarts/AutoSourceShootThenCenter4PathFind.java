// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GeometryUtil;

/** Add your docs here. */
public class AutoSourceShootThenCenter4PathFind extends SequentialCommandGroup {

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

        public AutoSourceShootThenCenter4PathFind(
                        CommandFactory cf,
                        PathPlannerPath path,
                        SwerveSubsystem swerve) {

                addCommands(
                                // shoot first note

                                cf.setStartPosebyAlliance(path),
                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),
                                cf.transferNoteToShooterCommand(),
                                // move to center note 4, pick up if there and move to shoot position then shoot
                                // if note at 4 not picked up, try note at 5 and shoot it
                                Commands.runOnce(() -> swerve.toLocation = 4),
                                new RunPPath(swerve,
                                                path,
                                                false),
                                new ParallelCommandGroup(
                                                new ConditionalCommand(
                                                                cf.autopickup(FieldConstants.centerNote4PickupBlue),
                                                                cf.autopickup(GeometryUtil.flipFieldPose(
                                                                                FieldConstants.centerNote4PickupBlue)),
                                                                () -> DriverStation.getAlliance().isPresent()
                                                                                && DriverStation.getAlliance()
                                                                                                .get() == Alliance.Blue),

                                                cf.doIntake()),
                                Commands.runOnce(() -> swerve.atLocation = 4));

        }

}
