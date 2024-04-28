// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Factories.AutoFactory;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class AutoSourceShootThenCenter4 extends SequentialCommandGroup {

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

        public AutoSourceShootThenCenter4(
                        CommandFactory cf,
                        PathPlannerPath path,
                        AutoFactory af,
                        PathFactory pf,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        ShooterSubsystem shooter,
                        ArmSubsystem arm,
                        TransferSubsystem transfer) {

                addCommands(
                                // shoot first note
                                cf.setStartPosebyAlliance(path),
                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),
                                cf.transferNoteToShooter(),
                                // move to center note 4, pick up if there and move to shoot position then shoot
                                // if note at 4 not picked up, try note at 5 and shoot it
                                new ParallelCommandGroup(
                                                new RunPPath(swerve,
                                                                path,
                                                                false),
                                                cf.doIntake()),

                                // cf.decideOn(
                                //                 pf.pathMaps.get(sourcepaths.Center4ToSourceShoot.name()),
                                //                 pf.pathMaps.get(sourcepaths.Center4ToCenter5.name()),
                                //                 pf.pathMaps.get(sourcepaths.Center5ToSourceShoot.name())),

                                // go back for note at center 5 if it wasn't tried before

                                cf.getSecondCenterNote(
                                                pf.pathMaps.get(sourcepaths.SourceShootToCenter5.name()),
                                                pf.pathMaps.get(sourcepaths.Center5ToSourceShoot.name())));

        }

}
