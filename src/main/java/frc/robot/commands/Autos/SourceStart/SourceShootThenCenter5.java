// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SourceStart;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoFactory;
import frc.robot.Constants;
import frc.robot.PathFactory;
import frc.robot.PathFactory.sourcepaths;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class SourceShootThenCenter5 extends SequentialCommandGroup {

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

        public SourceShootThenCenter5(
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
                                                Constants.subwfrShooterSpeed).asProxy(),

                                Commands.runOnce(() -> SmartDashboard.putNumber("TTTT", 999)),

                                cf.transferNoteToShooter(),
                                // move to center note 5, pick up if there and move to shoot position then shoot
                                // if note at 5 not picked up, try note at 4 and shoot it
                                new ParallelCommandGroup(
                                                new RunPPath(swerve,
                                                                path,
                                                                false),
                                                cf.doIntake()),

                                new ConditionalCommand(
                                                new CenterToSourceShoot(cf,
                                                                pf.pathMaps.get(sourcepaths.Center5ToSourceShoot
                                                                                .name()),
                                                                swerve),
                                                new SequentialCommandGroup(
                                                                new CenterToCenterPickup(cf,
                                                                                pf.pathMaps.get(sourcepaths.Center5ToCenter4
                                                                                                .name()),
                                                                                swerve),
                                                                new CenterToSourceShoot(
                                                                                cf,
                                                                                pf.pathMaps.get(sourcepaths.Center4ToSourceShoot
                                                                                                .name()),
                                                                                swerve)),

                                                () -> transfer.noteAtIntake()),

                                // go back for note at center 4 if it wasn't tried before
                                new ConditionalCommand(

                                                new ParallelCommandGroup(
                                                                new RunPPath(swerve, pf.pathMaps.get(
                                                                                sourcepaths.SourceShootToCenter4
                                                                                                .name()),
                                                                                false),
                                                                cf.doIntake()),
                                                Commands.none(),

                                                () -> !intake.noteMissed),

                                new ConditionalCommand(
                                                new CenterToSourceShoot(
                                                                cf,
                                                                pf.pathMaps.get(sourcepaths.Center4ToSourceShoot
                                                                                .name()),
                                                                swerve),
                                                Commands.none(),
                                                () -> transfer.noteAtIntake()),

                                cf.resetAll());

        }

}
