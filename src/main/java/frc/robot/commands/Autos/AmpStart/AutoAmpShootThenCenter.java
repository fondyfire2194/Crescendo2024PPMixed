// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AmpStart;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoAmpShootThenCenter extends SequentialCommandGroup {

        public AutoAmpShootThenCenter(
                        CommandFactory cf,
                        PathPlannerPath path,
                        SwerveSubsystem swerve) {

                addCommands(

                                // shoot first note
                                Commands.runOnce(() -> swerve.currentPlannerPath = path),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),
                                
                                cf.setStartPosebyAlliance(FieldConstants.ampStartPose),
                                
                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),
                                cf.transferNoteToShooterCommand(),
                                // move to center note , pick up if there and move to shoot position then shoot

                                new ParallelCommandGroup(
                                                Commands.runOnce(() -> swerve.toLocation = 2),
                                                new RunPPath(swerve,
                                                                path),                                                          
                                                new SequentialCommandGroup(
                                                                Commands.waitSeconds(1),
                                                                cf.doIntake())),

                                Commands.runOnce(() -> swerve.atLocation = 2));

        }

}
