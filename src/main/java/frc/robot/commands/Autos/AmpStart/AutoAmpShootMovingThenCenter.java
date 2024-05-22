// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AmpStart;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.CheckAlignedToSpeaker;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AutoAmpShootMovingThenCenter extends SequentialCommandGroup {

        public AutoAmpShootMovingThenCenter(
                        CommandFactory cf,
                        PathPlannerPath path,
                        SwerveSubsystem swerve) {

                addCommands(

                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),

                                // path auto shoots on the fly
                                // move to center note , pick up if there and move to shoot position then shoot
                                Commands.runOnce(() -> swerve.currentPlannerPath = path),
                                cf.setStartPosebyAlliance(FieldConstants.ampStartPose),
                                cf.setArmShooterValues(Constants.autoShootArmAngle, Constants.autoShootRPM),

                                Commands.deadline(
                                                new RunPPath(swerve,
                                                                path),
                                                new CheckAlignedToSpeaker(swerve, 2)),
                                Commands.sequence(
                                                new WaitCommand(2),
                                                cf.doIntake()),
                                Commands.runOnce(() -> swerve.autostep = 1));

        }

}
