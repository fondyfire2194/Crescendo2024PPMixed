// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SubwfrStart;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AutoSbwfrShootThenSequence extends SequentialCommandGroup {

        public AutoSbwfrShootThenSequence(
                        CommandFactory cf,
                        PathFactory pf, SwerveSubsystem swerve,
                        sbwfrpaths path1,
                        sbwfrpaths path2,
                        sbwfrpaths path3,
                        sbwfrpaths path4,
                        sbwfrpaths path5,
                        sbwfrpaths path6) {

                addCommands(

                                // shoot first note
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),

                                cf.setStartPosebyAlliance(FieldConstants.sbwfrStartPose),

                                shoot(cf, Constants.subwfrArmAngle, Constants.subwfrShooterSpeed),

                                moveAndPickup(swerve, path1, cf, pf),

                                moveandshoot(swerve, path2, cf, pf, Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),

                                moveAndPickup(swerve, path3, cf, pf),

                                moveandshoot(swerve, path4, cf, pf, Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),

                                moveAndPickup(swerve, path5, cf, pf),

                                moveandshoot(swerve, path6, cf, pf, Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),

                                cf.resetAll());

        }

        public AutoSbwfrShootThenSequence(
                        CommandFactory cf,
                        PathFactory pf, SwerveSubsystem swerve,
                        sbwfrpaths path1,
                        sbwfrpaths path2,
                        sbwfrpaths path3,
                        sbwfrpaths path4) {

                addCommands(

                                // shoot first note
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),

                                cf.setStartPosebyAlliance(FieldConstants.sbwfrStartPose),

                                shoot(cf, Constants.subwfrArmAngle, Constants.subwfrShooterSpeed),

                                moveAndPickup(swerve, path1, cf, pf),

                                moveandshoot(swerve, path2, cf, pf, Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),

                                moveAndPickup(swerve, path3, cf, pf),

                                moveandshoot(swerve, path4, cf, pf, Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),

                                cf.resetAll());

        }

        public AutoSbwfrShootThenSequence(
                        CommandFactory cf,
                        PathFactory pf, SwerveSubsystem swerve,
                        sbwfrpaths path1,
                        sbwfrpaths path2,
                        sbwfrpaths path3,
                        sbwfrpaths path4,
                        sbwfrpaths path5) {

                addCommands(
                                // shoot first note
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),

                                cf.setStartPosebyAlliance(FieldConstants.sbwfrStartPose),

                                shoot(cf, Constants.subwfrArmAngle, Constants.subwfrShooterSpeed),

                                moveAndPickup(swerve, path1, cf, pf),

                                shoot(cf, Constants.wing2ArmAngle, Constants.wing2ShooterSpeed),

                                moveAndPickup(swerve, path2, cf, pf),

                                moveandshoot(swerve, path3, cf, pf, Constants.wing2ArmAngle,
                                                Constants.wing2ShooterSpeed),

                                moveAndPickup(swerve, path4, cf, pf),

                                shoot(cf, Constants.wing3ArmAngle, Constants.wing3ShooterSpeed),

                                moveAndPickup(swerve, path5, cf, pf),

                                shoot(cf, Constants.wing1ArmAngle, Constants.wing1ShooterSpeed),

                                cf.resetAll());
        }

        private Command shoot(CommandFactory cf, double angle, double rpm) {
                return Commands.sequence(
                                cf.positionArmRunShooterSpecialCase(angle, rpm),
                                cf.transferNoteToShooterCommand());
        }

        private Command moveAndPickup(SwerveSubsystem swerve, sbwfrpaths path, CommandFactory cf, PathFactory pf) {

                return Commands.parallel(
                                new RunPPath(swerve,
                                                pf.pathMaps.get(path.name())),
                                Commands.sequence(
                                                Commands.waitSeconds(.25),
                                                cf.doIntake()));

        }

        private Command moveandshoot(SwerveSubsystem swerve, sbwfrpaths path, CommandFactory cf, PathFactory pf,
                        double angle, double rpm) {

                return Commands.sequence(
                                Commands.parallel(
                                                new RunPPath(swerve, pf.pathMaps.get(path.name())),
                                                cf.positionArmRunShooterSpecialCase(
                                                                angle, rpm)),
                                cf.transferNoteToShooterCommand());
        }

}
