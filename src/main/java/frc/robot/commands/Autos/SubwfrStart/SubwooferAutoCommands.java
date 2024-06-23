// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SubwfrStart;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class SubwooferAutoCommands {

        public SubwooferAutoCommands(SwerveSubsystem swerve, CommandFactory cf) {
        }

        public Command setsbwrstart(SwerveSubsystem swerve, CommandFactory cf) {
                return Commands.sequence(
                                Commands.runOnce(() -> swerve.pickupTargetX = AllianceUtil.getWingNoteX()),
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> swerve.inhibitVision = true),
                                cf.setStartPosebyAlliance(FieldConstants.sbwfrStartPose));
        }

        public Command shoot(CommandFactory cf) {
                return cf.transferNoteToShooterCommand();
        }

        public Command setArmShooter(CommandFactory cf, double angle, double rpm) {
                return cf.positionArmRunShooterSpecialCase(angle, rpm, 25);
        }

        public Command shoot(CommandFactory cf, double angle, double rpm) {
                return Commands.sequence(
                                setArmShooter(cf, angle, rpm),
                                shoot(cf));
        }

        public Command sbwfrShoot(CommandFactory cf) {
                return Commands.sequence(
                                setArmShooter(cf, Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed),
                                cf.checkAtTargets(20),
                                shoot(cf));
        }

        public Command shootbydistance(CommandFactory cf) {
                return Commands.sequence(
                                cf.positionArmRunShooterByDistance(false, true),

                                shoot(cf));
        }

        public Command move(sbwfrpaths path, SwerveSubsystem swerve, PathFactory pf) {
                return new RunPPath(swerve, pf.pathMaps.get(path.name()));
        }

        public Command moveandshoot(sbwfrpaths path, SwerveSubsystem swerve, CommandFactory cf, PathFactory pf,
                        ShooterSubsystem shooter, ArmSubsystem arm,
                        double angle, double rpm) {
                return Commands.sequence(
                                Commands.parallel(
                                                new RunPPath(swerve, pf.pathMaps.get(path.name())),
                                                setArmShooter(cf, angle, rpm)),
                                Commands.waitUntil(() -> shooter.bothAtSpeed(.2) && arm.getAtSetpoint()),
                                shoot(cf));
        }

        public Command sbwfrmoveandshoot(sbwfrpaths path, SwerveSubsystem swerve, CommandFactory cf, PathFactory pf) {
                return Commands.sequence(
                                Commands.parallel(
                                                move(path, swerve, pf),
                                                setArmShooter(cf, Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed)),
                                shoot(cf));
        }

        public Command moveAndPickup(sbwfrpaths path, SwerveSubsystem swerve, CommandFactory cf, PathFactory pf) {
                return Commands.parallel(
                                move(path, swerve, pf),
                                Commands.sequence(
                                                Commands.waitSeconds(.25),
                                                cf.doIntake(10)));
        }

        public Command moveAndPickupUsingVision(sbwfrpaths path, SwerveSubsystem swerve, CommandFactory cf,
                        PathFactory pf) {
                return Commands.parallel(
                                move(path, swerve, pf),
                                Commands.sequence(
                                                Commands.waitSeconds(.25),
                                                cf.doIntake(10)));
        }

}
