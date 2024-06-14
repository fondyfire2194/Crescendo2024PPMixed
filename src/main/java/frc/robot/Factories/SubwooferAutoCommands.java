// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class SubwooferAutoCommands {

    public SubwooferAutoCommands(SwerveSubsystem swerve, CommandFactory cf) {
    }

    public Command setsbwrstart(SwerveSubsystem swerve, CommandFactory cf) {
        return Commands.sequence(
                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),
                cf.setStartPosebyAlliance(FieldConstants.sbwfrStartPose));
    }

    public Command shoot(CommandFactory cf, double angle, double rpm) {
        return Commands.sequence(
                cf.positionArmRunShooterSpecialCase(angle, rpm),
                cf.transferNoteToShooterCommand());
    }

    public Command shootbydistance(CommandFactory cf) {
        return Commands.sequence(
                cf.positionArmRunShooterByDistance(false, true),
                cf.transferNoteToShooterCommand());
    }

    public Command sbwfrShoot(CommandFactory cf) {
        return Commands.sequence(
                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                        Constants.subwfrShooterSpeed),
                cf.transferNoteToShooterCommand());
    }

    public Command moveandshoot(sbwfrpaths path, SwerveSubsystem swerve, CommandFactory cf, PathFactory pf,
            double angle, double rpm) {
        return Commands.sequence(
                Commands.parallel(
                        new RunPPath(swerve, pf.pathMaps.get(path.name())),
                        cf.positionArmRunShooterSpecialCase(
                                angle, rpm)),
                cf.transferNoteToShooterCommand());
    }

    public Command sbwfrmoveandshoot(sbwfrpaths path, SwerveSubsystem swerve, CommandFactory cf, PathFactory pf) {
        return Commands.sequence(
                Commands.parallel(
                        new RunPPath(swerve, pf.pathMaps.get(path.name())),
                        cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                Constants.subwfrShooterSpeed)),
                cf.transferNoteToShooterCommand());
    }

    public Command moveAndPickup(sbwfrpaths path, SwerveSubsystem swerve, CommandFactory cf, PathFactory pf) {
        return Commands.parallel(
                new RunPPath(swerve,
                        pf.pathMaps.get(path.name())),
                Commands.sequence(
                        Commands.waitSeconds(.25),
                        cf.doIntake()));
    }

}
