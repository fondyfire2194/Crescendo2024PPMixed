// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AmpAutoCommands {

        public AmpAutoCommands(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf) {
        }

        public Command setAmpStart(SwerveSubsystem swerve, TransferSubsystem transfer,
                        IntakeSubsystem intake, CommandFactory cf) {
                return Commands.sequence(
                                Commands.runOnce(() -> transfer.simnoteatintake = false),
                                Commands.runOnce(() -> intake.resetIsIntakingSim()),
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> swerve.ampActive = true),
                                Commands.runOnce(() -> swerve.sourceActive = false),
                                Commands.runOnce(() -> swerve.pickupTargetX = AllianceUtil.getWingNoteX()),
                                cf.setStartPosebyAlliance(FieldConstants.ampStartPose));
        }

        public Command pickupNote(CommandFactory cf, PathPlannerPath path, SwerveSubsystem swerve, double nonotetime) {

                return Commands.parallel(
                                new RunPPath(swerve, path),
                                cf.doIntake(nonotetime));
        }

        public Command prepandshoot(CommandFactory cf, double armAngle, double shooterpm, double rpmtol) {
                return Commands.sequence(
                                cf.positionArmRunShooterSpecialCase(armAngle, shooterpm, rpmtol),
                                cf.checkAtTargets(20),
                                cf.transferNoteToShooterCommand());
        }

        public Command shootbydistance(CommandFactory cf) {
                return Commands.sequence(
                                cf.positionArmRunShooterByDistance(
                                                false, true),

                                cf.transferNoteToShooterCommand());
        }

        public Command moveShoot(CommandFactory cf, PathPlannerPath path, SwerveSubsystem swerve, double armAngle,
                        double shooterpm, double rpmtol) {

                return Commands.sequence(
                                Commands.parallel(
                                                new RunPPath(swerve, path),
                                                cf.positionArmRunShooterSpecialCase(armAngle, shooterpm, rpmtol)),
                                cf.transferNoteToShooterCommand());
        }

        public Command pickupCenter1_2(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.AmpToCenter2.name())),

                                                new RunPPath(swerve, pf.pathMaps.get(
                                                                amppaths.AmpToCenter1.name())),
                                                () -> innerNoteFirst),

                                cf.doIntake(2));
        }

        public Command moveShootCenter1_2(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        boolean innerNoteFirst) {
                return Commands.either(
                                new CenterToShoot(cf, pf.pathMaps.get(amppaths.Center2ToAmpShoot
                                                .name()),
                                                swerve),
                                new CenterToShoot(cf, pf.pathMaps.get(amppaths.Center1ToAmpShoot
                                                .name()),
                                                swerve),
                                () -> innerNoteFirst);
        }

        public Command pickUpNoteAfterShoot(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake, boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.AmpShootToCenter1
                                                                                .name())),
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(amppaths.AmpShootToCenter2
                                                                                .name())),
                                                () -> innerNoteFirst),
                                cf.doIntake(2));
        }

        public Command tryOtherNote(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, boolean innerNoteFirst) {
                return Commands.sequence(
                                Commands.parallel(
                                                Commands.either(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(sourcepaths.Center5ToCenter4
                                                                                                .name())),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(sourcepaths.Center4ToCenter5
                                                                                                .name())),
                                                                () -> innerNoteFirst),
                                                cf.doIntake(2)));
        }

}