// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class SourceAutoCommands {

        public SourceAutoCommands(SwerveSubsystem swerve, CommandFactory cf) {
        }

        public Command setSourceStart(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake, CommandFactory cf) {
                return Commands.sequence(
                                Commands.runOnce(() -> transfer.simnoteatintake = false),
                                Commands.runOnce(() -> intake.resetIsIntakingSim()),
                                Commands.runOnce(() -> swerve.targetPose = AllianceUtil.getSpeakerPose()),
                                Commands.runOnce(() -> swerve.ampActive = false),
                                Commands.runOnce(() -> swerve.sourceActive = true),
                                Commands.runOnce(() -> swerve.currentpathstartTime = Timer.getFPGATimestamp()),
                                Commands.runOnce(() -> swerve.pickupTargetX = FieldConstants.FIELD_LENGTH / 2),
                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose));

        }

        public Command pickupNote(CommandFactory cf, PathPlannerPath path, SwerveSubsystem swerve) {

                return Commands.parallel(
                                new RunPPath(swerve, path),
                                cf.doIntake(3));

        }

        public Command moveShoot(CommandFactory cf, PathPlannerPath path, SwerveSubsystem swerve, double armAngle,
                        double shooterpm, double rpmtol) {
                return Commands.sequence(
                                Commands.parallel(
                                                new RunPPath(swerve, path),
                                                cf.positionArmRunShooterSpecialCase(armAngle, shooterpm, rpmtol)),
                                cf.transferNoteToShooterCommand());
        }

        public Command pickupCenter4_5(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake,
                        boolean innerNoteFirst) {

                return Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(sourcepaths.SourceToCenter4
                                                                                .name())),
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(sourcepaths.SourceToCenter5
                                                                                .name())),
                                                () -> innerNoteFirst),
                                cf.doIntakeDelayed(2, 2));
        }

        public Command moveShootCenter4_5(CommandFactory cf, PathFactory pf, SwerveSubsystem swerve,
                        boolean innerNoteFirst) {
                return Commands.either(
                                new CenterToShoot(cf, pf.pathMaps.get(
                                                sourcepaths.Center4ToSourceShoot
                                                                .name()),
                                                swerve),
                                new CenterToShoot(cf, pf.pathMaps.get(sourcepaths.Center5ToSourceShoot
                                                .name()),
                                                swerve),
                                () -> innerNoteFirst);
        }

        public Command pickUpNoteAfterShoot(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve,
                        TransferSubsystem transfer, IntakeSubsystem intake, boolean innerNoteFirst) {

                return

                Commands.parallel(
                                Commands.either(
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(sourcepaths.SourceShootToCenter5
                                                                                .name())),
                                                new RunPPath(swerve,
                                                                pf.pathMaps.get(sourcepaths.SourceShootToCenter4
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