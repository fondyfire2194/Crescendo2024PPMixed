// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Drive.DriveToPickupNote;

import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.ShootingData;

public class MovePickupShootTest extends SequentialCommandGroup {
        /** Creates a new MovePickupShoot. */
        public MovePickupShootTest(
                        CommandFactory cf,
                        SwerveSubsystem swerve,
                        ArmSubsystem arm,
                        TransferSubsystem transfer,
                        IntakeSubsystem intake,
                        ShooterSubsystem shooter,
                        ShootingData sd,
                        String camname,
                        int n) {

                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                addCommands(

                                Commands.runOnce(() -> cf.testNotesRun = 0),

                                Commands.repeatingSequence(

                                                Commands.parallel(
                                                                cf.doIntake(10),
                                                                new DriveToPickupNote(swerve, transfer, intake, camname)),

                                                Commands.deadline(
                                                                cf.positionArmRunShooterByDistance(false, true),
                                                                new AutoAlignSpeaker(swerve, false)),

                                                Commands.either(
                                                                transfer.transferToShooterCommand(),
                                                                Commands.runOnce(() -> cf.testNotesRun = n + 1),
                                                                () -> transfer.noteAtIntake()),
                                                new RotateToAngle(swerve, 180),

                                                Commands.runOnce(() -> cf.testNotesRun++))
                                                .until(() -> cf.testNotesRun >= n));
        }
}