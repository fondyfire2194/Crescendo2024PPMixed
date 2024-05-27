// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CameraConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.AlignTargetOdometry;
import frc.robot.commands.Drive.DriveToPickupNote;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.ShootingData;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MovePickupShoot extends SequentialCommandGroup {
  /** Creates a new MovePickupShoot. */
  public MovePickupShoot(
      CommandFactory cf,
      SwerveSubsystem swerve,
      ArmSubsystem arm,
      TransferSubsystem transfer,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      ShootingData sd,
      double maxDistance,
      int n) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
        Commands.runOnce(() -> cf.testNotesRun = 0),

        Commands.repeatingSequence(

            Commands.parallel(
                cf.doIntake(),
                new DriveToPickupNote(swerve, transfer, intake, CameraConstants.rearCamera.camname,
                    true, maxDistance)),

            Commands.deadline(
                cf.positionArmRunShooterByDistance(false, false, true),
                new AlignTargetOdometry(swerve, () -> 0, () -> 0, () -> 0, false)),

            Commands.either(
                transfer.transferToShooterCommand(), Commands.runOnce(() -> cf.testNotesRun = n + 1),
                () -> transfer.noteAtIntake()),

            Commands.runOnce(() -> cf.testNotesRun++))
            .until(() -> cf.testNotesRun >= n));
  }
}