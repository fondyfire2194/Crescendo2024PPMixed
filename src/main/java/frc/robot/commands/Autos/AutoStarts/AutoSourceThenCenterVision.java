// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Drive.CheckOKSwitchToDrive;
import frc.robot.commands.Drive.DriveToPickupNote;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class AutoSourceThenCenterVision extends SequentialCommandGroup {

        public AutoSourceThenCenterVision(
                        CommandFactory cf,
                        PathFactory pf,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        boolean innerNoteFirst) {

                addCommands(
                                // shoot first note
                                cf.setStartPosebyAlliance(FieldConstants.sourceStartPose),

                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle - 5,
                                                Constants.subwfrShooterSpeed),
                                cf.transferNoteToShooterCommand(),
                                cf.armToIntake(),
                                new ParallelRaceGroup(
                                                new CheckOKSwitchToDrive(swerve, 2),
                                                Commands.either(
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                sourcepaths.SourceToCenter4
                                                                                                                .name())),
                                                                new RunPPath(swerve,
                                                                                pf.pathMaps.get(
                                                                                                sourcepaths.SourceShootToCenter5
                                                                                                                .name())),
                                                                () -> innerNoteFirst)),

                                Commands.parallel(
                                                new DriveToPickupNote(swerve, transfer, intake, null, true, 1),
                                                cf.doIntake()),
                                Commands.parallel(
                                                Commands.runOnce(() -> swerve.autostep = 1),
                                                Commands.runOnce(() -> cf.innerNoteFirst = innerNoteFirst)));
        }

}
