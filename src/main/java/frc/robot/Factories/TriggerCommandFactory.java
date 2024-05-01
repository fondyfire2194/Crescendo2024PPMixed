// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.Autos.SourceStart.Center4ToSourceShoot;
import frc.robot.commands.Autos.SourceStart.Center5ToSourceShoot;
import frc.robot.commands.Autos.SourceStart.SourceShootToCenter5Pickup;
import frc.robot.commands.Drive.DriveToPickupNote;
import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.GeometryUtil;
import monologue.Annotations.Log;
import monologue.Logged;

/** Add your docs here. */
public class TriggerCommandFactory implements Logged {

        private final SwerveSubsystem m_swerve;
        private final IntakeSubsystem m_intake;
        private final TransferSubsystem m_transfer;
        private final LimelightVision m_llv;
        private final PathFactory m_pf;
        private final CommandFactory m_cf;

        @Log.NT(key = "trig1")
        private boolean trig1;
        @Log.NT(key = "trig2")
        private boolean trig2;
        @Log.NT(key = "trig3")
        private boolean trig3;
        @Log.NT(key = "trig4")
        private boolean trig4;
        @Log.NT(key = "trig5")
        private boolean trig5;
        @Log.NT(key = "trig6")
        private boolean trig6;

        public TriggerCommandFactory(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        LimelightVision llv, PathFactory pf, CommandFactory cf) {
                m_swerve = swerve;
                m_intake = intake;
                m_transfer = transfer;
                m_llv = llv;
                m_pf = pf;
                m_cf = cf;
        }

        private void resettrigs() {
                trig1 = false;
                trig2 = false;
                trig3 = false;
                trig4 = false;
                trig5 = false;
                trig6 = false;
                resetLocations();
        }
        // location conditions from, to and at are used to sequnce the triggers
        // 0 = start, 10 = source shoot, 4, 5 are center notes

        private void resetLocations() {
                m_swerve.toLocation = 0;
                m_swerve.fromLocation = 0;
                m_swerve.atLocation = 0;

        }

        public void createSourceTriggersC4C5() {

                resettrigs();

                // move to shoot if a note C4 is picked up. Set source of move

                Trigger triggerC4ToSS = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && m_swerve.fromLocation == 0 && m_swerve.atLocation == 4 && m_transfer.noteAtIntake());

                triggerC4ToSS.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_swerve.fromLocation = 4),
                                Commands.runOnce(() -> m_swerve.toLocation = 10),
                                new Center4ToSourceShoot(m_cf, m_pf, m_swerve),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.atLocation = 10),
                                                Commands.runOnce(() -> trig1 = true))));

                // when shot from note C4 ends go try for note C5
                Trigger triggerSSToC5 = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && m_transfer.isStopped() && !m_transfer.noteAtIntake()
                                && m_swerve.fromLocation == 4
                                && m_swerve.atLocation == 10);

                triggerSSToC5.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_swerve.toLocation = 5),
                                Commands.runOnce(() -> m_swerve.fromLocation = 10),
                                new SourceShootToCenter5Pickup(m_cf, m_pf, m_swerve),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.atLocation = 5),
                                                Commands.runOnce(() -> trig2 = true))));

                // // if note C5 is picked up, go shoot it
                Trigger triggerC5ToSS = new Trigger(() -> DriverStation.isAutonomousEnabled() &&
                                m_swerve.isStopped() && m_transfer.isStopped() && m_transfer.noteAtIntake()
                                && m_swerve.atLocation == 5
                                && (m_swerve.fromLocation == 10 || m_swerve.fromLocation == 4));

                triggerC5ToSS.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_swerve.fromLocation = 5),
                                Commands.runOnce(() -> m_swerve.toLocation = 10),
                                new Center5ToSourceShoot(m_cf, m_pf, m_swerve),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.atLocation = 10),
                                                Commands.runOnce(() -> trig3 = true))));

                // // if note C4 isn't collected, go try C5
                Trigger triggerC4ToC5 = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && m_swerve.isStopped() && !m_transfer.noteAtIntake() && m_transfer.isStopped()
                                && m_swerve.fromLocation == 0 && m_swerve.toLocation == 4 && m_swerve.atLocation == 4);

                triggerC4ToC5.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_swerve.fromLocation = 4),
                                Commands.runOnce(() -> m_swerve.toLocation = 5),
                                new RotateToAngle(m_swerve, 90),
                                m_intake.startIntakeCommand(),
                                new TransferIntakeToSensor(m_transfer, m_intake, .6),
                                new DriveToPickupNote(m_swerve, m_transfer, m_intake,
                                                CameraConstants.rearCamera.camname, m_llv,
                                                4),
                                new ConditionalCommand(
                                                m_cf.autopickup(FieldConstants.sourceShootBlue),
                                                m_cf.autopickup(GeometryUtil
                                                                .flipFieldPose(FieldConstants.sourceShootBlue)),
                                                () -> DriverStation.getAlliance().isPresent()
                                                                && DriverStation.getAlliance()
                                                                                .get() == Alliance.Blue),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.atLocation = 5),
                                                Commands.runOnce(() -> trig4 = true))));

                // stop shooter and intake, move intake to pickup position, stop transfer
                // 2 conditions at shoot position from 5 and no note or at 5 from 4 or 10 and no
                // note

                Trigger resetAll = new Trigger(() -> (m_swerve.toLocation == 10 && m_swerve.atLocation == 10
                                && !m_transfer.noteAtIntake() && m_transfer.isStopped())
                                || m_swerve.atLocation == 5
                                                && (m_swerve.fromLocation == 4 || m_swerve.fromLocation == 10)
                                                && !m_transfer.noteAtIntake() && m_transfer.isStopped());

                resetAll.onTrue(Commands.sequence(
                                Commands.runOnce(() -> trig5 = true),
                                m_cf.resetAll(),
                                Commands.runOnce(() -> resetLocations())));

        }

        public void createAmpTriggers() {

        }

}