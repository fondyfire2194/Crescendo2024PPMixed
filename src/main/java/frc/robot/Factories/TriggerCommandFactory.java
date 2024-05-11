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
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Autos.AmpStart.AmpShootToCenterPickup;
import frc.robot.commands.Autos.SourceStart.CenterToSourceShoot;
import frc.robot.commands.Autos.SourceStart.SourceShootToCenterPickup;
import frc.robot.commands.Drive.PickUpAlternateNote;
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

                triggerC4ToSS.onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> m_swerve.setFromTo(4, 10)),
                                                new CenterToSourceShoot(m_cf,
                                                                m_pf.pathMaps.get(sourcepaths.Center4ToSourceShoot
                                                                                .name()),
                                                                m_swerve),
                                                Commands.parallel(
                                                                Commands.runOnce(() -> m_swerve.atLocation = 10),
                                                                Commands.runOnce(() -> trig1 = true))));

                // when shot from note C4 ends go try for note C5
                Trigger triggerSSToC5 = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && m_transfer.isStopped() && !m_transfer.noteAtIntake()
                                && m_swerve.fromLocation == 4
                                && m_swerve.atLocation == 10);

                triggerSSToC5.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_swerve.setFromTo(10, 5)),
                                new SourceShootToCenterPickup(m_cf, m_pf.pathMaps.get(sourcepaths.SourceShootToCenter5
                                                .name()), m_swerve),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.atLocation = 5),
                                                Commands.runOnce(() -> trig2 = true))));

                // // if note C5 is picked up, go shoot it
                Trigger triggerC5ToSS = new Trigger(() -> DriverStation.isAutonomousEnabled() &&
                                m_swerve.isStopped() && m_transfer.isStopped() && m_transfer.noteAtIntake()
                                && m_swerve.atLocation == 5
                                && (m_swerve.fromLocation == 10 || m_swerve.fromLocation == 4));

                triggerC5ToSS.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_swerve.setFromTo(4, 10)),
                                new CenterToSourceShoot(m_cf, m_pf.pathMaps.get(sourcepaths.Center5ToSourceShoot
                                                .name()), m_swerve),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.atLocation = 10),
                                                Commands.runOnce(() -> trig1 = true))));

                // // if note C4 isn't collected, go try C5
                Trigger triggerC4ToC5 = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && m_swerve.isStopped() && !m_transfer.noteAtIntake() && m_transfer.isStopped()
                                && m_swerve.fromLocation == 0 && m_swerve.toLocation == 4 && m_swerve.atLocation == 4);

                triggerC4ToC5.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_swerve.setFromTo(4, 5)),
                                new RotateToAngle(m_swerve, 90),
                                m_intake.startIntakeCommand(),
                                Commands.parallel(
                                                new TransferIntakeToSensor(m_transfer, m_intake, .6),
                                                new PickUpAlternateNote(m_swerve, m_transfer, m_intake,
                                                                CameraConstants.rearCamera.camname, m_llv,
                                                                5)),
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

                Trigger resetAll = new Trigger(() -> (m_swerve.fromLocation == 5 && m_swerve.atLocation == 10
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

                resettrigs();

                Trigger triggerC2ToAS = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && m_swerve.fromLocation == 0 && m_swerve.atLocation == 2 && m_transfer.noteAtIntake());

                triggerC2ToAS.onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> m_swerve.setFromTo(2, 11)),
                                                new CenterToSourceShoot(m_cf,
                                                                m_pf.pathMaps.get(amppaths.Center2ToAmpShoot
                                                                                .name()),
                                                                m_swerve),
                                                Commands.parallel(
                                                                Commands.runOnce(() -> m_swerve.atLocation = 11),
                                                                Commands.runOnce(() -> trig1 = true))));

                Trigger triggerASToC1 = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && m_transfer.isStopped() && !m_transfer.noteAtIntake()
                                && m_swerve.fromLocation == 2
                                && m_swerve.atLocation == 11);

                triggerASToC1.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_swerve.setFromTo(11, 1)),
                                new AmpShootToCenterPickup(m_cf, m_pf.pathMaps.get(amppaths.AmpShootToCenter1
                                                .name()), m_swerve),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.atLocation = 1),
                                                Commands.runOnce(() -> trig2 = true))));

                Trigger triggerC1ToAS = new Trigger(() -> DriverStation.isAutonomousEnabled() &&
                                m_swerve.isStopped() && m_transfer.isStopped() && m_transfer.noteAtIntake()
                                && m_swerve.atLocation == 1
                                && (m_swerve.fromLocation == 11 || m_swerve.fromLocation == 1));

                triggerC1ToAS.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_swerve.setFromTo(11, 1)),
                                new CenterToSourceShoot(m_cf, m_pf.pathMaps.get(amppaths.Center1ToAmpShoot
                                                .name()), m_swerve),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.atLocation = 11),
                                                Commands.runOnce(() -> trig1 = true))));

                Trigger triggerC2ToC1 = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && m_swerve.isStopped() && !m_transfer.noteAtIntake() && m_transfer.isStopped()
                                && m_swerve.fromLocation == 0 && m_swerve.toLocation == 2 && m_swerve.atLocation == 2);

                triggerC2ToC1.onTrue(Commands.sequence(
                                Commands.runOnce(() -> m_swerve.setFromTo(2, 1)),
                                new RotateToAngle(m_swerve, -90),
                                m_intake.startIntakeCommand(),
                                Commands.parallel(
                                                new TransferIntakeToSensor(m_transfer, m_intake, .6),
                                                new PickUpAlternateNote(m_swerve, m_transfer, m_intake,
                                                                CameraConstants.rearCamera.camname, m_llv,
                                                                1)),
                                new ConditionalCommand(
                                                m_cf.autopickup(FieldConstants.ampShootBlue),
                                                m_cf.autopickup(GeometryUtil
                                                                .flipFieldPose(FieldConstants.ampShootBlue)),
                                                () -> DriverStation.getAlliance().isPresent()
                                                                && DriverStation.getAlliance()
                                                                                .get() == Alliance.Blue),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.atLocation = 1),
                                                Commands.runOnce(() -> trig4 = true))));

                Trigger resetAll = new Trigger(() -> (m_swerve.fromLocation == 1 && m_swerve.atLocation == 11
                                && !m_transfer.noteAtIntake() && m_transfer.isStopped())
                                || m_swerve.atLocation == 1
                                                && (m_swerve.fromLocation == 2 || m_swerve.fromLocation == 11)
                                                && !m_transfer.noteAtIntake() && m_transfer.isStopped());

                resetAll.onTrue(Commands.sequence(
                                Commands.runOnce(() -> trig5 = true),
                                m_cf.resetAll(),
                                Commands.runOnce(() -> resetLocations())));

        }

}