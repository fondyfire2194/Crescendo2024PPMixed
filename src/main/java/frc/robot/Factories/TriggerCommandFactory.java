// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.CameraConstants;
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Autos.AmpStart.AmpShootToCenterPickup;
import frc.robot.commands.Autos.SourceStart.CenterToShoot;
import frc.robot.commands.Autos.SourceStart.SourceShootToCenterPickup;
import frc.robot.commands.Autos.SourceStart.SourceVisionPickup;
import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.commands.Drive.TryForAnotherNote;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import monologue.Logged;

/** Add your docs here. */
public class TriggerCommandFactory implements Logged {

        private final SwerveSubsystem m_swerve;
        private final IntakeSubsystem m_intake;
        private final TransferSubsystem m_transfer;
        private final PathFactory m_pf;
        private final CommandFactory m_cf;

        private boolean stepRunning;

        private int step0 = 0;
        private int checkfornote1 = 1;//
        private int movetoshootfrom1stpickup = 2;//
        private int movefromshoottosecondnotepickup = 3;
        private int movetoshootfromsecondpickup = 4;
        private int checkfornote2 = 5;
        private int notemissingfindanother = 6;
        private int endstep = 7;

        private int endit = 8;

        public TriggerCommandFactory(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        PathFactory pf, CommandFactory cf) {
                m_swerve = swerve;
                m_intake = intake;
                m_transfer = transfer;
                m_pf = pf;
                m_cf = cf;
        }

        private void resetSteps() {
                m_swerve.autostep = step0;
                stepRunning = false;
        }

        public void createCommonTriggers() {

                Trigger checkFirstNote = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && !stepRunning && m_swerve.autostep == checkfornote1);

                checkFirstNote.onTrue(Commands.sequence(
                                Commands.runOnce(() -> stepRunning = true),
                                Commands.either(
                                                Commands.runOnce(() -> m_swerve.autostep = movetoshootfrom1stpickup),
                                                Commands.runOnce(() -> m_swerve.autostep = notemissingfindanother),
                                                () -> m_transfer.noteAtIntake()),
                                Commands.runOnce(() -> stepRunning = false)));

                Trigger checkSecondNote = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && !stepRunning && m_swerve.autostep == checkfornote2);

                checkSecondNote.onTrue(Commands.sequence(
                                Commands.runOnce(() -> stepRunning = true),
                                Commands.either(
                                                Commands.runOnce(() -> m_swerve.autostep = movetoshootfromsecondpickup),
                                                Commands.runOnce(() -> m_swerve.autostep = notemissingfindanother),
                                                () -> m_transfer.noteAtIntake()),
                                Commands.runOnce(() -> stepRunning = false)));

                Trigger resetAll = new Trigger(() -> (m_swerve.autostep == endit && !stepRunning
                                && !m_transfer.noteAtIntake() && m_transfer.isStopped()));

                // stop shooter and intake, move intake to pickup position, stop transfer
                // 2 conditions at shoot position from 5 and no note or at 5 from 4 or 10 and no
                // note

                resetAll.onTrue(Commands.sequence(
                                Commands.runOnce(() -> stepRunning = true),
                                m_cf.resetAll(),
                                Commands.runOnce(() -> stepRunning = false),
                                Commands.runOnce(() -> m_swerve.autostep = step0)));

        }
        // location conditions from, to and at are used to sequnce the triggers
        // 0 = start, 10 = source shoot, 4, 5 are center notes

        public void createSourceTriggers() {

                resetSteps();
                m_swerve.ampActive = false;
                m_swerve.sourceActive = true;

                // Step 2 go shoot first note
                Trigger firstNoteToShootSource = new Trigger(
                                () -> DriverStation.isAutonomousEnabled() && m_swerve.sourceActive
                                                && m_swerve.isStopped()
                                                && !stepRunning && m_swerve.autostep == movetoshootfrom1stpickup);

                // when shot from 1st note ends go try for second note set next step to 3
                Trigger shootToSecondNoteSource = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && m_swerve.sourceActive && m_swerve.isStopped()
                                && m_transfer.isStopped() && !stepRunning
                                && m_swerve.autostep == movefromshoottosecondnotepickup);

                // // if second note is picked up, go shoot it
                Trigger secondNoteToShootSource = new Trigger(() -> DriverStation.isAutonomousEnabled() &&
                                m_swerve.sourceActive && m_swerve.isStopped() && m_transfer.isStopped()
                                && !stepRunning && m_swerve.autostep == movetoshootfromsecondpickup);

                // // if target note isn't collected, go try others
                Trigger firstNoteToSecondNoteSource = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && m_swerve.sourceActive && m_swerve.isStopped() && !m_transfer.noteAtIntake()
                                && m_transfer.isStopped()
                                && !stepRunning && m_swerve.autostep == notemissingfindanother);

                // set step 2 if note present or step 6 if not

                firstNoteToShootSource.onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> stepRunning = true),
                                                Commands.either(
                                                                new CenterToShoot(m_cf,
                                                                                m_pf.pathMaps.get(
                                                                                                sourcepaths.Center4ToSourceShoot
                                                                                                                .name()),
                                                                                m_swerve),
                                                                new CenterToShoot(m_cf,
                                                                                m_pf.pathMaps.get(
                                                                                                sourcepaths.Center5ToSourceShoot
                                                                                                                .name()),
                                                                                m_swerve),

                                                                () -> m_cf.innerNoteFirst),
                                                Commands.parallel(
                                                                Commands.runOnce(
                                                                                () -> m_swerve.autostep = movefromshoottosecondnotepickup),
                                                                Commands.runOnce(() -> stepRunning = false))));

                shootToSecondNoteSource.onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> stepRunning = true),
                                                new SourceVisionPickup(
                                                m_cf,
                                                m_pf.pathMaps.get(sourcepaths.SourceShootToCenter5.name()),
                                                m_pf.pathMaps.get(sourcepaths.SourceShootToCenter4.name()),
                                                m_transfer,
                                                m_intake,
                                                m_swerve,
                                                true),

                                                Commands.parallel(
                                                                Commands.runOnce(
                                                                                () -> m_swerve.autostep = checkfornote2),
                                                                Commands.runOnce(() -> stepRunning = false))));

                secondNoteToShootSource.onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> stepRunning = true),
                                                Commands.either(
                                                                new CenterToShoot(m_cf,
                                                                                m_pf.pathMaps.get(
                                                                                                sourcepaths.Center5ToSourceShoot
                                                                                                                .name()),
                                                                                m_swerve),
                                                                new CenterToShoot(m_cf,
                                                                                m_pf.pathMaps.get(
                                                                                                sourcepaths.Center4ToSourceShoot
                                                                                                                .name()),
                                                                                m_swerve),
                                                                () -> m_cf.innerNoteFirst),

                                                Commands.parallel(
                                                                Commands.runOnce(
                                                                                () -> m_swerve.autostep = endit),
                                                                Commands.runOnce(() -> stepRunning = false))));

                firstNoteToSecondNoteSource.onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> stepRunning = true),

                                                new RotateToAngle(m_swerve, -90)
                                                                .unless(() -> LimelightHelpers.getTV(
                                                                                CameraConstants.rearCamera.camname)),
                                                m_intake.startIntakeCommand(),
                                                Commands.deadline(
                                                                new TryForAnotherNote(m_swerve, m_transfer, m_intake,
                                                                                CameraConstants.rearCamera.camname),
                                                                new TransferIntakeToSensor(m_transfer, m_intake, 6)),

                                                Commands.either(
                                                                Commands.sequence(
                                                                                m_cf.autopickup(AllianceUtil
                                                                                                .getSourceClearStagePose()),
                                                                                m_cf.autopickup(AllianceUtil
                                                                                                .getSourceShootPose())),
                                                                Commands.none(),
                                                                () -> m_transfer.noteAtIntake()),
                                                Commands.parallel(
                                                                Commands.runOnce(() -> m_swerve.autostep = endit),
                                                                Commands.runOnce(() -> stepRunning = false))));
        }

        public void createAmpTriggers() {

                resetSteps();
                m_swerve.ampActive = true;
                m_swerve.sourceActive = false;
                // Step 2 go shoot first note
                Trigger firstNoteToShootAmp = new Trigger(
                                () -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                                && m_swerve.ampActive && !stepRunning
                                                && m_swerve.autostep == movetoshootfrom1stpickup);

                // when shot from 1st note ends go try for second note set next step to 3 if
                // note
                Trigger shootToSecondNoteAmp = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && m_swerve.isStopped() && m_swerve.ampActive
                                && m_transfer.isStopped() && !stepRunning
                                && m_swerve.autostep == movefromshoottosecondnotepickup);

                // // if second note is picked up, go shoot it
                Trigger secondNoteToShootAmp = new Trigger(() -> DriverStation.isAutonomousEnabled() &&
                                m_swerve.ampActive && m_swerve.isStopped() && m_transfer.isStopped()
                                && m_transfer.noteAtIntake()
                                && !stepRunning && m_swerve.autostep == movefromshoottosecondnotepickup);

                // // if note C4 isn't collected, go try C5
                Trigger firstNoteToSecondNoteAmp = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && m_swerve.ampActive && m_swerve.isStopped() && !m_transfer.noteAtIntake()
                                && m_transfer.isStopped()
                                && !stepRunning && m_swerve.autostep == notemissingfindanother);

                firstNoteToShootAmp.onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> stepRunning = true),

                                                Commands.either(
                                                                new CenterToShoot(m_cf, m_pf.pathMaps.get(
                                                                                amppaths.Center2ToAmpShoot
                                                                                                .name()),
                                                                                m_swerve),
                                                                new CenterToShoot(m_cf, m_pf.pathMaps.get(
                                                                                amppaths.Center1ToAmpShoot
                                                                                                .name()),
                                                                                m_swerve),
                                                                () -> m_cf.innerNoteFirst),

                                                Commands.parallel(
                                                                Commands.runOnce(
                                                                                () -> m_swerve.autostep = movefromshoottosecondnotepickup),
                                                                Commands.runOnce(() -> stepRunning = false))));

                shootToSecondNoteAmp.onTrue(Commands.sequence(
                                Commands.runOnce(() -> stepRunning = true),
                                Commands.either(
                                                new AmpShootToCenterPickup(m_cf,
                                                                m_pf.pathMaps.get(amppaths.AmpShootToCenter1
                                                                                .name()),
                                                                m_swerve),
                                                new AmpShootToCenterPickup(m_cf,
                                                                m_pf.pathMaps.get(amppaths.AmpShootToCenter2
                                                                                .name()),
                                                                m_swerve),

                                                () -> m_cf.innerNoteFirst),

                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.autostep = checkfornote2),
                                                Commands.runOnce(() -> stepRunning = false))));

                secondNoteToShootAmp.onTrue(Commands.sequence(
                                Commands.runOnce(() -> stepRunning = true),
                                Commands.either(
                                                new CenterToShoot(m_cf, m_pf.pathMaps.get(amppaths.Center1ToAmpShoot
                                                                .name()), m_swerve),
                                                new CenterToShoot(m_cf, m_pf.pathMaps.get(amppaths.Center2ToAmpShoot
                                                                .name()), m_swerve),
                                                () -> m_cf.innerNoteFirst),

                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.autostep = endit),
                                                Commands.runOnce(() -> stepRunning = false))));

                firstNoteToSecondNoteAmp.onTrue(Commands.sequence(
                                Commands.runOnce(() -> stepRunning = true),
                                new RotateToAngle(m_swerve, 90)
                                                .unless(() -> LimelightHelpers
                                                                .getTV(CameraConstants.rearCamera.camname)),
                                m_intake.startIntakeCommand(),
                                Commands.deadline(
                                                new TryForAnotherNote(m_swerve, m_transfer, m_intake,
                                                                CameraConstants.rearCamera.camname),
                                                new TransferIntakeToSensor(m_transfer, m_intake, 6)),

                                Commands.either(
                                                Commands.sequence(
                                                                m_cf.autopickup(AllianceUtil
                                                                                .getAmpClearStagePose()),
                                                                m_cf.autopickup(AllianceUtil
                                                                                .getAmpShootPose())),
                                                Commands.none(),
                                                () -> m_transfer.noteAtIntake()),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.autostep = endit),
                                                Commands.runOnce(() -> stepRunning = false))));
        }

}