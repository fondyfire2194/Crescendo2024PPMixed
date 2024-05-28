// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import com.pathplanner.lib.path.PathPlannerPath;

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
import frc.robot.commands.Autos.SourceStart.CenterToShoot;
import frc.robot.commands.Autos.SourceStart.SourceShootToCenterPickup;
import frc.robot.commands.Drive.FindNote;
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
        private boolean stepRunning;
        private int step0 = 0;
        private int step1 = 1;// check for note C2/C4 - step 2 if present step 6 if not
        private int step2 = 2;// move from C2/C4 note to shoot set step 3
        private int step3 = 3;// move from shoot to second note for pickup set step 4
        private int step4 = 4;// check for second note C1/C5 - step 5 if present step 7 if not
        private int step5 = 5;// move from second note to shoot set end step
        private int step6 = 6;// first note pickup failed find second one if possible and go to shoot it
        private int step7 = 7;// second note pickup failed set end step

        private int endit = 8;
        @Log.NT(key = "sourceauto")
        public boolean sourceActive = false;
        @Log.NT(key = "ampauto")
        public boolean ampActive = false;

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
                m_swerve.autostep = step0;
                stepRunning = false;
        }

        public void createCommonTriggers() {

                Trigger checkFirstNote = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && !stepRunning && m_swerve.autostep == step1);

                checkFirstNote.onTrue(Commands.sequence(
                                Commands.runOnce(() -> stepRunning = true),
                                Commands.either(
                                                Commands.runOnce(() -> m_swerve.autostep = 2),
                                                Commands.runOnce(() -> m_swerve.autostep = 6),
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
                                Commands.runOnce(() -> trig5 = true),
                                Commands.runOnce(() -> stepRunning = false),
                                Commands.runOnce(() -> m_swerve.autostep = step0)));

        }
        // location conditions from, to and at are used to sequnce the triggers
        // 0 = start, 10 = source shoot, 4, 5 are center notes

        public void createSourceTriggers() {

                resettrigs();
                ampActive = false;
                sourceActive = true;

                // Step 2 go shoot first note
                Trigger firstNoteToShootSource = new Trigger(
                                () -> DriverStation.isAutonomousEnabled() && sourceActive && m_swerve.isStopped()
                                                && !stepRunning && m_swerve.autostep == step2);

                // when shot from 1st note ends go try for second note set next step to 3 if
                // note
                Trigger shootToSecondNoteSource = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && sourceActive && m_swerve.isStopped()
                                && m_transfer.isStopped() && !stepRunning && m_swerve.autostep == step3);

                // // if second note is picked up, go shoot it
                Trigger secondNoteToShootSource = new Trigger(() -> DriverStation.isAutonomousEnabled() &&
                                sourceActive && m_swerve.isStopped() && m_transfer.isStopped()
                                && m_transfer.noteAtIntake()
                                && !stepRunning && m_swerve.autostep == step4);

                // // if note C4 isn't collected, go try C5
                Trigger firstNoteToSecondNoteSource = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && sourceActive && m_swerve.isStopped() && !m_transfer.noteAtIntake()
                                && m_transfer.isStopped()
                                && !stepRunning && m_swerve.autostep == step6);

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
                                                                Commands.runOnce(() -> m_swerve.autostep = step3),
                                                                Commands.runOnce(() -> trig1 = true),
                                                                Commands.runOnce(() -> stepRunning = false))));

                shootToSecondNoteSource.onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> stepRunning = true),
                                                Commands.either(
                                                                new SourceShootToCenterPickup(m_cf,
                                                                                m_pf.pathMaps.get(
                                                                                                sourcepaths.SourceShootToCenter5
                                                                                                                .name()),
                                                                                m_swerve),
                                                                new SourceShootToCenterPickup(m_cf,
                                                                                m_pf.pathMaps.get(
                                                                                                sourcepaths.SourceShootToCenter4
                                                                                                                .name()),
                                                                                m_swerve),
                                                                () -> m_cf.innerNoteFirst),
                                                Commands.parallel(
                                                                Commands.runOnce(() -> m_swerve.autostep = step4),
                                                                Commands.runOnce(() -> trig2 = true),
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
                                                                Commands.runOnce(() -> m_swerve.autostep = step4),
                                                                Commands.runOnce(() -> trig2 = true),
                                                                Commands.runOnce(() -> stepRunning = false))));

                firstNoteToSecondNoteSource.onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> stepRunning = true),
                                                new FindNote(m_swerve, true, m_llv, CameraConstants.rearCamera.camname),
                                                m_intake.startIntakeCommand(),
                                                Commands.parallel(
                                                                new TransferIntakeToSensor(m_transfer, m_intake, .6),
                                                                new PickUpAlternateNote(m_swerve, m_transfer, m_intake,
                                                                                CameraConstants.rearCamera.camname,
                                                                                m_llv,
                                                                                5)),
                                                Commands.either(
                                                                m_cf.autopickup(FieldConstants.sourceShootBlue),
                                                                m_cf.autopickup(GeometryUtil
                                                                                .flipFieldPose(FieldConstants.sourceShootBlue)),
                                                                () -> DriverStation.getAlliance().isPresent()
                                                                                && DriverStation.getAlliance()
                                                                                                .get() == Alliance.Blue),
                                                Commands.parallel(
                                                                Commands.runOnce(() -> m_swerve.autostep = endit),
                                                                Commands.runOnce(() -> trig4 = true),
                                                                Commands.runOnce(() -> stepRunning = false))));

        }

        public void createAmpTriggers() {

                resettrigs();
                ampActive = true;
                sourceActive = false;
                // Step 2 go shoot first note
                Trigger firstNoteToShootAmp = new Trigger(
                                () -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                                && ampActive && !stepRunning && m_swerve.autostep == step2);

                // when shot from 1st note ends go try for second note set next step to 3 if
                // note
                Trigger shootToSecondNoteAmp = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && m_swerve.isStopped() && ampActive
                                && m_transfer.isStopped() && !stepRunning && m_swerve.autostep == step3);

                // // if second note is picked up, go shoot it
                Trigger secondNoteToShootAmp = new Trigger(() -> DriverStation.isAutonomousEnabled() &&
                                ampActive && m_swerve.isStopped() && m_transfer.isStopped() && m_transfer.noteAtIntake()
                                && !stepRunning && m_swerve.autostep == step4);

                // // if note C4 isn't collected, go try C5
                Trigger firstNoteToSecondNoteAmp = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && ampActive && m_swerve.isStopped() && !m_transfer.noteAtIntake()
                                && m_transfer.isStopped()
                                && !stepRunning && m_swerve.autostep == step6);

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
                                                                Commands.runOnce(() -> m_swerve.autostep = step3),
                                                                Commands.runOnce(() -> trig1 = true),

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
                                                Commands.runOnce(() -> m_swerve.autostep = step4),
                                                Commands.runOnce(() -> trig2 = true),
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
                                                Commands.runOnce(() -> m_swerve.autostep = step4),
                                                Commands.runOnce(() -> trig2 = true),
                                                Commands.runOnce(() -> stepRunning = false))));

                firstNoteToSecondNoteAmp.onTrue(Commands.sequence(
                                Commands.runOnce(() -> stepRunning = true),
                                new RotateToAngle(m_swerve, -90),
                                m_intake.startIntakeCommand(),
                                Commands.parallel(
                                                new TransferIntakeToSensor(m_transfer, m_intake, .6),
                                                new PickUpAlternateNote(m_swerve, m_transfer, m_intake,
                                                                CameraConstants.rearCamera.camname, m_llv,
                                                                1)),
                                Commands.either(
                                                m_cf.autopickup(FieldConstants.ampShootBlue),
                                                m_cf.autopickup(GeometryUtil
                                                                .flipFieldPose(FieldConstants.ampShootBlue)),
                                                () -> DriverStation.getAlliance().isPresent()
                                                                && DriverStation.getAlliance()
                                                                                .get() == Alliance.Blue),
                                Commands.parallel(
                                                Commands.runOnce(() -> m_swerve.autostep = endit),
                                                Commands.runOnce(() -> trig4 = true),
                                                Commands.runOnce(() -> stepRunning = false))));

        }

}