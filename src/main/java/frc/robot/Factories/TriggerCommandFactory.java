// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos.SourceStart.Center4ToCenter5Pickup;
import frc.robot.commands.Autos.SourceStart.Center4ToSourceShoot;
import frc.robot.commands.Autos.SourceStart.Center5ToSourceShoot;
import frc.robot.commands.Autos.SourceStart.SourceShootToCenter5Pickup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import monologue.Annotations.Log;
import monologue.Logged;

/** Add your docs here. */
public class TriggerCommandFactory implements Logged {

        private final SwerveSubsystem m_swerve;

        private final IntakeSubsystem m_intake;

        private final TransferSubsystem m_transfer;

        private final ShooterSubsystem m_shooter;

        private final ArmSubsystem m_arm;

        private final LimelightVision m_llv;

        private final ClimberSubsystem m_climber;

        private final AutoFactory m_af;

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

        public TriggerCommandFactory(SwerveSubsystem swerve, ShooterSubsystem shooter, ArmSubsystem arm,
                        IntakeSubsystem intake, TransferSubsystem transfer, ClimberSubsystem climber,
                        LimelightVision llv, AutoFactory af, PathFactory pf, CommandFactory cf) {
                m_swerve = swerve;
                m_shooter = shooter;
                m_arm = arm;
                m_intake = intake;
                m_transfer = transfer;
                m_climber = climber;
                m_llv = llv;
                m_af = af;
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
        }

        public void createSourceTriggersC4C5() {

                resettrigs();

                // move to shoot if a note C4 is picked up

                Trigger triggerC4ToSS = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && m_transfer.noteAtIntake() && m_transfer.intaketries == 1);

                triggerC4ToSS.onTrue(new Center4ToSourceShoot(m_cf, m_pf, m_swerve))
                                .onTrue(Commands.runOnce(() -> m_swerve.fromLocation = 4))
                                .onTrue(Commands.runOnce(() -> trig1 = true));
                              
                // when shot from note C4 ends go try for note C5
                Trigger triggerSSToC5 = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && m_transfer.isStopped() && !m_transfer.noteAtIntake() && m_swerve.fromLocation == 4);

                triggerSSToC5.onTrue(new SourceShootToCenter5Pickup(m_cf, m_pf, m_swerve))
                                .onTrue(Commands.runOnce(() -> m_swerve.fromLocation = 10))
                                .onTrue(Commands.runOnce(() -> trig2 = true));
                                

                // // if note C5 is picked up, go shoot it
                Trigger triggerC5ToSS = new Trigger(() -> DriverStation.isAutonomousEnabled() &&
                                m_swerve.isStopped() && m_transfer.isStopped() && m_transfer.noteAtIntake()
                                && (m_swerve.fromLocation == 10 || m_swerve.fromLocation == 4)
                                && m_transfer.intaketries > 1);

                triggerC5ToSS.onTrue(new Center5ToSourceShoot(m_cf, m_pf, m_swerve))
                                .onTrue(Commands.runOnce(() -> m_swerve.fromLocation = 5))
                                .onTrue(Commands.runOnce(() -> trig3 = true));
                              

                // // if note C4 isn't collected, go try C5
                Trigger triggerC4ToC5 = new Trigger(() -> DriverStation.isAutonomousEnabled()
                                && m_swerve.isStopped()
                                && !m_transfer.noteAtIntake() && m_swerve.fromLocation == 0
                                && m_transfer.intaketries == 1);

                triggerC4ToC5.onTrue(new Center4ToCenter5Pickup(m_cf, m_pf, m_swerve))
                                .onTrue(Commands.runOnce(() -> m_swerve.fromLocation = 4))
                                .onTrue(Commands.runOnce(() -> trig4 = true));
                       

        }

}