// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Autos.AmpStart.AutoAmpShootThenCenter;
import frc.robot.commands.Autos.AutoStarts.AutoSourceShootCenterPathfind;
import frc.robot.commands.Autos.AutoStarts.AutoSourceShootThenCenter;
import frc.robot.commands.Autos.AutoStarts.AutoSourceThenCenterVision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.ShootingData;
import monologue.Annotations.Log;
import monologue.Logged;

/** Add your docs here. */
public class AutoFactory implements Logged {

        private final PathFactory m_pf;

        public final SendableChooser<Integer> m_ampStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Integer> m_sourceStartChooser = new SendableChooser<Integer>();

        public SendableChooser<Command> m_subwfrStartChooser;

        @Log.NT(key = "finalchoice")
        public int finalChoice = 0;

        int ampChoice;
        int ampChoiceLast;
        String subwfrcchoicelasst;
        int sourceChoice;
        int sourceChoiceLast;
        String subwfrcchoice;
        @Log.NT(key = "validstartchoice")
        public int validStartChoice = 0;
        String subwfrdefnam;
        public int minsourceauto;
        public int maxsourceauto;
        public int minampauto;
        public int maxampauto;

        private final SwerveSubsystem m_swerve;

        private final IntakeSubsystem m_intake;

        private final TransferSubsystem m_transfer;

        private final ShooterSubsystem m_shooter;

        private final ArmSubsystem m_arm;

        private final LimelightVision m_llv;

        private CommandFactory m_cf;

        public boolean validChoice;

        public AutoFactory(PathFactory pf,CommandFactory cf,SwerveSubsystem swerve, ShooterSubsystem shooter, ArmSubsystem arm,
                        IntakeSubsystem intake, TransferSubsystem transfer,
                        LimelightVision llv) {
                m_pf = pf;
                m_cf=cf;
                m_llv=llv;
                m_swerve=swerve;
                m_shooter=shooter;
                m_transfer=transfer;
                m_intake=intake;
                m_arm=arm;


                minsourceauto = 11;
                m_sourceStartChooser.setDefaultOption("Not Used", 10);
                m_sourceStartChooser.addOption("C4 Then C5", 11);
                m_sourceStartChooser.addOption("C5 Then C4", 12);
                m_sourceStartChooser.addOption("C4 Pathfind Then C5", 13);
                m_sourceStartChooser.addOption("C4 Vision Then C5", 14);
                maxsourceauto = 14;
                minampauto = 21;
                m_ampStartChooser.setDefaultOption("Not Used", 20);
                m_ampStartChooser.addOption("Shoot C2 then C1", 21);
                m_ampStartChooser.addOption("Shoot C1 then C2", 22);
                // m_ampStartChooser.addOption("Shoot Moving C2 then C1", 23);
                maxampauto = 22;
                m_subwfrStartChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Source Start", m_sourceStartChooser);
                SmartDashboard.putData("Amp Start", m_ampStartChooser);
                SmartDashboard.putData("SubwfrStart", m_subwfrStartChooser);

                subwfrdefnam = m_subwfrStartChooser.getSelected().getName();

        }

        // This method is run by an EventLoop in RobotContainer
        public boolean checkChoiceChange() {

                ampChoice = m_ampStartChooser.getSelected();// 20 start
                sourceChoice = m_sourceStartChooser.getSelected();// 10 start
                subwfrcchoice = m_subwfrStartChooser.getSelected().getName();
                SmartDashboard.putNumber("SWLGTH", subwfrcchoice.length());
                boolean temp = ampChoice != ampChoiceLast || sourceChoice != sourceChoiceLast
                                || subwfrcchoice != subwfrcchoicelasst;

                ampChoiceLast = ampChoice;
                sourceChoiceLast = sourceChoice;
                subwfrcchoicelasst = subwfrcchoice;
                return temp;
        }

        public int selectAndLoadPathFiles() {
                finalChoice = 0;
                validChoice = false;

                boolean validAmpChoice = ampChoice != 20;
                boolean validSourceChoice = sourceChoice != 10;
                boolean validSubwfrChoice = subwfrcchoice != subwfrdefnam;

                if (validAmpChoice && !validSourceChoice && !validSubwfrChoice) {
                        m_pf.linkAmpPaths();
                        validChoice = true;
                        finalChoice = ampChoice;

                }

                if (validSourceChoice && !validAmpChoice && !validSubwfrChoice) {
                        m_pf.linkSourcePaths();
                        validChoice = true;
                        finalChoice = sourceChoice;
                }

                if (!validAmpChoice && !validSourceChoice && validSubwfrChoice) {
                        validChoice = true;
                }

                SmartDashboard.putBoolean("Auto//Valid Auto Start Choice", validChoice);

                return finalChoice;
        }

        public Command finalCommand(int choice) {

                switch ((choice)) {

                        case 11:
                                return new AutoSourceShootThenCenter(m_cf, m_pf,
                                                m_swerve, true);

                        case 12:
                                return new AutoSourceShootThenCenter(m_cf, m_pf,
                                                m_swerve, false);

                        case 13:
                                return new AutoSourceShootCenterPathfind(m_cf, m_pf,
                                                m_swerve, true);
                        case 14:
                                return new AutoSourceThenCenterVision(m_cf, m_pf,
                                                m_swerve, m_llv, m_intake, m_transfer, true);

                        case 21:
                                return new AutoAmpShootThenCenter(m_cf, m_pf,
                                                m_swerve, true);
                        case 22:
                                return new AutoAmpShootThenCenter(m_cf, m_pf,
                                                m_swerve, false);
                        // case 23:
                        // return new AutoAmpShootThenCenter(this, m_pf,
                        // m_swerve, true);

                        // case 24:
                        // return new AutoAmpShootMovingThenCenter(this,
                        // m_pf.pathMaps.get(amppaths.AmpToCenter2.name()),
                        // m_swerve);

                        default:
                                return Commands.none();

                }

        }

        public Command getAutonomousCommand() {
                return finalCommand(finalChoice);
        }

}