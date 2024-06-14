// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Autos.AutoStarts.AutoAmpCompleteVis;
import frc.robot.commands.Autos.AutoStarts.AutoAmpCompletePF;
import frc.robot.commands.Autos.AutoStarts.AutoSourceCompleteVis;
import frc.robot.commands.Autos.AutoStarts.AutoSourceCompletePF;
import frc.robot.commands.Autos.SubwfrStart.AutoSbwfrShootThenSequence;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import monologue.Annotations.Log;
import monologue.Logged;

/** Add your docs here. */
public class AutoFactory implements Logged {

        private final PathFactory m_pf;

        public SendableChooser<Integer> m_subwfrStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Integer> m_ampStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Integer> m_sourceStartChooser = new SendableChooser<Integer>();

        @Log.NT(key = "finalchoice")
        public int finalChoice = 0;

        int ampChoice;
        int ampChoiceLast;
        int subwfrchoice;
        int subwfrchoicelast;
        int sourceChoice;
        int sourceChoiceLast;

        @Log.NT(key = "validstartchoice")
        public int validStartChoice = 0;

        public int minsbwfrauto;
        public int maxsbwfrauto;
        public int minsourceauto;
        public int maxsourceauto;
        public int minampauto;
        public int maxampauto;

        private final SwerveSubsystem m_swerve;

        private final IntakeSubsystem m_intake;

        private final TransferSubsystem m_transfer;

        private CommandFactory m_cf;

        public boolean validChoice;

        public AutoFactory(PathFactory pf, CommandFactory cf, SwerveSubsystem swerve, ShooterSubsystem shooter,
                        ArmSubsystem arm,
                        IntakeSubsystem intake, TransferSubsystem transfer) {
                m_pf = pf;
                m_cf = cf;
                m_swerve = swerve;
                m_transfer = transfer;
                m_intake = intake;

                minsbwfrauto = 1;
                m_subwfrStartChooser.setDefaultOption("Not Used", 0);
                m_subwfrStartChooser.addOption("        W2-W1-W3", 1);
                m_subwfrStartChooser.addOption("W2-W3-W1", 2);
                m_subwfrStartChooser.addOption("W2-W3", 3);
                m_subwfrStartChooser.addOption("W2-W1", 4);
                m_subwfrStartChooser.addOption("W2-C3-W3-W1", 5);
                maxsbwfrauto = 5;

                minsourceauto = 11;
                m_sourceStartChooser.setDefaultOption("Not Used", 10);
                m_sourceStartChooser.addOption("C4 Then C5 Vis", 11);
                m_sourceStartChooser.addOption("C5 Then C4 Vis", 12);
                m_sourceStartChooser.addOption("C4 Then C5 PF", 13);
                m_sourceStartChooser.addOption("C5 Then C4 PF", 14);

                maxsourceauto = 14;

                minampauto = 21;
                m_ampStartChooser.setDefaultOption("Not Used", 20);
                m_ampStartChooser.addOption("C2 then C1 Vis", 21);
                m_ampStartChooser.addOption("C1 then C2 Vis", 22);
                m_ampStartChooser.addOption("C2 then C1 PF", 23);
                m_ampStartChooser.addOption("C1 then C2 PF", 24);

                maxampauto = 24;

                SmartDashboard.putData("Source Start", m_sourceStartChooser);
                SmartDashboard.putData("Amp Start", m_ampStartChooser);
                SmartDashboard.putData("SubwfrStart", m_subwfrStartChooser);

        }

        // This method is run by an EventLoop in RobotContainer
        public boolean checkChoiceChange() {

                ampChoice = m_ampStartChooser.getSelected();// 20 start
                sourceChoice = m_sourceStartChooser.getSelected();// 10 start
                subwfrchoice = m_subwfrStartChooser.getSelected();// 0 start

                boolean temp = ampChoice != ampChoiceLast || sourceChoice != sourceChoiceLast
                                || subwfrchoice != subwfrchoicelast;

                ampChoiceLast = ampChoice;
                sourceChoiceLast = sourceChoice;
                subwfrchoicelast = subwfrchoice;
                return temp;
        }

        public int selectAndLoadPathFiles() {
                finalChoice = 0;
                validChoice = false;
                boolean validSubwfrChoice = subwfrchoice != 0;
                boolean validSourceChoice = sourceChoice != 10;
                boolean validAmpChoice = ampChoice != 20;

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
                        m_pf.linkSbwfrPaths();
                        validChoice = true;
                        finalChoice = subwfrchoice;
                }

                SmartDashboard.putBoolean("Auto//Valid Auto Start Choice", validChoice);

                return finalChoice;
        }

        public Command finalCommand(int choice) {

                switch ((choice)) {

                        case 1:
                                return new AutoSbwfrShootThenSequence(m_cf, m_pf, m_swerve,
                                                sbwfrpaths.SubwfrShootToWing2, sbwfrpaths.Wing2ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing1, sbwfrpaths.Wing1ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing3, sbwfrpaths.Wing3ToSubwfrShoot);
                        case 2:
                                return new AutoSbwfrShootThenSequence(m_cf, m_pf, m_swerve,
                                                sbwfrpaths.SubwfrShootToWing2, sbwfrpaths.Wing2ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing3, sbwfrpaths.Wing3ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing1, sbwfrpaths.Wing1ToSubwfrShoot);

                        case 3:
                                return new AutoSbwfrShootThenSequence(m_cf, m_pf, m_swerve,
                                                sbwfrpaths.SubwfrShootToWing2, sbwfrpaths.Wing2ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing3, sbwfrpaths.Wing3ToSubwfrShoot);

                        case 4:
                                return new AutoSbwfrShootThenSequence(m_cf, m_pf, m_swerve,
                                                sbwfrpaths.SubwfrShootToWing2, sbwfrpaths.Wing2ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing1, sbwfrpaths.Wing1ToSubwfrShoot);

                        case 5:
                                return new AutoSbwfrShootThenSequence(m_cf, m_pf, m_swerve,
                                                sbwfrpaths.SubwfrShootToWing2, sbwfrpaths.Wing2ToCenter3,
                                                sbwfrpaths.Center3ToWing2, sbwfrpaths.QuickToNote3,
                                                sbwfrpaths.Quick3ToNote1);

                        case 11:
                                return new AutoSourceCompleteVis(m_cf, m_pf, this,
                                                m_swerve, m_intake, m_transfer, true);
                        case 12:
                                return new AutoSourceCompleteVis(m_cf, m_pf, this,
                                                m_swerve, m_intake, m_transfer, false);

                        case 13:
                                return new AutoSourceCompletePF(m_cf, m_pf, this,
                                                m_swerve, m_intake, m_transfer, true);
                        case 14:
                                return new AutoSourceCompletePF(m_cf, m_pf, this,
                                                m_swerve, m_intake, m_transfer, false);

                        case 21:
                                return new AutoAmpCompleteVis(m_cf, m_pf, this,
                                                m_swerve, m_intake, m_transfer, true);

                        case 22:
                                return new AutoAmpCompleteVis(m_cf, m_pf, this,
                                                m_swerve, m_intake, m_transfer, false);
                        case 23:
                                return new AutoAmpCompletePF(m_cf, m_pf, this,
                                                m_swerve, m_intake, m_transfer, true);

                        case 24:
                                return new AutoAmpCompletePF(m_cf, m_pf, this,
                                                m_swerve, m_intake, m_transfer, false);

                        default:
                                return Commands.none();

                }

        }

        public Command getAutonomousCommand() {
                return finalCommand(finalChoice);
        }

}