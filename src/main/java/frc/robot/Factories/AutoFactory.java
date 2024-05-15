// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

        public boolean validChoice;

        public AutoFactory(PathFactory pf) {
                m_pf = pf;
                minsourceauto = 11;
                m_sourceStartChooser.setDefaultOption("Not Used", 10);
                m_sourceStartChooser.addOption("C4 Then C5", 11);
                m_sourceStartChooser.addOption("Shoot Moving C4 Then C5 ", 12);
                m_sourceStartChooser.addOption("C4 Pathfind Then C5", 13);
                m_sourceStartChooser.addOption("C4 Vision Then C5", 14);
                maxsourceauto = 14;
                minampauto = 21;
                m_ampStartChooser.setDefaultOption("Not Used", 20);
                m_ampStartChooser.addOption("Shoot C2 then C1", 21);
                m_ampStartChooser.addOption("Shoot Moving C2 the C1", 22);
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

}