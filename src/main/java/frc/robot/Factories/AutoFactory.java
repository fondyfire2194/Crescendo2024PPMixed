// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;
import monologue.Annotations.Log;
import monologue.Logged;

/** Add your docs here. */
public class AutoFactory implements Logged {

        private final PathFactory m_pf;

        public final SendableChooser<Integer> m_ampStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Integer> m_sourceStartChooser = new SendableChooser<Integer>();

        SendableChooser<Command> m_subwfrStartChooser;

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

        public AutoFactory(PathFactory pf) {
                m_pf = pf;

                m_ampStartChooser.setDefaultOption("Not Used", 20);
                m_ampStartChooser.addOption("C2 then C1", 21);
                m_ampStartChooser.addOption("C1 then C2", 22);

                m_sourceStartChooser.setDefaultOption("Not Used", 10);
                m_sourceStartChooser.addOption("C4 Then C5", 11);
                m_sourceStartChooser.addOption("Shoot Moving C4 Then C5 ", 12);
                m_sourceStartChooser.addOption("C4 Pathfind Then C5", 13);
                m_sourceStartChooser.addOption("C4 Vision Then C5", 14);

                m_subwfrStartChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Source Start", m_sourceStartChooser);
                SmartDashboard.putData("Amp Start", m_ampStartChooser);
                SmartDashboard.putData("SubwfrStart", m_subwfrStartChooser);

                subwfrdefnam = m_subwfrStartChooser.getSelected().getName();

        }

        public boolean checkChoiceChange() {

                ampChoice = m_ampStartChooser.getSelected();// 20 start
                sourceChoice = m_sourceStartChooser.getSelected();// 10 start
                subwfrcchoice = m_subwfrStartChooser.getSelected().getName();
                boolean temp = ampChoice != ampChoiceLast || sourceChoice != sourceChoiceLast
                                || subwfrcchoice != subwfrcchoicelasst;

                ampChoiceLast = ampChoice;
                sourceChoiceLast = sourceChoice;
                subwfrcchoicelasst = subwfrcchoice;
                return temp;
        }

        public int selectAndLoadPathFiles() {
                finalChoice = 0;
                boolean validChoice = false;

                SmartDashboard.putNumber("LENGTHAmp", 0);
                SmartDashboard.putNumber("LENGTHSource", 0);

                boolean validAmpChoice = ampChoice != 20;
                boolean validSourceChoice = sourceChoice != 10;
                boolean validSubwfrChoice = subwfrcchoice != subwfrdefnam;

                if (validAmpChoice && !validSourceChoice && !validSubwfrChoice) {
                        m_pf.linkAmpPaths();
                        validChoice = true;
                        finalChoice=ampChoice;
                        SmartDashboard.putNumber("LENGTHAmp", m_pf.pathMaps.size());
                }

                if (validSourceChoice && !validAmpChoice && !validSubwfrChoice) {
                        m_pf.linkSourcePaths();
                        validChoice = true;
                        finalChoice=sourceChoice;
                        SmartDashboard.putNumber("LENGTHSource", m_pf.pathMaps.size());
                }

                if (!validAmpChoice && !validSourceChoice && validSubwfrChoice) {
                        validChoice = true;
                }

                SmartDashboard.putBoolean("ValidChoice", validChoice);

                return finalChoice;
        }

}