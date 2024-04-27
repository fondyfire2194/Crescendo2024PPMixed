// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants;
import frc.robot.PathFactory.sourcepaths;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Autos.AutoStarts.AutoSourceShootThenCenter4;
import frc.robot.commands.Autos.AutoStarts.AutoSourceShootThenCenter5;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoFactory {

        private final PathFactory m_pf;

        private final SwerveSubsystem m_swerve;

        public final SendableChooser<Integer> m_ampStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Integer> m_sourceStartChooser = new SendableChooser<Integer>();

        public int finalChoice = 0;

        int ampChoice;
        int ampChoiceLast;

        int sourceChoice;
        int sourceChoiceLast;

        public int validStartChoice = 0;

        public AutoFactory(PathFactory pf, SwerveSubsystem swerve) {

                m_pf = pf;

                m_swerve = swerve;

                m_ampStartChooser.setDefaultOption("Not Used", 0);
                m_ampStartChooser.addOption("Leave Zone", 1);
                m_ampStartChooser.addOption("C2 then C1", 2);
                m_ampStartChooser.addOption("C1 then C2", 3);

                m_sourceStartChooser.setDefaultOption("Not Used", 10);
                m_sourceStartChooser.addOption("C4 Then C5", 11);
                m_sourceStartChooser.addOption("C5 Then C4", 12);
                m_sourceStartChooser.addOption("C4 Then C5 Triggers", 13);
                

                Shuffleboard.getTab("Autonomous").add("AmpStart", m_ampStartChooser)
                                .withSize(3, 1).withPosition(0, 0);

                Shuffleboard.getTab("Autonomous").add("SourceStart", m_sourceStartChooser)
                                .withSize(3, 1).withPosition(6, 0);

                Shuffleboard.getTab("Autonomous").addBoolean("Valid Choice", () -> finalChoice != 0)
                                .withSize(10, 1).withPosition(0, 1)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                ShuffleboardLayout camLayout = Shuffleboard.getTab("Autonomous")
                                .getLayout("Cameras", BuiltInLayouts.kList).withPosition(0, 2)
                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP"));

                camLayout.addBoolean("FrontLeftCamera", () -> CameraConstants.frontLeftCamera.isActive)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                camLayout.addBoolean("FrontRightCamera", () -> CameraConstants.frontRightCamera.isActive)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                camLayout.addBoolean("RearCamera", () -> CameraConstants.rearCamera.isActive)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                ShuffleboardLayout fileCheckLayout = Shuffleboard.getTab("Autonomous")
                                .getLayout("PathFilesOK", BuiltInLayouts.kList).withPosition(1, 2)
                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP"));

                fileCheckLayout.addBoolean("AmpFiles", () -> m_pf.ampFilesOK);
                fileCheckLayout.addBoolean("SourceFiles", () -> m_pf.sourceFilesOK);

                // rearLayout.addNumber("BatteryVolts", () ->
                // RobotController.getBatteryVoltage());

        }

        public boolean checkChoiceChange() {

                ampChoice = m_ampStartChooser.getSelected();// 0 start
                sourceChoice = m_sourceStartChooser.getSelected();// 10 start

                boolean temp = ampChoice != ampChoiceLast || sourceChoice != sourceChoiceLast;

                ampChoiceLast = ampChoice;

                sourceChoiceLast = sourceChoice;

                return temp;
        }

        public int selectAndLoadPathFiles() {
                finalChoice = 0;
                if (ampChoice != 0 && sourceChoice == 10)
                        finalChoice = ampChoice;

                if (ampChoice == 0 && sourceChoice != 10)
                        finalChoice = sourceChoice;

                SmartDashboard.putNumber("FC", finalChoice);

                if (finalChoice > 0 && finalChoice < 10) {
                        m_pf.linkAmpPaths();
                        SmartDashboard.putNumber("LENGTHAmp", m_pf.pathMaps.size());

                }

                if (finalChoice > 10 && finalChoice < 20) {
                        m_pf.linkSourcePaths();
                        SmartDashboard.putNumber("LENGTHSource", m_pf.pathMaps.size());
                }

                return finalChoice;
        }

}