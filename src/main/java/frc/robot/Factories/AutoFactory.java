// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.commands.Autos.AutoStarts.AutoAmpCompleteVis;
import frc.robot.commands.Autos.AutoStarts.AutoAmpCompletePF;
import frc.robot.commands.Autos.AutoStarts.AutoSourceCompleteVis;
import frc.robot.commands.Autos.Autos.PickupUsingVision;
import frc.robot.commands.Autos.AutoStarts.AutoSourceCompletePF;
import frc.robot.commands.Autos.SubwfrStart.AutoSbwfrShootThenSequence;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

/** Add your docs here. */
public class AutoFactory {

        private final PathFactory m_pf;

        private final SubwooferAutoCommands m_sac;

        public SendableChooser<Integer> m_subwfrStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Integer> m_ampStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Integer> m_sourceStartChooser = new SendableChooser<Integer>();

        public int finalChoice = 0;

        int ampChoice;
        int ampChoiceLast;
        int subwfrchoice;
        int subwfrchoicelast;
        int sourceChoice;
        int sourceChoiceLast;

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

        public AutoFactory(PathFactory pf, CommandFactory cf, SubwooferAutoCommands sac, SwerveSubsystem swerve,
                        ShooterSubsystem shooter,
                        ArmSubsystem arm,
                        IntakeSubsystem intake, TransferSubsystem transfer) {
                m_pf = pf;
                m_cf = cf;
                m_sac = sac;
                m_swerve = swerve;
                m_transfer = transfer;
                m_intake = intake;

                minsbwfrauto = 1;
                m_subwfrStartChooser.setDefaultOption("Not Used", 0);
                m_subwfrStartChooser.addOption("W2-W1-W3", 1);
                m_subwfrStartChooser.addOption("W2-W3-W1", 2);
                m_subwfrStartChooser.addOption("W2-W3", 3);
                m_subwfrStartChooser.addOption("W2-W1", 4);
                m_subwfrStartChooser.addOption("W2-C3-SBWFR-W3", 5);
                m_subwfrStartChooser.addOption("W2-C3-SBWFR-W1", 6);
                m_subwfrStartChooser.addOption("W2-C3-SBWFR-W3-Vis", 7);
                m_subwfrStartChooser.addOption("W2-C3-SBWFR-W1-Vis", 8);

                maxsbwfrauto = 8;

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
                                return new AutoSbwfrShootThenSequence(m_cf, m_pf, m_sac, m_swerve,
                                                sbwfrpaths.SubwfrShootToWing2, sbwfrpaths.Wing2ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing1, sbwfrpaths.Wing1ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing3, sbwfrpaths.Wing3ToSubwfrShoot);
                        case 2:
                                return new AutoSbwfrShootThenSequence(m_cf, m_pf, m_sac, m_swerve,
                                                sbwfrpaths.SubwfrShootToWing2, sbwfrpaths.Wing2ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing3, sbwfrpaths.Wing3ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing1, sbwfrpaths.Wing1ToSubwfrShoot);

                        case 3:
                                return new AutoSbwfrShootThenSequence(m_cf, m_pf, m_sac, m_swerve,
                                                sbwfrpaths.SubwfrShootToWing3, sbwfrpaths.Wing3ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing2, sbwfrpaths.Wing2ToSubwfrShoot);

                        case 4:
                                return new AutoSbwfrShootThenSequence(m_cf, m_pf, m_sac, m_swerve,
                                                sbwfrpaths.SubwfrShootToWing2, sbwfrpaths.Wing2ToSubwfrShoot,
                                                sbwfrpaths.SubwfrShootToWing1, sbwfrpaths.Wing1ToSubwfrShoot);

                        case 5:
                                return Commands.sequence(
                                                m_sac.setsbwrstart(m_swerve, m_cf),
                                                m_sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing2, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.shootbydistance(m_cf),
                                                m_sac.moveAndPickup(sbwfrpaths.Wing2ToCenter3, m_swerve, m_cf, m_pf),
                                                m_sac.move(sbwfrpaths.Center3ToWing2, m_swerve, m_pf),
                                                m_sac.sbwfrmoveandshoot(sbwfrpaths.Wing2ToSubwfrShoot, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing3Shoot, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.shootbydistance(m_cf));

                        case 6:
                                return Commands.sequence(
                                                m_sac.setsbwrstart(m_swerve, m_cf),
                                                m_sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing2, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.shootbydistance(m_cf),
                                                m_sac.moveAndPickup(sbwfrpaths.Wing2ToCenter3, m_swerve, m_cf, m_pf),
                                                m_sac.move(sbwfrpaths.Center3ToWing2, m_swerve, m_pf),
                                                m_sac.sbwfrmoveandshoot(sbwfrpaths.Wing2ToSubwfrShoot, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing1Shoot, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.shootbydistance(m_cf));

                        case 7:
                                return Commands.sequence(
                                                m_sac.setsbwrstart(m_swerve, m_cf),
                                                m_sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing2, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.shootbydistance(m_cf),
                                                new PickupUsingVision(m_cf,
                                                                m_pf.pathMaps.get(sbwfrpaths.Wing2ToCenter3.name()),
                                                                m_pf.pathMaps.get(sbwfrpaths.Wing2ToCenter3.name()),
                                                                m_transfer, m_intake, m_swerve, true, -1,
                                                                1, -1, 1),
                                                m_sac.move(sbwfrpaths.Center3ToWing2, m_swerve, m_pf),
                                                m_sac.sbwfrmoveandshoot(sbwfrpaths.Wing2ToSubwfrShoot, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing3Shoot, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.shootbydistance(m_cf));

                        case 8:
                                return Commands.sequence(
                                                m_sac.setsbwrstart(m_swerve, m_cf),
                                                m_sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing2, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.shootbydistance(m_cf),
                                                new PickupUsingVision(m_cf,
                                                                m_pf.pathMaps.get(sbwfrpaths.Wing2ToCenter3.name()),
                                                                m_pf.pathMaps.get(sbwfrpaths.Wing2ToCenter3.name()),
                                                                m_transfer, m_intake, m_swerve, true, -1,
                                                                1, -1, 1),
                                                m_sac.move(sbwfrpaths.Center3ToWing2, m_swerve, m_pf),
                                                m_sac.sbwfrmoveandshoot(sbwfrpaths.Wing2ToSubwfrShoot, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.moveAndPickup(sbwfrpaths.SubwfrShootToWing1Shoot, m_swerve, m_cf,
                                                                m_pf),
                                                m_sac.shootbydistance(m_cf));

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

        Command shoot(CommandFactory cf, double angle, double rpm) {
                return Commands.sequence(
                                cf.positionArmRunShooterSpecialCase(angle, rpm),
                                cf.transferNoteToShooterCommand());
        }

        Command moveandshoot(SwerveSubsystem swerve, sbwfrpaths path, CommandFactory cf, PathFactory pf,
                        double angle, double rpm) {
                return Commands.sequence(
                                Commands.parallel(
                                                new RunPPath(swerve, pf.pathMaps.get(path.name())),
                                                cf.positionArmRunShooterSpecialCase(
                                                                angle, rpm)),
                                cf.transferNoteToShooterCommand());
        }

        Command moveAndPickup(SwerveSubsystem swerve, sbwfrpaths path, CommandFactory cf, PathFactory pf) {
                return Commands.parallel(
                                new RunPPath(swerve,
                                                pf.pathMaps.get(path.name())),
                                Commands.sequence(
                                                Commands.waitSeconds(.25),
                                                cf.doIntake()));
        }

        public Command getAutonomousCommand() {
                return finalCommand(finalChoice);
        }

}