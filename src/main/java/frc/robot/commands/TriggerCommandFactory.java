// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.impl.FilteredBeanPropertyWriter;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoFactory;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimelightHelpers;
import frc.robot.PathFactory;
import frc.robot.PathFactory.sourcepaths;
import frc.robot.commands.Arm.CheckArmAtTarget;
import frc.robot.commands.Autos.AutoStarts.AutoSourceShootThenCenter4;
import frc.robot.commands.Autos.AutoStarts.AutoSourceShootThenCenter4Triggers;
import frc.robot.commands.Autos.AutoStarts.AutoSourceShootThenCenter5;
import frc.robot.commands.Autos.SourceStart.Center4ToCenter5Pickup;
import frc.robot.commands.Autos.SourceStart.Center4ToSourceShoot;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.commands.Shooter.CheckShooterAtSpeed;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
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

        private Trigger triggerA;

        private Trigger triggerb;

        public boolean atNote4;

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

        public void createSourceTriggers() {
                triggerA = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && m_transfer.noteAtIntake());

                triggerA.onTrue(new Center4ToSourceShoot(m_cf, m_pf, m_swerve))
                                .onTrue(Commands.runOnce(() -> SmartDashboard.putNumber("trigA", 909)));

                triggerb = new Trigger(() -> DriverStation.isAutonomousEnabled() && m_swerve.isStopped()
                                && m_transfer.isStopped() && !m_transfer.noteAtIntake());

                triggerA.onTrue(new SourceShootToCenter(m_cf,
                                m_pf.pathMaps.get(sourcepaths.SourceShootToCenter5.name()), m_af, m_pf, m_swerve,
                                m_intake, m_shooter, m_arm,
                                m_transfer))
                                .onTrue(Commands.runOnce(() -> SmartDashboard.putNumber("trigB", 909)));
        }

}