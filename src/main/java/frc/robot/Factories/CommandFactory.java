// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import org.ejml.ops.FConvertArrays;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Pref;
import frc.robot.Factories.PathFactory.amppaths;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Autos.AmpStart.AutoAmpShootThenCenter;
import frc.robot.commands.Autos.AutoStarts.AutoSourceShootCenter4Pathfind;
import frc.robot.commands.Autos.AutoStarts.AutoSourceShootMovingThenCenter;
import frc.robot.commands.Autos.AutoStarts.AutoSourceShootThenCenter;
import frc.robot.commands.Autos.AutoStarts.AutoSourceThenCenter4Vision;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import monologue.Annotations.Log;
import monologue.Logged;

/** Add your docs here. */
public class CommandFactory implements Logged {

        private final SwerveSubsystem m_swerve;

        private final IntakeSubsystem m_intake;

        private final TransferSubsystem m_transfer;

        private final ShooterSubsystem m_shooter;

        private final ArmSubsystem m_arm;

        private final LimelightVision m_llv;

        private final ClimberSubsystem m_climber;

        private final AutoFactory m_af;

        private final PathFactory m_pf;
        @Log.NT(key = "startpose")
        Pose2d tempPose2d = new Pose2d();

        private Trigger runShooterTrigger;
        @Log.NT(key = "startontheflyshoot")
        public boolean startShoot;

        public CommandFactory(SwerveSubsystem swerve, ShooterSubsystem shooter, ArmSubsystem arm,
                        IntakeSubsystem intake, TransferSubsystem transfer, ClimberSubsystem climber,
                        LimelightVision llv, AutoFactory af, PathFactory pf) {
                m_swerve = swerve;
                m_shooter = shooter;
                m_arm = arm;
                m_intake = intake;
                m_transfer = transfer;
                m_climber = climber;
                m_llv = llv;
                m_af = af;
                m_pf = pf;

                runShooterTrigger = new Trigger(() -> startShoot);

                runShooterTrigger.onTrue(transferNoteToShooterCommand());

        }

        public Command autopickup(Pose2d targetPose) {
                return AutoBuilder.pathfindToPose(
                                targetPose,
                                new PathConstraints(
                                                3.25, 4.0,
                                                Units.degreesToRadians(360), Units.degreesToRadians(540)),
                                0,
                                0);
        }

        public Command positionArmRunShooterByDistance() {
                return new FunctionalCommand(
                                () -> Commands.none(),
                                () -> {
                                        m_shooter.startShooter(
                                                        Constants.shooterRPMMap.get(m_swerve.getDistanceFromSpeaker()));
                                        m_arm.setGoal(Units.degreesToRadians(
                                                        Constants.armAngleMap.get(m_swerve.getDistanceFromSpeaker())));
                                },
                                (interrupted) -> Commands.none(),
                                () -> false);
        }

        public Command positionArmRunShooterByDistanceLob() {
                return new FunctionalCommand(
                                () -> Commands.none(),
                                () -> {
                                        m_shooter.startShooter(
                                                        Constants.shooterLobRPMMap.get(m_swerve.getDistanceFromLobTarget()));
                                        m_arm.setGoal(Units.degreesToRadians(
                                                        Constants.armLobAngleMap.get(m_swerve.getDistanceFromLobTarget())));
                                },
                                (interrupted) -> Commands.none(),
                                () -> false);
        }

        // @Log.NT(key = "posarmrunshootercommand")
        public Command positionArmRunShooterSpecialCase(double armAngleDeg, double shooterSpeed) {
                return Commands.parallel(
                                m_arm.setGoalCommand(Units.degreesToRadians(armAngleDeg)),
                                m_shooter.startShooterCommand(shooterSpeed));

        }

        // @Log.NT(key = "dointakecommand")
        public Command doIntake() {
                return new ParallelCommandGroup(
                                m_intake.startIntakeCommand(),
                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians),
                                new TransferIntakeToSensor(m_transfer, m_intake, 3));
        }

        public Command transferNoteToShooterCommand() {
                return Commands.parallel(
                                m_transfer.transferToShooterCommand(),
                                Commands.runOnce(() -> startShoot = false));
        }

        public Command alignShootCommand() {
                return Commands.sequence(alignToTag(),
                                m_transfer.transferToShooterCommand());
        }

        public Command alignToTag() {
                return new FunctionalCommand(
                                () -> m_llv.setAlignSpeakerPipeline(),
                                () -> m_swerve.alignToAngle(
                                                LimelightHelpers.getTX(CameraConstants.frontLeftCamera.camname)),
                                (interrupted) -> m_llv.setAprilTag_ALL_Pipeline(),
                                () -> Math.abs(LimelightHelpers.getTX(CameraConstants.frontLeftCamera.camname)) < 1,
                                m_swerve);
        }

        public Command alignShootCommand(double meters) {
                return Commands.parallel(alignToTag(),
                                Commands.run(() -> m_arm.trackDistance(meters)),
                                Commands.run(() -> m_shooter.rpmTrackDistance(meters)));
        }

        public Command clearStickyFaultsCommand() {
                return Commands.parallel(
                                m_arm.clearFaultsCommand(),
                                m_intake.clearFaultsCommand(),
                                m_transfer.clearFaultsCommand(),
                                m_climber.clearFaultsCommand(),
                                m_swerve.clearFaultsCommand());
        }

        public Command rumbleCommand(CommandXboxController controller) {
                return Commands.run(() -> {
                        if (m_swerve.getOnTarget())
                                controller.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
                        else
                                controller.getHID().setRumble(RumbleType.kLeftRumble, 0.0);

                        if (m_transfer.noteAtIntake())
                                controller.getHID().setRumble(RumbleType.kRightRumble, 1.0);
                        else
                                controller.getHID().setRumble(RumbleType.kRightRumble, 0.0);

                }).finallyDo(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0));
        }

        public Command setStartPosebyAlliance(Pose2d startPose) {
                tempPose2d = startPose;
                if (AllianceUtil.isRedAlliance())
                        tempPose2d = GeometryUtil.flipFieldPose(startPose);
                return Commands.runOnce(() -> m_swerve.resetPoseEstimator(tempPose2d));
        }

        public Command finalCommand(int choice) {

                switch ((choice)) {

                        case 11:
                                return new AutoSourceShootThenCenter(this,
                                                m_pf.pathMaps.get(sourcepaths.SourceToCenter4.name()),
                                                m_swerve);
                        case 12:
                                return new AutoSourceShootMovingThenCenter(this,
                                                m_pf.pathMaps.get(sourcepaths.SourceToCenter4ShootMoving.name()),
                                                m_swerve);
                        case 13:
                                return new AutoSourceShootCenter4Pathfind(this,
                                                m_pf, m_pf.pathMaps.get(sourcepaths.SourceToCenter4.name()),
                                                m_swerve);
                        case 14:
                                return new AutoSourceThenCenter4Vision(this,
                                                m_pf.pathMaps.get(sourcepaths.SourceToNearCenter4.name()),
                                                m_swerve, m_llv, m_intake, m_transfer);

                        case 21:
                                return new AutoAmpShootThenCenter(this, m_pf.pathMaps.get(amppaths.AmpToCenter2.name()),
                                                m_swerve);
                        case 22:
                                return new AutoAmpShootThenCenter(this, m_pf.pathMaps.get(amppaths.AmpToCenter2.name()),
                                                m_swerve);

                        default:
                                return Commands.none();

                }

        }

        public Command getAutonomousCommand() {
                return finalCommand(m_af.finalChoice);
        }

        public Command resetAll() {
                return new ParallelCommandGroup(
                                m_shooter.stopShooterCommand(),
                                m_intake.stopIntakeCommand(),
                                m_transfer.stopTransferCommand(),
                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians)
                                                .withName("Reset All"))
                                .asProxy();
        }

        public Command doAmpShot() {
                return new SequentialCommandGroup(
                                Commands.runOnce(() -> m_arm.setUseMotorEncoder(true)),
                                m_arm.setGoalCommand(Units.degreesToRadians(90)),
                                // new CheckArmAtTarget(m_arm),
                                m_arm.setGoalCommand(Units.degreesToRadians(Pref.getPref("AmpArmDegrees"))),
                                m_shooter.startShooterCommand(
                                                Pref.getPref("AmpTopRPM"), Pref.getPref("AmpBottomRPM")),
                                // new CheckShooterAtSpeed(m_shooter, .05),
                                // new CheckArmAtTarget(m_arm),
                                Commands.parallel(
                                                m_transfer.transferToShooterCommand(),
                                                new WaitCommand(Pref.getPref("AmpArmIncrementDelay")),
                                                m_arm.setGoalCommand(
                                                                Units.degreesToRadians(Pref.getPref("AmpArmDegrees")) +
                                                                                Units.degreesToRadians(Pref.getPref(
                                                                                                "AmpDegreeIncrement"))),
                                                new WaitCommand(1)),

                                Commands.parallel(
                                                m_shooter.stopShooterCommand(),
                                                m_arm.setGoalCommand(ArmConstants.armMinRadians)));
        }
}
