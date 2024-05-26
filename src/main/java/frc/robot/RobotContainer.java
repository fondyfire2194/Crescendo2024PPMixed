// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Factories.AutoFactory;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.TriggerCommandFactory;
import frc.robot.commands.JogClimber;
import frc.robot.commands.ViewArmShooterByDistance;
import frc.robot.commands.Drive.AlignTargetOdometry;
import frc.robot.commands.Drive.AlignToNote;
import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Shooter.ShootWhileMoving;
import frc.robot.commands.Shooter.ShootWhileMovingQuadratic;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.ShootingData;
import monologue.Annotations.Log;
import monologue.Logged;

public class RobotContainer implements Logged {
        /* Subsystems */
        final SwerveSubsystem m_swerve = new SwerveSubsystem();

        final IntakeSubsystem m_intake = new IntakeSubsystem();

        final TransferSubsystem m_transfer = new TransferSubsystem();

        final ArmSubsystem m_arm = new ArmSubsystem();

        final ClimberSubsystem m_climber = new ClimberSubsystem();

        final PowerDistribution m_pd = new PowerDistribution(1, ModuleType.kRev);

        public final LimelightVision m_llv = new LimelightVision();

        public final SendableChooser<Double> m_startDelayChooser = new SendableChooser<Double>();

        public final SendableChooser<String> m_batteryChooser = new SendableChooser<String>();

        private final CommandXboxController driver = new CommandXboxController(0);

        private final CommandXboxController codriver = new CommandXboxController(1);

        private final CommandXboxController setup = new CommandXboxController(2);

        final ShooterSubsystem m_shooter = new ShooterSubsystem();

        ShootingData m_sd = new ShootingData();

        public final PathFactory m_pf = new PathFactory(m_swerve);

        public final AutoFactory m_af = new AutoFactory(m_pf);

        public final CommandFactory m_cf = new CommandFactory(m_swerve, m_shooter, m_arm, m_intake, m_transfer,
                        m_climber, m_llv, m_af, m_pf, m_sd);

        public final TriggerCommandFactory m_tcf = new TriggerCommandFactory(m_swerve, m_transfer, m_intake,
                        m_llv, m_pf, m_cf);

        BooleanSupplier keepAngle;

        public BooleanSupplier fieldRelative;

        private Trigger doLobShot;
        private Trigger doMovingShot;

        private Trigger logShotTrigger;

        EventLoop checkAutoSelectLoop;

        private BooleanEvent doAutoSetup;

        private Trigger canivoreCheck;

        public CANBusStatus canInfo;
        @Log.NT(key = "canivoreutil")
        public float busUtil;

        public RobotContainer() {

                registerNamedCommands();

                if (RobotBase.isReal()) {
                        // Pref.deleteUnused();
                        Pref.addMissing();
                }

                m_arm.setKPKIKD();

                m_pd.resetTotalEnergy();

                m_transfer.setVelPID();

                SmartDashboard.putData("TestCan", this.testAllCan().ignoringDisable(true));

                SmartDashboard.putData("ClearStickyFaults", this.clearAllStickyFaultsCommand().ignoringDisable(true));

                SmartDashboard.putData("ViewRPMAngles",
                                new ViewArmShooterByDistance(m_cf, m_sd, m_arm).ignoringDisable(true));

                configureDriverBindings();

                configureCodriverBindings();

                configureChoosers();

                m_intake.setPID();

                configureSetupBindings();

                m_shooter.setTopKpKdKi();

                configureCommandScheduler();

                setDefaultCommands();

                m_shooter.setBottomKpKdKi();

                doLobShot = new Trigger(() -> m_transfer.lobbing
                                && m_transfer.noteAtIntake()
                                && m_shooter.bothAtSpeed(1)
                                && m_arm.getAtSetpoint()
                                && m_swerve.alignedToTarget
                                && Math.abs(m_swerve.getChassisSpeeds().vxMetersPerSecond) < 1
                                && m_swerve.getDistanceFromLobTarget() > SwerveConstants.minLobDistance
                                && m_swerve.getDistanceFromLobTarget() < SwerveConstants.maxLobDistance);

                doLobShot.onTrue(m_cf.transferNoteToShooterCommand());

                doMovingShot = new Trigger(() -> m_transfer.shootmoving
                                && m_transfer.OKShootMoving
                                && m_transfer.noteAtIntake()
                                && m_shooter.bothAtSpeed(1)
                                && m_arm.getAtSetpoint()
                                && m_swerve.alignedToTarget
                                && Math.abs(m_swerve.getChassisSpeeds().vxMetersPerSecond) < 1
                                && m_swerve.getDistanceFromSpeaker() < SwerveConstants.maxMovingShotDistance);

                doMovingShot.onTrue(m_cf.transferNoteToShooterCommand());

                logShotTrigger = new Trigger(() -> m_transfer.logShot == true);

                logShotTrigger.onTrue(
                                Commands.sequence(

                                                Commands.runOnce(() -> m_swerve.poseWhenShooting = m_swerve.getPose()),
                                                Commands.runOnce(() -> m_arm.angleDegWhenShooting = m_arm
                                                                .getAngleDegrees()),
                                                Commands.runOnce(() -> m_shooter.rpmWhenShooting = m_shooter
                                                                .getRPMTop()),
                                                Commands.runOnce(() -> m_transfer.logShot = false)));

                m_tcf.createCommonTriggers();

                checkAutoSelectLoop = new EventLoop();

                doAutoSetup = new BooleanEvent(checkAutoSelectLoop, m_af::checkChoiceChange);

                doAutoSetup.ifHigh(() -> setAutoData());

                if (RobotBase.isReal()) {

                        canInfo = CANBus.getStatus("CV1");
                        busUtil = canInfo.BusUtilization;

                        canivoreCheck = new Trigger(
                                        () -> !canInfo.Status.isOK() || canInfo.Status.isError()
                                                        || canInfo.Status.isWarning());

                        canivoreCheck.onTrue(Commands.runOnce(() -> logCanivore()));
                }

        }

        public void logCanivore() {
                log("errcanivore", canInfo.Status.isError());
                log("warncanivore", canInfo.Status.isWarning());
                log("okcanivore", canInfo.Status.isOK());
                log("canivoredesc", canInfo.Status.getDescription());
        }

        private void configureDriverBindings() {

                // KEEP IN BUTTON ORDER

                fieldRelative = driver.y().negate().and(driver.rightBumper().negate());

                keepAngle = () -> false;
                // align for speaker shots
                driver.leftTrigger().and(driver.a().negate()).whileTrue(
                                Commands.parallel(
                                                new AlignTargetOdometry(
                                                                m_swerve,
                                                                () -> -driver.getLeftY(),
                                                                () -> driver.getLeftX(),
                                                                () -> driver.getRightX(), false),
                                                m_cf.positionArmRunShooterByDistance(false, false)));

                driver.leftTrigger().and(driver.a()).whileTrue(
                                Commands.parallel(
                                                new AlignTargetOdometry(
                                                                m_swerve,
                                                                () -> -driver.getLeftY(),
                                                                () -> driver.getLeftX(),
                                                                () -> driver.getRightX(), false),
                                                                new ShootWhileMovingQuadratic(m_arm, m_transfer, m_shooter, m_swerve, m_sd)));
                                              //  new ShootWhileMoving(m_arm, m_transfer, m_shooter, m_swerve, m_sd)));

                driver.rightBumper().and(driver.a().negate()).onTrue(Commands.parallel(
                                m_intake.startIntakeCommand(),
                                new TransferIntakeToSensor(m_transfer, m_intake, 120),
                                m_cf.rumbleCommand(driver),
                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians))
                                .withTimeout(10));

                // pick up notes with vision align
                driver.rightBumper().and(driver.a()).onTrue(
                                Commands.sequence(
                                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians),
                                                Commands.none().until(() -> m_arm.getAtSetpoint()),
                                                m_intake.startIntakeCommand(),
                                                Commands.deadline(
                                                                new TransferIntakeToSensor(m_transfer,
                                                                                m_intake, 120),
                                                                new AlignToNote(
                                                                                m_swerve,
                                                                                m_llv,
                                                                                CameraConstants.rearCamera.camname,
                                                                                () -> -driver.getLeftY(),
                                                                                () -> driver.getLeftX(),
                                                                                () -> driver.getRightX())),

                                                m_cf.rumbleCommand(driver)));

                // align with amp corner for lob shots
                driver.leftBumper().whileTrue(
                                Commands.parallel(
                                                new AlignTargetOdometry(
                                                                m_swerve,
                                                                () -> -driver.getLeftY(),
                                                                () -> driver.getLeftX(),
                                                                () -> driver.getRightX(), true),
                                                m_cf.positionArmRunShooterByDistance(true, false)))

                                .onFalse(
                                                Commands.parallel(
                                                                m_shooter.stopShooterCommand(),
                                                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians)));

                // shoot
                driver.rightTrigger().onTrue(
                                Commands.sequence(
                                                Commands.waitUntil(() -> m_arm.getAtSetpoint()),
                                                m_cf.transferNoteToShooterCommand(),
                                                m_shooter.stopShooterCommand(),
                                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians),
                                                m_intake.stopIntakeCommand()));

                driver.b().onTrue(m_shooter.stopShooterCommand());

                driver.x().onTrue(m_shooter.startShooterCommand(3500));

                driver.a().and(driver.leftTrigger().negate()).and(driver.rightBumper().negate())
                                .onTrue(m_cf.doAmpShot());

                driver.povUp().onTrue(m_shooter.increaseRPMCommand(100));

                driver.povDown().onTrue(m_shooter.decreaseRPMCommand(100));

                driver.povRight().onTrue(Commands.runOnce(() -> m_arm.incrementArmAngle(1)));

                driver.povLeft().onTrue(Commands.runOnce(() -> m_arm.decrementArmAngle(1)));

                driver.start().onTrue(Commands.runOnce(() -> m_swerve.zeroGyro()));

                driver.back().onTrue(
                                Commands.sequence(
                                                m_cf.positionArmRunShooterSpecialCase(45,
                                                                4750),
                                                Commands.waitSeconds(2),
                                                m_cf.transferNoteToShooterCommand(),
                                                new WaitCommand(1))
                                                .andThen(
                                                                Commands.parallel(
                                                                                m_arm.setGoalCommand(
                                                                                                ArmConstants.pickupAngleRadians),
                                                                                m_shooter.stopShooterCommand())));

        }

        private void configureCodriverBindings() {
                // CoDriver
                // KEEP IN BUTTON ORDER
                // jogs are in case note gets stuck

                codriver.leftTrigger().whileTrue(m_climber.raiseClimberArmsCommand(0.6))
                                .onFalse(m_climber.stopClimberCommand());

                // codriver.leftBumper().onTrue(m_arm.positionToIntakeUDACommand());
                codriver.leftBumper().whileTrue(new JogClimber(m_climber, codriver));

                codriver.rightTrigger().whileTrue(m_climber.lowerClimberArmsCommand(0.6))
                                .onFalse(m_climber.stopClimberCommand());

                // codriver.rightBumper().onTrue(

                codriver.a().onTrue(m_cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                Constants.subwfrShooterSpeed));

                // codriver.b().onTrue(m_cf.positionArmRunShooterSpecialCase(Constants.safeStageArmAngle,
                // Constants.safeStageShooterSpeed));

                codriver.x().onTrue(m_cf.positionArmRunShooterSpecialCase(Constants.tapeLineArmAngle,
                                Constants.tapeLineShooterSpeed));

                // codriver.y().whileTrue(

                codriver.povUp().onTrue(m_climber.raiseClimberArmsCommand(.3));

                codriver.povDown().onTrue(m_climber.lowerClimberArmsCommand(.3));

                codriver.povLeft().whileTrue(Commands.runOnce(() -> m_transfer.transferMotor.setVoltage(-.5)))
                                .onFalse(Commands.runOnce(() -> m_transfer.transferMotor.setVoltage(0)));

                // codriver.povRight().onTrue(

                codriver.start().onTrue(Commands.runOnce(() -> m_swerve.resetModuleEncoders()));

                codriver.back().whileTrue(Commands.runOnce(() -> m_intake.intakeMotor.setVoltage(-8)))
                                .onFalse(Commands.runOnce(() -> m_intake.intakeMotor.setVoltage(0)));

        }

        private void configureSetupBindings() {
                // Setup
                // KEEP IN BUTTON ORDER
                // jogs are in case note gets stuck

                // setup.leftTrigger().whileTrue(new JogIntake(m_intake, setup));

                // setup.leftBumper().whileTrue(new JogArm(m_arm, setup));

                // setup.rightTrigger().whileTrue(new JogTransfer(m_transfer, setup));

                setup.rightBumper().whileTrue(new JogClimber(m_climber, setup));

                setup.a().onTrue(m_climber.lockClimberCommand());

                setup.b().onTrue(m_climber.unlockClimberCommand());

                // setup.leftBumper().whileTrue(m_swerve.quasistaticForward());

                // setup.leftTrigger().whileTrue(m_swerve.quasistaticBackward());

                // setup.rightBumper().whileTrue(m_swerve.dynamicForward());

                // setup.rightTrigger().whileTrue(m_swerve.dynamicBackward());

                // setup.a().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(25)));

                // setup.b().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(40)));

                setup.x().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(50)));

                setup.y().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(70)));

                // setup.povUp().onTrue(

                setup.povDown().onTrue(new RotateToAngle(m_swerve, 0));

                setup.povLeft().onTrue(new RotateToAngle(m_swerve, 90));

                setup.povRight().onTrue(new RotateToAngle(m_swerve, -90));

                // setup.start()

                // setup.back()
        }

        private void setDefaultCommands() {

                m_swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                m_swerve,
                                                () -> -driver.getLeftY(),
                                                () -> -driver.getLeftX(),
                                                () -> -driver.getRawAxis(4),
                                                fieldRelative,
                                                keepAngle));
        }

        private void configureCommandScheduler() {
                SmartDashboard.putData("CommSchd", CommandScheduler.getInstance());
        }

        private void registerNamedCommands() {

                NamedCommands.registerCommand("ResetAll", m_cf.resetAll());

                NamedCommands.registerCommand("Arm Shooter Pre Wing 2", m_cf.positionArmRunShooterSpecialCase(44,
                                3000).asProxy()
                                .withName("Arm Shooter Pre Wing 2"));

                NamedCommands.registerCommand("Arm Shooter Wing 2", m_cf.positionArmRunShooterSpecialCase(35,
                                3200).asProxy()
                                .withName("Arm Shooter Wing 2"));

                NamedCommands.registerCommand("Prestart Shooter Wheels",
                                m_shooter.startShooterCommand(4000).asProxy()
                                                .withName("Prestart Shooter Wheels"));

                NamedCommands.registerCommand("Stop Intake", m_intake.stopIntakeCommand().asProxy()
                                .withName("Stop Intake"));

                NamedCommands.registerCommand(
                                "Start Intake", m_intake.startIntakeCommand().asProxy()
                                                .withName("Start Intake"));

                NamedCommands.registerCommand(
                                "DoIntake", m_cf.doIntake().asProxy()
                                                .withName("Do Intake"));

                NamedCommands.registerCommand(
                                "Transfer Stop", m_transfer.stopTransferCommand().asProxy()
                                                .withName("Transfer Stop"));

                NamedCommands.registerCommand("Arm To Intake",
                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians).asProxy()
                                                .withName("ArmToIntake"));

                // NamedCommands.registerCommand("Shooter Low Speed",
                // m_shooter.setRPMCommand(1000, false));

                NamedCommands.registerCommand("Shoot", m_cf.transferNoteToShooterCommand()
                                .withName("Shoot"));

                NamedCommands.registerCommand("Align Then Shoot", m_cf.alignShootCommand().asProxy());

                NamedCommands.registerCommand("Arm Shooter SubWfr",
                                m_cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed).asProxy()
                                                .withName("Arm Shooter SubWfr")); // Constants.subwfrShooterSpeed
                                                                                  // lower speed to decrease
                                                                                  // time
                NamedCommands.registerCommand("Arm Shooter While Moving",
                                m_cf.positionArmRunShooterSpecialCase(Constants.autoShootArmAngle,
                                                Constants.autoShootRPM).asProxy()
                                                .withName("Arm Shooter While Moving")); // Constants.subwfrShooterSpeed
                                                                                        // lower speed to decrease
                                                                                        // time

                NamedCommands.registerCommand("Arm Shooter Wing 1",
                                m_cf.positionArmRunShooterSpecialCase(Constants.wing1ArmAngle,
                                                Constants.wing1ShooterSpeed).asProxy()
                                                .withName("Arm Shooter Wing 1"));

                NamedCommands.registerCommand("Arm Shooter Wing 2",
                                m_cf.positionArmRunShooterSpecialCase(Constants.wing2ArmAngle,
                                                Constants.wing2ShooterSpeed).asProxy()
                                                .withName("Arm Shooter Wing 2"));

                NamedCommands.registerCommand("Arm Shooter Wing 3",
                                m_cf.positionArmRunShooterSpecialCase(Constants.wing3ArmAngle,
                                                Constants.wing3ShooterSpeed).asProxy()
                                                .withName("Arm Shooter Wing 3"));

                NamedCommands.registerCommand("Arm Shooter Amp Shoot",
                                m_cf.positionArmRunShooterSpecialCase(Constants.ampStartArmAngle,
                                                Constants.ampStartShooterSpeed).asProxy()
                                                .withName("Arm Shooter Amp Shoot"));

                NamedCommands.registerCommand("Arm Shooter Source",
                                m_cf.positionArmRunShooterSpecialCase(Constants.sourceShootAngle,
                                                Constants.sourceShootSpeed).asProxy()
                                                .withName("Arm Shooter Source"));

                NamedCommands.registerCommand("Stop Shooter", m_shooter.stopShooterCommand().asProxy()
                                .withName("Stop Shooter"));

                NamedCommands.registerCommand("Pathfind to Pickup C4",
                                m_cf.autopickup(FieldConstants.centerNote4PickupBlue)
                                                .withName("Pathfind to Pickup C4"));

                NamedCommands.registerCommand("Pathfind to Pickup C5",
                                m_cf.autopickup(FieldConstants.centerNote5PickupBlue)
                                                .withName("Pathfind to Pickup C5"));

                NamedCommands.registerCommand("CheckForNote",
                                Commands.runOnce(() -> m_swerve.checkNote = true));

        }

        private void configureChoosers() {

                m_startDelayChooser.setDefaultOption("0 sec", 0.);
                m_startDelayChooser.addOption("1 sec", 1.);
                m_startDelayChooser.addOption("2 sec", 2.);
                m_startDelayChooser.addOption("3 sec", 3.);
                m_startDelayChooser.addOption("4 sec", 4.);
                m_startDelayChooser.addOption("5 sec", 5.);

                m_batteryChooser.setDefaultOption("A", "A");
                m_batteryChooser.addOption("B", "B");
                m_batteryChooser.addOption("C", "C");
                m_batteryChooser.addOption("D", "D");
                m_batteryChooser.addOption("E", "E");
                m_batteryChooser.addOption("F", "F");

                SmartDashboard.putData("DelayChooser", m_startDelayChooser);

                SmartDashboard.putData("BatteryChooser", m_batteryChooser);

        }

        void setAutoData() {
                m_af.validStartChoice = m_af.selectAndLoadPathFiles();

                if (m_af.validStartChoice >= m_af.minsourceauto && m_af.validStartChoice <= m_af.maxsourceauto) {
                        m_tcf.createSourceTriggers();
                        m_cf.setStartPosebyAlliance(FieldConstants.sourceStartPose).runsWhenDisabled();
                }
                if (m_af.validStartChoice >= m_af.minampauto && m_af.validStartChoice <= m_af.maxampauto) {
                        m_tcf.createAmpTriggers();
                        m_cf.setStartPosebyAlliance(FieldConstants.ampStartPose).runsWhenDisabled();
                }
        }

        public Command testAllCan() {
                return Commands.sequence(
                                m_arm.testCan(),
                                m_climber.testCan(),
                                m_intake.testCan(),
                                m_transfer.testCan(),
                                m_shooter.testCan(),
                                m_swerve.testAllCan());
        }

        @Log.NT(key = "canOK")
        public boolean isCanOK() {
                return m_arm.armMotorConnected
                                && m_intake.intakeMotorConnected
                                && m_transfer.transferMotorConnected
                                && m_shooter.topMotorConnected && m_shooter.bottomMotorConnected
                                && m_climber.leftMotorConnected && m_climber.rightMotorConnected
                                && m_swerve.mod0connected && m_swerve.mod1connected
                                && m_swerve.mod2connected && m_swerve.mod3connected;

        }

        public Command clearAllStickyFaultsCommand() {
                return Commands.sequence(
                                m_arm.clearStickyFaultsCommand(),
                                m_climber.clearStickyFaultsCommand(),
                                m_intake.clearStickyFaultsCommand(),
                                m_transfer.clearStickyFaultsCommand(),
                                m_shooter.clearStickyFaultsCommand(),
                                m_swerve.clearStickFaultsCommand());
        }

        @Log.NT(key = "stickyfault")
        public boolean getStickyFaults() {
                return m_arm.getStickyFaults() != 0
                                && m_intake.getStickyFaults() != 0
                                && m_transfer.getStickyFaults() != 0
                                && m_shooter.getTopStickyFaults() != 0 && m_shooter.getBottomStickyFaults() != 0
                                && m_climber.getLeftStickyFaults() != 0 && m_climber.getRightStickyFaults() != 0
                                && m_swerve.getModuleStickyFaults() != 0;

        }

}