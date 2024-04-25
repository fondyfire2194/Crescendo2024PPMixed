// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.TransferConstants;
import monologue.Annotations.Log;
import monologue.Logged;
import frc.robot.Pref;

public class TransferSubsystem extends SubsystemBase implements Logged {

  public CANSparkMax transferMotor;
  public SparkPIDController transferController;
  RelativeEncoder transferEncoder;

  public boolean m_showScreens;

  private int loopctr;

  @Log.NT(key = "transfercommandrpm")
  private double commandrpm;

  public SparkLimitSwitch m_limitSwitch;

  /** Creates a new transfer. */
  public TransferSubsystem(boolean showScreens) {
    m_showScreens = showScreens;
    transferMotor = new CANSparkMax(CANIDConstants.transferID, MotorType.kBrushless);
    transferEncoder = transferMotor.getEncoder();
    transferController = transferMotor.getPIDController();

    configMotor(transferMotor, transferEncoder, true);

    m_limitSwitch = transferMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_limitSwitch.enableLimitSwitch(true);

    if (m_showScreens) {

      Shuffleboard.getTab("IntakeSubsystem").add(this)
          .withSize(2, 1)
          .withPosition(3, 0);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("TransferRPM",
          () -> round2dp(getRPM(), 0))
          .withSize(1, 1)
          .withPosition(3, 1);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("TransferAmps",
          () -> round2dp(getAmps(), 1))
          .withSize(1, 1)
          .withPosition(4, 1);

      // Shuffleboard.getTab("IntakeSubsystem").addNumber("NoteSensorInches",
      // () -> round2dp(getSensorDistanceInches(), 1))
      // .withSize(1, 1)
      // .withPosition(3, 2);

      Shuffleboard.getTab("IntakeSubsystem").addBoolean("NoteSensed", () -> noteAtIntake())
          .withSize(1, 1)
          .withPosition(4, 2)
          .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

      Shuffleboard.getTab("IntakeSubsystem")
          .addBoolean("StickyFaultTfr", () -> getStickyFaults() != 0)
          .withPosition(3, 3).withSize(1, 1)
          .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "black"));

      Shuffleboard.getTab("IntakeSubsystem")
          .add("ClearFaultTfr", clearFaultsCommand())
          .withPosition(4, 3).withSize(1, 1);

    }

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder,
      boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(TransferConstants.transferContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(TransferConstants.transferIdleMode);
    // encoder.setVelocityConversionFactor(TransferConstants.transferConversionVelocityFactor);
    // encoder.setPositionConversionFactor(TransferConstants.transferConversionPositionFactor);
    motor.enableVoltageCompensation(TransferConstants.voltageComp);
    // motor.setOpenLoopRampRate(1);
    // motor.setClosedLoopRampRate(1);
    motor.burnFlash();
    encoder.setPosition(0.0);

  }

  public void stopMotor() {
    runAtVelocity(0);
    transferMotor.stopMotor();
    commandrpm = 0;

  }

  public Command stopTransferCommand() {
    commandrpm = 0;
    return Commands.runOnce(() -> stopMotor(), this);
  }
  @Log.NT(key = "transfertoshootercommand")
  public Command transferToShooterCommand() {
    return Commands
        .run(() -> transferToShooter(), this)
        .withTimeout(TransferConstants.clearShooterTime)
        .andThen(stopTransferCommand());
  }

  public void transferToShooter() {
    enableLimitSwitch(false);
    commandrpm = Pref.getPref("TransferToShootSpeed");
    runAtVelocity(commandrpm);
  }

  public void runToSensor() {
    enableLimitSwitch(true);
    commandrpm = Pref.getPref("TransferIntakingSpeed");
    runAtVelocity(commandrpm);

  }

  public void runAtVelocity(double rpm) {
    if (RobotBase.isReal())
      transferController.setReference(rpm, ControlType.kVelocity);
  }

  @Log.NT(key = "transfernoteatintake")
  public boolean noteAtIntake() { // we can get rid of the TimeOfFlight
    return m_limitSwitch.isPressed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableLimitSwitch(boolean enable) {
    m_limitSwitch.enableLimitSwitch(enable);
  }

  public boolean getLimitSwitchEnabled() {
    return m_limitSwitch.isLimitSwitchEnabled();
  }

  @Log.NT(key = "transferamps")
  public double getAmps() {
    return transferMotor.getOutputCurrent();
  }

  @Log.NT(key = "transferrpm")
  public double getRPM() {
    if (RobotBase.isReal())
      return transferEncoder.getVelocity();
    else
      return commandrpm;
  }

  public void setVelPID() {
    REVLibError ok = transferController.setFF(TransferConstants.transferKFF, 0);
    REVLibError okp = transferController.setP(TransferConstants.transferKp, 0);
  }

  public Command setTransferPIDCommand() {
    return Commands.runOnce(() -> setVelPID());
  }

  public boolean onPlusHardwareLimit() {
    return transferMotor.getFault(FaultID.kHardLimitRev);
  }

  public boolean onMinusHardwareLimit() {
    return transferMotor.getFault(FaultID.kHardLimitRev);
  }

  public int getFaults() {
    return transferMotor.getFaults();
  }

  public int getStickyFaults() {
    return transferMotor.getStickyFaults();
  }

  public void clearFaults() {
    transferMotor.clearFaults();
  }

  public Command clearFaultsCommand() {
    return Commands.runOnce(() -> transferMotor.clearFaults());
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

  public double getPosition() {
    return transferEncoder.getPosition();
  }

  public double getVelocity() {
    return transferEncoder.getVelocity();
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < Pref.getPref("TransferIntakingSpeed") / 20;
  }

}
