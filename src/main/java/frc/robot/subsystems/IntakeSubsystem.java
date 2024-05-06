// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import monologue.Annotations.Log;
import monologue.Logged;
import frc.robot.Pref;

public class IntakeSubsystem extends SubsystemBase implements Logged {

  public CANSparkMax intakeMotor;
  RelativeEncoder intakeEncoder;
  public SparkPIDController intakeController;

  private int loopctr;

  @Log.NT(key = "intakerun")
  private boolean runIntake;
  public boolean jogging;
  private SlewRateLimiter intakeLimiter = new SlewRateLimiter(1500);
  @Log.NT(key = "intakecommandrpm")
  private double commandrpm;
  public boolean noteMissed;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.CANIDConstants.intakeID, MotorType.kBrushless);
    intakeController = intakeMotor.getPIDController();
    intakeEncoder = intakeMotor.getEncoder();
    configMotor(intakeMotor, intakeEncoder, false);

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.IntakeConstants.intakeContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.IntakeConstants.intakeIdleMode);
    encoder.setVelocityConversionFactor(Constants.IntakeConstants.intakeConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.IntakeConstants.intakeConversionPositionFactor);
    motor.enableVoltageCompensation(Constants.IntakeConstants.voltageComp);
    // intakeMotor.setClosedLoopRampRate(1);
    // intakeMotor.setOpenLoopRampRate(1);
    motor.burnFlash();
    encoder.setPosition(0.0);

  }

  public void stopMotor() {
    intakeMotor.stopMotor();
    intakeController.setReference(0, ControlType.kVelocity);
    resetRunIntake();
    commandrpm = 0;
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(() -> stopMotor());
  }

  public Command startIntakeCommand() {
    return Commands.runOnce(() -> setRunIntake());
  }

  public void setRunIntake() {
    runIntake = true;
  }

  public void resetRunIntake() {
    runIntake = false;
  }

  public boolean getRunIntake() {
    return runIntake;
  }

  @Log.NT(key = "intakerpm")
  public double getRPM() {
    return intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    loopctr++;

    if (runIntake) {
      // double rpm = intakeLimiter.calculate(Pref.getPref("IntakeSpeed"));
      commandrpm = Pref.getPref("IntakeSpeed");
      runAtVelocity(commandrpm);
    }
    if (!runIntake && !jogging) {
      stopMotor();
      // intakeLimiter.reset(0);
    }
  }

  private void runAtVelocity(double rpm) {
    intakeController.setReference(rpm, ControlType.kVelocity);
  }

  public void reverseMotor() {
    runAtVelocity(IntakeConstants.reverseRPM);
  }

  @Log.NT(key = "intakeAmps")
  public double getAmps() {
    return intakeMotor.getOutputCurrent();
  }

  public void setPID() {
    intakeController.setP(Pref.getPref("IntakeKp"));
    intakeController.setFF(IntakeConstants.intakeKFF);
  }

  public Command clearFaultsCommand() {
    return Commands.runOnce(() -> intakeMotor.clearFaults());
  }

  public int getFaults() {
    return intakeMotor.getFaults();
  }

  public int getStickyFaults() {
    return intakeMotor.getStickyFaults();
  }

  public String getFirmwareVersion() {
    return intakeMotor.getFirmwareString();
  }
}
