// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Pref;
import monologue.Annotations.Log;
import monologue.Logged;

public class ClimberSubsystem extends SubsystemBase implements Logged {
  /** Creates a new Climber. */
  CANSparkMax climberMotorLeft;
  CANSparkMax climberMotorRight;
  Servo climberLock;

  RelativeEncoder climberEncoderLeft;
  RelativeEncoder climberEncoderRight;
  public boolean showClimber = true;
  private int loopctr;

  public boolean leftMotorConnected;
  public boolean rightMotorConnected;

  public ClimberSubsystem() {
    climberMotorLeft = new CANSparkMax(CANIDConstants.climberIDLeft, MotorType.kBrushless);
    climberMotorRight = new CANSparkMax(CANIDConstants.climberIDRight, MotorType.kBrushless);
    climberEncoderLeft = climberMotorLeft.getEncoder();
    climberEncoderRight = climberMotorRight.getEncoder();

    climberLock = new Servo(0);
    unlockClimber();
    configMotor(climberMotorRight, climberEncoderRight, false);
    configMotor(climberMotorLeft, climberEncoderLeft, true);

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kMinimal);
    motor.setSmartCurrentLimit(Constants.ClimberConstants.climberContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.ClimberConstants.climberIdleMode);
    encoder.setVelocityConversionFactor(Constants.ClimberConstants.climberConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.ClimberConstants.climberConversionPositionFactor);
    motor.enableVoltageCompensation(Constants.ClimberConstants.voltageComp);
    // motor.setOpenLoopRampRate(3);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!leftMotorConnected) {
      leftMotorConnected = checkMotorCanOK(climberMotorLeft);
      SmartDashboard.putBoolean("Climber//OKLClimber", leftMotorConnected);
    }

    if (!rightMotorConnected) {
      rightMotorConnected = checkMotorCanOK(climberMotorRight);
      SmartDashboard.putBoolean("Climber//OKRClimber", rightMotorConnected);
    }

    SmartDashboard.putNumber("Climber// Left RPM", getRPMLeft());
    SmartDashboard.putNumber("Climber// Right RPM", getRPMRight());

    SmartDashboard.putNumber("Climber// Left Amps", climberMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Climber// Left Position", climberEncoderLeft.getPosition());
    SmartDashboard.putNumber("Climber// Right Amps", climberMotorRight.getOutputCurrent());
    SmartDashboard.putNumber("Climber// Right Position", climberEncoderRight.getPosition());

  }

  private boolean checkMotorCanOK(CANSparkMax motor) {
    double temp = motor.getOpenLoopRampRate();
    return RobotBase.isSimulation() || motor.setOpenLoopRampRate(temp) == REVLibError.kOk;
  }

  public Command testCan() {
    return Commands.parallel(
        Commands.runOnce(() -> leftMotorConnected = false),
        runOnce(() -> rightMotorConnected = false));
  }

  public void stopMotors() {
    climberMotorLeft.stopMotor();
    climberMotorLeft.setVoltage(0);
    climberMotorRight.stopMotor();
    climberMotorRight.setVoltage(0);
  }

  public Command stopClimberCommand() {
    return Commands.runOnce(() -> stopMotors(), this);
  }

  public void runClimberMotor(double speed) {
    if (getPositionLeft() > 130) {
      speed = speed * 0.5;
    }
    climberMotorLeft.setVoltage(speed * RobotController.getBatteryVoltage());
    climberMotorRight.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public void lowerClimber(double speed) {
    // if (climberEncoder.getPosition() > 60) {
    // runClimberMotor(speed);
    // } else {
    // runClimberMotor(speed * .5);
    // }
    if (getPositionLeft() < 10) {
      runClimberMotor(speed * 0.2);
    } else {
      runClimberMotor(speed);
    }
  }

  public Command lowerClimberArmsCommand(double speed) {
    return Commands.run(() -> lowerClimber(-speed));
  }

  public Command raiseClimberArmsCommand(double speed) {
    return Commands.run(() -> runClimberMotor(speed));
  }

  @Log.NT(key = "ClimberLeftRPM")
  public double getRPMLeft() {
    return climberEncoderLeft.getVelocity();
  }

  @Log.NT(key = "ClimberRightRPM")
  public double getRPMRight() {
    return climberEncoderRight.getVelocity();
  }

  @Log.NT(key = "ClimberPositionLeft")
  public double getPositionLeft() {
    return climberEncoderLeft.getPosition();
  }

  @Log.NT(key = "ClimberPositionRight")
  public double getPositionRight() {
    return climberEncoderRight.getPosition();
  }

  @Log.NT(key = "climberleftstickyfault")
  public int getLeftStickyFaults() {
    return climberMotorLeft.getStickyFaults();
  }

  @Log.NT(key = "climberrightstickyfault")
  public int getRightStickyFaults() {
    return climberMotorRight.getStickyFaults();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> climberMotorLeft.clearFaults()),
        runOnce(() -> climberMotorRight.clearFaults()));
  }

  @Log.NT(key = "ClimberLeftAmps")
  public double getLeftAmps() {
    return climberMotorLeft.getOutputCurrent();
  }

  @Log.NT(key = "ClimberRightAmps")
  public double getRightAmps() {
    return climberMotorRight.getOutputCurrent();
  }

  public void lockClimber() {
    climberLock.set(Pref.getPref("LockNumber"));
  }

  public Command lockClimberCommand() {
    return Commands.runOnce(() -> lockClimber());
  }

  public void unlockClimber() {
    climberLock.set(Pref.getPref("UnlockNumber"));
  }

  public Command unlockClimberCommand() {
    return Commands.runOnce(() -> unlockClimber());
  }

}
