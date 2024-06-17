// Copytop (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Pref;
import monologue.Annotations.Log;
import monologue.Logged;

public class ShooterSubsystem extends SubsystemBase implements Logged {

  public CANSparkMax topRoller;
  public SparkPIDController topController;
  public SparkPIDController bottomController;
  RelativeEncoder topEncoder;

  public CANSparkMax bottomRoller;
  RelativeEncoder bottomEncoder;
  @Log.NT(key = "shtrtopcommandrpm")
  public double topCommandRPM = 500;
  @Log.NT(key = "shtrbottomcommandrpm")
  public double bottomCommandRPM = 500;

  private double topSimRPM = 0;
  private double bottomSimRPM = 0;
  @Log.NT(key = "shtratspeed;")
  private boolean shootersatspeed;

  @Log.NT(key = "shtrrunatvel")
  private boolean runShooterVel;

  private SlewRateLimiter topSpeedLimiter = new SlewRateLimiter(10000);
  private SlewRateLimiter bottomSpeedLimiter = new SlewRateLimiter(10000);
  public boolean topMotorConnected;
  public boolean bottomMotorConnected;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {

    topRoller = new CANSparkMax(Constants.CANIDConstants.topShooterID, MotorType.kBrushless);
    topController = topRoller.getPIDController();
    topEncoder = topRoller.getEncoder();

    configMotor(topRoller, topEncoder, true);

    bottomRoller = new CANSparkMax(Constants.CANIDConstants.bottomShooterID, MotorType.kBrushless);
    bottomEncoder = bottomRoller.getEncoder();
    bottomController = bottomRoller.getPIDController();

    configMotor(bottomRoller, bottomEncoder, true);

    topController.setOutputRange(0, 1);

    bottomController.setOutputRange(0, 1);

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder,
      boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.ShooterConstants.shooterContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.ShooterConstants.shooterIdleMode);
    encoder.setVelocityConversionFactor(Constants.ShooterConstants.shooterConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.ShooterConstants.shooterConversionPositionFactor);
    encoder.setAverageDepth(4);
    encoder.setMeasurementPeriod(32);
    motor.enableVoltageCompensation(Constants.ShooterConstants.voltageComp);

    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public void stopMotors() {
    runShooterVel = false;
    topController.setReference(0, ControlType.kVelocity);
    bottomController.setReference(0, ControlType.kVelocity);
    topRoller.stopMotor();
    bottomRoller.stopMotor();
    topSpeedLimiter.reset(0);
    bottomSpeedLimiter.reset(0);
    topSimRPM = 0;
    bottomSimRPM = 0;
  }

  public Command stopShooterCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> stopMotors()),
        Commands.runOnce(() -> topCommandRPM = 500),
        Commands.runOnce(() -> bottomCommandRPM = 500));

  }

  public Command startShooterCommand(double rpm) {
    return Commands.run(() -> startShooter(rpm))
        .until(() -> bothAtSpeed(5));
  }

  public Command startShooterCommand(double toprpm, double bottomrpm) {
    return Commands.run(() -> startShooter(toprpm, bottomrpm))
        .until(() -> bothAtSpeed(5));
  }

  public void startShooter(double rpm) {
    topCommandRPM = rpm;
    bottomCommandRPM = rpm;
    setRunShooter();
  }

  public void startShooter(double toprpm, double bottomrpm) {
    topCommandRPM = toprpm;
    bottomCommandRPM = bottomrpm;
    setRunShooter();
  }

  public void setRunShooter() {
    runShooterVel = true;
  }

  public void resetRunShooter() {
    runShooterVel = false;
  }

  public boolean getRunShooter() {
    return runShooterVel;
  }

  // o// }

  @Log.NT(key = "shtrtoprpm")
  public double getRPMTop() {
    if (RobotBase.isReal())
      return topEncoder.getVelocity();
    else
      return topSimRPM;
  }

  @Log.NT(key = "shtrbottomrpm")
  public double getRPMBottom() {
    if (RobotBase.isReal())
      return bottomEncoder.getVelocity();
    else
      return bottomSimRPM;
  }

  public void setCommandRPM(double rpm) {
    topCommandRPM = rpm;
    bottomCommandRPM = rpm;
  }

  public void increaseShooterRPM(double val) {
    topCommandRPM += val;
    bottomCommandRPM = topCommandRPM;
    if (topCommandRPM > ShooterConstants.maxShooterMotorRPM)
      topCommandRPM = ShooterConstants.maxShooterMotorRPM;
  }

  public Command increaseRPMCommand(double val) {
    return Commands.runOnce(() -> increaseShooterRPM(val));
  }

  public void decreaseShooterRPM(double val) {
    topCommandRPM -= val;
    bottomCommandRPM = topCommandRPM;
    if (topCommandRPM < ShooterConstants.minShooterMotorRPM)
      topCommandRPM = ShooterConstants.minShooterMotorRPM;
  }

  public Command decreaseRPMCommand(double val) {
    return Commands.runOnce(() -> decreaseShooterRPM(val));
  }

  public boolean topAtSpeed(double pct) {
    return topCommandRPM != 0 && Math.abs(topCommandRPM - getRPMTop()) < topCommandRPM * pct / 100;
  }

  public boolean bottomAtSpeed(double pct) {
    return bottomCommandRPM != 0 && Math.abs(bottomCommandRPM - getRPMBottom()) < bottomCommandRPM * pct / 100;
  }

  @Log.NT(key = "rpmerrortop")
  public double getTopRPMError() {
    return topCommandRPM - getRPMTop();
  }

  @Log.NT(key = "rpmerrorbottom")
  public double getBottomRPMError() {
    return bottomCommandRPM - getRPMBottom();
  }

  public boolean bothAtSpeed(double pct) {
    shootersatspeed = topAtSpeed(pct) && bottomAtSpeed(pct);
    return shootersatspeed;
  }

  @Log.NT(key = "shtrtopamps")
  public double getTopAmps() {
    return topRoller.getOutputCurrent();
  }

  @Log.NT(key = "shtrbottomamps")
  public double getBottomAmps() {
    return bottomRoller.getOutputCurrent();
  }

  @Log.NT(key = "shootertopstickyfault")
  public int getTopStickyFaults() {
    return topRoller.getStickyFaults();
  }

  @Log.NT(key = "shooterbottomstickyfault")
  public int getBottomStickyFaults() {
    return bottomRoller.getStickyFaults();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> topRoller.clearFaults()),
        runOnce(() -> bottomRoller.clearFaults()));
  }

  @Override
  public void simulationPeriodic() {
    double simrpmdiff = topCommandRPM - topSimRPM;
    double speedAdder = 50;// every 20 ms so 1000 per sec
    if (simrpmdiff < 0)
      speedAdder = -speedAdder;
    if (topSimRPM != topCommandRPM)
      topSimRPM += speedAdder;

    simrpmdiff = bottomCommandRPM - bottomSimRPM;
    speedAdder = 50;
    if (simrpmdiff < 0)
      speedAdder = -speedAdder;
    if (bottomSimRPM != bottomCommandRPM)
      bottomSimRPM += speedAdder;

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/RPM", topCommandRPM);

    if (runShooterVel) {
      double toprpm = getTopCommandRPM();
      double bottomrpm = getBottomCommandRPM();
      if (RobotBase.isReal()) {
        topController.setReference(topSpeedLimiter.calculate(toprpm), ControlType.kVelocity, 0);
        bottomController.setReference(bottomSpeedLimiter.calculate(bottomrpm), ControlType.kVelocity, 0);
      }
    } else {
      stopMotors();
    }
    if (!topMotorConnected) {
      topMotorConnected = checkMotorCanOK(topRoller);
      SmartDashboard.putBoolean("Shooter//OKTShooter", topMotorConnected);
    }

    if (!bottomMotorConnected) {
      bottomMotorConnected = checkMotorCanOK(bottomRoller);
      SmartDashboard.putBoolean("Shooter//OKBShooter", bottomMotorConnected);
    }

  }

  private boolean checkMotorCanOK(CANSparkMax motor) {
    double temp = motor.getOpenLoopRampRate();
    return RobotBase.isSimulation() || motor.setOpenLoopRampRate(temp) == REVLibError.kOk;
  }

  public Command testCan() {
    return Commands.parallel(
        Commands.runOnce(() -> topMotorConnected = false),
        runOnce(() -> bottomMotorConnected = false));
  }

  private double getTopCommandRPM() {
    return topCommandRPM;
  }

  private double getBottomCommandRPM() {
    return bottomCommandRPM;
  }

  public Command setTopKpKdKiCommand() {
    return Commands.runOnce(() -> setTopKpKdKi());
  }

  public Command setBottomKpKdKiCommand() {
    return Commands.runOnce(() -> setBottomKpKdKi());
  }

  public void setTopKpKdKi() {

    topController.setFF(Constants.ShooterConstants.topShooterKFF, 0);
    topController.setP(.0004, 0);// (Pref.getPref("ShooterTopKp"), 0);
    topController.setD(.01, 0);// (Pref.getPref("ShooterTopKd"), 0);
    topController.setI(0., 0);// (Pref.getPref("ShooterTopKi"), 0);
  }

  public void setBottomKpKdKi() {
    bottomController.setFF(Constants.ShooterConstants.bottomShooterKFF);
    bottomController.setP(.0004, 0);// (Pref.getPref("ShooterBottomKp"), 0);
    bottomController.setD(.0001, 0);// (Pref.getPref("ShooterBottomKd"), 0);
    bottomController.setI(.001, 0);// (Pref.getPref("ShooterBottomKi"), 0);
  }

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (volts) -> {
            bottomRoller.setVoltage(volts.in(Volts));
            topRoller.setVoltage(volts.in(Volts));
          },
          log -> {
            log.motor("Top")
                .voltage(Volts.of(topRoller.getAppliedOutput() * topRoller.getBusVoltage()))
                .angularVelocity(Rotations.per(Minute).of(topEncoder.getVelocity()))
                .angularPosition(Rotations.of(topEncoder.getPosition()));
            // log.motor("Bottom")
            // .voltage(Volts.of(bottomRoller.getAppliedOutput() *
            // bottomRoller.getBusVoltage()))
            // .angularVelocity(Rotations.per(Minute).of(bottomEncoder.getVelocity()))
            // .angularPosition(Rotations.of(bottomEncoder.getPosition()));
          },
          this));

  public Command quasistaticForward() {
    return sysIdRoutine.quasistatic(Direction.kForward);
  }

  public Command quasistaticBackward() {
    return sysIdRoutine.quasistatic(Direction.kReverse);
  }

  public Command dynamicForward() {
    return sysIdRoutine.dynamic(Direction.kForward);
  }

  public Command dynamicBackward() {
    return sysIdRoutine.dynamic(Direction.kReverse);
  }

}