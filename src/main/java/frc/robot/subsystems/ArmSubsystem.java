package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Pref;
import monologue.Logged;
import monologue.Annotations.Log;

import static edu.wpi.first.units.Units.Volts;

public class ArmSubsystem extends ProfiledPIDSubsystem implements Logged {

    public final CANSparkMax armMotor = new CANSparkMax(CANIDConstants.armID, MotorType.kBrushless);

    public final CANcoder armCancoder;

    private final RelativeEncoder armEncoder;

    public ArmFeedforward armfeedforward;

    public boolean armMotorConnected;

    public double appliedOutput;

    private boolean useSoftwareLimit;
    public boolean inIZone;
    public double armVolts;
    @Log.NT(key = "armfeedforward")
    private double feedforward;
    private double acceleration;
    private double lastTime;
    private double lastSpeed;
    private double lastPosition = 0;
    public double appliedVolts;
    public double armAngleRads;
    @Log.NT(key = "armpidout")
    private double pidout;
    private PIDController pid = new PIDController(ArmConstants.armKp, 0.0, 0);
    public double angleToleranceRads = ArmConstants.angleTolerance;
    @Log.NT(key = "enablearm")
    public boolean enableArm;

    private double activeKv;
    @Log.NT(key = "simanglerads")
    private double simAngleRads;
    @Log.NT(key = "shootingangle")
    public double angleDegWhenShooting;
    private boolean cancoderconnected;
    private int checkCancoderCounter;
    private final DCMotor m_armGearbox = DCMotor.getNEO(1);

    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            m_armGearbox,
            ArmConstants.NET_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ArmConstants.armLength, ArmConstants.armMass),
            ArmConstants.armLength,
            ArmConstants.reverseMovementLimitAngle,
            ArmConstants.forwardMovementLimitAngle,
            true,
            0,
            VecBuilder.fill(ArmConstants.RADIANS_PER_ENCODER_REV) // Add noise with a std-dev of 1 tick
    );

    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 1, -90));
    private final MechanismLigament2d m_arm = m_armPivot.append(
            new MechanismLigament2d(
                    "Arm",
                    30,
                    Units.radiansToDegrees(armAngleRads),
                    6,
                    new Color8Bit(Color.kYellow)));

    private final MechanismLigament2d m_armTarget = m_armPivot.append(
            new MechanismLigament2d(
                    "ArmTarget",
                    30,
                    Units.radiansToDegrees(getCurrentGoalRads()),
                    6,
                    new Color8Bit(Color.kRed)));
    @Log.NT(key = "usemotorencoder")
    public boolean useMotorEncoder;

    Trigger setMotorEncoderToCancoder;

    private int cancdrokctr;

    public ArmSubsystem() {
        super(
                new ProfiledPIDController(
                        ArmConstants.armKp,
                        ArmConstants.armKi,
                        ArmConstants.armKd,
                        new TrapezoidProfile.Constraints(
                                ArmConstants.kTrapVelocityRadPerSecond,
                                ArmConstants.kTrapAccelerationRadPerSecSquared)),
                0);

        useSoftwareLimit = false;

        armEncoder = armMotor.getEncoder();
        armCancoder = new CANcoder(CANIDConstants.armCancoderID, "CV1");

        configMotor(armMotor, armEncoder, false);

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        armfeedforward = new ArmFeedforward(ArmConstants.armKs, ArmConstants.armKg, ArmConstants.armKv,
                ArmConstants.armKa);

        setUseMotorEncoder(false);

        if (RobotBase.isReal()) {
            presetArmEncoderToCancoder();
            setGoalCommand(getCanCoderRad());
        } else {
            armEncoder.setPosition(ArmConstants.armMinRadians);
            setGoal(ArmConstants.armMinRadians);
            simAngleRads = ArmConstants.armMinRadians;
        }
        resetController();
        pid.reset();
        setKp();

        SmartDashboard.putData("Arm//Arm Sim", m_mech2d);
        m_armTower.setColor(new Color8Bit(Color.kBlue));
    }

    private void configMotor(CANSparkMax motor, RelativeEncoder encoder, boolean reverse) {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(ArmConstants.armContinuousCurrentLimit);
        motor.setInverted(reverse);
        motor.setIdleMode(ArmConstants.armIdleMode);
        encoder.setVelocityConversionFactor(ArmConstants.armConversionVelocityFactor);
        encoder.setPositionConversionFactor(ArmConstants.armConversionPositionFactor);
        motor.enableVoltageCompensation(ArmConstants.voltageComp);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kPositionOnly);
        motor.burnFlash();
    }

    public void periodicRobot() {
        armAngleRads = getAngleRadians();
        if (!enableArm) {
            setGoal(armAngleRads);
        }

        // if (getCurrentGoalRads() > ArmConstants.armMaxRadians)
        // setGoalCommand(ArmConstants.armMaxRadians);
        // if (getCurrentGoalRads() < ArmConstants.armMinRadians)
        // setGoalCommand(ArmConstants.armMinRadians);

        checkCancoderCounter++;
        if (checkCancoderCounter == 10) {
            cancoderconnected = RobotBase.isSimulation() || checkCancoderCanOK();
            checkCancoderCounter = 0;
            SmartDashboard.putBoolean("Arm//OKCancoder", cancoderconnected);
        }

        if (!armMotorConnected) {
            armMotorConnected = checkMotorCanOK(armMotor);
            SmartDashboard.putBoolean("Arm//OKArmMotor", armMotorConnected);
        }

        SmartDashboard.putNumber("Arm/CanCdrRads", getCanCoderRad());
        SmartDashboard.putBoolean("Arm/isenab", isEnabled());
    }

    private boolean checkCancoderCanOK() {
        SmartDashboard.putNumber("CCC/Ctr", cancdrokctr);
        double currentPosition = getAngleDegrees();
        SmartDashboard.putNumber("CCC/Cp", currentPosition);
        SmartDashboard.putNumber("CCC/Lp", lastPosition);
        if (currentPosition == lastPosition) {
            cancdrokctr++;
        } else {
            cancdrokctr = 0;
        }
        lastPosition = currentPosition;
        return RobotBase.isSimulation() || cancdrokctr < 10;
    }

    private boolean checkMotorCanOK(CANSparkMax motor) {
        double temp = motor.getOpenLoopRampRate();
        return RobotBase.isSimulation() || motor.setOpenLoopRampRate(temp) == REVLibError.kOk;
    }

    @Override
    public void simulationPeriodic() {

        double diff = getCurrentGoalRads() - simAngleRads;

        if (diff != 0)
            simAngleRads += diff / 10;

        // Update the Mechanism Arm angle based on the simulated arm angle
        m_arm.setAngle(Units.radiansToDegrees(getAngleRadians()));
        m_armTarget.setAngle(Units.radiansToDegrees(getCurrentGoalRads()));
    }

    @Override
    protected void useOutput(double output, State goalState) {
        pidout = pid.calculate(armAngleRads, getController().getSetpoint().position);
        acceleration = (getController().getSetpoint().velocity - lastSpeed)
                / (Timer.getFPGATimestamp() - lastTime);
        boolean tuning = false;
        if (!tuning) {

            feedforward = armfeedforward.calculate(getController().getSetpoint().position,
                    getController().getSetpoint().velocity,
                    acceleration);
        } else {
            feedforward = Pref.getPref("armFFKs") *
                    Math.signum(getController().getSetpoint().velocity)
                    + Pref.getPref("armFFKg") * Math.cos(getController().getSetpoint().position)
                    + Pref.getPref("armFFKv") * getController().getSetpoint().velocity // this
                    // was commented out for some reason?
                    + activeKv * getController().getSetpoint().velocity
                    + Pref.getPref("armFFKa") * acceleration;
        }

        // Add the feedforward to the PID output to get the motor output

        lastSpeed = getController().getSetpoint().velocity;
        lastPosition = getController().getSetpoint().position;

        lastTime = Timer.getFPGATimestamp();

        double out = pidout + feedforward;

        armMotor.setVoltage(out);
    }

    @Override
    protected double getMeasurement() {
        return armAngleRads;
    }

    public void resetController() {
        getController().reset(getAngleRadians());
    }

    public void setTolerance(double toleranceRads) {
        angleToleranceRads = toleranceRads;
    }

    public void setTarget(double anglerads) {
        if (anglerads > ArmConstants.armMaxRadians)
            anglerads = ArmConstants.armMaxRadians;
        if (anglerads < ArmConstants.armMinRadians)
            anglerads = ArmConstants.armMinRadians;
        setGoal(anglerads);
    }

    // public Command setGoalCommand(double anglerads) {
    // return Commands.sequence(
    // Commands.runOnce(() -> setTarget(anglerads)),
    // Commands.either(
    // Commands.runOnce(() -> enable()),
    // Commands.none(),
    // () -> isEnabled()));
    // }

    public Command setGoalCommand(double angleRads) {
        return Commands.sequence(
                Commands.runOnce(() -> SmartDashboard.putNumber("ArmGoal", angleRads)),
                Commands.runOnce(() -> setTolerance(ArmConstants.angleTolerance)),
                Commands.runOnce(() -> resetController()),
                Commands.runOnce(() -> setGoal(angleRads), this),
                Commands.runOnce(() -> enable(), this));
    }

    public void incrementArmAngle(double valdeg) {
        double temp = getCurrentGoalRads();
        temp += Units.degreesToRadians(valdeg);
        if (temp > ArmConstants.armMaxRadians)
            temp = ArmConstants.armMaxRadians;
        setGoal(temp);
    }

    public void decrementArmAngle(double valdeg) {
        double temp = getCurrentGoalRads();
        temp -= Units.degreesToRadians(valdeg);
        if (temp < ArmConstants.armMinRadians)
            temp = ArmConstants.armMinRadians;
        setGoal(temp);
    }

    @Log.NT(key = "armgoalrads")
    public double getCurrentGoalRads() {
        return getController().getGoal().position;
    }

    @Log.NT(key = "armgoaldeg")
    public double getCurrentGoalDeg() {
        return round2dp(Units.radiansToDegrees(getCurrentGoalRads()), 2);
    }

    @Log.NT(key = "armrads")
    public double getAngleRadians() {
        if (RobotBase.isReal()) {
            if (!useMotorEncoder)
                return getCanCoderRad();
            else
                return armEncoder.getPosition();
        } else
            return simAngleRads;
    }

    @Log.NT(key = "armerrorrads")
    public double getAngleErrorRadians() {
        return getCurrentGoalRads() - getAngleRadians();
    }

    @Log.NT(key = "armerrordeg")
    public double getAngleErrorDegrees() {
        return Units.radiansToDegrees(getAngleErrorRadians());
    }

    @Log.NT(key = "armdegs")
    public double getAngleDegrees() {
        return round2dp(Units.radiansToDegrees(getAngleRadians()), 2);
    }

    public void setUseMotorEncoder(boolean on) {
        useMotorEncoder = on;
    }

    public boolean getUseMotorEncoder() {
        return useMotorEncoder;
    }

    private void presetArmEncoderToCancoder() {
        armEncoder.setPosition(getCanCoderRad());
    }

    @Log.NT(key = "armatsetpoint")
    public boolean getAtSetpoint() {
        return Math.abs(getAngleErrorRadians()) < angleToleranceRads;
    }

    public double getVoltsPerRadPerSec() {
        appliedVolts = armMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        double temp = appliedVolts / getRadsPerSec();
        if (temp < 1 || temp > 3)
            temp = 0;
        return temp;
    }

    public double getRadsPerSec() {
        return armEncoder.getVelocity();
    }

    public double getCanCoderRadsPerSec() {
        return Math.PI * armCancoder.getVelocity().getValueAsDouble();
    }

    public double getDegreesPerSec() {
        return Units.radiansToDegrees(armEncoder.getVelocity());
    }

    public boolean onPlusSoftwareLimit() {
        return armMotor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean onMinusSoftwareLimit() {
        return armMotor.getFault(FaultID.kSoftLimitRev);
    }

    public boolean onPlusHardwareLimit() {
        return armMotor.getFault(FaultID.kHardLimitRev);
    }

    public boolean onMinusHardwareLimit() {
        return armMotor.getFault(FaultID.kHardLimitRev);
    }

    public boolean onLimit() {
        return onPlusHardwareLimit() || onMinusHardwareLimit() || onPlusSoftwareLimit() || onMinusSoftwareLimit();
    }

    public void stop() {
        armMotor.setVoltage(0);
    }

    @Log.NT(key = "armamps")
    public double getAmps() {
        return armMotor.getOutputCurrent();
    }

    public void setSoftwareLimits() {
        armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) ArmConstants.armMinRadians);
        armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) ArmConstants.armMaxRadians);
        armMotor.setIdleMode(IdleMode.kBrake);
    }

    public void enableSoftLimits(boolean on) {
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, on);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, on);
    }

    public boolean isBraked() {
        return armMotor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return armMotor.isSoftLimitEnabled(SoftLimitDirection.kForward)
                || armMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    @Log.NT(key = "armstickyfault")
    public int getStickyFaults() {
        return armMotor.getStickyFaults();
    }

    @Log.NT(key = "armscancoderstickyfault")
    public int getCancoderStickyFaults() {
        return armCancoder.getStickyFaultField().getValue();
    }

    public Command clearStickyFaultsCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> armMotor.clearFaults()),
                runOnce(() -> armCancoder.clearStickyFaults()));
    }

    public static double round2dp(double number, int dp) {
        double temp = Math.pow(10, dp);
        double temp1 = Math.round(number * temp);
        return temp1 / temp;
    }

    public double getCanCoderDeg() {
        return Units.radiansToDegrees(getCanCoderRad());
    }

    public double getCanCoderRad() {
        double temp = (armCancoder.getAbsolutePosition().getValueAsDouble()
                * Math.PI) + ArmConstants.cancoderOffsetRadiansAtCalibration;
        if (temp > Math.PI)
            temp = temp - Math.PI;
        return temp;
    }

    @Log.NT(key = "cancodervelocity")
    public double getCanCoderRadPerSec() {
        return armCancoder.getVelocity().getValueAsDouble() * Math.PI;

    }

    @Log.NT(key = "isstopped")
    public boolean isStopped() {
        return Math.abs(getCanCoderRadPerSec()) < Units.degreesToRadians(1);
    }

    public Command testCan() {
        return Commands.parallel(
                Commands.runOnce(() -> armMotorConnected = false),
                runOnce(() -> cancoderconnected = false));
    }

    public void setKp() {
        pid.setP(ArmConstants.armKp);// (Pref.getPref("armKp"));
    }

    public void setKd() {
        pid.setD(0);// (Pref.getPref("armKd"));
    }

    public void setKi() {
        pid.setI(.5);// (Pref.getPref("armKi"));
    }

    public Command setPIDGainsCommand() {
        return Commands.runOnce(() -> setKPKIKD());
    }

    public void setKPKIKD() {
        setKp();
        setKi();
        setKd();
    }

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (volts) -> {
                        armMotor.setVoltage(volts.in(Volts));
                    },
                    null,
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
