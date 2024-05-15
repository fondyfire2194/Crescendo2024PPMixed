package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
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

    private boolean m_showScreens;

    public double armVolts;

    private double feedforward;

    private double acceleration;

    private double lastTime;

    private double lastSpeed;

    private double lastPosition;

    public double appliedVolts;

    public double armAngleRads;

    private PIDController pid = new PIDController(.1, 0.0, 0);

    private double pidout;

    public double angleTolerance = ArmConstants.angleTolerance;

    @Log.NT(key = "armenamble")
    public boolean enableArm;
    @Log.NT(key = "armpidenambled")
    public boolean armpidenabled;

    private double activeKv;

    private double lastGoal;

    @Log.NT(key = "simanglerads")
    private double simAngleRads;

    private boolean cancoderok;

    private int checkCounter;

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
                    Units.radiansToDegrees(getCurrentGoal()),
                    6,
                    new Color8Bit(Color.kRed)));
    @Log.NT(key = "usemotorencoder")
    private boolean useMotorEncoder;

    Trigger setMotorEncoderToCancoder;

    private boolean newGoal;

    public ArmSubsystem() {
        super(
                new ProfiledPIDController(
                        5,
                        0,
                        0,
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
            setGoal(getCanCoderRad());
        } else {
            armEncoder.setPosition(ArmConstants.armMinRadians);
            setGoal(ArmConstants.armMinRadians);
            simAngleRads = ArmConstants.armMinRadians;
        }

        pid.reset();
        setKp();

        SmartDashboard.putData("Arm//Arm Sim", m_mech2d);
        m_armTower.setColor(new Color8Bit(Color.kBlue));

        // setMotorEncoderToCancoder = new Trigger(
        // () -> isStopped() && !useMotorEncoder && getCurrentGoal() ==
        // ArmConstants.armMinRadians
        // && getAtSetpoint());

        // setMotorEncoderToCancoder.onTrue(Commands.runOnce(() ->
        // presetArmEncoderToCancoder()));
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
        getpidenebled();
        checkCounter++;
        if (checkCounter == 50) {
            int temp = 0;
            boolean armMotorOK = false;
            temp = getCanCoderID();
            cancoderok = (temp == CANIDConstants.armCancoderID);
            SmartDashboard.putBoolean("Arm//ArmCancoderOK", cancoderok);
            if (DriverStation.isDisabled()) {
                temp = armMotor.getDeviceId();
                armMotorOK = (temp == CANIDConstants.armID);
                SmartDashboard.putBoolean("Arm//ArmCanOK", armMotorOK);
            }
            checkCounter = 0;
        }
    }

    @Override
    public void simulationPeriodic() {

        double diff = getCurrentGoal() - simAngleRads;

        if (diff != 0)
            simAngleRads += diff / 10;

        // Update the Mechanism Arm angle based on the simulated arm angle
        m_arm.setAngle(Units.radiansToDegrees(getAngleRadians()));

        m_armTarget.setAngle(Units.radiansToDegrees(getCurrentGoal()));

    }

    @Override
    protected void useOutput(double output, State goalState) {

        pidout = pid.calculate(armAngleRads, getController().getSetpoint().position);

        double tempv = getController().getSetpoint().velocity;
        double tempp = getController().getSetpoint().position;

        acceleration = (getController().getSetpoint().velocity - lastSpeed)
                / (Timer.getFPGATimestamp() - lastTime);

        // feedforward =
        // armfeedforward.calculate(getController().getSetpoint().position,
        // getController().getSetpoint().velocity,
        // acceleration);

        feedforward = Pref.getPref("armFFKs") *
                Math.signum(getController().getSetpoint().velocity)
                + Pref.getPref("armFFKg") * Math.cos(getController().getSetpoint().position)
                + Pref.getPref("armFFKv") * getController().getSetpoint().velocity // this was commented out for some
                                                                                   // reason?
                + activeKv * getController().getSetpoint().velocity
                + Pref.getPref("armFFKa") * acceleration;
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

    public void trackDistance(double meters) {
        setTolerance(ArmConstants.angleTolerance);
        double angle = Constants.armAngleMap.get(meters);
        setGoal(angle);
    }

    public void resetController() {
        getController().reset(getAngleRadians());
    }

    public void setTolerance(double tolerance) {
        angleTolerance = tolerance;
    }

    private double checkArmLimits(double val) {
        double temp = 0;
        if (val >= ArmConstants.armMaxRadians)
            temp = ArmConstants.armMaxRadians;
        if (val <= ArmConstants.armMinRadians)
            temp = ArmConstants.armMinRadians;
        return temp;
    }

    public Command setGoalCommand(double angleRads) {
        newGoal = true;
        double armrads = checkArmLimits(angleRads);
        return Commands.sequence(
                runOnce(() -> setGoal(armrads)),
                runOnce(() -> setTolerance(ArmConstants.angleTolerance)),
                runOnce(() -> resetController()),
                runOnce(() -> enable()),
                runOnce(() -> newGoal = false));
    }

    public Command setGoalCommand(double angleRads, double tolerance) {
        newGoal = true;
        double armrads = checkArmLimits(angleRads);
        return Commands.sequence(
                runOnce(() -> setGoal(armrads)),
                runOnce(() -> angleTolerance = Units.degreesToRadians(tolerance)),
                runOnce(() -> resetController()),
                runOnce(() -> setUpDownKv(armrads)),

                Commands.runOnce(() -> enable()));
    }

    public Command positionToIntakeUDACommand() {
        return Commands.sequence(
                setGoalCommand(Units.degreesToRadians(20)),
                new WaitCommand(1),
                setGoalCommand(ArmConstants.pickupAngleRadians));
    }

    public void setUpDownKv(double rads) {
        activeKv = Pref.getPref("armFFKv");
        if (rads > lastGoal)
            activeKv = Pref.getPref("armUpFFKv");
        lastGoal = rads;
    }

    public void incrementArmAngle(double valdeg) {
        double temp = getCurrentGoal();
        temp += Units.degreesToRadians(valdeg);
        if (temp > ArmConstants.armMaxRadians)
            temp = ArmConstants.armMaxRadians;
        setGoal(temp);

    }

    public void decrementArmAngle(double valdeg) {
        double temp = getCurrentGoal();
        temp -= Units.degreesToRadians(valdeg);
        if (temp < ArmConstants.armMinRadians)
            temp = ArmConstants.armMinRadians;
        setGoal(temp);

    }

    @Log.NT(key = "armgoalrads")
    public double getCurrentGoal() {
        return getController().getGoal().position;

    }

    @Log.NT(key = "armgoaldeg")
    public double getCurrentGoalDeg() {
        return Units.radiansToDegrees(getCurrentGoal());
    }

    @Log.NT(key = "armrads")
    public double getAngleRadians() {
        if (RobotBase.isReal()) {
            if (useMotorEncoder)
                return getCanCoderRad();
            else
                return armEncoder.getPosition();
        } else
            return simAngleRads;
    }

    @Log.NT(key = "armdegs")
    public double getAngleDegrees() {
        return Units.radiansToDegrees(getAngleRadians());
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
        return !newGoal && isStopped() && Math.abs(getCurrentGoal() - getAngleRadians()) < angleTolerance;
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

    public boolean getpidenebled() {
        armpidenabled = isEnabled();
        return armpidenabled;
    }

    public void stop() {
        armMotor.setVoltage(0);
    }

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

    public Command clearFaultsCommand() {
        return Commands.runOnce(() -> armMotor.clearFaults());
    }

    public int getFaults() {
        return armMotor.getFaults();
    }

    public int getStickyFaults() {
        return armMotor.getStickyFaults();
    }

    public double round2dp(double number) {
        number = Math.round(number * 100);
        number /= 100;
        return number;
    }

    public double getCanCoderDeg() {
        return Units.radiansToDegrees(getCanCoderRad());
    }

    public double getCanCoderRad() {
        double temp = (armCancoder.getAbsolutePosition().getValueAsDouble()
                * Math.PI) + ArmConstants.cancoderOffsetRadians;
        if (temp > Math.PI)
            temp = temp - Math.PI;
        return temp;
    }

    public double getCanCoderRadPerSec() {
        return armCancoder.getVelocity().getValueAsDouble() * Math.PI;
    }

    public boolean isStopped() {
        return Math.abs(getCanCoderRadPerSec()) < Units.degreesToRadians(1);
    }

    public int getCanCoderID() {
        return armCancoder.getDeviceID();
    }

    public void setKp() {
        pid.setP(Pref.getPref("armKp"));

    }

    public void setKd() {
        // getController().setP(Pref.getPref("armKd"));
        pid.setD(Pref.getPref("armKd"));
    }

    public void setKi() {
        // getController().setP(Pref.getPref("armKi"));
        pid.setI(Pref.getPref("armKi"));
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
