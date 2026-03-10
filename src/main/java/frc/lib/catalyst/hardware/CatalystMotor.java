package frc.lib.catalyst.hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Unified TalonFX motor wrapper with builder-style configuration,
 * simplified control methods, and automatic telemetry.
 */
public class CatalystMotor {

    private final TalonFX motor;
    private TalonFX follower;
    private final int canId;
    private final String name;

    // Control requests (reused to avoid GC pressure)
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    // Telemetry publishers
    private final DoublePublisher positionPub;
    private final DoublePublisher velocityPub;
    private final DoublePublisher voltagePub;
    private final DoublePublisher currentPub;
    private final DoublePublisher tempPub;

    // Config storage
    private double gearRatio = 1.0;
    private double positionConversionFactor = 1.0; // rotations to mechanism units

    private CatalystMotor(Builder builder) {
        this.canId = builder.canId;
        this.name = builder.name != null ? builder.name : "Motor" + canId;
        this.motor = new TalonFX(canId, builder.canBus);
        this.gearRatio = builder.gearRatio;
        this.positionConversionFactor = builder.positionConversionFactor;

        // Apply configuration
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output
        config.MotorOutput.Inverted = builder.inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = builder.brakeMode
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        // Current limits
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = builder.currentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = builder.statorCurrentLimit;

        // PID (Slot 0)
        config.Slot0.kP = builder.kP;
        config.Slot0.kI = builder.kI;
        config.Slot0.kD = builder.kD;
        config.Slot0.kS = builder.kS;
        config.Slot0.kV = builder.kV;
        config.Slot0.kA = builder.kA;
        config.Slot0.kG = builder.kG;
        config.Slot0.GravityType = builder.gravityType;

        // Motion Magic
        if (builder.motionMagicCruiseVelocity > 0) {
            config.MotionMagic.MotionMagicCruiseVelocity = builder.motionMagicCruiseVelocity;
            config.MotionMagic.MotionMagicAcceleration = builder.motionMagicAcceleration;
            config.MotionMagic.MotionMagicJerk = builder.motionMagicJerk;
        }

        // Soft limits
        if (builder.forwardSoftLimit != Double.MAX_VALUE) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = builder.forwardSoftLimit;
        }
        if (builder.reverseSoftLimit != -Double.MAX_VALUE) {
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = builder.reverseSoftLimit;
        }

        // Voltage compensation
        if (builder.voltageCompensation > 0) {
            config.Voltage.PeakForwardVoltage = builder.voltageCompensation;
            config.Voltage.PeakReverseVoltage = -builder.voltageCompensation;
        }

        // Ramp rates
        if (builder.openLoopRampRate > 0) {
            config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = builder.openLoopRampRate;
            config.OpenLoopRamps.VoltageOpenLoopRampPeriod = builder.openLoopRampRate;
        }
        if (builder.closedLoopRampRate > 0) {
            config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = builder.closedLoopRampRate;
            config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = builder.closedLoopRampRate;
        }

        // Gear ratio for rotor-to-sensor
        config.Feedback.SensorToMechanismRatio = builder.gearRatio;

        // Apply config with retries
        for (int i = 0; i < 5; i++) {
            var status = motor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        // Set up follower if configured
        if (builder.followerCanId >= 0) {
            follower = new TalonFX(builder.followerCanId, builder.canBus);
            TalonFXConfiguration followerConfig = new TalonFXConfiguration();
            followerConfig.MotorOutput.NeutralMode = builder.brakeMode
                    ? NeutralModeValue.Brake
                    : NeutralModeValue.Coast;
            followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            followerConfig.CurrentLimits.SupplyCurrentLimit = builder.currentLimit;
            followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            followerConfig.CurrentLimits.StatorCurrentLimit = builder.statorCurrentLimit;

            for (int i = 0; i < 5; i++) {
                var status = follower.getConfigurator().apply(followerConfig);
                if (status.isOK()) break;
            }
            follower.setControl(new Follower(canId,
                    builder.followerOppose
                            ? MotorAlignmentValue.Opposed
                            : MotorAlignmentValue.Aligned));
        }

        // Set up telemetry
        NetworkTable table = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable(this.name);
        positionPub = table.getDoubleTopic("Position").publish();
        velocityPub = table.getDoubleTopic("Velocity").publish();
        voltagePub = table.getDoubleTopic("Voltage").publish();
        currentPub = table.getDoubleTopic("Current").publish();
        tempPub = table.getDoubleTopic("Temperature").publish();
    }

    // --- Control Methods ---

    /** Set motor output as a percentage [-1, 1]. */
    public void setPercent(double percent) {
        motor.setControl(dutyCycleRequest.withOutput(percent));
    }

    /** Set motor output voltage [-12, 12]. */
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    /** Set closed-loop position target in mechanism units. */
    public void setPosition(double position) {
        motor.setControl(positionRequest.withPosition(position));
    }

    /** Set closed-loop position target with arbitrary feedforward voltage. */
    public void setPosition(double position, double feedforwardVolts) {
        motor.setControl(positionRequest.withPosition(position).withFeedForward(feedforwardVolts));
    }

    /** Set Motion Magic position target in mechanism units. */
    public void setMotionMagicPosition(double position) {
        motor.setControl(motionMagicRequest.withPosition(position));
    }

    /** Set Motion Magic position target with arbitrary feedforward voltage. */
    public void setMotionMagicPosition(double position, double feedforwardVolts) {
        motor.setControl(motionMagicRequest.withPosition(position).withFeedForward(feedforwardVolts));
    }

    /** Set closed-loop velocity target in mechanism rotations per second. */
    public void setVelocity(double velocityRPS) {
        motor.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    /** Stop the motor (neutral output). */
    public void stop() {
        motor.setControl(neutralRequest);
    }

    /** Set the motor's encoder position. */
    public void setEncoderPosition(double position) {
        motor.setPosition(position);
    }

    /** Zero the motor's encoder. */
    public void zeroEncoder() {
        motor.setPosition(0);
    }

    // --- Getters ---

    /** Get mechanism position (after gear ratio). */
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /** Get mechanism velocity in rotations per second (after gear ratio). */
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    /** Get applied motor voltage. */
    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    /** Get stator current draw in amps. */
    public double getStatorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    /** Get supply current draw in amps. */
    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    /** Get motor temperature in Celsius. */
    public double getTemperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

    /** Get the underlying TalonFX for advanced configuration. */
    public TalonFX getTalonFX() {
        return motor;
    }

    /** Get the follower TalonFX, if configured. */
    public TalonFX getFollowerTalonFX() {
        return follower;
    }

    /** Update telemetry. Call from subsystem periodic(). */
    public void updateTelemetry() {
        positionPub.set(getPosition());
        velocityPub.set(getVelocity());
        voltagePub.set(getAppliedVoltage());
        currentPub.set(getStatorCurrent());
        tempPub.set(getTemperature());
    }

    // --- Builder ---

    public static Builder builder(int canId) {
        return new Builder(canId);
    }

    public static class Builder {
        private final int canId;
        private String canBus = "";
        private String name;
        private boolean inverted = false;
        private boolean brakeMode = true;
        private double currentLimit = 40;
        private double statorCurrentLimit = 80;
        private double gearRatio = 1.0;
        private double positionConversionFactor = 1.0;
        private double kP = 0, kI = 0, kD = 0;
        private double kS = 0, kV = 0, kA = 0, kG = 0;
        private GravityTypeValue gravityType = GravityTypeValue.Elevator_Static;
        private double motionMagicCruiseVelocity = 0;
        private double motionMagicAcceleration = 0;
        private double motionMagicJerk = 0;
        private double forwardSoftLimit = Double.MAX_VALUE;
        private double reverseSoftLimit = -Double.MAX_VALUE;
        private int followerCanId = -1;
        private boolean followerOppose = false;
        private double voltageCompensation = 0; // 0 = disabled
        private double openLoopRampRate = 0; // seconds 0->full, 0 = disabled
        private double closedLoopRampRate = 0;

        private Builder(int canId) {
            this.canId = canId;
        }

        public Builder canBus(String canBus) { this.canBus = canBus; return this; }
        public Builder name(String name) { this.name = name; return this; }
        public Builder inverted(boolean inverted) { this.inverted = inverted; return this; }
        public Builder brakeMode(boolean brakeMode) { this.brakeMode = brakeMode; return this; }
        public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
        public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }
        public Builder gearRatio(double ratio) { this.gearRatio = ratio; return this; }
        public Builder positionConversionFactor(double factor) { this.positionConversionFactor = factor; return this; }

        public Builder pid(double kP, double kI, double kD) {
            this.kP = kP; this.kI = kI; this.kD = kD; return this;
        }

        public Builder feedforward(double kS, double kV) {
            this.kS = kS; this.kV = kV; return this;
        }

        public Builder feedforward(double kS, double kV, double kA) {
            this.kS = kS; this.kV = kV; this.kA = kA; return this;
        }

        public Builder gravityGain(double kG, GravityTypeValue type) {
            this.kG = kG; this.gravityType = type; return this;
        }

        public Builder motionMagic(double cruiseVelocity, double acceleration, double jerk) {
            this.motionMagicCruiseVelocity = cruiseVelocity;
            this.motionMagicAcceleration = acceleration;
            this.motionMagicJerk = jerk;
            return this;
        }

        public Builder softLimits(double reverse, double forward) {
            this.reverseSoftLimit = reverse;
            this.forwardSoftLimit = forward;
            return this;
        }

        public Builder withFollower(int canId, boolean oppose) {
            this.followerCanId = canId;
            this.followerOppose = oppose;
            return this;
        }

        /**
         * Enable voltage compensation. Motor output will be scaled to behave
         * consistently regardless of battery voltage.
         * @param nominalVoltage typical voltage (usually 12.0)
         */
        public Builder voltageCompensation(double nominalVoltage) {
            this.voltageCompensation = nominalVoltage;
            return this;
        }

        /**
         * Set open-loop ramp rate (seconds from 0 to full output).
         * Prevents sudden acceleration in duty cycle and voltage control.
         * @param seconds ramp period (e.g., 0.25 = 250ms to full)
         */
        public Builder openLoopRampRate(double seconds) {
            this.openLoopRampRate = seconds;
            return this;
        }

        /**
         * Set closed-loop ramp rate (seconds from 0 to full output).
         * Prevents sudden acceleration in PID/MotionMagic control.
         * @param seconds ramp period
         */
        public Builder closedLoopRampRate(double seconds) {
            this.closedLoopRampRate = seconds;
            return this;
        }

        public CatalystMotor build() {
            return new CatalystMotor(this);
        }
    }
}
