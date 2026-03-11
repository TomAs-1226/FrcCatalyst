package frc.lib.catalyst.hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
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
 *
 * <p><b>Encoder Architecture:</b> By default, uses the TalonFX's built-in encoder
 * with {@code SensorToMechanismRatio} for gear reduction. This is the simplest
 * and most reliable feedback path — no external sensors needed.
 *
 * <p>For mechanisms requiring absolute positioning (e.g., swerve steering, arms
 * that start in unknown positions), you can optionally fuse a CANcoder using
 * {@link Builder#fusedCANcoder(int, double)} or {@link Builder#syncCANcoder(int, double)}.
 * Fused CANcoder combines the high-resolution internal encoder with the CANcoder's
 * absolute position on startup (requires Phoenix Pro). SyncCANcoder is the
 * non-Pro alternative that synchronizes once on boot.
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

        // Feedback configuration
        if (builder.fusedCancoderId >= 0) {
            // Fused CANcoder: combines internal encoder + CANcoder absolute position.
            // The TalonFX uses its internal encoder for high-resolution feedback
            // but seeds/fuses with the CANcoder's absolute position on startup.
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            config.Feedback.FeedbackRemoteSensorID = builder.fusedCancoderId;
            config.Feedback.RotorToSensorRatio = builder.rotorToSensorRatio;
            config.Feedback.SensorToMechanismRatio = builder.sensorToMechanismRatio;
        } else if (builder.syncCancoderId >= 0) {
            // Sync CANcoder: non-Pro alternative. Seeds the internal encoder
            // with the CANcoder's absolute position on boot, then uses internal only.
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
            config.Feedback.FeedbackRemoteSensorID = builder.syncCancoderId;
            config.Feedback.RotorToSensorRatio = builder.rotorToSensorRatio;
            config.Feedback.SensorToMechanismRatio = builder.sensorToMechanismRatio;
        } else if (builder.remoteCancoderId >= 0) {
            // Remote CANcoder: uses CANcoder directly as feedback (lower bandwidth).
            // Only use when the internal encoder cannot see the mechanism position
            // (e.g., mechanism on the other side of a belt/chain with slip).
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            config.Feedback.FeedbackRemoteSensorID = builder.remoteCancoderId;
            config.Feedback.SensorToMechanismRatio = builder.sensorToMechanismRatio;
        } else {
            // Default: internal encoder only. This is the simplest and best option
            // for most mechanisms. SensorToMechanismRatio converts rotor rotations
            // to mechanism rotations (e.g., 10.0 means 10 motor turns = 1 mechanism turn).
            config.Feedback.SensorToMechanismRatio = builder.gearRatio;
        }

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

    /**
     * Check if the motor has any faults (hardware faults from Phoenix status signals).
     * @return true if any fault is detected
     */
    public boolean hasFault() {
        return motor.getFault_Hardware().getValue()
                || motor.getFault_DeviceTemp().getValue()
                || motor.getFault_BootDuringEnable().getValue();
    }

    /**
     * Check if the motor temperature is above a threshold.
     * @param thresholdCelsius temperature threshold
     */
    public boolean isOverTemp(double thresholdCelsius) {
        return getTemperature() > thresholdCelsius;
    }

    /**
     * Calculate a temperature-based derating factor.
     * Returns 1.0 when cool, linearly decreases to 0 as temperature approaches cutoff.
     * Use this to scale motor output when the motor gets hot.
     *
     * @param warningTemp temperature where derating begins (e.g., 60C)
     * @param cutoffTemp temperature where output should be zero (e.g., 80C)
     * @return derating factor [0.0, 1.0]
     */
    public double getTemperatureDerating(double warningTemp, double cutoffTemp) {
        double temp = getTemperature();
        if (temp <= warningTemp) return 1.0;
        if (temp >= cutoffTemp) return 0.0;
        return 1.0 - (temp - warningTemp) / (cutoffTemp - warningTemp);
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
        private int fusedCancoderId = -1;   // -1 = disabled
        private int syncCancoderId = -1;    // -1 = disabled
        private int remoteCancoderId = -1;  // -1 = disabled
        private double rotorToSensorRatio = 1.0;
        private double sensorToMechanismRatio = 1.0;
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
         * Fuse a CANcoder with the internal encoder for absolute positioning.
         * The TalonFX uses its high-resolution internal encoder for closed-loop control
         * but fuses the CANcoder's absolute position to eliminate startup drift.
         * <b>Requires Phoenix Pro license.</b>
         *
         * @param cancoderId CAN ID of the CANcoder
         * @param rotorToSensorRatio ratio of motor rotor rotations to CANcoder rotations
         *                           (e.g., if the CANcoder is on the mechanism output
         *                           and the gear ratio is 10:1, this is 10.0)
         */
        public Builder fusedCANcoder(int cancoderId, double rotorToSensorRatio) {
            this.fusedCancoderId = cancoderId;
            this.rotorToSensorRatio = rotorToSensorRatio;
            return this;
        }

        /**
         * Sync a CANcoder with the internal encoder (non-Pro alternative).
         * Seeds the internal encoder with the CANcoder's absolute position on boot,
         * then uses the internal encoder exclusively. Good for mechanisms that
         * need to know their absolute position at startup but don't need
         * continuous absolute tracking.
         *
         * @param cancoderId CAN ID of the CANcoder
         * @param rotorToSensorRatio ratio of motor rotor rotations to CANcoder rotations
         */
        public Builder syncCANcoder(int cancoderId, double rotorToSensorRatio) {
            this.syncCancoderId = cancoderId;
            this.rotorToSensorRatio = rotorToSensorRatio;
            return this;
        }

        /**
         * Use a remote CANcoder as the feedback sensor instead of the internal encoder.
         * Only use this when the internal encoder cannot see the mechanism
         * (e.g., belt/chain with slip between motor and mechanism).
         * Lower bandwidth than internal/fused — prefer fusedCANcoder when possible.
         *
         * @param cancoderId CAN ID of the CANcoder
         */
        public Builder remoteCANcoder(int cancoderId) {
            this.remoteCancoderId = cancoderId;
            return this;
        }

        /**
         * Set the sensor-to-mechanism ratio when using a CANcoder.
         * This is the ratio from the CANcoder to the mechanism output.
         * Only needed with fusedCANcoder/syncCANcoder/remoteCANcoder.
         *
         * @param ratio CANcoder rotations per mechanism rotation
         */
        public Builder sensorToMechanismRatio(double ratio) {
            this.sensorToMechanismRatio = ratio;
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
