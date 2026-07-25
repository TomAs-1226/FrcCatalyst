package frc.lib.catalyst.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.catalyst.logging.CatalystLog;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

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

    /** Specification for one follower motor. */
    public record FollowerSpec(int canId, boolean oppose) {}

    private final TalonFX motor;
    private final List<TalonFX> followers = new ArrayList<>();
    private final int canId;
    private final String name;

    // Control requests (reused to avoid GC pressure)
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    // Config storage
    private double gearRatio = 1.0;
    private double positionConversionFactor = 1.0; // rotations to mechanism units
    // Retained so live-tuning helpers can rebuild Slot0Configs without losing the
    // gravity model the mechanism was originally configured with.
    private GravityTypeValue gravityType;
    // Retained so a runtime current-limit change re-sends the WHOLE CurrentLimits
    // group (Phoenix's apply(CurrentLimitsConfigs) overwrites all of it), instead
    // of silently dropping the other limit and its thermal protection.
    private double supplyCurrentLimit;
    private double statorCurrentLimit;

    private CatalystMotor(Builder builder) {
        this.canId = builder.canId;
        this.name = builder.name != null ? builder.name : "Motor" + canId;

        // Claim our CAN id with the registry before constructing the TalonFX,
        // so a duplicate id surfaces at robotInit with a clear message rather
        // than waiting for Phoenix to fail later.
        CANRegistry.register(this.name, canId, builder.canBus, "TalonFX");

        this.motor = new TalonFX(canId, builder.canBus);
        this.gearRatio = builder.gearRatio;
        this.positionConversionFactor = builder.positionConversionFactor;
        this.gravityType = builder.gravityType;
        this.supplyCurrentLimit = builder.currentLimit;
        this.statorCurrentLimit = builder.statorCurrentLimit;

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

        // Torque-current peaks (only meaningful for *TorqueCurrentFOC requests; harmless otherwise)
        config.TorqueCurrent.PeakForwardTorqueCurrent = builder.peakForwardTorqueCurrent;
        config.TorqueCurrent.PeakReverseTorqueCurrent = builder.peakReverseTorqueCurrent;

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

        // Claim any attached CANcoder. Type label matches the feedback mode
        // so the wiring summary names it correctly.
        if (builder.fusedCancoderId >= 0) {
            CANRegistry.register(this.name + "CANcoder", builder.fusedCancoderId, builder.canBus, "CANcoder (fused)");
        } else if (builder.syncCancoderId >= 0) {
            CANRegistry.register(this.name + "CANcoder", builder.syncCancoderId, builder.canBus, "CANcoder (sync)");
        } else if (builder.remoteCancoderId >= 0) {
            CANRegistry.register(this.name + "CANcoder", builder.remoteCancoderId, builder.canBus, "CANcoder (remote)");
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

        // Set up followers. Each one gets a fresh TalonFX, shared current/neutral
        // config, and a Follower control request pointing at the leader.
        for (FollowerSpec spec : builder.followerSpecs) {
            CANRegistry.register(this.name + "Follower" + spec.canId(), spec.canId(), builder.canBus, "TalonFX (follower)");
            TalonFX follower = new TalonFX(spec.canId(), builder.canBus);
            TalonFXConfiguration followerConfig = new TalonFXConfiguration();
            followerConfig.MotorOutput.NeutralMode = builder.brakeMode
                    ? NeutralModeValue.Brake
                    : NeutralModeValue.Coast;
            followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            followerConfig.CurrentLimits.SupplyCurrentLimit = builder.currentLimit;
            followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            followerConfig.CurrentLimits.StatorCurrentLimit = builder.statorCurrentLimit;
            followerConfig.TorqueCurrent.PeakForwardTorqueCurrent = builder.peakForwardTorqueCurrent;
            followerConfig.TorqueCurrent.PeakReverseTorqueCurrent = builder.peakReverseTorqueCurrent;

            for (int i = 0; i < 5; i++) {
                var status = follower.getConfigurator().apply(followerConfig);
                if (status.isOK()) break;
            }
            follower.setControl(new Follower(canId,
                    spec.oppose()
                            ? MotorAlignmentValue.Opposed
                            : MotorAlignmentValue.Aligned));
            followers.add(follower);
        }

        // Optional CAN-bus hygiene: raise only the status signals we actually
        // read to the configured rate, then stop every other signal from
        // broadcasting. On a high-device-count robot this meaningfully cuts
        // bus utilization. Opt-in (default off) so it never silently starves a
        // signal a team depends on.
        if (builder.optimizeCanBus) {
            BaseStatusSignal.setUpdateFrequencyForAll(builder.canUpdateHz,
                    motor.getPosition(), motor.getVelocity(), motor.getMotorVoltage(),
                    motor.getStatorCurrent(), motor.getSupplyCurrent(), motor.getDeviceTemp(),
                    motor.getFault_Hardware(), motor.getFault_DeviceTemp(),
                    motor.getFault_BootDuringEnable());
            motor.optimizeBusUtilization();
            for (TalonFX f : followers) {
                BaseStatusSignal.setUpdateFrequencyForAll(builder.canUpdateHz,
                        f.getStatorCurrent(), f.getDeviceTemp());
                f.optimizeBusUtilization();
            }
        }
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
    /**
     * Closed-loop velocity via torque-current FOC (requires a Phoenix Pro license on the device).
     *
     * <p>In this mode the Slot 0 gains are in <b>amps</b>, not volts: kP is A per rps of error, kS
     * is A, kV is A per rps. Voltage-mode gains are not transferable — running them through this
     * request will be violent. Configure {@code Builder.torqueCurrentLimits(...)} so the request
     * has headroom (Phoenix defaults the peaks conservatively).
     *
     * @param velocityRPS target velocity in rotations per second
     * @param feedforwardAmps additive feedforward in amps, recomputable every loop (the classic use
     *     is compensating a shooter for the ball being fed into it)
     */
    public void setVelocityTorqueCurrent(double velocityRPS, double feedforwardAmps) {
        motor.setControl(
                velocityTorqueRequest.withVelocity(velocityRPS).withFeedForward(feedforwardAmps));
    }

    /** Closed-loop torque-current velocity with no additive feedforward. */
    public void setVelocityTorqueCurrent(double velocityRPS) {
        setVelocityTorqueCurrent(velocityRPS, 0.0);
    }

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

    /**
     * Get the first follower TalonFX, if any are configured.
     * Returns {@code null} when no followers exist. Prefer {@link #getFollowerTalonFXs()}
     * when you need access to all followers.
     */
    public TalonFX getFollowerTalonFX() {
        return followers.isEmpty() ? null : followers.get(0);
    }

    /**
     * Get all follower TalonFX motors in the order they were added.
     * Returns an empty list when no followers are configured.
     */
    public List<TalonFX> getFollowerTalonFXs() {
        return Collections.unmodifiableList(followers);
    }

    /** Number of follower motors configured. */
    public int getFollowerCount() {
        return followers.size();
    }

    /**
     * Get the stator current of every follower in configuration order.
     * Returns an empty array when no followers are configured.
     */
    public double[] getFollowerStatorCurrents() {
        double[] currents = new double[followers.size()];
        for (int i = 0; i < followers.size(); i++) {
            currents[i] = followers.get(i).getStatorCurrent().getValueAsDouble();
        }
        return currents;
    }

    /**
     * Get the temperature of every follower in configuration order, in degrees Celsius.
     * Returns an empty array when no followers are configured.
     */
    public double[] getFollowerTemperatures() {
        double[] temps = new double[followers.size()];
        for (int i = 0; i < followers.size(); i++) {
            temps[i] = followers.get(i).getDeviceTemp().getValueAsDouble();
        }
        return temps;
    }

    /**
     * Hot-reload Slot 0 PID + feedforward gains on the running motor. Used by
     * {@link frc.lib.catalyst.util.TunableGains} for live tuning; teams normally
     * don't call this directly. Preserves the gravity model from initial config.
     */
    public void updateSlot0(double kP, double kI, double kD,
                            double kS, double kV, double kA, double kG) {
        Slot0Configs slot = new Slot0Configs();
        slot.kP = kP; slot.kI = kI; slot.kD = kD;
        slot.kS = kS; slot.kV = kV; slot.kA = kA; slot.kG = kG;
        slot.GravityType = gravityType;
        motor.getConfigurator().apply(slot);
    }

    /**
     * Change the supply current limit at runtime (e.g. state-based power
     * budgeting). Re-sends the whole {@link CurrentLimitsConfigs} group so the
     * stator limit and both enables are preserved.
     */
    public void setSupplyCurrentLimit(double amps) {
        this.supplyCurrentLimit = amps;
        applyCurrentLimits();
    }

    /** Change the stator current limit at runtime. Preserves the supply limit. */
    public void setStatorCurrentLimit(double amps) {
        this.statorCurrentLimit = amps;
        applyCurrentLimits();
    }

    /** Change both current limits at runtime in a single config apply. */
    public void setCurrentLimits(double supplyAmps, double statorAmps) {
        this.supplyCurrentLimit = supplyAmps;
        this.statorCurrentLimit = statorAmps;
        applyCurrentLimits();
    }

    private void applyCurrentLimits() {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = supplyCurrentLimit;
        limits.StatorCurrentLimitEnable = true;
        limits.StatorCurrentLimit = statorCurrentLimit;
        motor.getConfigurator().apply(limits);
    }

    /**
     * Hot-reload Slot 1 PID + feedforward gains. Slot 1 is used by Catalyst's
     * differential mechanisms (e.g. {@link frc.lib.catalyst.mechanisms.DifferentialWristMechanism})
     * for the differential-axis controller while Slot 0 handles the average axis.
     */
    public void updateSlot1(double kP, double kI, double kD,
                            double kS, double kV, double kA) {
        Slot1Configs slot = new Slot1Configs();
        slot.kP = kP; slot.kI = kI; slot.kD = kD;
        slot.kS = kS; slot.kV = kV; slot.kA = kA;
        motor.getConfigurator().apply(slot);
    }

    /**
     * Hot-reload Motion Magic profile constants (cruise velocity, acceleration,
     * jerk) on the running motor. Used by
     * {@link frc.lib.catalyst.util.TunableGains} for live tuning.
     */
    public void updateMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = cruiseVelocity;
        mm.MotionMagicAcceleration = acceleration;
        mm.MotionMagicJerk = jerk;
        motor.getConfigurator().apply(mm);
    }

    /** Update telemetry. Call from subsystem periodic(). Publishes through CatalystLog. */
    public void updateTelemetry() {
        CatalystLog.log(name + "/Position", getPosition());
        CatalystLog.log(name + "/Velocity", getVelocity());
        CatalystLog.log(name + "/Voltage", getAppliedVoltage());
        CatalystLog.log(name + "/Current", getStatorCurrent());
        CatalystLog.log(name + "/Temperature", getTemperature());
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

    // --- SysId ---
    //
    // Phoenix 6 already logs every TalonFX signal through SignalLogger when
    // it's started. We hand WPILib a SysIdRoutine that drives the master
    // motor with VoltageOut and let SignalLogger take care of the data
    // capture — no per-signal logging boilerplate in the mechanism code.
    //
    // Teams need to call SignalLogger.start() once in Robot.robotInit()
    // for the WPILib SysId tooling to find the data.

    /** Build a {@link SysIdRoutine} for this motor with the given subsystem requirement. */
    public SysIdRoutine sysIdRoutine(SubsystemBase requirement) {
        return sysIdRoutine(requirement, defaultSysIdConfig());
    }

    /** Build a {@link SysIdRoutine} for this motor with a custom config. */
    public SysIdRoutine sysIdRoutine(SubsystemBase requirement, SysIdRoutine.Config config) {
        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (voltage) -> setVoltage(voltage.in(Volts)),
                        null,            // Phoenix SignalLogger captures signals; no per-signal callback needed.
                        requirement,
                        name));
    }

    /** Default routine: 1 V/s ramp, 4 V step, 10 s timeout. */
    public static SysIdRoutine.Config defaultSysIdConfig() {
        return new SysIdRoutine.Config(
                null,            // 1 V/s default
                Volts.of(4),     // dynamic step voltage
                null,            // 10 s default timeout
                (state) -> SignalLogger.writeString("SysIdState", state.toString()));
    }

    /** Quasistatic SysId command — slow ramp, characterises kS / kV. */
    public Command sysIdQuasistatic(SubsystemBase requirement, Direction dir) {
        return sysIdRoutine(requirement).quasistatic(dir);
    }

    /** Dynamic SysId command — step input, characterises kA. */
    public Command sysIdDynamic(SubsystemBase requirement, Direction dir) {
        return sysIdRoutine(requirement).dynamic(dir);
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
        private boolean optimizeCanBus = false;
        private double canUpdateHz = 50.0;
        private double currentLimit = 40;
        private double statorCurrentLimit = 80;
        // Phoenix's own defaults (+/-800 A are the config maximums; Phoenix ships narrower).
        // Kept at Phoenix defaults unless torqueCurrentLimits() is called.
        private double peakForwardTorqueCurrent = 800;
        private double peakReverseTorqueCurrent = -800;
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
        private final List<FollowerSpec> followerSpecs = new ArrayList<>();
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

        /**
         * Reduce CAN-bus load: raise only the status signals Catalyst reads
         * (position, velocity, voltage, currents, temp, faults) to
         * {@code updateHz}, then stop every other signal on this device from
         * broadcasting. Worth enabling on high-device-count robots. Off by
         * default — when on, don't read raw TalonFX signals Catalyst doesn't,
         * or raise them yourself first.
         *
         * @param updateHz status-signal rate (50 Hz is a good default)
         */
        public Builder optimizeCanBus(double updateHz) {
            this.optimizeCanBus = true;
            this.canUpdateHz = updateHz;
            return this;
        }

        /** Optimize CAN bus utilization at the default 50 Hz. */
        public Builder optimizeCanBus() { return optimizeCanBus(50.0); }
        public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
        public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }

        /**
         * Peak torque-current for *TorqueCurrentFOC requests (Pro-licensed devices). Phoenix's own
         * default is the full +/-800 A range; shooters commonly want something like +200/-200 so a
         * ball being fed in cannot command an unbounded correction.
         *
         * @param peakForwardAmps positive amps
         * @param peakReverseAmps negative amps
         */
        public Builder torqueCurrentLimits(double peakForwardAmps, double peakReverseAmps) {
            this.peakForwardTorqueCurrent = peakForwardAmps;
            this.peakReverseTorqueCurrent = peakReverseAmps;
            return this;
        }
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

        /**
         * Add a follower motor that mirrors this motor's output.
         * <p>This method is <b>additive</b>: call it once per follower. Adding two
         * followers, for example, creates two follower TalonFX instances pointed
         * at the leader CAN ID.
         *
         * @param canId the follower's CAN ID
         * @param oppose if true, the follower runs in the opposite direction (mirrored)
         */
        public Builder withFollower(int canId, boolean oppose) {
            this.followerSpecs.add(new FollowerSpec(canId, oppose));
            return this;
        }

        /** Add a follower that runs in the same direction as the leader. */
        public Builder withFollower(int canId) {
            return withFollower(canId, false);
        }

        /**
         * Add multiple followers in a single call.
         * @param specs follower specifications in any order
         */
        public Builder withFollowers(FollowerSpec... specs) {
            for (FollowerSpec s : specs) this.followerSpecs.add(s);
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
