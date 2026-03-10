package frc.lib.catalyst.mechanisms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;

import java.util.function.DoubleSupplier;

/**
 * Generic winch mechanism. Use for climbers, deployment mechanisms,
 * or any mechanism that extends/retracts using a spool or winch.
 *
 * <p>Provides simple extend/retract commands with position limits
 * and current monitoring for safety.
 *
 * <p>Example usage:
 * <pre>{@code
 * WinchMechanism climber = new WinchMechanism(
 *     WinchMechanism.Config.builder()
 *         .name("Climber")
 *         .motor(25)
 *         .gearRatio(25.0)
 *         .spoolRadius(0.02)
 *         .range(0.0, 0.6)
 *         .extendSpeed(0.8)
 *         .retractSpeed(-1.0)
 *         .currentLimit(80)
 *         .build());
 * }</pre>
 */
public class WinchMechanism extends CatalystMechanism {

    private final Config config;
    private final CatalystMotor motor;
    private final CatalystMotor secondMotor;

    public WinchMechanism(Config config) {
        super(config.name);
        this.config = config;

        CatalystMotor.Builder motorBuilder = CatalystMotor.builder(config.motorCanId)
                .name(config.name + "Motor")
                .canBus(config.canBus)
                .inverted(config.inverted)
                .brakeMode(true) // always brake for safety
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit)
                .gearRatio(config.gearRatio);

        // Soft limits in mechanism rotations
        if (config.spoolRadius > 0) {
            double minRot = config.minPosition / (2.0 * Math.PI * config.spoolRadius);
            double maxRot = config.maxPosition / (2.0 * Math.PI * config.spoolRadius);
            motorBuilder.softLimits(minRot, maxRot);
        }

        this.motor = motorBuilder.build();

        // Second motor (for dual climber arms)
        if (config.secondMotorCanId >= 0) {
            this.secondMotor = CatalystMotor.builder(config.secondMotorCanId)
                    .name(config.name + "Motor2")
                    .canBus(config.canBus)
                    .inverted(config.secondInverted)
                    .brakeMode(true)
                    .currentLimit(config.currentLimit)
                    .statorCurrentLimit(config.statorCurrentLimit)
                    .gearRatio(config.gearRatio)
                    .build();
        } else {
            this.secondMotor = null;
        }
    }

    // --- Conversions ---

    private double getPositionMeters() {
        if (config.spoolRadius <= 0) return motor.getPosition();
        return motor.getPosition() * (2.0 * Math.PI * config.spoolRadius);
    }

    // --- Getters ---

    /** Get current position in meters (if spool radius set) or rotations. */
    public double getPosition() {
        return getPositionMeters();
    }

    /** Get current draw in amps. */
    public double getCurrent() {
        return motor.getStatorCurrent();
    }

    /** Check if fully extended. */
    public boolean isFullyExtended() {
        return getPosition() >= config.maxPosition - 0.01;
    }

    /** Check if fully retracted. */
    public boolean isFullyRetracted() {
        return getPosition() <= config.minPosition + 0.01;
    }

    // --- Triggers ---

    public Trigger fullyExtendedTrigger() {
        return new Trigger(this::isFullyExtended);
    }

    public Trigger fullyRetractedTrigger() {
        return new Trigger(this::isFullyRetracted);
    }

    // --- Command Factories ---

    /** Command to extend the mechanism at the configured extend speed. */
    public Command extend() {
        return run(() -> {
            setMotors(config.extendSpeed);
            setState("Extending");
        }).finallyDo(() -> {
            stopMotors();
            setState("Idle");
        }).withName(name + ".Extend");
    }

    /** Command to retract the mechanism at the configured retract speed. */
    public Command retract() {
        return run(() -> {
            setMotors(config.retractSpeed);
            setState("Retracting");
        }).finallyDo(() -> {
            stopMotors();
            setState("Idle");
        }).withName(name + ".Retract");
    }

    /** Command to run at a custom speed [-1, 1]. */
    public Command runAtSpeed(double speed) {
        return run(() -> {
            setMotors(speed);
            setState("Running " + String.format("%.0f%%", speed * 100));
        }).finallyDo(() -> {
            stopMotors();
            setState("Idle");
        }).withName(name + ".RunAt(" + String.format("%.0f%%", speed * 100) + ")");
    }

    /** Command to control with a joystick axis. */
    public Command manualControl(DoubleSupplier speedSupplier) {
        return run(() -> {
            double speed = speedSupplier.getAsDouble();
            if (Math.abs(speed) < 0.05) {
                stopMotors();
                setState("Hold");
            } else {
                setMotors(speed);
                setState("Manual " + String.format("%.0f%%", speed * 100));
            }
        }).finallyDo(() -> {
            stopMotors();
            setState("Idle");
        }).withName(name + ".ManualControl");
    }

    /** Command to zero the encoder. */
    public Command zero() {
        return runOnce(() -> {
            motor.zeroEncoder();
            if (secondMotor != null) secondMotor.zeroEncoder();
            setState("Zeroed");
        }).withName(name + ".Zero");
    }

    // --- Helpers ---

    private void setMotors(double speed) {
        motor.setPercent(speed);
        if (secondMotor != null) secondMotor.setPercent(speed);
    }

    private void stopMotors() {
        motor.stop();
        if (secondMotor != null) secondMotor.stop();
    }

    // --- Internals ---

    @Override
    protected void stop() {
        stopMotors();
        setState("Stopped");
    }

    @Override
    protected void updateTelemetry() {
        motor.updateTelemetry();
        log("Position", getPosition());
        log("CurrentAmps", getCurrent());
        log("FullyExtended", isFullyExtended());
        log("FullyRetracted", isFullyRetracted());
        if (secondMotor != null) {
            secondMotor.updateTelemetry();
        }
    }

    public CatalystMotor getMotor() { return motor; }
    public CatalystMotor getSecondMotor() { return secondMotor; }

    // ===========================================
    //                  CONFIG
    // ===========================================

    public static class Config {
        final String name;
        final int motorCanId;
        final int secondMotorCanId;
        final String canBus;
        final boolean inverted;
        final boolean secondInverted;
        final double gearRatio;
        final double spoolRadius;
        final double minPosition;
        final double maxPosition;
        final double currentLimit;
        final double statorCurrentLimit;
        final double extendSpeed;
        final double retractSpeed;

        private Config(Builder b) {
            this.name = b.name;
            this.motorCanId = b.motorCanId;
            this.secondMotorCanId = b.secondMotorCanId;
            this.canBus = b.canBus;
            this.inverted = b.inverted;
            this.secondInverted = b.secondInverted;
            this.gearRatio = b.gearRatio;
            this.spoolRadius = b.spoolRadius;
            this.minPosition = b.minPosition;
            this.maxPosition = b.maxPosition;
            this.currentLimit = b.currentLimit;
            this.statorCurrentLimit = b.statorCurrentLimit;
            this.extendSpeed = b.extendSpeed;
            this.retractSpeed = b.retractSpeed;
        }

        public static Builder builder() {
            return new Builder();
        }

        public static class Builder {
            private String name = "WinchMechanism";
            private int motorCanId = 0;
            private int secondMotorCanId = -1;
            private String canBus = "";
            private boolean inverted = false;
            private boolean secondInverted = false;
            private double gearRatio = 25.0;
            private double spoolRadius = 0.02; // meters
            private double minPosition = 0;
            private double maxPosition = 0.6;
            private double currentLimit = 80;
            private double statorCurrentLimit = 120;
            private double extendSpeed = 0.8;
            private double retractSpeed = -1.0;

            public Builder name(String name) { this.name = name; return this; }
            public Builder motor(int canId) { this.motorCanId = canId; return this; }

            /** Add a second motor (independent, for dual-arm climbers). */
            public Builder secondMotor(int canId) { this.secondMotorCanId = canId; return this; }

            public Builder canBus(String canBus) { this.canBus = canBus; return this; }
            public Builder inverted(boolean inverted) { this.inverted = inverted; return this; }
            public Builder secondInverted(boolean inverted) { this.secondInverted = inverted; return this; }
            public Builder gearRatio(double ratio) { this.gearRatio = ratio; return this; }

            /** Spool/drum radius in meters. Used for position <-> rotation conversion. */
            public Builder spoolRadius(double meters) { this.spoolRadius = meters; return this; }

            /** Min and max position in meters. */
            public Builder range(double minMeters, double maxMeters) {
                this.minPosition = minMeters;
                this.maxPosition = maxMeters;
                return this;
            }

            public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
            public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }

            /** Extend speed as duty cycle [0, 1]. */
            public Builder extendSpeed(double speed) { this.extendSpeed = speed; return this; }

            /** Retract speed as duty cycle [-1, 0]. */
            public Builder retractSpeed(double speed) { this.retractSpeed = speed; return this; }

            public Config build() {
                if (motorCanId == 0) {
                    throw new IllegalStateException("Motor CAN ID must be set");
                }
                return new Config(this);
            }
        }
    }
}
