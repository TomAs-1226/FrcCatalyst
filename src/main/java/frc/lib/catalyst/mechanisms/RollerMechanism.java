package frc.lib.catalyst.mechanisms;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;

/**
 * Generic roller mechanism. Use for intakes, conveyors, indexers,
 * or any mechanism that spins rollers at a set speed.
 *
 * <p>Supports game piece detection via:
 * <ul>
 *   <li>Stall current detection (current spike = game piece acquired)</li>
 *   <li>Beam break sensor (DIO)</li>
 * </ul>
 *
 * <p>Example usage:
 * <pre>{@code
 * RollerMechanism intake = new RollerMechanism(
 *     RollerMechanism.Config.builder()
 *         .name("Intake")
 *         .motor(16)
 *         .intakeSpeed(0.8)
 *         .ejectSpeed(-0.6)
 *         .currentLimit(30)
 *         .stallDetection(25, 0.2)
 *         .build());
 * }</pre>
 */
public class RollerMechanism extends CatalystMechanism {

    private final Config config;
    private final CatalystMotor motor;
    private final DigitalInput beamBreak;

    // Stall detection state
    private final Timer stallTimer = new Timer();
    private boolean hasPiece = false;
    private boolean stallTimerStarted = false;

    public RollerMechanism(Config config) {
        super(config.name);
        this.config = config;

        this.motor = CatalystMotor.builder(config.motorCanId)
                .name(config.name + "Motor")
                .canBus(config.canBus)
                .inverted(config.inverted)
                .brakeMode(config.brakeMode)
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit)
                .build();

        // Beam break sensor
        if (config.beamBreakPort >= 0) {
            beamBreak = new DigitalInput(config.beamBreakPort);
        } else {
            beamBreak = null;
        }
    }

    // --- Getters ---

    /** Check if a game piece is detected (via beam break or stall detection). */
    public boolean hasPiece() {
        if (beamBreak != null) {
            return !beamBreak.get(); // beam break is normally open, false = broken = has piece
        }
        return hasPiece;
    }

    /** Trigger that fires when a game piece is detected. */
    public Trigger hasPieceTrigger() {
        return new Trigger(this::hasPiece);
    }

    /** Get current motor speed as a fraction [-1, 1]. */
    public double getSpeed() {
        return motor.getAppliedVoltage() / 12.0;
    }

    /** Get current draw in amps. */
    public double getCurrent() {
        return motor.getStatorCurrent();
    }

    // --- Command Factories ---

    /**
     * Command to run intake rollers.
     * If stall detection or beam break is configured, ends when game piece is detected.
     * Otherwise runs indefinitely (use whileTrue).
     */
    public Command intake() {
        return run(() -> {
            motor.setPercent(config.intakeSpeed);
            updateStallDetection();
            setState("Intaking");
        }).until(this::hasPiece)
                .finallyDo(() -> {
                    motor.stop();
                    setState("Idle");
                })
                .withName(name + ".Intake");
    }

    /**
     * Command to intake without auto-stopping.
     * Useful when you want manual control over when to stop.
     */
    public Command intakeContinuous() {
        return run(() -> {
            motor.setPercent(config.intakeSpeed);
            setState("Intaking");
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".IntakeContinuous");
    }

    /** Command to eject game pieces. */
    public Command eject() {
        return run(() -> {
            motor.setPercent(config.ejectSpeed);
            hasPiece = false;
            setState("Ejecting");
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".Eject");
    }

    /** Command to run rollers at a custom speed [-1, 1]. */
    public Command runAtSpeed(double speed) {
        return run(() -> {
            motor.setPercent(speed);
            setState("Running " + String.format("%.0f%%", speed * 100));
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".RunAt(" + String.format("%.0f%%", speed * 100) + ")");
    }

    /** Command to run rollers at a custom voltage [-12, 12]. */
    public Command runAtVoltage(double volts) {
        return run(() -> {
            motor.setVoltage(volts);
            setState("Running " + String.format("%.1fV", volts));
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".RunAt(" + String.format("%.1fV", volts) + ")");
    }

    /** Reset the has-piece state. */
    public Command resetPieceDetection() {
        return runOnce(() -> {
            hasPiece = false;
            stallTimerStarted = false;
            setState("Reset");
        }).withName(name + ".ResetDetection");
    }

    // --- Stall Detection ---

    private void updateStallDetection() {
        if (config.stallCurrentThreshold <= 0) return;

        double current = motor.getStatorCurrent();
        if (current >= config.stallCurrentThreshold) {
            if (!stallTimerStarted) {
                stallTimer.restart();
                stallTimerStarted = true;
            }
            if (stallTimer.get() >= config.stallTimeThreshold) {
                hasPiece = true;
            }
        } else {
            stallTimerStarted = false;
        }
    }

    // --- Internals ---

    @Override
    protected void stop() {
        motor.stop();
        setState("Stopped");
    }

    @Override
    protected void updateTelemetry() {
        motor.updateTelemetry();
        log("Speed", getSpeed());
        log("CurrentAmps", getCurrent());
        log("HasPiece", hasPiece());
    }

    /** Get the underlying motor for advanced use. */
    public CatalystMotor getMotor() {
        return motor;
    }

    // ===========================================
    //                  CONFIG
    // ===========================================

    public static class Config {
        final String name;
        final int motorCanId;
        final String canBus;
        final boolean inverted;
        final boolean brakeMode;
        final double currentLimit;
        final double statorCurrentLimit;
        final double intakeSpeed;
        final double ejectSpeed;
        final double stallCurrentThreshold;
        final double stallTimeThreshold;
        final int beamBreakPort;

        private Config(Builder b) {
            this.name = b.name;
            this.motorCanId = b.motorCanId;
            this.canBus = b.canBus;
            this.inverted = b.inverted;
            this.brakeMode = b.brakeMode;
            this.currentLimit = b.currentLimit;
            this.statorCurrentLimit = b.statorCurrentLimit;
            this.intakeSpeed = b.intakeSpeed;
            this.ejectSpeed = b.ejectSpeed;
            this.stallCurrentThreshold = b.stallCurrentThreshold;
            this.stallTimeThreshold = b.stallTimeThreshold;
            this.beamBreakPort = b.beamBreakPort;
        }

        public static Builder builder() {
            return new Builder();
        }

        public static class Builder {
            private String name = "RollerMechanism";
            private int motorCanId = 0;
            private String canBus = "";
            private boolean inverted = false;
            private boolean brakeMode = false;
            private double currentLimit = 30;
            private double statorCurrentLimit = 60;
            private double intakeSpeed = 0.8;
            private double ejectSpeed = -0.6;
            private double stallCurrentThreshold = -1; // disabled by default
            private double stallTimeThreshold = 0.2;
            private int beamBreakPort = -1; // disabled by default

            public Builder name(String name) { this.name = name; return this; }
            public Builder motor(int canId) { this.motorCanId = canId; return this; }
            public Builder canBus(String canBus) { this.canBus = canBus; return this; }
            public Builder inverted(boolean inverted) { this.inverted = inverted; return this; }
            public Builder brakeMode(boolean brakeMode) { this.brakeMode = brakeMode; return this; }
            public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
            public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }

            /** Intake speed as duty cycle [-1, 1]. Positive = intake direction. */
            public Builder intakeSpeed(double speed) { this.intakeSpeed = speed; return this; }

            /** Eject speed as duty cycle [-1, 1]. Should be opposite sign of intakeSpeed. */
            public Builder ejectSpeed(double speed) { this.ejectSpeed = speed; return this; }

            /**
             * Enable stall-based game piece detection.
             * @param currentAmps current threshold in amps
             * @param timeSeconds how long current must exceed threshold
             */
            public Builder stallDetection(double currentAmps, double timeSeconds) {
                this.stallCurrentThreshold = currentAmps;
                this.stallTimeThreshold = timeSeconds;
                return this;
            }

            /** Enable beam break sensor for game piece detection. */
            public Builder beamBreak(int dioPort) {
                this.beamBreakPort = dioPort;
                return this;
            }

            public Config build() {
                if (motorCanId == 0) {
                    throw new IllegalStateException("Motor CAN ID must be set");
                }
                return new Config(this);
            }
        }
    }
}
