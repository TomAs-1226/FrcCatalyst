package frc.lib.catalyst.mechanisms;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.io.ClawMechanismInputs;

/**
 * Motor-driven claw / gripper mechanism.
 *
 * <p>Designed for the common FRC pattern: one motor (with optional follower)
 * that closes onto a game piece, then holds it with a low passive grip voltage.
 * Stall-current detection auto-transitions from {@code CLOSING} to
 * {@code HOLDING} once a piece is grabbed, so the motor doesn't cook itself
 * trying to squeeze harder.
 *
 * <p>For pneumatic claws, use {@code PneumaticMechanism} instead.
 *
 * <p>Example usage:
 * <pre>{@code
 * ClawMechanism claw = new ClawMechanism(
 *     ClawMechanism.Config.builder()
 *         .name("Claw")
 *         .motor(30)
 *         .closeVoltage(6.0)
 *         .openVoltage(-4.0)
 *         .holdVoltage(1.5)
 *         .stallDetection(25.0, 0.2)
 *         .currentLimit(40)
 *         .build());
 *
 * controller.a().onTrue(claw.closeUntilGripped());
 * controller.b().onTrue(claw.open());
 * }</pre>
 */
public class ClawMechanism extends CatalystMechanism {

    private final Config config;
    private final CatalystMotor motor;
    private final DigitalInput beamBreak;

    private final Timer stallTimer = new Timer();
    private boolean stallTimerStarted = false;
    private boolean hasPiece = false;
    private String gripState = "IDLE";

    private final ClawMechanismInputs inputs = new ClawMechanismInputs();

    public ClawMechanism(Config config) {
        super(config.name);
        this.config = config;

        CatalystMotor.Builder motorBuilder = CatalystMotor.builder(config.motorCanId)
                .name(config.name + "Motor")
                .canBus(config.canBus)
                .inverted(config.inverted)
                .brakeMode(config.brakeMode)
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit);

        if (config.followerCanId >= 0) {
            motorBuilder.withFollower(config.followerCanId, config.followerOppose);
        }

        this.motor = motorBuilder.build();

        this.beamBreak = config.beamBreakPort >= 0 ? new DigitalInput(config.beamBreakPort) : null;
    }

    // --- Getters ---

    /** True when the claw believes it is currently holding a game piece. */
    public boolean hasPiece() {
        if (beamBreak != null) return !beamBreak.get();
        return hasPiece;
    }

    /** Trigger that fires whenever {@link #hasPiece()} is true. */
    public Trigger hasPieceTrigger() {
        return new Trigger(this::hasPiece);
    }

    /** Current grip state — one of {@code OPEN}, {@code CLOSING}, {@code HOLDING}, {@code OPENING}, {@code IDLE}. */
    public String getGripState() {
        return gripState;
    }

    /** Current stator current draw in amps. */
    public double getCurrent() {
        return motor.getStatorCurrent();
    }

    // --- Command Factories ---

    /**
     * Close the claw and run until commanded otherwise.
     * Does not auto-stop on stall — pair with {@link #closeUntilGripped()}
     * if you want hands-off piece detection.
     */
    public Command close() {
        return run(() -> {
            motor.setVoltage(config.closeVoltage);
            gripState = "CLOSING";
            setState("Closing");
        }).finallyDo(() -> {
            motor.stop();
            gripState = "IDLE";
            setState("Idle");
        }).withName(name + ".Close");
    }

    /**
     * Close the claw until a piece is gripped (stall current or beam break),
     * then transition to a low passive hold voltage and continue holding
     * until the command is interrupted.
     */
    public Command closeUntilGripped() {
        return run(() -> {
            if (hasPiece()) {
                motor.setVoltage(config.holdVoltage);
                gripState = "HOLDING";
                setState("Holding");
            } else {
                motor.setVoltage(config.closeVoltage);
                gripState = "CLOSING";
                updateStallDetection();
                setState("Closing");
            }
        }).finallyDo(() -> {
            motor.stop();
            gripState = "IDLE";
            stallTimerStarted = false;
            setState("Idle");
        }).withName(name + ".CloseUntilGripped");
    }

    /**
     * Hold the current grip at low passive voltage. Use this as the default
     * command after {@link #closeUntilGripped()} succeeds and the scheduler
     * has moved on, so the piece doesn't drop.
     */
    public Command hold() {
        return run(() -> {
            motor.setVoltage(config.holdVoltage);
            gripState = "HOLDING";
            setState("Holding");
        }).finallyDo(() -> {
            motor.stop();
            gripState = "IDLE";
            setState("Idle");
        }).withName(name + ".Hold");
    }

    /** Open the claw at the configured open voltage. Clears the has-piece state. */
    public Command open() {
        return run(() -> {
            motor.setVoltage(config.openVoltage);
            hasPiece = false;
            stallTimerStarted = false;
            gripState = "OPENING";
            setState("Opening");
        }).finallyDo(() -> {
            motor.stop();
            gripState = "OPEN";
            setState("Open");
        }).withName(name + ".Open");
    }

    /** Run the claw at a custom voltage. Does not touch grip-state tracking. */
    public Command runAtVoltage(double volts) {
        return run(() -> {
            motor.setVoltage(volts);
            setState("Voltage " + String.format("%.1fV", volts));
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".RunAt(" + String.format("%.1fV", volts) + ")");
    }

    /** Manually reset the {@code hasPiece} state — useful after a drop / fault. */
    public Command resetPieceDetection() {
        return runOnce(() -> {
            hasPiece = false;
            stallTimerStarted = false;
            setState("Reset");
        }).withName(name + ".ResetDetection");
    }

    // --- Stall detection ---

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
        gripState = "IDLE";
        setState("Stopped");
    }

    @Override
    protected void updateTelemetry() {
        motor.updateTelemetry();

        inputs.dutyCycle = motor.getAppliedVoltage() / 12.0;
        inputs.statorCurrentAmps = motor.getStatorCurrent();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent();
        inputs.appliedVolts = motor.getAppliedVoltage();
        inputs.temperatureC = motor.getTemperature();
        inputs.followerStatorCurrentAmps = motor.getFollowerStatorCurrents();
        inputs.followerTemperatureC = motor.getFollowerTemperatures();
        inputs.gripState = gripState;
        inputs.hasPiece = hasPiece();
        inputs.beamBreakBroken = beamBreak != null && !beamBreak.get();
        inputs.stallDetected = stallTimerStarted;
        processInputs(inputs);

        log("GripState", inputs.gripState);
        log("HasPiece", inputs.hasPiece);
        log("CurrentAmps", inputs.statorCurrentAmps);
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
        final int followerCanId;
        final boolean followerOppose;
        final String canBus;
        final boolean inverted;
        final boolean brakeMode;
        final double currentLimit;
        final double statorCurrentLimit;
        final double closeVoltage;
        final double openVoltage;
        final double holdVoltage;
        final double stallCurrentThreshold;
        final double stallTimeThreshold;
        final int beamBreakPort;

        private Config(Builder b) {
            this.name = b.name;
            this.motorCanId = b.motorCanId;
            this.followerCanId = b.followerCanId;
            this.followerOppose = b.followerOppose;
            this.canBus = b.canBus;
            this.inverted = b.inverted;
            this.brakeMode = b.brakeMode;
            this.currentLimit = b.currentLimit;
            this.statorCurrentLimit = b.statorCurrentLimit;
            this.closeVoltage = b.closeVoltage;
            this.openVoltage = b.openVoltage;
            this.holdVoltage = b.holdVoltage;
            this.stallCurrentThreshold = b.stallCurrentThreshold;
            this.stallTimeThreshold = b.stallTimeThreshold;
            this.beamBreakPort = b.beamBreakPort;
        }

        public static Builder builder() { return new Builder(); }

        public static class Builder {
            private String name = "ClawMechanism";
            private int motorCanId = 0;
            private int followerCanId = -1;
            private boolean followerOppose = false;
            private String canBus = "";
            private boolean inverted = false;
            private boolean brakeMode = true;
            private double currentLimit = 30;
            private double statorCurrentLimit = 60;
            private double closeVoltage = 6.0;
            private double openVoltage = -4.0;
            private double holdVoltage = 1.5;
            private double stallCurrentThreshold = -1;
            private double stallTimeThreshold = 0.2;
            private int beamBreakPort = -1;

            public Builder name(String name) { this.name = name; return this; }
            public Builder motor(int canId) { this.motorCanId = canId; return this; }

            /** Add a single follower motor sharing the same control output. */
            public Builder follower(int canId, boolean oppose) {
                this.followerCanId = canId;
                this.followerOppose = oppose;
                return this;
            }

            public Builder canBus(String canBus) { this.canBus = canBus; return this; }
            public Builder inverted(boolean inverted) { this.inverted = inverted; return this; }
            public Builder brakeMode(boolean brake) { this.brakeMode = brake; return this; }
            public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
            public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }

            /** Voltage applied while actively closing onto a piece. */
            public Builder closeVoltage(double volts) { this.closeVoltage = volts; return this; }

            /** Voltage applied while opening (should be opposite sign of close voltage). */
            public Builder openVoltage(double volts) { this.openVoltage = volts; return this; }

            /** Passive hold voltage applied once a piece has been gripped. Keep this small to avoid cooking the motor. */
            public Builder holdVoltage(double volts) { this.holdVoltage = volts; return this; }

            /**
             * Enable stall-current grip detection.
             * @param currentAmps current threshold in amps
             * @param timeSeconds how long current must stay above threshold before {@code hasPiece} flips true
             */
            public Builder stallDetection(double currentAmps, double timeSeconds) {
                this.stallCurrentThreshold = currentAmps;
                this.stallTimeThreshold = timeSeconds;
                return this;
            }

            /** Add a beam-break sensor as an alternative / additional piece-detection signal. */
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
