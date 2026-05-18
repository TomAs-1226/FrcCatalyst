package frc.lib.catalyst.mechanisms;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.hardware.CatalystMotor.FollowerSpec;
import frc.lib.catalyst.io.ClawMechanismInputs;
import frc.lib.catalyst.util.HealthCheck;
import frc.lib.catalyst.util.HealthMonitor;

import java.util.ArrayList;
import java.util.List;

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

        for (FollowerSpec spec : config.followers) {
            motorBuilder.withFollower(spec.canId(), spec.oppose());
        }

        this.motor = motorBuilder.build();

        this.beamBreak = config.beamBreakPort >= 0 ? new DigitalInput(config.beamBreakPort) : null;

        HealthMonitor.standardMotorChecks(name, motor, config.statorCurrentLimit, 70);

        // Phoenix followers (no separate CatalystMotor wrapper) — register one
        // OverCurrent / HighTemp pair per follower so multi-motor claws each
        // get individual fault signals.
        final double warnAmps = config.statorCurrentLimit * 0.9;
        for (int i = 0; i < config.followers.size(); i++) {
            final int idx = i;
            final int canId = config.followers.get(i).canId();
            HealthCheck.builder(name, "OverCurrentFollower" + canId)
                    .severity(HealthCheck.Severity.WARN)
                    .description("Follower " + canId + " stator current near limit")
                    .when(() -> {
                        double[] cs = motor.getFollowerStatorCurrents();
                        return cs.length > idx && cs[idx] > warnAmps;
                    })
                    .detail(() -> {
                        double[] cs = motor.getFollowerStatorCurrents();
                        return cs.length > idx ? String.format("%.1f A", cs[idx]) : "";
                    })
                    .debounce(0.5)
                    .clearAfter(1.0)
                    .register();
            HealthCheck.builder(name, "HighTempFollower" + canId)
                    .severity(HealthCheck.Severity.WARN)
                    .description("Follower " + canId + " temperature high")
                    .when(() -> {
                        double[] ts = motor.getFollowerTemperatures();
                        return ts.length > idx && ts[idx] > 70;
                    })
                    .detail(() -> {
                        double[] ts = motor.getFollowerTemperatures();
                        return ts.length > idx ? String.format("%.0f C", ts[idx]) : "";
                    })
                    .debounce(1.0)
                    .clearAfter(5.0)
                    .register();
        }
    }

    // --- Getters ---

    /**
     * True when the claw believes it is currently holding a game piece.
     * Returns true if EITHER the beam-break is broken OR stall detection
     * has latched — so configuring both signals gives redundant detection.
     */
    public boolean hasPiece() {
        boolean beamBroken = beamBreak != null && !beamBreak.get();
        return beamBroken || hasPiece;
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

        HealthMonitor.getInstance().update();
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
        final List<FollowerSpec> followers;
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
            this.followers = List.copyOf(b.followers);
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
            private final List<FollowerSpec> followers = new ArrayList<>();
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

            /**
             * Add a follower motor that shares the same control output as the
             * primary. Call this multiple times to add several followers
             * (claws with two or three motors per side are common).
             *
             * @param canId  CAN ID of the follower TalonFX
             * @param oppose true if the follower spins the opposite direction
             *               of the primary (mirrored mounting)
             */
            public Builder follower(int canId, boolean oppose) {
                this.followers.add(new FollowerSpec(canId, oppose));
                return this;
            }

            /** Convenience: add a follower with {@code oppose = false}. */
            public Builder follower(int canId) {
                return follower(canId, false);
            }

            /** Add several followers in one call. */
            public Builder followers(FollowerSpec... specs) {
                for (FollowerSpec s : specs) this.followers.add(s);
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
