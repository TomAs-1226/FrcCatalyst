package frc.lib.catalyst.mechanisms;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.io.PneumaticMechanismInputs;
import frc.lib.catalyst.util.AlertManager;

/**
 * Pneumatic actuator mechanism — wraps a single or double solenoid as a
 * Catalyst mechanism with logging, command factories, and optional pressure
 * guarding.
 *
 * <p>Covers the FRC pneumatic actuation pattern that's currently absent from
 * Catalyst: climbers, hatch ejectors, gear shifters, brake cylinders, kicker
 * bars. For motor-driven grippers use {@link ClawMechanism} instead.
 *
 * <p>When a REVPH analog pressure sensor is wired, configure
 * {@code requirePressureAbove(psi)} on the builder and the mechanism will
 * refuse to actuate below that threshold (raising an {@link AlertManager}
 * warning instead) — a small safety net against firing a piston with no air.
 *
 * <p>Example usage:
 * <pre>{@code
 * PneumaticMechanism climbHook = new PneumaticMechanism(
 *     PneumaticMechanism.Config.builder()
 *         .name("ClimbHook")
 *         .doubleSolenoid(PneumaticsModuleType.REVPH, 0, 1)
 *         .compressor(PneumaticsModuleType.REVPH)
 *         .requirePressureAbove(40.0)
 *         .build());
 *
 * controller.x().onTrue(climbHook.extend());
 * controller.y().onTrue(climbHook.retract());
 * }</pre>
 */
public class PneumaticMechanism extends CatalystMechanism {

    /** Logical state of a pneumatic actuator. */
    public enum State { FORWARD, REVERSE, OFF }

    private final Config config;
    private final DoubleSolenoid doubleSolenoid;
    private final Solenoid singleSolenoid;
    private final Compressor compressor;

    private State state = State.OFF;
    private double lastTransitionTimestamp = 0.0;
    private long transitionCount = 0L;

    private final PneumaticMechanismInputs inputs = new PneumaticMechanismInputs();

    public PneumaticMechanism(Config config) {
        super(config.name);
        this.config = config;

        if (config.isDouble) {
            this.doubleSolenoid = new DoubleSolenoid(
                    config.moduleType,
                    config.forwardChannel,
                    config.reverseChannel);
            this.singleSolenoid = null;
        } else {
            this.doubleSolenoid = null;
            this.singleSolenoid = new Solenoid(config.moduleType, config.forwardChannel);
        }

        this.compressor = config.attachCompressor ? new Compressor(config.moduleType) : null;
    }

    // --- Getters ---

    /** Current commanded state. */
    public State getState() {
        return state;
    }

    /** True when commanded forward / extended. */
    public boolean isForward() {
        return state == State.FORWARD;
    }

    /** True when commanded reverse (double solenoid only — always false for single). */
    public boolean isReverse() {
        return state == State.REVERSE;
    }

    /** Trigger for {@link #isForward()}. */
    public Trigger forwardTrigger() {
        return new Trigger(this::isForward);
    }

    /** Trigger for {@link #isReverse()}. */
    public Trigger reverseTrigger() {
        return new Trigger(this::isReverse);
    }

    /**
     * Current analog pressure in psi, or {@code -1} when no compressor was
     * attached. REVPH only — CTRE PCM has no analog sensor channel.
     */
    public double getPressure() {
        if (compressor == null) return -1.0;
        return compressor.getPressure();
    }

    // --- Command Factories ---

    /** Extend the actuator (drive solenoid forward). */
    public Command extend() {
        return runOnce(() -> applyState(State.FORWARD))
                .withName(name + ".Extend");
    }

    /**
     * Retract the actuator. On a double solenoid this commands the reverse
     * channel; on a single solenoid this de-energizes the coil.
     */
    public Command retract() {
        return runOnce(() -> applyState(config.isDouble ? State.REVERSE : State.OFF))
                .withName(name + ".Retract");
    }

    /** De-energize the solenoid. */
    public Command off() {
        return runOnce(() -> applyState(State.OFF))
                .withName(name + ".Off");
    }

    /** Flip between forward and reverse / off based on current state. */
    public Command toggle() {
        return runOnce(() -> {
            State target = isForward()
                    ? (config.isDouble ? State.REVERSE : State.OFF)
                    : State.FORWARD;
            applyState(target);
        }).withName(name + ".Toggle");
    }

    /**
     * Pulse the actuator forward for {@code durationSeconds}, then return to
     * the previous state. Useful for kickers and ejection mechanisms.
     */
    public Command pulse(double durationSeconds) {
        final Timer timer = new Timer();
        final State[] previous = new State[] { State.OFF };
        return run(() -> {
            // body runs every loop; nothing to do once commanded
        })
                .beforeStarting(() -> {
                    previous[0] = state;
                    applyState(State.FORWARD);
                    timer.restart();
                })
                .until(() -> timer.hasElapsed(durationSeconds))
                .finallyDo(() -> applyState(previous[0]))
                .withName(name + ".Pulse(" + String.format("%.2fs", durationSeconds) + ")");
    }

    // --- Internals ---

    private void applyState(State target) {
        if (target == State.FORWARD && config.minPressurePSI > 0) {
            double psi = getPressure();
            if (psi >= 0 && psi < config.minPressurePSI) {
                AlertManager.getInstance().warning(name,
                        String.format("Refusing to actuate: pressure %.1f psi < required %.1f psi",
                                psi, config.minPressurePSI));
                return;
            }
        }
        if (state != target) {
            lastTransitionTimestamp = Timer.getFPGATimestamp();
            transitionCount++;
        }
        state = target;
        applyHardware();
        setState(target.name());
    }

    private void applyHardware() {
        if (config.isDouble) {
            switch (state) {
                case FORWARD -> doubleSolenoid.set(DoubleSolenoid.Value.kForward);
                case REVERSE -> doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
                case OFF -> doubleSolenoid.set(DoubleSolenoid.Value.kOff);
            }
        } else {
            singleSolenoid.set(state == State.FORWARD);
        }
    }

    @Override
    protected void stop() {
        applyState(State.OFF);
    }

    @Override
    protected void updateTelemetry() {
        inputs.state = state.name();
        inputs.forwardCommanded = state == State.FORWARD;
        inputs.reverseCommanded = state == State.REVERSE;
        inputs.pressurePSI = getPressure();
        inputs.lastTransitionTimestamp = lastTransitionTimestamp;
        inputs.transitionCount = transitionCount;
        processInputs(inputs);

        log("State", inputs.state);
        log("PressurePSI", inputs.pressurePSI);
        log("TransitionCount", inputs.transitionCount);
    }

    /** Get the underlying DoubleSolenoid for advanced use ({@code null} for single-solenoid configs). */
    public DoubleSolenoid getDoubleSolenoid() { return doubleSolenoid; }

    /** Get the underlying Solenoid for advanced use ({@code null} for double-solenoid configs). */
    public Solenoid getSolenoid() { return singleSolenoid; }

    // ===========================================
    //                  CONFIG
    // ===========================================

    public static class Config {
        final String name;
        final PneumaticsModuleType moduleType;
        final int forwardChannel;
        final int reverseChannel;
        final boolean isDouble;
        final boolean attachCompressor;
        final double minPressurePSI;

        private Config(Builder b) {
            this.name = b.name;
            this.moduleType = b.moduleType;
            this.forwardChannel = b.forwardChannel;
            this.reverseChannel = b.reverseChannel;
            this.isDouble = b.isDouble;
            this.attachCompressor = b.attachCompressor;
            this.minPressurePSI = b.minPressurePSI;
        }

        public static Builder builder() { return new Builder(); }

        public static class Builder {
            private String name = "PneumaticMechanism";
            private PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
            private int forwardChannel = -1;
            private int reverseChannel = -1;
            private boolean isDouble = false;
            private boolean attachCompressor = false;
            private double minPressurePSI = -1;

            public Builder name(String name) { this.name = name; return this; }

            /**
             * Configure as a double solenoid with explicit forward and reverse channels.
             * Use this for FRC pistons that need both directions actively driven.
             */
            public Builder doubleSolenoid(PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
                this.moduleType = moduleType;
                this.forwardChannel = forwardChannel;
                this.reverseChannel = reverseChannel;
                this.isDouble = true;
                return this;
            }

            /**
             * Configure as a single solenoid (one channel). The piston returns
             * by spring / pressure when the coil de-energizes.
             */
            public Builder singleSolenoid(PneumaticsModuleType moduleType, int channel) {
                this.moduleType = moduleType;
                this.forwardChannel = channel;
                this.reverseChannel = -1;
                this.isDouble = false;
                return this;
            }

            /**
             * Attach a {@link Compressor} for this mechanism to read pressure from.
             * Required for {@link #requirePressureAbove(double)} to actually gate actuation.
             */
            public Builder compressor(PneumaticsModuleType moduleType) {
                this.moduleType = moduleType;
                this.attachCompressor = true;
                return this;
            }

            /**
             * Refuse to drive forward when measured pressure is below {@code psi}.
             * Requires {@link #compressor(PneumaticsModuleType)} to be set
             * (REVPH analog pressure sensor only).
             */
            public Builder requirePressureAbove(double psi) {
                this.minPressurePSI = psi;
                return this;
            }

            public Config build() {
                if (forwardChannel < 0) {
                    throw new IllegalStateException(
                            "Pneumatic channel must be set via doubleSolenoid(...) or singleSolenoid(...)");
                }
                if (isDouble && reverseChannel < 0) {
                    throw new IllegalStateException("Double solenoid requires a reverse channel");
                }
                return new Config(this);
            }
        }
    }
}
