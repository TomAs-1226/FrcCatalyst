package frc.lib.catalyst.mechanisms;

import frc.lib.catalyst.hardware.CatalystCANBus;
import frc.lib.catalyst.command.CatalystCommand;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.pneumatic.Compressor;
import org.wpilib.hardware.pneumatic.DoubleSolenoid;
import org.wpilib.hardware.pneumatic.PneumaticsModuleType;
import org.wpilib.hardware.pneumatic.Solenoid;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import frc.lib.catalyst.io.PneumaticMechanismInputs;
import frc.lib.catalyst.util.AlertManager;
import frc.lib.catalyst.util.HealthCheck;
import frc.lib.catalyst.util.HealthMonitor;

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
 *         .doubleSolenoid(PneumaticsModuleType.REV_PH, 0, 1)
 *         .compressor(PneumaticsModuleType.REV_PH)
 *         .requirePressureAbove(40.0)
 *         .build());
 *
 * controller.x().onTrue(climbHook.extend());
 * controller.y().onTrue(climbHook.retract());
 * }</pre>
 */
public class PneumaticMechanism extends CatalystMechanism {

    // WPILib's CANPort constants indexed the way CatalystCANBus numbers its buses. Written out
    // rather than resolved by name so a rename upstream fails the build instead of the match.
    private static final CANPort[] SYSTEMCORE_PORTS = {
            CANPort.CAN_S0, CANPort.CAN_S1, CANPort.CAN_S2, CANPort.CAN_S3, CANPort.CAN_S4
    };

    private static final CANPort[] MOTIONCORE_PORTS = {
            CANPort.CAN_D0, CANPort.CAN_D1, CANPort.CAN_D2, CANPort.CAN_D3, CANPort.CAN_D4,
            CANPort.CAN_D5, CANPort.CAN_D6, CANPort.CAN_D7, CANPort.CAN_D8, CANPort.CAN_D9,
            CANPort.CAN_D10, CANPort.CAN_D11, CANPort.CAN_D12, CANPort.CAN_D13, CANPort.CAN_D14,
            CANPort.CAN_D15, CANPort.CAN_D16, CANPort.CAN_D17, CANPort.CAN_D18, CANPort.CAN_D19
    };

    /**
     * The WPILib port a Catalyst bus corresponds to.
     *
     * <p>Lives here rather than on {@link CatalystCANBus} because that type's {@code wpilib()}
     * accessor still refuses to hand a port out — it was written off when the released alpha-6 had
     * no such WPILib type at all. Fold this back into {@code CatalystCANBus.wpilib()} once that
     * accessor is restored; every WPILib device will want the same mapping.
     */
    private static CANPort canPort(CatalystCANBus bus) {
        int index = bus.index().orElse(-1);
        if (bus.isSystemcore() && index >= 0 && index < SYSTEMCORE_PORTS.length) {
            return SYSTEMCORE_PORTS[index];
        }
        if (bus.isMotioncore() && index >= 0 && index < MOTIONCORE_PORTS.length) {
            return MOTIONCORE_PORTS[index];
        }
        throw new IllegalStateException(
                "WPILib's CANPort enum covers only can_s0-can_s4 and can_d0-can_d19, so " + bus.name()
                        + " cannot carry a pneumatics module. CANivores are Phoenix-only.");
    }

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

        // 2027 requires a CAN bus: Systemcore has no rio-attached pneumatics module, so a
        // REVPH/CTREPCM is reached over CAN like any other device.
        CANPort port = canPort(config.canBus);

        // Every overload below omits the module id, leaving WPILib to fill in the default for the
        // module type - 1 for a REV PH, 0 for a CTRE PCM. Alpha-6 addressed the module by a single
        // int that had to serve as both, so Catalyst hardcoded 1 and quietly mis-addressed a PCM.
        // A module moved off its default id still needs a config field Catalyst does not have.
        if (config.isDouble) {
            this.doubleSolenoid = new DoubleSolenoid(
                    port,
                    config.moduleType,
                    config.forwardChannel,
                    config.reverseChannel);
            this.singleSolenoid = null;
        } else {
            this.doubleSolenoid = null;
            this.singleSolenoid = new Solenoid(port, config.moduleType, config.forwardChannel);
        }

        this.compressor = config.attachCompressor
                ? new Compressor(port, config.moduleType)
                : null;

        if (compressor != null && config.minPressurePSI > 0) {
            HealthCheck.builder(name, "LowPressure")
                    .severity(HealthCheck.Severity.WARN)
                    .description("Air pressure below operating threshold")
                    .when(() -> {
                        double psi = getPressure();
                        return psi >= 0 && psi < config.minPressurePSI;
                    })
                    .detail(() -> {
                        double psi = getPressure();
                        if (psi < 0) return "no sensor";
                        return String.format("%.0f psi (min %.0f)", psi, config.minPressurePSI);
                    })
                    .debounce(0.5)
                    .clearAfter(2.0)
                    .register();
        }
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

    @Override
    public MechanismView describe() {
        // Solenoids have no continuous position; the value carries pressure when
        // an analog sensor is present, and the commanded state rides in extras.
        MechanismView.Builder view = MechanismView.of(name, "pneumatic");
        double psi = getPressure();
        if (psi >= 0) {
            view.value(psi, "psi");
        }
        return view
                .extra("state", getState().name())
                .extra("forward", isForward())
                .extra("reverse", isReverse())
                .build();
    }

    /**
     * Seconds since the last state transition.
     *
     * <p>Useful for sequencing: "wait until the cylinder has been extended for
     * at least 0.25s before releasing the next stage." Returns 0 before the
     * first transition.
     *
     * <pre>{@code
     * climbHook.extend().then(Commands.waitUntil(() -> climbHook.timeInState() > 0.25));
     * }</pre>
     */
    public double timeInState() {
        if (lastTransitionTimestamp <= 0) return 0.0;
        return Timer.getTimestamp() - lastTransitionTimestamp;
    }

    /** Total number of state transitions since construction. */
    public long getTransitionCount() {
        return transitionCount;
    }

    // --- Command Factories ---

    /** Extend the actuator (drive solenoid forward). */
    public CatalystCommand extend() {
        return runOnce(() -> applyState(State.FORWARD))
                .withName(name + ".Extend");
    }

    /**
     * Retract the actuator. On a double solenoid this commands the reverse
     * channel; on a single solenoid this de-energizes the coil.
     */
    public CatalystCommand retract() {
        return runOnce(() -> applyState(config.isDouble ? State.REVERSE : State.OFF))
                .withName(name + ".Retract");
    }

    /** De-energize the solenoid. */
    public CatalystCommand off() {
        return runOnce(() -> applyState(State.OFF))
                .withName(name + ".Off");
    }

    /** Flip between forward and reverse / off based on current state. */
    public CatalystCommand toggle() {
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
    public CatalystCommand pulse(double durationSeconds) {
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
                .untilTrue(() -> timer.hasElapsed(durationSeconds))
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
            lastTransitionTimestamp = Timer.getTimestamp();
            transitionCount++;
        }
        state = target;
        applyHardware();
        setState(target.name());
    }

    private void applyHardware() {
        if (config.isDouble) {
            switch (state) {
                case FORWARD -> doubleSolenoid.set(DoubleSolenoid.Value.FORWARD);
                case REVERSE -> doubleSolenoid.set(DoubleSolenoid.Value.REVERSE);
                case OFF -> doubleSolenoid.set(DoubleSolenoid.Value.OFF);
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

        HealthMonitor.getInstance().update();
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
        /** CAN bus the pneumatics module lives on. Systemcore has no rio-attached module. */
        final CatalystCANBus canBus;
        final int forwardChannel;
        final int reverseChannel;
        final boolean isDouble;
        final boolean attachCompressor;
        final double minPressurePSI;

        private Config(Builder b) {
            this.name = b.name;
            this.moduleType = b.moduleType;
            this.canBus = b.canBus;
            this.forwardChannel = b.forwardChannel;
            this.reverseChannel = b.reverseChannel;
            this.isDouble = b.isDouble;
            this.attachCompressor = b.attachCompressor;
            this.minPressurePSI = b.minPressurePSI;
        }

        public static Builder builder() { return new Builder(); }

        public static class Builder {
            private String name = "PneumaticMechanism";
            private PneumaticsModuleType moduleType = PneumaticsModuleType.REV_PH;
            private CatalystCANBus canBus = CatalystCANBus.DEFAULT;
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
            /**
             * CAN bus the pneumatics module is on. Defaults to {@link CatalystCANBus#DEFAULT}
             * ({@code can_s0}), which is where a single-bus robot will have put it.
             */
            public Builder canBus(CatalystCANBus bus) {
                this.canBus = bus;
                return this;
            }

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
