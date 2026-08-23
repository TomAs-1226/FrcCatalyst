package frc.lib.catalyst.sysid;

import frc.lib.catalyst.command.CatalystCommand;
import frc.lib.catalyst.command.Commands;

import org.wpilib.command3.Mechanism;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.system.Timer;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Velocity;
import org.wpilib.units.measure.Voltage;

import java.util.function.Consumer;

import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

/**
 * System identification routine — Catalyst's replacement for the one WPILib removed in 2027.
 *
 * <p><b>What happened.</b> WPILib 2027 kept {@link SysIdRoutineLog}, the half that writes data in
 * the format the SysId desktop tool reads, and removed the half that drove the mechanism, because
 * that half was built on commands v2. Nothing in commands v3 replaced it.
 *
 * <p>Characterisation is not optional for Catalyst — feedforward gains, {@code
 * MotionConstraintCalculator} and Physics Core's own identifiers all assume a team can measure kS,
 * kV and kA. So Catalyst supplies the driving half itself, on v3 coroutines, and keeps writing
 * through {@code SysIdRoutineLog}. <b>The desktop SysId tool sees exactly the same data it always
 * did</b>, because the logging side is untouched.
 *
 * <p>The API mirrors the one it replaces, so existing Catalyst code and team code carry over:
 *
 * <pre>{@code
 * SysIdRoutine routine = new SysIdRoutine(
 *         new SysIdRoutine.Config(Volts.per(Second).of(1.0), Volts.of(7.0), Seconds.of(10.0)),
 *         new SysIdRoutine.Mechanism(
 *                 volts -> motor.setVoltage(volts.in(Volts)),
 *                 log -> log.motor("elevator")
 *                           .voltage(Volts.of(motor.getAppliedVoltage()))
 *                           .linearPosition(Meters.of(motor.getPosition()))
 *                           .linearVelocity(MetersPerSecond.of(motor.getVelocity())),
 *                 subsystem));
 *
 * controller.a().whileTrue(routine.quasistatic(SysIdRoutine.Direction.FORWARD));
 * }</pre>
 *
 * <p><b>Safety note.</b> These commands drive open-loop voltage with no soft limits of their own.
 * Run them with the mechanism free to travel, and bind them to held buttons so releasing the button
 * stops the test — every command here sets the output back to zero when it ends, however it ends.
 *
 * @since 2.0.0
 */
public class SysIdRoutine {

    /** Which way the test drives the mechanism. */
    public enum Direction {
        /** Positive voltage. */
        FORWARD(1.0),
        /** Negative voltage. */
        REVERSE(-1.0);

        final double sign;

        Direction(double sign) {
            this.sign = sign;
        }
    }

    /**
     * Test parameters.
     *
     * @param rampRateVoltsPerSecond quasistatic ramp rate
     * @param stepVoltage            dynamic step amplitude
     * @param timeout                maximum length of a single test
     * @param stateCallback          notified whenever the test state changes; may be null
     */
    public record Config(
            double rampRateVoltsPerSecond,
            Voltage stepVoltage,
            Time timeout,
            Consumer<SysIdRoutineLog.State> stateCallback) {

        /** WPILib's defaults: 1 V/s ramp, 7 V step, 10 s timeout. */
        public static final double DEFAULT_RAMP_RATE = 1.0;

        /** Defaults matching WPILib's: 1 V/s ramp, 7 V step, 10 s timeout. */
        public Config() {
            this(DEFAULT_RAMP_RATE, Volts.of(7.0), Seconds.of(10.0), null);
        }

        /** Parameters without a state callback. */
        public Config(double rampRateVoltsPerSecond, Voltage stepVoltage, Time timeout) {
            this(rampRateVoltsPerSecond, stepVoltage, timeout, null);
        }

        /**
         * Units-typed ramp rate, matching how the WPILib routine was configured
         * ({@code Volts.per(Second).of(1.0)}). Kept so existing team code carries over unchanged.
         */
        public Config(Velocity<VoltageUnit> rampRate, Voltage stepVoltage, Time timeout) {
            this(rampRate == null ? DEFAULT_RAMP_RATE : rampRate.baseUnitMagnitude(),
                    stepVoltage, timeout, null);
        }

        /**
         * Any argument may be null to take the default. Catalyst's own motor wrapper relies on
         * this — it only ever overrides the step voltage and the state callback.
         */
        public Config(Double rampRateVoltsPerSecond, Voltage stepVoltage, Time timeout,
                      Consumer<SysIdRoutineLog.State> stateCallback) {
            this(rampRateVoltsPerSecond == null ? DEFAULT_RAMP_RATE : rampRateVoltsPerSecond,
                    stepVoltage == null ? Volts.of(7.0) : stepVoltage,
                    timeout == null ? Seconds.of(10.0) : timeout,
                    stateCallback);
        }
    }

    /**
     * How to drive and observe the mechanism under test.
     *
     * @param drive       applies a voltage to the mechanism
     * @param log         records one sample; called every loop while a test runs
     * @param requirement the mechanism the test commands require
     * @param name        routine name, used as the log prefix
     */
    public record Mechanism(
            Consumer<Voltage> drive,
            Consumer<SysIdRoutineLog> log,
            org.wpilib.command3.Mechanism requirement,
            String name) {

        /** Name defaults to {@code "sysid"}. */
        public Mechanism(Consumer<Voltage> drive,
                         Consumer<SysIdRoutineLog> log,
                         org.wpilib.command3.Mechanism requirement) {
            this(drive, log, requirement, "sysid");
        }
    }

    private final Config config;
    private final Mechanism mechanism;
    private final SysIdRoutineLog routineLog;

    /**
     * @param config    test parameters
     * @param mechanism how to drive and observe the mechanism
     */
    public SysIdRoutine(Config config, Mechanism mechanism) {
        this.config = config;
        this.mechanism = mechanism;
        this.routineLog = new SysIdRoutineLog(mechanism.name());
    }

    /**
     * Slow voltage ramp, for kS and kV.
     *
     * <p>Voltage climbs at the configured ramp rate until the timeout expires or the command is
     * cancelled, whichever comes first.
     */
    public CatalystCommand quasistatic(Direction direction) {
        SysIdRoutineLog.State state = direction == Direction.FORWARD
                ? SysIdRoutineLog.State.QUASISTATIC_FORWARD
                : SysIdRoutineLog.State.QUASISTATIC_REVERSE;

        return test(state, elapsed -> config.rampRateVoltsPerSecond() * elapsed * direction.sign)
                .withName(mechanism.name() + ".quasistatic(" + direction + ")");
    }

    /**
     * Voltage step, for kA.
     *
     * <p>The full step voltage is applied immediately and held until the timeout expires or the
     * command is cancelled.
     */
    public CatalystCommand dynamic(Direction direction) {
        SysIdRoutineLog.State state = direction == Direction.FORWARD
                ? SysIdRoutineLog.State.DYNAMIC_FORWARD
                : SysIdRoutineLog.State.DYNAMIC_REVERSE;

        double volts = config.stepVoltage().in(Volts) * direction.sign;
        return test(state, elapsed -> volts)
                .withName(mechanism.name() + ".dynamic(" + direction + ")");
    }

    /**
     * The body both tests share: announce the state, drive the profile, sample every loop, and
     * always stop the mechanism on the way out.
     *
     * @param state   state to record for the SysId tool
     * @param profile voltage as a function of seconds elapsed
     */
    private CatalystCommand test(SysIdRoutineLog.State state,
                                 java.util.function.DoubleUnaryOperator profile) {
        double timeoutSeconds = config.timeout().in(Seconds);

        return Commands.of(mechanism.name() + ".sysid", coroutine -> {
            recordState(state);
            double start = Timer.getTimestamp();

            while (true) {
                double elapsed = Timer.getTimestamp() - start;
                if (elapsed >= timeoutSeconds) {
                    break;
                }
                mechanism.drive().accept(Volts.of(profile.applyAsDouble(elapsed)));
                if (mechanism.log() != null) {
                    mechanism.log().accept(routineLog);
                }
                coroutine.yield();
            }
        }, mechanism.requirement())
                // Runs on a normal finish and on cancellation alike, which is what makes it safe to
                // bind these to a held button.
                .finallyDo(interrupted -> {
                    mechanism.drive().accept(Volts.of(0));
                    recordState(SysIdRoutineLog.State.NONE);
                });
    }

    private void recordState(SysIdRoutineLog.State state) {
        routineLog.recordState(state);
        if (config.stateCallback() != null) {
            config.stateCallback().accept(state);
        }
    }
}
