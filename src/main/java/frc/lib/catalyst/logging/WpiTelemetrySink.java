package frc.lib.catalyst.logging;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.telemetry.DiscardTelemetryBackend;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.util.struct.Struct;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Default {@link LogSink} on WPILib 2027 and later. Forwards every Catalyst log call to
 * {@link Telemetry}, WPILib's own telemetry facade.
 *
 * <p><b>Why this replaced {@link NetworkTablesSink} as the default.</b> WPILib 2027 introduced
 * {@code org.wpilib.telemetry}, which is the same design Catalyst already had — a static facade
 * over a swappable backend — and made it the route every 2027 dashboard reads. Publishing straight
 * to NetworkTables still works, but it only reaches tools that know to look under
 * {@code /Catalyst/...}. Publishing through {@code Telemetry} reaches Elastic, AdvantageScope, the
 * FIRST Driver Station and anything else that speaks the standard backend, without any of them
 * knowing Catalyst exists.
 *
 * <p>Nothing about Catalyst's own API changed to make this work. {@link CatalystLog#log} has the
 * same signatures it always had; only what sits behind it moved. A team that wants the old
 * behaviour back gets it in one line:
 *
 * <pre>{@code
 * CatalystLog.setSink(new NetworkTablesSink());
 * }</pre>
 *
 * <p>Keys are written relative to a root table (default {@code "Catalyst"}), so the published
 * paths match what {@link NetworkTablesSink} produced and existing dashboards, the Health
 * Dashboard and Catalyst Console keep resolving the same names.
 *
 * <h2>Logging before there is anywhere to log</h2>
 *
 * <p>WPILib installs its telemetry backend in {@code RobotBase}'s constructor - read off the
 * alpha-6 jar, not inferred. That is later than it sounds: a {@code static final} field on the
 * robot class is initialised when the class loads, which is before any instance of it exists and
 * therefore before that constructor has run. Anything
 * a robot publishes before that lands in a registry with no backend, and WPILib answers each one
 * with a warning and a stack trace rather than storing it — so the value is gone and the console is
 * full of traces about it.
 *
 * <p>That window is not an edge case. Hardware is claimed in constructors, and the recommended
 * shape for a container that owns hardware is a single instance built once — often a {@code static
 * final} field, because building it twice would have two objects claiming the same CAN ids. A
 * {@code static final} field is initialised during class loading, which is before
 * {@code startRobot} has got anywhere. Every {@code CANRegistry} publish, every mechanism
 * registering itself, every identity field written during construction falls in it.
 *
 * <p>So this sink buffers until a backend appears and then replays. Only the newest value per key
 * is kept: telemetry is state, not a stream of events, and a dashboard connecting later wants the
 * current value rather than the history of a boot sequence. The buffer is bounded, and once a
 * backend exists the sink writes straight through with nothing but a volatile read in the way.
 *
 * @since 2.0.0
 */
public final class WpiTelemetrySink implements LogSink {

    /**
     * How many distinct keys may be held while there is no backend.
     *
     * <p>Generous — a robot publishes a few dozen keys during construction, not thousands — and
     * bounded anyway, because a sink used outside a robot program (a unit test, a desktop tool)
     * never gets a backend and must not grow without limit.
     */
    private static final int MAX_BUFFERED_KEYS = 512;

    /** Root table every key is published beneath. */
    private final TelemetryTable root;

    /** The table name, which is also the path asked about when checking for a backend. */
    private final String rootName;

    /**
     * Newest pending write per key, in the order the keys were first seen.
     *
     * <p>Insertion-ordered so a replay reproduces the order things were first published in, which
     * is the order they would have arrived in had a backend been there. Null once flushed — the
     * common case is a robot that has been up for more than a second, and it should carry no state.
     */
    private Map<String, Runnable> pending = new LinkedHashMap<>();

    /** Whether a backend has been seen. Volatile because logging is not always on the main thread. */
    private volatile boolean live;

    /** Whether the buffer overflowed, so it is said once rather than per dropped key. */
    private boolean overflowed;

    /** When this sink was built, so buffering can be given a deadline. */
    private final long born = System.nanoTime();

    /**
     * How long to hold writes waiting for a backend.
     *
     * <p>Generous next to robot startup, which installs the backend in well under a second, and
     * short next to a match.
     */
    private static final long GRACE_NANOS = 5_000_000_000L;

    /** This sink's grace period; {@link #GRACE_NANOS} except in tests. */
    private final long grace;



    /** Publish under {@code /Catalyst}. */
    public WpiTelemetrySink() {
        this("Catalyst");
    }

    /**
     * Publish under a custom root table.
     *
     * @param rootTable table name to nest all Catalyst keys under, e.g. {@code "Catalyst"}
     */
    public WpiTelemetrySink(String rootTable) {
        this(rootTable, GRACE_NANOS);
    }

    /**
     * Test seam. A grace period of zero makes the sink write straight through from the first call,
     * which is how the give-up path is checked without a test that sleeps for five seconds.
     */
    WpiTelemetrySink(String rootTable, long graceNanos) {
        this.rootName = rootTable;
        this.root = Telemetry.getTable(rootTable);
        this.grace = graceNanos;
    }

    /** Test hook. Whether anything is currently being held back. Not for production code. */
    boolean isBuffering() {
        synchronized (this) {
            return pending != null && !pending.isEmpty();
        }
    }

    /**
     * Send a write through, or hold it until there is somewhere for it to go.
     *
     * <p>The check costs a volatile read once a backend exists, which is every loop after the first
     * fraction of a second. Before that it is one write attempt per call, on a few dozen calls.
     */
    private void write(String key, Runnable send) {
        if (live) {
            send.run();
            return;
        }
        synchronized (this) {
            if (live) {
                send.run();
                return;
            }
            if (hasBackend()) {
                flush();
                send.run();
                return;
            }
            // Buffering is a bridge across robot startup, not a place to keep telemetry. If no real
            // backend has arrived by now there probably is not going to be one - a unit test, a
            // desktop tool, a simulation harness that never installs it - and holding values
            // forever would turn "logged nowhere, loudly" into "logged nowhere, silently", which
            // is the worse of the two. Give up and behave exactly as this class did before.
            if (System.nanoTime() - born >= grace) {
                giveUp();
                send.run();
                return;
            }
            hold(key, send);
        }
    }

    /**
     * Whether WPILib has installed the backend that everything resolves through.
     *
     * <p>The path asked about is the empty one, because that is the exact path
     * {@code RobotBase.startRobot} registers {@code NetworkTablesTelemetryBackend} at. Asking about
     * the Catalyst table instead was tried and is wrong: it answers non-null in states where an
     * entry write still lands nowhere, so the buffer flushed early and the replay warned.
     *
     * <p>Observing whether a write warned was also tried and is worse. WPILib reports a missing
     * backend once per path, so the second write to any path looks like a success and the sink
     * declares itself live while nothing is being stored.
     *
     * <p>The probe is silenced while it runs: asking is itself reportable, and without this the
     * console fills with traces about the check that exists to keep the console clean. Warnings
     * that are not ours pass through to whatever handler the team installed.
     */
    private boolean hasBackend() {
        java.util.function.BiConsumer<String, String> previous = TelemetryRegistry.getReportWarning();
        try {
            TelemetryRegistry.setReportWarning((path, message) -> { });
            // Not merely non-null. Before RobotBase installs the real one, the registry answers
            // every path with a DiscardTelemetryBackend - a backend that exists and throws
            // everything away. Treating that as somewhere to log is exactly the bug this class was
            // written to fix, one level further in.
            return !(TelemetryRegistry.getBackend("") instanceof DiscardTelemetryBackend);
        } catch (RuntimeException e) {
            return false;
        } finally {
            TelemetryRegistry.setReportWarning(previous);
        }
    }

    /** Remember the newest value for {@code key}, within the cap. */
    private void hold(String key, Runnable send) {
        if (pending == null) {
            return;
        }
        // Replacing an existing key is always allowed - it does not grow the map, and dropping the
        // newer value in favour of a stale one would be the wrong way round.
        if (pending.size() >= MAX_BUFFERED_KEYS && !pending.containsKey(key)) {
            if (!overflowed) {
                overflowed = true;
                DriverStationErrors.reportWarning(
                        "Catalyst telemetry buffered " + MAX_BUFFERED_KEYS + " keys before WPILib had "
                                + "a backend, and is dropping further new ones. This is normal outside "
                                + "a robot program; on a robot it means telemetry is being written "
                                + "from a loop that runs before startRobot.", false);
            }
            return;
        }
        pending.put(key, send);
    }

    /**
     * Stop buffering without a backend having appeared, discarding what was held.
     *
     * <p>Deliberately quiet about the values. They were going to a discard backend either way, and
     * a robot program that reaches here has bigger news than a telemetry warning. Caller holds the
     * monitor.
     */
    private void giveUp() {
        pending = null;
        live = true;
    }

    /** Replay everything held, oldest key first. Caller holds the monitor. */
    private void flush() {
        Map<String, Runnable> held = pending;
        pending = null;
        live = true;
        if (held == null) {
            return;
        }
        for (Runnable send : held.values()) {
            try {
                send.run();
            } catch (RuntimeException e) {
                // One bad value must not cost the rest of the replay. It has already been logged
                // once by whatever produced it.
                DriverStationErrors.reportWarning(
                        "Catalyst telemetry replay failed for one key: " + e, false);
            }
        }
    }

    // Arrays are copied on the buffered path only. A caller is entitled to reuse its array after a
    // log call returns, and a replay seconds later would otherwise publish whatever it holds by
    // then. On the live path - every loop after startup - nothing is copied.
    @Override public void log(String key, double value)    { write(key, () -> root.log(key, value)); }
    @Override public void log(String key, boolean value)   { write(key, () -> root.log(key, value)); }
    @Override public void log(String key, long value)      { write(key, () -> root.log(key, value)); }
    @Override public void log(String key, String value)    { write(key, () -> root.log(key, value)); }

    @Override public void log(String key, double[] value) {
        if (live) { root.log(key, value); } else { double[] c = value.clone(); write(key, () -> root.log(key, c)); }
    }

    @Override public void log(String key, boolean[] value) {
        if (live) { root.log(key, value); } else { boolean[] c = value.clone(); write(key, () -> root.log(key, c)); }
    }

    @Override public void log(String key, long[] value) {
        if (live) { root.log(key, value); } else { long[] c = value.clone(); write(key, () -> root.log(key, c)); }
    }

    @Override public void log(String key, String[] value) {
        if (live) { root.log(key, value); } else { String[] c = value.clone(); write(key, () -> root.log(key, c)); }
    }

    @Override
    public <T> void log(String key, Struct<T> struct, T value) {
        write(key, () -> root.log(key, value, struct));
    }

    @Override
    public <T> void log(String key, Struct<T> struct, T[] values) {
        if (live) {
            root.log(key, values, struct);
        } else {
            T[] copy = values.clone();
            write(key, () -> root.log(key, copy, struct));
        }
    }
}
