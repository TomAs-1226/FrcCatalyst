package frc.lib.catalyst.logging;

/**
 * The single entry point for all Catalyst telemetry.
 *
 * <p>{@code CatalystLog} is a static facade in front of a single
 * {@link LogSink}. Mechanisms call {@code CatalystLog.log("Elevator/Position", ...)},
 * the sink decides where it goes. Out of the box the sink is
 * {@link NetworkTablesSink}, so everything appears under
 * {@code /Catalyst/...} in NetworkTables (NT4) with zero setup.
 *
 * <h2>Why a facade?</h2>
 * Catalyst is intentionally a self-contained library. We do not bundle
 * another team's logging framework. But we also do not want to force teams
 * who already standardized on a framework (AdvantageKit, DataLog,
 * Shuffleboard recording, custom recorders) to use a parallel system. The
 * facade pattern means a team that wants AdvantageKit can install a
 * 20-line bridge in their own code:
 *
 * <pre>{@code
 * CatalystLog.setSink(new CompoundSink(
 *     new NetworkTablesSink(),  // keep NT for the dashboard
 *     new LogSink() {           // forward everything to AdvantageKit
 *         public void log(String k, double v)    { Logger.recordOutput(k, v); }
 *         public void log(String k, boolean v)   { Logger.recordOutput(k, v); }
 *         // ... one line per type
 *         public void processInputs(String p, CatalystInputs in) {
 *             LogTable t = new LogTable();
 *             in.toLog(t);
 *             // forward t to AK's LogTable
 *         }
 *     }
 * ));
 * }</pre>
 *
 * <h2>Threading</h2>
 * All calls happen on the robot main thread; this class is not thread-safe.
 *
 * <h2>Replay</h2>
 * For replay-style debugging, teams can install a sink that records each
 * call to a file and a separate replay harness that swaps the sink with a
 * playback implementation. The {@link CatalystInputs} contract is designed
 * with this in mind.
 */
public final class CatalystLog {

    private static LogSink sink = new NetworkTablesSink();
    private static boolean loggingInputs = true;
    private CatalystLog() {}

    /**
     * Install a new sink. Replaces (does not chain) the previous sink — use
     * {@link CompoundSink} to fan out.
     *
     * @param newSink non-null sink to receive every subsequent log call
     */
    public static void setSink(LogSink newSink) {
        if (newSink == null) throw new IllegalArgumentException("sink must not be null");
        sink = newSink;
    }

    /** The currently active sink. */
    public static LogSink getSink() { return sink; }

    /** Record a {@code double} under {@code key}. */
    public static void log(String key, double value)    { sink.log(key, value); }
    /** Record a {@code boolean} under {@code key}. */
    public static void log(String key, boolean value)   { sink.log(key, value); }
    /** Record a {@code long} under {@code key}. */
    public static void log(String key, long value)      { sink.log(key, value); }
    /** Record a {@code String} under {@code key}. */
    public static void log(String key, String value)    { sink.log(key, value); }
    /** Record a {@code double[]} under {@code key}. */
    public static void log(String key, double[] value)  { sink.log(key, value); }
    /** Record a {@code boolean[]} under {@code key}. */
    public static void log(String key, boolean[] value) { sink.log(key, value); }
    /** Record a {@code long[]} under {@code key}. */
    public static void log(String key, long[] value)    { sink.log(key, value); }
    /** Record a {@code String[]} under {@code key}. */
    public static void log(String key, String[] value)  { sink.log(key, value); }

    /**
     * Record a WPILib struct-serializable {@code value} (e.g. a {@code Pose2d}
     * or {@code SwerveModuleState}) under {@code key} using its {@link
     * edu.wpi.first.util.struct.Struct} descriptor, so AdvantageScope renders it
     * as a real 2D/3D object instead of loose numbers.
     */
    public static <T> void log(String key, edu.wpi.first.util.struct.Struct<T> struct, T value) {
        sink.log(key, struct, value);
    }

    /** Record an array of struct-serializable values (e.g. {@code SwerveModuleState[]}) under {@code key}. */
    public static <T> void log(String key, edu.wpi.first.util.struct.Struct<T> struct, T[] values) {
        sink.log(key, struct, values);
    }

    /**
     * Process a mechanism's input snapshot.
     * <p>Delegates to {@link LogSink#processInputs(String, CatalystInputs)}, which
     * by default serializes the inputs into a {@link LogTable} and writes one entry
     * per field. AdvantageKit-bridge sinks may instead forward the {@code inputs}
     * directly to AK's own {@code LoggableInputs} pipeline.
     */
    public static void processInputs(String prefix, CatalystInputs inputs) {
        if (loggingInputs) {
            sink.processInputs(prefix, inputs);        
        }
    }
    /**
     * use this to disable logging to the sink the inputs can help with loop over run as youu send less to NT
     * @param enable if the inputs will be logged or not
     */
    public static void enableLoggingInputs(boolean enable){
        loggingInputs = enable;
    }
}
