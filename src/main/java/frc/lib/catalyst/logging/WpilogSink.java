package frc.lib.catalyst.logging;

import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * {@link LogSink} that records everything Catalyst logs to a standard WPILib
 * {@code .wpilog} file — which opens directly in AdvantageScope, the
 * DataLogTool, or any WPILOG reader. No extra vendordep; DataLog ships with
 * WPILib.
 *
 * <p>This is the lightweight "record" half of replay-style debugging: you get
 * a complete, timestamped log of every mechanism's inputs and every output,
 * scrubable after the match. (For full deterministic <em>replay</em> — re-run
 * modified code against the log — forward the sink into AdvantageKit's
 * {@code Logger} instead; the {@link CatalystInputs} {@code toLog}/{@code fromLog}
 * layer is built for exactly that. See {@code docs/advanced/logging.md}.)
 *
 * <p>By default it also keeps logging to NetworkTables so live dashboards
 * keep working — wrap with {@link CompoundSink} or use the {@code alsoToNt}
 * constructor flag.
 *
 * <pre>{@code
 * public void robotInit() {
 *     // Records to /U/logs (USB) or ~/logs, plus NT capture, plus DS data.
 *     CatalystLog.setSink(new WpilogSink());
 * }
 * }</pre>
 */
public final class WpilogSink implements LogSink {

    private final DataLog log;
    private final LogSink alsoTo;   // may be null

    private final Map<String, DoubleLogEntry> doubles = new HashMap<>();
    private final Map<String, BooleanLogEntry> booleans = new HashMap<>();
    private final Map<String, IntegerLogEntry> longs = new HashMap<>();
    private final Map<String, StringLogEntry> strings = new HashMap<>();
    private final Map<String, DoubleArrayLogEntry> doubleArrays = new HashMap<>();
    private final Map<String, BooleanArrayLogEntry> booleanArrays = new HashMap<>();
    private final Map<String, IntegerArrayLogEntry> longArrays = new HashMap<>();
    private final Map<String, StringArrayLogEntry> stringArrays = new HashMap<>();
    private final Map<String, StructLogEntry<?>> structs = new HashMap<>();

    /**
     * Start (or reuse) {@link DataLogManager}'s log — auto-locates a USB stick
     * or falls back to the roboRIO logs directory, and also captures NT + DS
     * data. Keeps mirroring to NetworkTables so live dashboards still work.
     */
    public WpilogSink() {
        this(true);
    }

    /**
     * @param alsoToNt if true, also forward to a {@link NetworkTablesSink} so
     *                 live dashboards keep updating while logging to file.
     */
    public WpilogSink(boolean alsoToNt) {
        DataLogManager.start();
        this.log = DataLogManager.getLog();
        this.alsoTo = alsoToNt ? new NetworkTablesSink() : null;
    }

    /** Use a caller-managed {@link DataLog} (e.g. a custom directory/filename). */
    public WpilogSink(DataLog log, boolean alsoToNt) {
        this.log = log;
        this.alsoTo = alsoToNt ? new NetworkTablesSink() : null;
    }

    @Override
    public void log(String key, double value) {
        doubles.computeIfAbsent(key, k -> new DoubleLogEntry(log, k)).append(value);
        if (alsoTo != null) alsoTo.log(key, value);
    }

    @Override
    public void log(String key, boolean value) {
        booleans.computeIfAbsent(key, k -> new BooleanLogEntry(log, k)).append(value);
        if (alsoTo != null) alsoTo.log(key, value);
    }

    @Override
    public void log(String key, long value) {
        longs.computeIfAbsent(key, k -> new IntegerLogEntry(log, k)).append(value);
        if (alsoTo != null) alsoTo.log(key, value);
    }

    @Override
    public void log(String key, String value) {
        strings.computeIfAbsent(key, k -> new StringLogEntry(log, k)).append(value);
        if (alsoTo != null) alsoTo.log(key, value);
    }

    @Override
    public void log(String key, double[] value) {
        doubleArrays.computeIfAbsent(key, k -> new DoubleArrayLogEntry(log, k)).append(value);
        if (alsoTo != null) alsoTo.log(key, value);
    }

    @Override
    public void log(String key, boolean[] value) {
        booleanArrays.computeIfAbsent(key, k -> new BooleanArrayLogEntry(log, k)).append(value);
        if (alsoTo != null) alsoTo.log(key, value);
    }

    @Override
    public void log(String key, long[] value) {
        longArrays.computeIfAbsent(key, k -> new IntegerArrayLogEntry(log, k)).append(value);
        if (alsoTo != null) alsoTo.log(key, value);
    }

    @Override
    public void log(String key, String[] value) {
        stringArrays.computeIfAbsent(key, k -> new StringArrayLogEntry(log, k)).append(value);
        if (alsoTo != null) alsoTo.log(key, value);
    }

    @Override
    public <T> void log(String key, Struct<T> struct, T value) {
        ((StructLogEntry<T>) structs.computeIfAbsent(key, k -> StructLogEntry.create(log, key, struct))).append(value);
        if (alsoTo != null) alsoTo.log(key, struct, value);
    }
}
