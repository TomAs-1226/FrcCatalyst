package frc.lib.catalyst.logging;

import edu.wpi.first.util.struct.Struct;

/**
 * Pluggable destination for everything Catalyst logs.
 *
 * <p>Catalyst publishes telemetry through a single sink owned by
 * {@link CatalystLog}. The default is {@link NetworkTablesSink}; teams that
 * want to fan out into AdvantageKit, DataLog, file logging, or any other
 * system simply install a custom sink at robot init.
 *
 * <p>This is the seam that lets Catalyst stay self-contained while still
 * being completely composable with other logging frameworks. There is no
 * AdvantageKit dependency in Catalyst — instead, a team can implement
 * {@code LogSink} in their own code and forward calls to
 * {@code org.littletonrobotics.junction.Logger}. See
 * {@code docs/advanced/logging.md}.
 *
 * <p>Implementations must be safe to call from the robot main thread every
 * loop. They do not need to handle concurrency.
 */
public interface LogSink {

    /** Record a {@code double} under {@code key}. */
    void log(String key, double value);

    /** Record a {@code boolean} under {@code key}. */
    void log(String key, boolean value);

    /** Record a {@code long} under {@code key}. */
    void log(String key, long value);

    /** Record a {@code String} under {@code key}. */
    void log(String key, String value);

    /** Record a {@code double[]} under {@code key}. The array should be treated as read-only. */
    void log(String key, double[] value);

    /** Record a {@code boolean[]} under {@code key}. The array should be treated as read-only. */
    void log(String key, boolean[] value);

    /** Record a {@code long[]} under {@code key}. The array should be treated as read-only. */
    void log(String key, long[] value);

    /** Record a {@code String[]} under {@code key}. The array should be treated as read-only. */
    void log(String key, String[] value);

    /** Record a {@code String[]} under {@code key}. The array should be treated as read-only. */
    <T> void log(String key, Struct<T> struct, T value);

    /**
     * Process a mechanism's input snapshot.
     * <p>The default implementation serializes the inputs into a {@link LogTable}
     * and forwards each entry through the typed {@code log(...)} methods. Override
     * to integrate with frameworks that have first-class support for "process
     * inputs" semantics (e.g., AdvantageKit replay).
     *
     * @param prefix key prefix (e.g., {@code "Catalyst/Elevator"})
     * @param inputs the inputs snapshot to record
     */
    default void processInputs(String prefix, CatalystInputs inputs) {
        LogTable table = new LogTable();
        inputs.toLog(table);
        for (var e : table.entries().entrySet()) {
            String fullKey = prefix + "/" + e.getKey();
            switch (e.getValue().type()) {
                case DOUBLE        -> log(fullKey, (double) e.getValue().value());
                case BOOLEAN       -> log(fullKey, (boolean) e.getValue().value());
                case LONG          -> log(fullKey, (long) e.getValue().value());
                case STRING        -> log(fullKey, (String) e.getValue().value());
                case DOUBLE_ARRAY  -> log(fullKey, (double[]) e.getValue().value());
                case BOOLEAN_ARRAY -> log(fullKey, (boolean[]) e.getValue().value());
                case LONG_ARRAY    -> log(fullKey, (long[]) e.getValue().value());
                case STRING_ARRAY  -> log(fullKey, (String[]) e.getValue().value());
            }
        }
    }
}
