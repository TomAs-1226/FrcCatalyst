package frc.lib.catalyst.logging;

import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.util.struct.Struct;

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
 * @since 2.0.0
 */
public final class WpiTelemetrySink implements LogSink {

    /** Root table every key is published beneath. */
    private final TelemetryTable root;

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
        this.root = Telemetry.getTable(rootTable);
    }

    @Override public void log(String key, double value)    { root.log(key, value); }
    @Override public void log(String key, boolean value)   { root.log(key, value); }
    @Override public void log(String key, long value)      { root.log(key, value); }
    @Override public void log(String key, String value)    { root.log(key, value); }
    @Override public void log(String key, double[] value)  { root.log(key, value); }
    @Override public void log(String key, boolean[] value) { root.log(key, value); }
    @Override public void log(String key, long[] value)    { root.log(key, value); }
    @Override public void log(String key, String[] value)  { root.log(key, value); }

    @Override
    public <T> void log(String key, Struct<T> struct, T value) {
        root.log(key, value, struct);
    }

    @Override
    public <T> void log(String key, Struct<T> struct, T[] values) {
        root.log(key, values, struct);
    }
}
