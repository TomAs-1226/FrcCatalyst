package frc.lib.catalyst.logging;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.util.struct.Struct;

/**
 * A {@link LogSink} that forwards every call to a list of underlying sinks.
 *
 * <p>Useful when a team wants Catalyst telemetry to appear in two places at
 * once — e.g., NetworkTables for live driver dashboards plus an
 * AdvantageKit-bridge sink for replay logging. Order of installation is the
 * order of forwarding.
 *
 * <pre>{@code
 * CatalystLog.setSink(new CompoundSink(
 *     new NetworkTablesSink(),
 *     myTeam.advantageKitBridgeSink()
 * ));
 * }</pre>
 */
public final class CompoundSink implements LogSink {

    private final List<LogSink> sinks;

    /** Build a sink that fans out to {@code sinks} in order. */
    public CompoundSink(LogSink... sinks) {
        this.sinks = Arrays.asList(sinks);
    }

    @Override public void log(String key, double value)    { for (LogSink s : sinks) s.log(key, value); }
    @Override public void log(String key, boolean value)   { for (LogSink s : sinks) s.log(key, value); }
    @Override public void log(String key, long value)      { for (LogSink s : sinks) s.log(key, value); }
    @Override public void log(String key, String value)    { for (LogSink s : sinks) s.log(key, value); }
    @Override public void log(String key, double[] value)  { for (LogSink s : sinks) s.log(key, value); }
    @Override public void log(String key, boolean[] value) { for (LogSink s : sinks) s.log(key, value); }
    @Override public void log(String key, long[] value)    { for (LogSink s : sinks) s.log(key, value); }
    @Override public void log(String key, String[] value)  { for (LogSink s : sinks) s.log(key, value); }
    @Override
    public <T> void log(String key, Struct<T> struct, T value) { for (LogSink s : sinks) s.log(key, struct, value); }
    @Override
    public <T> void log(String key, Struct<T> struct, T[] values) { for (LogSink s : sinks) s.log(key, struct, values); }
    @Override
    public void processInputs(String prefix, CatalystInputs inputs) {
        for (LogSink s : sinks) s.processInputs(prefix, inputs);
    }

}
