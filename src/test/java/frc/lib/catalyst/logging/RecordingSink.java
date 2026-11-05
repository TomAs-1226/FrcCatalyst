package frc.lib.catalyst.logging;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A {@link LogSink} that keeps what was written, so a test can assert on telemetry.
 *
 * <p>Catalyst publishes a great deal of its reasoning - which behaviour won and why, whether a goal
 * is ready and what is missing, which phase a co-pilot is in - and none of it was reachable from a
 * test while those classes wrote straight at NetworkTables. Installing this instead of the default
 * sink turns every published key into an assertion target with no NT server, no HAL and no robot.
 *
 * <p>Keeps both the latest value per key and the full ordered history, because the two questions are
 * different: "what does the dashboard show now" and "did it write this three times when it should
 * have written it once".
 *
 * <pre>{@code
 * RecordingSink sink = RecordingSink.install();
 * try {
 *     director.pursue(SCORE).schedule();
 *     assertEquals("SCORE", sink.string("Goal/Active"));
 *     assertEquals(1, sink.writeCount("Goal/Active"));
 * } finally {
 *     sink.restore();
 * }
 * }</pre>
 */
public final class RecordingSink implements LogSink {

    /** One write, in order. */
    public record Entry(String key, Object value) {}

    private final Map<String, Object> latest = new LinkedHashMap<>();
    private final List<Entry> history = new ArrayList<>();
    private LogSink previous;

    /** Install a fresh recorder as the process-wide sink, remembering what to put back. */
    public static RecordingSink install() {
        RecordingSink sink = new RecordingSink();
        sink.previous = CatalystLog.getSink();
        CatalystLog.setSink(sink);
        return sink;
    }

    /** Put back whatever sink was installed before. Call this in a finally or an @AfterEach. */
    public void restore() {
        if (previous != null) {
            CatalystLog.setSink(previous);
            previous = null;
        }
    }

    private void put(String key, Object value) {
        latest.put(key, value);
        history.add(new Entry(key, value));
    }

    @Override public void log(String key, double value)    { put(key, value); }
    @Override public void log(String key, boolean value)   { put(key, value); }
    @Override public void log(String key, long value)      { put(key, value); }
    @Override public void log(String key, String value)    { put(key, value); }
    @Override public void log(String key, double[] value)  { put(key, value); }
    @Override public void log(String key, boolean[] value) { put(key, value); }
    @Override public void log(String key, long[] value)    { put(key, value); }
    @Override public void log(String key, String[] value)  { put(key, value); }

    // ------------------------------------------------------------------ reading it back

    /** The most recent value written under {@code key}, or null if nothing ever was. */
    public Object value(String key) {
        return latest.get(key);
    }

    /** The most recent value as a String, or null. Fails loudly if it was written as another type. */
    public String string(String key) {
        Object v = latest.get(key);
        if (v == null) {
            return null;
        }
        if (!(v instanceof String s)) {
            throw new AssertionError(key + " was written as " + v.getClass().getSimpleName() + ", not String");
        }
        return s;
    }

    /** The most recent boolean under {@code key}, or null if nothing was written. */
    public Boolean bool(String key) {
        Object v = latest.get(key);
        return v == null ? null : (Boolean) v;
    }

    /** The most recent number under {@code key} as a double, or null. */
    public Double number(String key) {
        Object v = latest.get(key);
        if (v == null) {
            return null;
        }
        return ((Number) v).doubleValue();
    }

    /** Whether anything was ever written under {@code key}. */
    public boolean has(String key) {
        return latest.containsKey(key);
    }

    /** How many times {@code key} was written. The point of publish-on-change is that this stays low. */
    public int writeCount(String key) {
        int n = 0;
        for (Entry e : history) {
            if (e.key().equals(key)) {
                n++;
            }
        }
        return n;
    }

    /** Every key written, in first-write order. */
    public List<String> keys() {
        return List.copyOf(latest.keySet());
    }

    /** Every write, in order. */
    public List<Entry> history() {
        return List.copyOf(history);
    }

    /** Forget everything recorded so far, keeping this installed. */
    public void clear() {
        latest.clear();
        history.clear();
    }
}
