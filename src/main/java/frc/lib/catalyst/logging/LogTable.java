package frc.lib.catalyst.logging;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * An ordered key-value table used to ship telemetry between mechanisms,
 * the active {@link LogSink}, and replay tooling.
 *
 * <p><b>Design intent.</b> Catalyst ships a fully in-house logging stack so
 * teams pulling in this library do not pick up another framework's runtime
 * as a transitive dependency. The shape of {@code LogTable} is deliberately
 * a near-mirror of the structures used by popular FRC replay frameworks
 * (one slot per primitive type, ordered insertion, string keys), so a team
 * that already runs AdvantageKit can bridge the two with a tiny adapter
 * (see {@code docs/advanced/logging.md}) without Catalyst ever importing
 * anything from that framework.
 *
 * <p>{@code LogTable} is intentionally a plain Java object — no NetworkTables,
 * no DataLog, no I/O. {@link LogSink} implementations are the only things
 * that translate a {@code LogTable} into a side effect (publishing,
 * recording, forwarding).
 *
 * <p>Iteration order matches insertion order; this is the order keys appear
 * when serialized to dashboards.
 *
 * <p>This class is not thread-safe. All Catalyst logging happens on the
 * robot main thread.
 */
public final class LogTable {

    /** Discriminator for the primitive variant of each value in the table. */
    public enum Type {
        DOUBLE, BOOLEAN, STRING, LONG, DOUBLE_ARRAY, BOOLEAN_ARRAY, LONG_ARRAY, STRING_ARRAY
    }

    /** One entry in the table; pairs a type tag with the boxed value. */
    public record Entry(Type type, Object value) {}

    private final Map<String, Entry> data = new LinkedHashMap<>();

    /** Insert (or overwrite) a {@code double} value. */
    public void put(String key, double value) { data.put(key, new Entry(Type.DOUBLE, value)); }

    /** Insert (or overwrite) a {@code boolean} value. */
    public void put(String key, boolean value) { data.put(key, new Entry(Type.BOOLEAN, value)); }

    /** Insert (or overwrite) a {@code long} value. */
    public void put(String key, long value) { data.put(key, new Entry(Type.LONG, value)); }

    /** Insert (or overwrite) a {@code String} value. {@code null} is stored as the empty string. */
    public void put(String key, String value) { data.put(key, new Entry(Type.STRING, value == null ? "" : value)); }

    /** Insert (or overwrite) a {@code double[]} value. The array is referenced, not copied. */
    public void put(String key, double[] value) { data.put(key, new Entry(Type.DOUBLE_ARRAY, value)); }

    /** Insert (or overwrite) a {@code boolean[]} value. The array is referenced, not copied. */
    public void put(String key, boolean[] value) { data.put(key, new Entry(Type.BOOLEAN_ARRAY, value)); }

    /** Insert (or overwrite) a {@code long[]} value. The array is referenced, not copied. */
    public void put(String key, long[] value) { data.put(key, new Entry(Type.LONG_ARRAY, value)); }

    /** Insert (or overwrite) a {@code String[]} value. The array is referenced, not copied. */
    public void put(String key, String[] value) { data.put(key, new Entry(Type.STRING_ARRAY, value)); }

    /** Read a {@code double}, falling back to {@code defaultValue} if the key is missing or another type. */
    public double get(String key, double defaultValue) {
        Entry e = data.get(key);
        return (e != null && e.type == Type.DOUBLE) ? (double) e.value : defaultValue;
    }

    /** Read a {@code boolean}, falling back to {@code defaultValue} if the key is missing or another type. */
    public boolean get(String key, boolean defaultValue) {
        Entry e = data.get(key);
        return (e != null && e.type == Type.BOOLEAN) ? (boolean) e.value : defaultValue;
    }

    /** Read a {@code long}, falling back to {@code defaultValue} if the key is missing or another type. */
    public long get(String key, long defaultValue) {
        Entry e = data.get(key);
        return (e != null && e.type == Type.LONG) ? (long) e.value : defaultValue;
    }

    /** Read a {@code String}, falling back to {@code defaultValue} if the key is missing or another type. */
    public String get(String key, String defaultValue) {
        Entry e = data.get(key);
        return (e != null && e.type == Type.STRING) ? (String) e.value : defaultValue;
    }

    /** Read a {@code double[]}, falling back to {@code defaultValue} if the key is missing or another type. */
    public double[] get(String key, double[] defaultValue) {
        Entry e = data.get(key);
        return (e != null && e.type == Type.DOUBLE_ARRAY) ? (double[]) e.value : defaultValue;
    }

    /** Read a {@code boolean[]}, falling back to {@code defaultValue} if the key is missing or another type. */
    public boolean[] get(String key, boolean[] defaultValue) {
        Entry e = data.get(key);
        return (e != null && e.type == Type.BOOLEAN_ARRAY) ? (boolean[]) e.value : defaultValue;
    }

    /** Read a {@code long[]}, falling back to {@code defaultValue} if the key is missing or another type. */
    public long[] get(String key, long[] defaultValue) {
        Entry e = data.get(key);
        return (e != null && e.type == Type.LONG_ARRAY) ? (long[]) e.value : defaultValue;
    }

    /** Read a {@code String[]}, falling back to {@code defaultValue} if the key is missing or another type. */
    public String[] get(String key, String[] defaultValue) {
        Entry e = data.get(key);
        return (e != null && e.type == Type.STRING_ARRAY) ? (String[]) e.value : defaultValue;
    }

    /** Live view of the underlying map. Mutating the returned map mutates the table. */
    public Map<String, Entry> entries() { return data; }

    /** Remove every entry. */
    public void clear() { data.clear(); }

    /** True when the table has no entries. */
    public boolean isEmpty() { return data.isEmpty(); }

    /** Number of entries currently in the table. */
    public int size() { return data.size(); }

    /** True when the table contains an entry under {@code key}. */
    public boolean containsKey(String key) { return data.containsKey(key); }
}
