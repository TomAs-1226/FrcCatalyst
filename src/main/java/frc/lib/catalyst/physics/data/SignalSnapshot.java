package frc.lib.catalyst.physics.data;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.Set;

/**
 * Several named signals, all read at the same instant.
 *
 * <p>The point of a snapshot is that the numbers inside it are comparable. Reading
 * {@code gyro.getRate()} and {@code module.getVelocity()} on consecutive lines gives you two values
 * from two slightly different moments, and any residual you compute from them includes that skew.
 * {@link TimestampSynchronizer} produces a {@code SignalSnapshot} instead, where every value has been
 * interpolated to one common {@link #timestampSeconds()}.
 *
 * <p>Immutable. {@link #get(String)} returns an empty {@code OptionalDouble} for a signal that was
 * not available at that instant, rather than a zero that would silently look like a real reading.
 *
 * @since 1.5.0
 */
public final class SignalSnapshot {

    private final double timestampSeconds;
    private final Map<String, Double> values;

    /**
     * @param timestampSeconds the instant every value in {@code values} was interpolated to
     * @param values           signal name to value; copied, so the caller may reuse its map
     */
    public SignalSnapshot(double timestampSeconds, Map<String, Double> values) {
        this.timestampSeconds = timestampSeconds;
        this.values = Collections.unmodifiableMap(new LinkedHashMap<>(values));
    }

    /** The instant this snapshot describes, on the FPGA clock. */
    public double timestampSeconds() {
        return timestampSeconds;
    }

    /** The value of {@code name} at {@link #timestampSeconds()}, or empty if it was unavailable. */
    public OptionalDouble get(String name) {
        Double value = values.get(name);
        return value == null ? OptionalDouble.empty() : OptionalDouble.of(value);
    }

    /** The value of {@code name}, or {@code fallback} if it was unavailable. */
    public double getOr(String name, double fallback) {
        Double value = values.get(name);
        return value == null ? fallback : value;
    }

    /** True if {@code name} was available at this instant. */
    public boolean has(String name) {
        return values.containsKey(name);
    }

    /** The names present in this snapshot, in the order they were registered. */
    public Set<String> names() {
        return values.keySet();
    }

    /** How many signals resolved at this instant. */
    public int size() {
        return values.size();
    }

    @Override
    public String toString() {
        return "SignalSnapshot[t=" + timestampSeconds + ", " + values + "]";
    }
}
