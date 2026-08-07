package frc.lib.catalyst.identity;

import frc.lib.catalyst.logging.CatalystLog;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

/**
 * A spec sheet under construction: an ordered set of facts, and a hard refusal to record one that
 * nobody knows.
 *
 * <p>Every setter takes an {@code Optional}, never a bare value, and that is the whole design.
 * Catalyst Console draws a dash for a key the robot never published and a number for one it did, so
 * an absent key is the only honest way to say "unknown" — a placeholder ({@code 0}, {@code -1},
 * {@code ""}) is a lie the dashboard cannot detect. Leaving no overload that accepts a plain
 * {@code double} means writing the lie takes more effort than writing the truth, rather than less.
 *
 * <p>Nothing reaches NetworkTables until {@link #publish(String)} runs, so a sheet can be assembled,
 * inspected and asserted against with no HAL and no NT instance anywhere in sight.
 *
 * @since 1.10.0
 */
public final class SpecSheet {

    private final Map<String, Object> entries = new LinkedHashMap<>();

    /** Record a string, unless it is null or blank. */
    public SpecSheet putText(String key, Optional<String> value) {
        value.filter(s -> !s.isBlank()).ifPresent(s -> entries.put(key, s.trim()));
        return this;
    }

    /** Record a number, unless it is absent, NaN or infinite. */
    public SpecSheet putNumber(String key, OptionalDouble value) {
        if (value.isPresent() && Double.isFinite(value.getAsDouble())) {
            entries.put(key, value.getAsDouble());
        }
        return this;
    }

    /** Record a whole number, unless it is absent. */
    public SpecSheet putInt(String key, OptionalInt value) {
        value.ifPresent(v -> entries.put(key, (long) v));
        return this;
    }

    /**
     * Record a flag.
     *
     * <p>The only setter that takes a bare value, because a boolean has no "unknown" to encode: a
     * caller that does not know reaches this line via {@code Optional.ifPresent} or does not reach
     * it at all.
     */
    public SpecSheet putFlag(String key, boolean value) {
        entries.put(key, value);
        return this;
    }

    /**
     * Record a list of strings, unless it is empty.
     *
     * <p>An empty list is treated as unknown rather than as "none". A robot with no cameras and a
     * robot whose cameras Catalyst never saw look identical from here, and publishing an empty
     * array would tell the dashboard the first with the confidence it has earned for neither.
     */
    public SpecSheet putList(String key, List<String> values) {
        if (values != null && !values.isEmpty()) {
            entries.put(key, values.toArray(new String[0]));
        }
        return this;
    }

    /** Record an array of numbers, unless it is empty or any element is not finite. */
    public SpecSheet putNumbers(String key, double[] values) {
        if (values == null || values.length == 0) return this;
        for (double v : values) {
            if (!Double.isFinite(v)) return this;
        }
        entries.put(key, values.clone());
        return this;
    }

    /** Every fact recorded so far, in the order it was added. Keys are relative to the sub-table. */
    public Map<String, Object> entries() {
        return Collections.unmodifiableMap(entries);
    }

    /** How many facts are on the sheet. */
    public int size() {
        return entries.size();
    }

    /**
     * Write the sheet through {@link CatalystLog}, prefixing every key.
     *
     * <p>Goes through the log facade rather than straight to NetworkTables so a team that has
     * installed a WPILOG or AdvantageKit sink gets the spec sheet in their log alongside everything
     * else, and so the keys land under {@code /Catalyst/} with the rest of Catalyst's telemetry —
     * {@code TelemetryUtil} would have dropped them under {@code /SmartDashboard/} instead.
     */
    void publish(String prefix) {
        for (Map.Entry<String, Object> entry : entries.entrySet()) {
            String key = prefix + entry.getKey();
            Object value = entry.getValue();
            if (value instanceof String s) {
                CatalystLog.log(key, s);
            } else if (value instanceof Double d) {
                CatalystLog.log(key, d.doubleValue());
            } else if (value instanceof Long l) {
                CatalystLog.log(key, l.longValue());
            } else if (value instanceof Boolean b) {
                CatalystLog.log(key, b.booleanValue());
            } else if (value instanceof String[] a) {
                CatalystLog.log(key, a);
            } else if (value instanceof double[] a) {
                CatalystLog.log(key, a);
            }
        }
    }

    // -------------------------------------------------------------
    // Wrappers for the sources that signal "I don't know" in-band.
    // -------------------------------------------------------------

    /** A string, or empty when it is null or blank — which is how the HAL reports "unset". */
    public static Optional<String> text(String value) {
        return value == null || value.isBlank() ? Optional.empty() : Optional.of(value.trim());
    }

    /**
     * A number, or empty unless it is finite and above zero.
     *
     * <p>For the many WPILib and vendor calls that return {@code 0} when they have nothing: an
     * unset team number, an FPGA version off a simulated rio, a brownout threshold read before the
     * HAL is up. None of those zeroes are measurements, and every one of them would render as a
     * confident number on a dashboard.
     */
    public static OptionalDouble positive(double value) {
        return Double.isFinite(value) && value > 0 ? OptionalDouble.of(value) : OptionalDouble.empty();
    }

    /** As {@link #positive(double)}, for the whole-number sources. */
    public static OptionalInt positive(int value) {
        return value > 0 ? OptionalInt.of(value) : OptionalInt.empty();
    }
}
