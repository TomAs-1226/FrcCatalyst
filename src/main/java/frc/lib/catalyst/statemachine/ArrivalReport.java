package frc.lib.catalyst.statemachine;

/**
 * One binding's contribution to a completed transition — did it arrive, how close did it get, and
 * how long did it take.
 *
 * <p>The {@code arrivalSeconds} field is the reason this record exists: with one per binding
 * attached to every transition, "which mechanism is our critical path" stops being a scrubbing
 * exercise and becomes a column. If the arm arrives in 0.88 s, the wrist in 1.02 s and the
 * elevator in 1.31 s, the elevator is the only one worth tuning.
 *
 * @param key            the binding key
 * @param kind           the binding kind ({@code "linear"}, {@code "claw"}, …)
 * @param goalLabel      the low-cardinality goal label
 * @param arrived        whether it reached its goal before the transition ended
 * @param observable     {@code false} when arrival was a settle timer rather than a sensor
 * @param error          signed error at the moment the transition ended, or {@code NaN}
 * @param tolerance      the tolerance band, or {@code NaN}
 * @param unit           unit for {@code error} and {@code tolerance}
 * @param arrivalSeconds seconds from goal application to arrival, or {@code NaN} if never arrived
 * @since 1.2.0
 */
public record ArrivalReport(String key, String kind, String goalLabel, boolean arrived,
                            boolean observable, double error, double tolerance,
                            String unit, double arrivalSeconds) {

    /** Normalises nulls so serialization can never NPE inside a logging call. */
    public ArrivalReport {
        key = key == null ? "" : key;
        kind = kind == null ? "" : kind;
        goalLabel = goalLabel == null ? "" : goalLabel;
        unit = unit == null ? "" : unit;
    }

    /**
     * Pipe-delimited single line for a {@code String[]} log entry:
     * {@code "elevator|linear|1.10m|true|true|0.004|0.020|m|0.83"}.
     *
     * <p>Pipes inside any field are replaced with underscores so the record always splits into
     * exactly nine columns — a dashboard table parser can rely on that.
     */
    public String serialize() {
        return String.join("|",
                sanitize(key), sanitize(kind), sanitize(goalLabel),
                Boolean.toString(arrived), Boolean.toString(observable),
                fmt(error), fmt(tolerance), sanitize(unit), fmt(arrivalSeconds));
    }

    private static String sanitize(String s) {
        return s.replace('|', '_');
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "" : String.format("%.4f", v);
    }

    @Override
    public String toString() {
        String state = arrived ? "arrived" : "MISSED";
        String time = Double.isNaN(arrivalSeconds) ? "" : String.format(" in %.2fs", arrivalSeconds);
        return key + " " + state + time;
    }
}
