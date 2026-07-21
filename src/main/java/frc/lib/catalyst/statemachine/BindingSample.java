package frc.lib.catalyst.statemachine;

/**
 * A snapshot of one binding's live situation, published every loop under
 * {@code Bindings/<key>/} and exposed to code through {@code sample(key)}.
 *
 * <p>Two of these fields carry most of the diagnostic weight. {@code owned} goes false the moment
 * another command takes the subsystem — which is how a driver override stops looking like a
 * broken state machine. {@code observable} goes false when {@code atGoal} is really a settle timer
 * rather than a sensor reading, so nobody reads a log and believes a claw was measured shut when
 * in fact 0.3 seconds simply elapsed.
 *
 * @param key            binding key
 * @param kind           binding kind
 * @param unit           unit for {@code measured}, {@code error} and {@code tolerance}
 * @param goalLabel      low-cardinality goal label; {@code ""} when released
 * @param goalDetail     human-readable goal detail, may contain live values
 * @param arrived        {@code atGoal} for the currently applied goal
 * @param owned          whether the state machine currently holds this mechanism
 * @param observable     {@code false} when arrival is a timer, not a sensor
 * @param gating         whether this binding gates arrival for the active state
 * @param measured       live measured value, or {@code NaN}
 * @param error          signed error toward the applied goal, or {@code NaN}
 * @param tolerance      tolerance band, or {@code NaN}
 * @param arrivalSeconds seconds from goal application to arrival, or {@code NaN}
 * @param note           free-form runtime note from the binding
 * @since 1.2.0
 */
public record BindingSample(String key, String kind, String unit, String goalLabel,
                            String goalDetail, boolean arrived, boolean owned,
                            boolean observable, boolean gating, double measured,
                            double error, double tolerance, double arrivalSeconds, String note) {

    /** Normalises nulls so a sink can log every field without null checks. */
    public BindingSample {
        key = key == null ? "" : key;
        kind = kind == null ? "" : kind;
        unit = unit == null ? "" : unit;
        goalLabel = goalLabel == null ? "" : goalLabel;
        goalDetail = goalDetail == null ? "" : goalDetail;
        note = note == null ? "" : note;
    }

    /** True when this binding is gating the active state and has not arrived — i.e. it is the holdup. */
    public boolean blocking() {
        return gating && owned && !arrived;
    }
}
