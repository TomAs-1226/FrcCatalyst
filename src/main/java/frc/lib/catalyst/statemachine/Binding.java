package frc.lib.catalyst.statemachine;

import java.util.function.Consumer;

/**
 * The only thing the state machine engine knows about hardware.
 *
 * <p>This interface deliberately imports <b>no WPILib types</b>. That is what lets
 * {@link StateMachineCore} be unit-tested on a laptop with no HAL, no NetworkTables and
 * no command scheduler — a test implements {@code Binding} in ten lines with a mutable
 * {@code double}. The robot-side extension that adds {@code Command} plumbing is
 * {@link frc.lib.catalyst.statemachine.robot.Actuator}.
 *
 * @param <G> the goal type for this binding. It <b>must</b> have value-based
 *            {@code equals}/{@code hashCode} — a {@code record}, an enum, or a sealed
 *            hierarchy of records. Never an array. The engine compares goals with
 *            {@link java.util.Objects#equals} to decide when to re-apply actuation, so an
 *            identity-equality goal type would rebuild its command every loop.
 *
 * @since 1.2.0
 */
public interface Binding<G> {

    /** Stable, unique, log-safe key. Becomes {@code Bindings/<key>/...} in telemetry. */
    String key();

    /**
     * A {@link frc.lib.catalyst.mechanisms.MechanismView}-compatible kind string —
     * {@code "linear"}, {@code "rotational"}, {@code "roller"}, … or {@code "custom"}.
     */
    default String kind() { return "custom"; }

    /** Unit label for {@link #measured()}: {@code "m"}, {@code "deg"}, {@code "rps"}, or {@code ""}. */
    default String unit() { return ""; }

    /**
     * Has the mechanism physically reached {@code goal}?
     *
     * <p>Must be cheap and allocation-free — this is called every loop for every binding.
     *
     * <p><b>Must be a pure function</b> of live sensor state, {@code goal}, and
     * {@code secondsSinceApplied}. In particular it must not consult a field recording
     * "the goal we last applied", because {@link StateMachineCore#isAt(Enum)} queries this
     * against the goals of states the machine is <em>not</em> currently in — that is how
     * {@code isAt} stays a measurement rather than a latch.
     *
     * @param goal               the goal to test against
     * @param secondsSinceApplied seconds since this exact goal was first applied, or
     *                            {@code 0.0} when the goal is not currently applied. This is
     *                            the only sanctioned source of elapsed time; a binding must
     *                            never call {@code Timer.getFPGATimestamp()} itself, or it
     *                            becomes untestable and disabled-mode accounting breaks.
     */
    boolean atGoal(G goal, double secondsSinceApplied);

    /** Live measured value in {@link #unit()}, or {@code NaN} when not applicable. */
    default double measured() { return Double.NaN; }

    /** Signed error toward {@code goal}, or {@code NaN}. Logging only. */
    default double error(G goal) { return Double.NaN; }

    /** Tolerance band for {@code goal}, or {@code NaN}. Logging only. */
    default double tolerance(G goal) { return Double.NaN; }

    /**
     * {@code false} when arrival cannot actually be sensed and {@link #atGoal} is really a
     * settle timer. Published as {@code Bindings/<key>/Observable} so nobody mistakes a
     * timer for a sensor when reading a log.
     */
    default boolean observable(G goal) { return true; }

    /**
     * Stable, <b>low-cardinality</b> label for {@code goal}. Must not interpolate a live
     * value — this becomes a logged string that is edge-detected, so a label carrying a
     * changing float would write at 50 Hz forever.
     */
    default String label(G goal) { return String.valueOf(goal); }

    /** Human-readable detail. May interpolate live values. Never edge-detected. */
    default String detail(G goal) { return label(goal); }

    /** Free-form runtime note surfaced in {@code BlockerDetail}. {@code ""} when nothing to say. */
    default String note(G goal) { return ""; }

    /**
     * Is the mechanism homed? A state whose gating bindings are not all zeroed is rejected
     * with {@link RejectReason#NOT_ZEROED} rather than attempted. Defaults to {@code true}
     * for mechanisms with no zero concept.
     */
    default boolean zeroed() { return true; }

    /**
     * Build-time self-check for one goal. Report problems by accepting strings; the builder
     * aggregates them all into a single {@link StateMachineConfigException}.
     *
     * <p>This is where out-of-range setpoints, unknown named-position presets and
     * structurally impossible goals (a {@code REVERSE} goal on a single solenoid, a
     * secondary-RPS goal on a one-motor flywheel) are caught — on a laptop, at build, all at
     * once, instead of one exception per deploy cycle in a pit.
     */
    default void validate(G goal, Consumer<String> problems) {}

    /** Called when the machine relinquishes this binding (state release, or override). */
    default void release() {}
}
