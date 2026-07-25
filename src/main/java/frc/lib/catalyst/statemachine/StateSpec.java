package frc.lib.catalyst.statemachine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * Declares what one state means: where every mechanism goes, what must be true to enter it, and
 * what happens on the way in and out.
 *
 * <p>Handed to a lambda by {@code Builder.state(STOW, s -> s.set(...).set(...))}, so there is no
 * {@code done()} to forget — the lambda's closing brace closes the state.
 *
 * <p><b>A state's goal set never depends on where you came from.</b> It is the builder-wide
 * defaults, overridden by this state's own {@code set} calls, and nothing else. There is
 * deliberately no {@code keep()} method: path-dependent goals would make "am I in SCORE_HIGH?"
 * ambiguous, and an ambiguous state is exactly the mess this package exists to replace.
 *
 * @param <S> the machine's state enum
 * @since 1.2.0
 */
public final class StateSpec<S extends Enum<S>> {

    private final S state;
    final Map<Handle<?>, Object> goals = new HashMap<>();
    final Set<Handle<?>> released = new HashSet<>();
    final Set<Handle<?>> nonGating = new HashSet<>();
    double settleSeconds = 0.0;
    BooleanSupplier entryGuard;
    String entryGuardReason = "";
    double timeoutSeconds = Double.NaN;
    FaultPolicy faultPolicy;
    S recoveryState;
    final List<Runnable> onEnter = new ArrayList<>();
    final List<Runnable> onExit = new ArrayList<>();

    StateSpec(S state) {
        this.state = state;
    }

    /**
     * Send a bound mechanism to {@code goal} while in this state.
     *
     * <p>The generic signature is what makes the builder safe: {@code handle} carries its goal
     * type, so passing a {@code RotationalGoal} to an elevator handle does not compile.
     */
    public <G> StateSpec<S> set(Handle<G> handle, G goal) {
        if (handle == null) throw new IllegalArgumentException("handle must not be null");
        if (goal == null) throw new IllegalArgumentException(
                "goal for '" + handle.key() + "' in state " + state + " must not be null; "
                        + "use release(handle) to hand the mechanism back instead");
        goals.put(handle, goal);
        released.remove(handle);
        return this;
    }

    /**
     * Hand this mechanism back for the duration of this state: no goal is applied, its
     * {@code GoalRunner} idles, and {@link Binding#release()} is called.
     *
     * <p>Use it for a mechanism a driver should control freely in a given state — a climber winch
     * during endgame, say. Determinate, unlike "just don't mention it", which under the defaults
     * rule would silently inherit the default goal.
     */
    public StateSpec<S> release(Handle<?> handle) {
        released.add(handle);
        goals.remove(handle);
        return this;
    }

    /**
     * Drive this mechanism in this state, but do not wait for it before declaring arrival.
     *
     * <p>The right choice for open-loop outputs whose "arrival" is meaningless — a continuously
     * running indexer, an LED pattern — where gating on them would stall every transition.
     */
    public StateSpec<S> dontWait(Handle<?> handle) {
        nonGating.add(handle);
        return this;
    }

    /**
     * Require every gating binding to report at-goal continuously for this long before the state
     * counts as reached.
     *
     * <p>Guards against a mechanism that clips through its tolerance band on the way past.
     */
    public StateSpec<S> settleFor(double seconds) {
        this.settleSeconds = Math.max(0.0, seconds);
        return this;
    }

    /**
     * Refuse to enter this state unless {@code condition} holds, evaluated at request time.
     *
     * @param condition must be true to enter
     * @param reason    short, stable text logged with the rejection — {@code "gripped"}, not
     *                  {@code "claw current was 34.2 A"}
     */
    public StateSpec<S> entryGuard(BooleanSupplier condition, String reason) {
        this.entryGuard = condition;
        this.entryGuardReason = reason == null ? "" : reason;
        return this;
    }

    /** Deadline for reaching this state, overriding the builder default. */
    public StateSpec<S> timeout(double seconds) {
        this.timeoutSeconds = seconds;
        return this;
    }

    /** What to do if the deadline blows on the way here. */
    public StateSpec<S> onFault(FaultPolicy policy) {
        this.faultPolicy = policy;
        return this;
    }

    /** Shorthand for {@code onFault(RECOVER_TO)} plus the destination. */
    public StateSpec<S> recoverTo(S state) {
        this.recoveryState = state;
        this.faultPolicy = FaultPolicy.RECOVER_TO;
        return this;
    }

    /**
     * Run on <b>confirmed arrival only</b> — never on a timeout, an abort or an interrupt.
     *
     * <p>That restriction is the point. An entry action that fires when the machine merely
     * <em>gave up</em> on reaching a state is how "the LEDs said we were ready" happens.
     */
    public StateSpec<S> onEnter(Runnable action) {
        if (action != null) onEnter.add(action);
        return this;
    }

    /** Run when a transition out of a <b>confirmed</b> occupancy of this state actually begins. */
    public StateSpec<S> onExit(Runnable action) {
        if (action != null) onExit.add(action);
        return this;
    }

    /** The state being declared. */
    public S state() {
        return state;
    }
}
