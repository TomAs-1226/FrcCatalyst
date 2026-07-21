package frc.lib.catalyst.statemachine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Declares what one specific transition means: when it is allowed, how long it may take, and in
 * what order the mechanisms move.
 *
 * <p>{@link #stage(Handle...)} is the collision-prevention feature, and it replaces the old
 * {@code SuperstructureCoordinator.addTransitionRule(...)} with something declarative that the
 * machine can inspect and log. Instead of handing the library a hand-built command whose sequencing
 * is opaque, you say which mechanisms move first:
 *
 * <pre>{@code
 * .edge(STOW, SCORE_HIGH, e -> e
 *     .stage(elevator)      // raise first, so the arm clears the chassis
 *     .stage(arm, wrist))   // then deploy arm and wrist together
 * }</pre>
 *
 * <p>Stage N is applied only once every gating binding in stage N-1 reports at-goal. Any handle
 * never named in a stage moves immediately, in parallel with stage 0. The staging is published to
 * {@code Graph/Stages/<FROM>-><TO>}, so what the robot will actually do is readable before it
 * does it.
 *
 * @param <S> the machine's state enum
 * @since 1.2.0
 */
public final class EdgeSpec<S extends Enum<S>> {

    private final S from;
    private final S to;
    BooleanSupplier guard;
    String guardReason = "";
    double timeoutSeconds = Double.NaN;
    double cost = 1.0;
    final List<List<Handle<?>>> stages = new ArrayList<>();
    final List<Runnable> onTransit = new ArrayList<>();

    EdgeSpec(S from, S to) {
        this.from = from;
        this.to = to;
    }

    /**
     * Allow this transition only while {@code condition} holds.
     *
     * @param condition must be true for the edge to be taken, and for breadth-first routing to
     *                  consider it passable
     * @param reason    short, stable text logged with the rejection — this is what a driver sees
     *                  as {@code Blocker = "guard:endgame"}, so keep it a noun, not a sentence
     */
    public EdgeSpec<S> guard(BooleanSupplier condition, String reason) {
        this.guard = condition;
        this.guardReason = reason == null ? "" : reason;
        return this;
    }

    /** Deadline for this specific transition, taking precedence over the target state's timeout. */
    public EdgeSpec<S> timeout(double seconds) {
        this.timeoutSeconds = seconds;
        return this;
    }

    /**
     * Relative weight for breadth-first routing under {@link Routing#SHORTEST_PATH}, and the hook
     * a future priority-based scheduler would use. Purely advisory today; logged either way.
     */
    public EdgeSpec<S> cost(double cost) {
        this.cost = cost;
        return this;
    }

    /**
     * Append an actuation stage. Mechanisms in this stage begin moving only once every gating
     * mechanism in the previous stage has arrived.
     */
    public EdgeSpec<S> stage(Handle<?>... handles) {
        if (handles != null && handles.length > 0) {
            stages.add(List.copyOf(Arrays.asList(handles)));
        }
        return this;
    }

    /** Run once, when this edge is accepted and begins. */
    public EdgeSpec<S> onTransit(Runnable action) {
        if (action != null) onTransit.add(action);
        return this;
    }

    /** Origin state. */
    public S from() {
        return from;
    }

    /** Destination state. */
    public S to() {
        return to;
    }
}
