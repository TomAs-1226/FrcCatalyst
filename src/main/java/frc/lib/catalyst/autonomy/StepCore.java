package frc.lib.catalyst.autonomy;

/**
 * What a reactive sequence does when it reaches a step: run it, substitute, skip, or abort.
 *
 * <p>Lifted out of {@code BehaviorEngine} so the decision can be read and tested on its own. It was
 * a switch nested inside a deferred lambda inside a builder, reachable only by scheduling a command
 * on a robot, and it is the whole point of that class - "if the piece is not where you expected, do
 * something sensible instead of driving to an empty spot".
 *
 * <p>Pure: same inputs, same answer, no clock and no hardware. The labels it returns are the exact
 * strings the engine has always published, because a dashboard is watching them.
 *
 * @since 2.1.0
 */
public final class StepCore {
    private StepCore() {}

    /** What to do with this step. */
    public enum Outcome {
        /** The step's own action can start; run it. */
        RUN,
        /** It cannot, but its substitute can; run that. */
        SUBSTITUTE,
        /** It cannot, and there is nothing to put in its place; move on. */
        SKIP,
        /** It cannot, and the sequence said that is fatal; stop here. */
        ABORT
    }

    /** How a step behaves when its action cannot start. Mirrors the engine's builder. */
    public enum Fallback {
        /** Move to the next step. */
        SKIP,
        /** End the whole sequence. */
        ABORT,
        /** Try the substitute; skip if that cannot start either. */
        SUBSTITUTE
    }

    /**
     * The decision, with the telemetry the engine publishes alongside it.
     *
     * @param outcome     what to do
     * @param actionLabel the string published as {@code Action}; unchanged from before the extraction
     * @param fellBack    the value published as {@code FellBack}
     */
    public record Decision(Outcome outcome, String actionLabel, boolean fellBack) {}

    /**
     * Decide one step.
     *
     * <p>Both {@code canStart} flags are booleans rather than suppliers on purpose: evaluating team
     * code is the caller's job, so that a precondition which throws is handled where the guard
     * belongs and this stays a pure function.
     *
     * @param actionName        the step's action name
     * @param actionCanStart    its precondition, already evaluated and guarded
     * @param fallback          what to do when it cannot start
     * @param substituteName    the substitute's name, or null when there is none
     * @param substituteCanStart the substitute's precondition, already evaluated and guarded
     */
    public static Decision decide(String actionName, boolean actionCanStart, Fallback fallback,
                                  String substituteName, boolean substituteCanStart) {
        if (actionCanStart) {
            return new Decision(Outcome.RUN, actionName, false);
        }
        if (fallback == Fallback.SUBSTITUTE && substituteName != null && substituteCanStart) {
            return new Decision(Outcome.SUBSTITUTE, substituteName + " (sub)", true);
        }
        if (fallback == Fallback.ABORT) {
            return new Decision(Outcome.ABORT, "(abort at " + actionName + ")", true);
        }
        // SKIP, and SUBSTITUTE whose substitute is absent or also unable to start.
        return new Decision(Outcome.SKIP, "(skipped " + actionName + ")", true);
    }
}
