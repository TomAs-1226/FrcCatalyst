package frc.lib.catalyst.statemachine.goals;

/**
 * Every goal that a {@code ClawMechanism} binding knows how to pursue.
 *
 * <p>This is a sealed hierarchy of records rather than an enum because three of the six
 * goals carry a number — a settle time or a timeout — that the caller genuinely needs to
 * vary per call site. Records give the value-based {@code equals}/{@code hashCode} that
 * {@link frc.lib.catalyst.statemachine.Binding} requires: the engine calls
 * {@link java.util.Objects#equals} against the active goal on every loop to decide whether
 * to rebuild actuation, so a goal type with identity equality would tear down and rebuild
 * its command fifty times a second and the claw would never actually move.
 *
 * <p>Because this library targets Java 17, pattern-matching {@code switch} is not available
 * (it stayed a preview feature through Java 20). A binding dispatches over this hierarchy
 * with an {@code instanceof} chain, which is final Java 16 syntax and compiles cleanly:
 *
 * <pre>{@code
 * if (g instanceof ClawGoal.Grip grip) {
 *     return claw.closeUntilGripped();
 * } else if (g instanceof ClawGoal.Close close) {
 *     return claw.close();
 * }
 * }</pre>
 *
 * <p>Sealing the interface is what makes that chain safe to review: the permitted subtypes
 * are listed in one place, so a reviewer can confirm by inspection that a dispatch chain
 * handles all six rather than trusting that nobody added a seventh goal in another file.
 *
 * <h2>Why so many goals are settle timers</h2>
 *
 * <p>Three facts about {@code ClawMechanism} shape this hierarchy, and each one is a trap
 * that a naive binding falls into:
 *
 * <p>First, {@code ClawMechanism.open()} sets its grip state to {@code "OPENING"} in the
 * command body and to {@code "OPEN"} only inside {@code finallyDo}. The state machine
 * <em>hosts</em> a pursue command instead of scheduling it, and never lets it finish, so
 * that {@code finallyDo} never runs on the happy path. A binding that tested
 * {@code "OPEN".equals(claw.getGripState())} would therefore wait forever on a claw that
 * had physically been open for ten seconds. Open arrival must be a settle timer, and the
 * binding must report {@code observable(goal) == false} so that a log reader is never
 * fooled into treating that timer as a sensor reading.
 *
 * <p>Second, {@code ClawMechanism.close()} never calls the mechanism's stall-detection
 * update; only {@code closeUntilGripped()} does. Closing therefore has no arrival signal at
 * all — {@code hasPiece()} can never latch under {@code close()} — which is why
 * {@link Close} is a dwell and {@link Grip} is a separate goal rather than a flag on it.
 * Reaching for {@code close()} when you meant "close onto the game piece and keep it" is
 * the single most common way to end a match holding nothing.
 *
 * <p>Third, {@code runAtVoltage} does not touch grip-state tracking whatsoever, so
 * {@link Volts} likewise has nothing to sense and carries its own dwell.
 *
 * <p>{@link Grip} is the one goal in this file with a real sensed arrival: it runs
 * {@code closeUntilGripped()}, so either the beam break or the stall latch can raise
 * {@code hasPiece()} and end the wait honestly.
 *
 * @see frc.lib.catalyst.statemachine.Binding
 * @see frc.lib.catalyst.statemachine.robot.Actuator
 * @since 1.2.0
 */
public sealed interface ClawGoal
        permits ClawGoal.Close, ClawGoal.Grip, ClawGoal.Hold,
                ClawGoal.Open, ClawGoal.Idle, ClawGoal.Volts {

    /**
     * Dwell applied by the zero-argument factories, in seconds.
     *
     * <p>0.30 s is chosen to be comfortably longer than a typical motor-driven claw takes to
     * travel end to end under its configured open or close voltage. It is deliberately a
     * little generous: the cost of an over-long dwell is a few tenths of wasted cycle time,
     * whereas the cost of a short one is a superstructure that advances to the next state
     * while the claw is still mid-travel and shears a game piece against the field element.
     * Teams that have measured their own claw should pass an explicit value.
     */
    double DEFAULT_SETTLE_SECONDS = 0.30;

    /**
     * Fallback timeout substituted when {@link #grip(double)} is handed a value that cannot
     * mean anything, in seconds.
     *
     * <p>This exists only so that a nonsensical input degrades into a bounded wait rather
     * than an instant false arrival. It is not a recommended default, and it is not exposed
     * as a zero-argument {@code grip()} factory — see {@link #grip(double)} for why.
     */
    double DEFAULT_GRIP_TIMEOUT_SECONDS = 2.0;

    /**
     * Normalises a requested dwell into one the arrival logic can actually use.
     *
     * <p>A negative dwell and a {@code NaN} dwell are both unrepresentable as a wait: the
     * comparison {@code secondsSinceApplied >= settleSeconds} would be trivially true for the
     * former and permanently false for the latter, so a {@code NaN} would hang the state
     * machine on this goal for the rest of the match. Both collapse to
     * {@link #DEFAULT_SETTLE_SECONDS}.
     *
     * <p>Zero is passed through untouched, because "arrive the instant this is applied" is a
     * coherent thing to ask for on a fast claw and the caller who typed {@code 0.0} meant it.
     *
     * @param requested the dwell as supplied by the caller
     * @return a finite, non-negative dwell in seconds
     */
    private static double normaliseSettle(double requested) {
        return (Double.isNaN(requested) || requested < 0.0) ? DEFAULT_SETTLE_SECONDS : requested;
    }

    /**
     * Close the claw at its configured close voltage and keep closing.
     *
     * <p>Arrival is a dwell of {@link #DEFAULT_SETTLE_SECONDS}, not a measurement, for the
     * reason given in the type documentation: {@code ClawMechanism.close()} never runs stall
     * detection, so there is nothing to sense.
     *
     * <p>If what you actually want is "close onto the game piece and keep hold of it", this
     * is the wrong goal and {@link #grip(double)} is the right one. Using {@code close()} for
     * that fails quietly and late — the claw squeezes, {@code hasPiece()} never latches, no
     * fault is raised, and the mistake surfaces as a dropped piece on the way to the scoring
     * position.
     *
     * @return a close goal with the default dwell
     */
    static ClawGoal close() {
        return new Close(DEFAULT_SETTLE_SECONDS);
    }

    /**
     * Close the claw at its configured close voltage, with an explicit dwell before the state
     * machine treats the claw as closed.
     *
     * <p>Use this when the default dwell is visibly wrong for your claw — a long-travel
     * gripper that needs half a second, or a stubby one that is shut in 0.10 s and should not
     * hold up the rest of the superstructure.
     *
     * @param settleSeconds seconds to dwell before reporting arrival. Negative and
     *                      {@code NaN} values are replaced with {@link #DEFAULT_SETTLE_SECONDS};
     *                      zero is honoured as an immediate arrival.
     * @return a close goal with the given dwell
     */
    static ClawGoal close(double settleSeconds) {
        return new Close(settleSeconds);
    }

    /**
     * Close onto a game piece using stall and beam-break detection, then hold it.
     *
     * <p>This is the only goal in this file whose arrival is genuinely sensed. It maps to
     * {@code ClawMechanism.closeUntilGripped()}, which is the only command factory on the
     * mechanism that runs the stall-detection update, and which drops to the low passive hold
     * voltage of its own accord once {@code hasPiece()} raises — so the motor stops trying to
     * squeeze harder against a piece it already has.
     *
     * <p><b>There is deliberately no zero-argument form.</b> The timeout is a mandatory
     * constructor argument precisely so that "wait forever for a piece" cannot be written
     * down. An unbounded grip is the classic way to lose an autonomous routine: the claw
     * closes on empty air because the piece was knocked away in the first two seconds of the
     * match, {@code hasPiece()} never raises, and the superstructure sits in that state until
     * the buzzer while every downstream state waits behind it. Forcing the author to name a
     * number makes them think about what should happen when the piece is not there.
     *
     * <p>Note that reaching the timeout counts as arrival, not as a fault — the state machine
     * moves on. A binding for this goal should say so in its {@code note}, and a routine that
     * cares about the difference should check {@code hasPiece()} itself afterwards rather
     * than inferring possession from having left this state.
     *
     * @param maxSeconds the longest the machine will wait for a piece before giving up and
     *                   reporting arrival anyway. Values that are not positive and finite
     *                   are replaced with {@link #DEFAULT_GRIP_TIMEOUT_SECONDS}, since a
     *                   zero or negative timeout would mean "abandon the grip before the
     *                   claw has moved" and is never what anyone intends.
     * @return a grip goal bounded by the given timeout
     */
    static ClawGoal grip(double maxSeconds) {
        return new Grip(maxSeconds);
    }

    /**
     * Apply the configured low passive hold voltage and nothing else.
     *
     * <p>Arrival is immediate: applying a voltage is instantaneous and there is no travel to
     * wait on, so this goal costs the superstructure no cycle time.
     *
     * <p>Reach for this when possession is already established and you want the claw to keep
     * its grip through a long traversal without re-running detection — for instance after a
     * {@link #grip(double)} has succeeded and the piece has settled deep enough in the
     * gripper that the beam break flickers. Holding at the passive voltage rather than the
     * full close voltage is what keeps the motor from cooking over a fifteen-second haul.
     *
     * @return the hold goal
     */
    static ClawGoal hold() {
        return Hold.INSTANCE;
    }

    /**
     * Open the claw at its configured open voltage, releasing any piece.
     *
     * <p>Arrival is a dwell of {@link #DEFAULT_SETTLE_SECONDS} and a binding for this goal
     * must report itself unobservable. This is not laziness: as described on the type,
     * {@code ClawMechanism.open()} publishes {@code "OPEN"} only from {@code finallyDo}, and
     * a hosted pursue command is never allowed to finish, so that string is unreachable while
     * the state machine is driving. A binding that waited for it would deadlock the
     * superstructure on a claw that is standing wide open.
     *
     * @return an open goal with the default dwell
     */
    static ClawGoal open() {
        return new Open(DEFAULT_SETTLE_SECONDS);
    }

    /**
     * Open the claw at its configured open voltage, with an explicit dwell before the state
     * machine treats the claw as open.
     *
     * <p>This dwell is worth tuning more often than the closing one, because it is what
     * separates the release from whatever motion follows it. Set it too short and the arm
     * starts retracting while the claw is still fouling the game piece it just placed.
     *
     * @param settleSeconds seconds to dwell before reporting arrival. Negative and
     *                      {@code NaN} values are replaced with {@link #DEFAULT_SETTLE_SECONDS};
     *                      zero is honoured as an immediate arrival.
     * @return an open goal with the given dwell
     */
    static ClawGoal open(double settleSeconds) {
        return new Open(settleSeconds);
    }

    /**
     * Stop driving the claw entirely.
     *
     * <p>Arrival is immediate. This is the explicit "no voltage" goal, and it exists so that
     * a state which does not care about the claw can still say so out loud. Leaving a claw
     * parked on {@link #hold()} when there is no piece in it means burning the passive hold
     * voltage into a closed gripper for as long as that state is active, which is a slow but
     * entirely avoidable way to overheat a motor.
     *
     * @return the idle goal
     */
    static ClawGoal idle() {
        return Idle.INSTANCE;
    }

    /**
     * Drive the claw at an arbitrary voltage.
     *
     * <p>This is the escape hatch, meant for characterisation, for a manual-override state
     * bound to an operator trigger, and for the occasional mechanism whose real behaviour the
     * five named goals do not describe. It maps to {@code ClawMechanism.runAtVoltage}, which
     * does not touch grip-state tracking at all, so there is nothing whatsoever to sense and
     * arrival is necessarily a dwell.
     *
     * <p>Prefer a named goal wherever one fits. A superstructure whose states are described
     * in volts rather than in intentions is one that nobody can debug from a log, because the
     * log no longer records what the robot was trying to do.
     *
     * @param volts         the voltage to apply. This is <b>not</b> normalised or clamped,
     *                      deliberately: silently rewriting a physical setpoint would hide
     *                      exactly the mistake that a binding's build-time {@code validate}
     *                      is there to catch on a laptop, where it can be read and fixed,
     *                      rather than diagnosed from robot behaviour in a pit.
     * @param settleSeconds seconds to dwell before reporting arrival. Negative and
     *                      {@code NaN} values are replaced with {@link #DEFAULT_SETTLE_SECONDS};
     *                      zero is honoured as an immediate arrival.
     * @return a voltage goal
     */
    static ClawGoal volts(double volts, double settleSeconds) {
        return new Volts(volts, settleSeconds);
    }

    /**
     * Close the claw and keep closing, treating the claw as closed after a fixed dwell.
     *
     * @param settleSeconds seconds to dwell before arrival; normalised by the compact
     *                      constructor so that the value stored here is always finite and
     *                      non-negative
     */
    record Close(double settleSeconds) implements ClawGoal {

        /**
         * Normalises the dwell so that arrival logic downstream never has to defend itself
         * against a {@code NaN} or a negative. Nothing is thrown here — an out-of-character
         * dwell is reported far more usefully by a binding's build-time {@code validate},
         * which can aggregate every complaint about every goal into one exception instead of
         * failing the robot on whichever bad value happened to be constructed first.
         */
        public Close {
            settleSeconds = ClawGoal.normaliseSettle(settleSeconds);
        }
    }

    /**
     * Close onto a game piece with detection running, bounded by a mandatory timeout.
     *
     * @param maxSeconds the longest the machine waits for {@code hasPiece()} before reporting
     *                   arrival regardless; normalised by the compact constructor so that it
     *                   is always positive and finite
     */
    record Grip(double maxSeconds) implements ClawGoal {

        /**
         * Normalises the timeout upward rather than downward. A non-positive or {@code NaN}
         * timeout is not a stricter version of this goal, it is a broken one — it would
         * either declare arrival before the claw had moved at all, or never declare it — so
         * the value collapses to {@link ClawGoal#DEFAULT_GRIP_TIMEOUT_SECONDS}. As above,
         * this does not throw; build-time validation reports the mistake better.
         */
        public Grip {
            if (Double.isNaN(maxSeconds) || maxSeconds <= 0.0) {
                maxSeconds = ClawGoal.DEFAULT_GRIP_TIMEOUT_SECONDS;
            }
        }
    }

    /**
     * Apply the configured passive hold voltage. Arrival is immediate.
     */
    record Hold() implements ClawGoal {

        /**
         * The single shared instance handed out by {@link ClawGoal#hold()}.
         *
         * <p>A component-less record already compares equal to every other instance of
         * itself, so this is purely about not allocating: goals are constructed on paths that
         * run every loop, and the state machine is expected to be allocation-free in steady
         * state so that the garbage collector never contributes to loop overrun.
         */
        private static final Hold INSTANCE = new Hold();
    }

    /**
     * Open the claw, treating it as open after a fixed dwell.
     *
     * @param settleSeconds seconds to dwell before arrival; normalised by the compact
     *                      constructor so that the value stored here is always finite and
     *                      non-negative
     */
    record Open(double settleSeconds) implements ClawGoal {

        /** Normalises the dwell exactly as {@link Close} does, and for the same reasons. */
        public Open {
            settleSeconds = ClawGoal.normaliseSettle(settleSeconds);
        }
    }

    /**
     * Stop driving the claw. Arrival is immediate.
     */
    record Idle() implements ClawGoal {

        /** The single shared instance handed out by {@link ClawGoal#idle()}. */
        private static final Idle INSTANCE = new Idle();
    }

    /**
     * Drive the claw at an arbitrary voltage, treating the goal as reached after a dwell.
     *
     * @param volts         the voltage to apply, stored exactly as supplied
     * @param settleSeconds seconds to dwell before arrival; normalised by the compact
     *                      constructor so that the value stored here is always finite and
     *                      non-negative
     */
    record Volts(double volts, double settleSeconds) implements ClawGoal {

        /**
         * Normalises only the dwell. The voltage is left alone on purpose — see
         * {@link ClawGoal#volts(double, double)} for why a physical setpoint is never
         * silently rewritten.
         */
        public Volts {
            settleSeconds = ClawGoal.normaliseSettle(settleSeconds);
        }
    }
}
