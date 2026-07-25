package frc.lib.catalyst.statemachine.goals;

/**
 * Every goal a {@code RollerMechanism} binding knows how to pursue.
 *
 * <p>This is a sealed hierarchy of records rather than an enum because half of these goals
 * carry a number — a timeout, a duty cycle, a voltage — and the state machine engine compares
 * goals with {@link java.util.Objects#equals} on every loop to decide whether actuation needs
 * to be re-applied. Records give that comparison value semantics for free. A goal type built
 * from anything with identity equality (an array component being the classic mistake) would
 * compare unequal to itself every iteration and rebuild its command fifty times a second.
 *
 * <p>The shape of this hierarchy is dictated by three facts about {@code RollerMechanism} that
 * are not obvious from its API, and each of them is the reason some goal here exists:
 *
 * <ul>
 *   <li>{@code RollerMechanism.hasPiece()} early-returns on the beam break whenever one is
 *       configured, which makes the stall-current latch unreachable dead code on those robots.
 *       A roller with neither a beam break nor a stall threshold can therefore <em>never</em>
 *       report a game piece, and a "run until you have it" goal on such a robot would wait
 *       forever. That is why {@link IntakeUntilPiece} demands an explicit timeout and offers
 *       no zero-argument form: the state machine must always have a way out.</li>
 *   <li>{@code RollerMechanism.intake()} calls {@code updateStallDetection()} in its run body
 *       and ends itself on {@code hasPiece()}; {@code intakeContinuous()} does neither. So a
 *       roller running under {@link IntakeContinuous} will never latch a piece by stall current
 *       even when stall detection is configured — only a beam break can see one. Choosing
 *       between the two goals is therefore a real decision about detection, not just a
 *       preference about who stops the rollers.</li>
 *   <li>Nothing a roller does is positional. There is no sensor that says "the eject finished"
 *       or "the rollers reached the commanded duty cycle" in any meaningful sense, so most of
 *       these goals arrive on a settle timer and their binding reports
 *       {@code observable() == false}. That is also why the open-loop goals carry their own
 *       settle time instead of inheriting a single global one — a 0.25 s indexer nudge and a
 *       1.5 s deep eject are the same goal type with genuinely different dwell.</li>
 * </ul>
 *
 * <p>Times on this type are always seconds measured from the moment the goal was applied, which
 * the binding reads from the {@code secondsSinceApplied} parameter it is handed. No goal here
 * captures a timestamp and no binding for one may call {@code Timer.getFPGATimestamp()}; doing
 * so would make arrival untestable off-robot and would keep counting through a disabled period.
 *
 * @since 1.2.0
 */
public sealed interface RollerGoal
        permits RollerGoal.IntakeUntilPiece,
                RollerGoal.IntakeContinuous,
                RollerGoal.Eject,
                RollerGoal.Idle,
                RollerGoal.Speed,
                RollerGoal.FeedVolts {

    /**
     * Timeout substituted when {@link #intakeUntilPiece(double)} is handed a value that could
     * not possibly serve as a timeout. Five seconds is longer than any credible intake cycle
     * and short enough that an autonomous routine still recovers within a scoring window.
     */
    double DEFAULT_INTAKE_TIMEOUT_SECONDS = 5.0;

    /**
     * Dwell substituted when {@link #eject(double)} is handed a value that would make the eject
     * finish before the rollers had turned. Half a second clears a piece on essentially every
     * roller geometry a rookie team is likely to build.
     */
    double DEFAULT_EJECT_SECONDS = 0.5;

    // =========================================================================
    //                                 GOALS
    // =========================================================================

    /**
     * Run the intake until a game piece is detected, giving up after {@code maxSeconds}.
     *
     * <p>The timeout is not a safety net bolted on for tidiness — it is the only thing standing
     * between a mis-wired beam break and a state machine that sits in {@code INTAKING} for the
     * rest of the match. A binding for this goal reports arrival when either
     * {@code hasPiece()} is true or {@code secondsSinceApplied >= maxSeconds}, and it should say
     * which of the two happened in {@code note()} so a log reader can tell a successful pickup
     * from an expiry.
     *
     * @param maxSeconds how long to keep trying before declaring arrival anyway. Non-positive
     *                   and non-finite values are replaced with
     *                   {@link #DEFAULT_INTAKE_TIMEOUT_SECONDS}, because a zero timeout would
     *                   arrive on the first loop and quietly convert this goal into a no-op.
     */
    record IntakeUntilPiece(double maxSeconds) implements RollerGoal {

        /**
         * Normalises the timeout so that a mistyped or defaulted value degrades into a long
         * timeout rather than an instant false arrival. This deliberately does not throw: a
         * badly chosen but usable timeout is exactly the kind of thing the binding's
         * {@code validate} hook reports better, at build time, alongside every other problem.
         */
        public IntakeUntilPiece {
            maxSeconds = positiveOr(maxSeconds, DEFAULT_INTAKE_TIMEOUT_SECONDS);
        }
    }

    /**
     * Run the intake rollers and keep running them until the state is left.
     *
     * <p>Use this when something other than the roller decides when intaking stops — a driver
     * releasing a button, a handoff sequence, a note already seated further down the path. Be
     * aware that the underlying {@code intakeContinuous()} command never calls
     * {@code updateStallDetection()}, so on a stall-detection-only robot {@code hasPiece()} will
     * stay false throughout. Arrival for this goal is a short settle timer and its binding
     * reports {@code observable() == false}; treating it as a piece detector is a mistake this
     * javadoc exists to prevent.
     */
    record IntakeContinuous() implements RollerGoal {

        /**
         * The single shared instance. A record with no components is immutable and all of its
         * instances are {@code equals}, so sharing one costs nothing and saves the engine's
         * per-loop comparison an allocation it never needed.
         */
        public static final IntakeContinuous INSTANCE = new IntakeContinuous();
    }

    /**
     * Run the rollers in the eject direction for a fixed dwell.
     *
     * <p>Ejecting is purely open loop. {@code RollerMechanism.eject()} clears the stall latch
     * and spins the motor, but nothing observes the piece leaving, so arrival here is a timer
     * and the binding must supply a {@code holdCommand} that stops the motor. Without that hold
     * the rollers keep grinding in the eject direction for as long as the state is held, which
     * on a compliant-wheel intake is how teams cook a motor between matches.
     *
     * @param seconds how long to spin before declaring the piece gone. Non-positive and
     *                non-finite values become {@link #DEFAULT_EJECT_SECONDS}, since an eject
     *                that arrives immediately never actually ejects anything.
     */
    record Eject(double seconds) implements RollerGoal {

        /** Normalises the dwell so a zero or {@code NaN} cannot turn an eject into a no-op. */
        public Eject {
            seconds = positiveOr(seconds, DEFAULT_EJECT_SECONDS);
        }
    }

    /**
     * Hold the rollers stopped.
     *
     * <p>This is the resting goal and the one every state that does not care about the roller
     * should name explicitly. Leaving a roller unbound is not the same thing: an unbound roller
     * keeps whatever the previous state left it doing, whereas {@code Idle} asserts zero output
     * and arrives immediately. A binding for this goal is trivially at goal, which is what makes
     * it safe to use as the roller's contribution to a state that is really about something
     * else.
     */
    record Idle() implements RollerGoal {

        /** The single shared instance; see {@link IntakeContinuous#INSTANCE} for why. */
        public static final Idle INSTANCE = new Idle();
    }

    /**
     * Run the rollers at an explicit duty cycle for a settle time.
     *
     * <p>The escape hatch for everything the named goals do not cover: a slow indexer creep, a
     * reverse nudge to unjam, a half-speed hand-off. Duty cycle is battery-dependent, so prefer
     * {@link FeedVolts} whenever the behaviour needs to be repeatable between a fresh battery
     * and a tired one.
     *
     * @param dutyCycle     commanded output in {@code [-1, 1]}, positive in the intake
     *                      direction. Deliberately <b>not</b> clamped here: an out-of-range duty
     *                      cycle is a configuration error, and the binding's {@code validate}
     *                      hook reports it at build time with the offending state's name
     *                      attached, which is far more useful than a value silently becoming
     *                      1.0 and appearing to work.
     * @param settleSeconds how long after application to declare arrival. Zero is legitimate and
     *                      means "arrived immediately"; negative and non-finite values are
     *                      normalised to zero.
     */
    record Speed(double dutyCycle, double settleSeconds) implements RollerGoal {

        /**
         * Normalises only the settle time. The duty cycle is passed through untouched on
         * purpose — see the parameter documentation above for why clamping it would hide a
         * problem that build-time validation reports better.
         */
        public Speed {
            settleSeconds = nonNegativeOrZero(settleSeconds);
        }
    }

    /**
     * Run the rollers at an explicit voltage for a settle time.
     *
     * <p>Voltage control is the right choice for feeding: a duty cycle of 0.4 is a different
     * roller surface speed at 12.6 V than it is at 11.2 V late in a match, and a feed that
     * changes speed as the battery sags is a feed that eventually mis-times a shot. Commanding
     * volts makes the behaviour repeatable until the battery can no longer supply them.
     *
     * @param volts         commanded voltage, conventionally within {@code [-12, 12]} and
     *                      positive in the intake direction. Not clamped, for the same reason
     *                      {@link Speed#dutyCycle()} is not: build-time validation names the
     *                      state that got it wrong.
     * @param settleSeconds how long after application to declare arrival. Zero is legitimate;
     *                      negative and non-finite values are normalised to zero.
     */
    record FeedVolts(double volts, double settleSeconds) implements RollerGoal {

        /** Normalises only the settle time; {@code volts} is left for {@code validate} to judge. */
        public FeedVolts {
            settleSeconds = nonNegativeOrZero(settleSeconds);
        }
    }

    // =========================================================================
    //                               FACTORIES
    // =========================================================================

    /**
     * Intake until a piece is detected or {@code maxSeconds} elapses.
     *
     * <p>There is no zero-argument form of this factory and there will not be one. A roller
     * whose detection is broken, unconfigured, or simply looking at an empty intake would park
     * the whole state machine in this state indefinitely, and a state machine that cannot leave
     * a state is worse than one that leaves it early. Naming a timeout at every call site forces
     * the author to decide how long is too long.
     *
     * @param maxSeconds the give-up time in seconds; non-positive and non-finite values fall
     *                   back to {@link #DEFAULT_INTAKE_TIMEOUT_SECONDS}
     * @return an intake goal that always terminates
     */
    static RollerGoal intakeUntilPiece(double maxSeconds) {
        return new IntakeUntilPiece(maxSeconds);
    }

    /**
     * Intake for as long as the state is held, without waiting for detection.
     *
     * <p>Correct when the exit condition lives somewhere else — a button release, a downstream
     * sensor, the end of an autonomous path. Remember that the underlying command does not run
     * stall detection, so a robot relying on stall current will not notice a piece while this
     * goal is active.
     *
     * @return the shared continuous-intake goal
     */
    static RollerGoal intakeContinuous() {
        return IntakeContinuous.INSTANCE;
    }

    /**
     * Eject for a fixed dwell.
     *
     * <p>Pick the dwell by timing the slowest piece off the rollers and adding margin; there is
     * no sensor to fall back on if it is too short, and the state simply moves on with the piece
     * still aboard.
     *
     * @param seconds dwell in seconds; non-positive and non-finite values fall back to
     *                {@link #DEFAULT_EJECT_SECONDS}
     * @return an eject goal
     */
    static RollerGoal eject(double seconds) {
        return new Eject(seconds);
    }

    /**
     * Stop the rollers and hold them stopped.
     *
     * <p>Name this explicitly in every state that does not want the roller moving, rather than
     * omitting the roller from the state. Omission inherits the previous state's behaviour;
     * this asserts zero.
     *
     * @return the shared idle goal
     */
    static RollerGoal idle() {
        return Idle.INSTANCE;
    }

    /**
     * Run at an explicit duty cycle, arriving after {@code settleSeconds}.
     *
     * <p>Prefer {@link #feed(double, double)} when the behaviour must survive a sagging battery
     * unchanged.
     *
     * @param dutyCycle     output in {@code [-1, 1]}, positive toward intake; range is checked
     *                      at build time rather than clamped here
     * @param settleSeconds seconds after application at which the goal is considered reached;
     *                      negative and non-finite values are normalised to zero
     * @return a duty-cycle goal
     */
    static RollerGoal speed(double dutyCycle, double settleSeconds) {
        return new Speed(dutyCycle, settleSeconds);
    }

    /**
     * Feed at an explicit voltage, arriving after {@code settleSeconds}.
     *
     * <p>This is the repeatable option. Use it for anything whose timing another mechanism
     * depends on — handing a piece to a shooter, indexing to a known seat — because voltage
     * control keeps the roller surface speed steady as the battery drops.
     *
     * @param volts         commanded voltage, conventionally within {@code [-12, 12]} and
     *                      positive toward intake; range is checked at build time rather than
     *                      clamped here
     * @param settleSeconds seconds after application at which the goal is considered reached;
     *                      negative and non-finite values are normalised to zero
     * @return a voltage feed goal
     */
    static RollerGoal feed(double volts, double settleSeconds) {
        return new FeedVolts(volts, settleSeconds);
    }

    // =========================================================================
    //                             NORMALISATION
    // =========================================================================

    /**
     * Returns {@code value} when it is a finite, strictly positive duration, and {@code fallback}
     * otherwise. Used for durations where zero is not a meaningful answer because it would
     * collapse the goal into an instant arrival that never actuated anything.
     */
    private static double positiveOr(double value, double fallback) {
        return (Double.isFinite(value) && value > 0.0) ? value : fallback;
    }

    /**
     * Returns {@code value} when it is a finite, non-negative duration, and {@code 0.0}
     * otherwise. Used for settle times, where zero genuinely means "arrived on application" and
     * only negatives and {@code NaN} need correcting.
     */
    private static double nonNegativeOrZero(double value) {
        return (Double.isFinite(value) && value > 0.0) ? value : 0.0;
    }
}
