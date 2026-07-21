package frc.lib.catalyst.statemachine.goals;

/**
 * The goal type for a {@link frc.lib.catalyst.mechanisms.WinchMechanism} binding.
 *
 * <p>A winch is the least sensed mechanism Catalyst ships. It has no closed loop, no
 * setpoint and no encoder-backed notion of "where it should be" — only a duty cycle that
 * the motor is currently being fed and two coarse limit predicates,
 * {@code isFullyExtended()} and {@code isFullyRetracted()}, that compare the raw position
 * against the configured travel range with a 1 cm dead zone. Every goal below is therefore
 * an <em>open-loop drive command</em> plus a rule for deciding when to stop driving.
 *
 * <p><b>There is deliberately no positional variant.</b> One is not omitted for later; it
 * is omitted because it cannot be implemented honestly. {@code WinchMechanism} exposes no
 * position control whatsoever: its command factories are {@code extend()},
 * {@code retract()}, {@code runAtSpeed(double)} and {@code manualControl(DoubleSupplier)},
 * all of which write a percent output, and none of which accept a target. A
 * {@code WinchGoal.Position(double)} could only be serviced by bit-banging duty cycle
 * against {@code getPosition()} inside the binding, which would put an unturned,
 * untestable bang-bang controller in the state machine layer and hide it behind a goal
 * that <em>looks</em> closed-loop to everyone reading the state table. Teams that need a
 * winch to stop at an intermediate height should configure the mechanism's soft limits, or
 * use {@code LinearMechanism} — which has Motion Magic, a tolerance and a real
 * {@code holdPosition()} — instead.
 *
 * <p>Two consequences of the mechanism's implementation shape every goal here, and binding
 * authors must respect both:
 *
 * <ul>
 *   <li>{@code extend()} and {@code retract()} <b>never end</b>. They are {@code run(...)}
 *       commands with no {@code isFinished}, so a {@code GoalRunner} that hosts one will
 *       drive the winch into its hard stop forever unless arrival swaps in a hold. Every
 *       driving goal below therefore <b>requires</b> a non-null {@code holdCommand} of
 *       {@code m.runAtSpeed(0)}. Returning {@code null} — correct for the closed-loop
 *       mechanisms — would stall a climber motor against a mechanical limit at full
 *       current for the rest of the match.</li>
 *   <li>When {@code Config.spoolRadius <= 0} the mechanism is in rotation mode and the
 *       constructor <b>never applies soft limits</b> to the motor, so the firmware will not
 *       save you either. In that configuration the limit predicates compare rotations
 *       against the range, and they are the only thing standing between the goal and a
 *       broken gearbox.</li>
 * </ul>
 *
 * <p>Value semantics are load-bearing, as for every Catalyst goal type: each variant is a
 * record, so {@code equals}/{@code hashCode} compare components rather than identity and
 * the engine's per-loop {@code Objects.equals(want, active)} check does not rebuild the
 * pursue command fifty times a second.
 *
 * @since 1.2.0
 */
public sealed interface WinchGoal
        permits WinchGoal.Extend, WinchGoal.Retract, WinchGoal.Stop, WinchGoal.Speed {

    /**
     * Drive the winch out at the configured {@code extendSpeed} until
     * {@code isFullyExtended()} reports true.
     *
     * <p>This variant carries no components because the speed is a property of the
     * mechanism's {@code Config}, not of the goal. Putting it here would let two states ask
     * the same climber to extend at two different rates, which reads as a tuning knob but
     * is really a way to desynchronise a dual-motor climber.
     *
     * <p>Arrival is genuinely observable — {@code isFullyExtended()} reads the encoder — so
     * a binding should report {@code observable(goal) == true} for this goal. It is still
     * only accurate to the 1 cm dead zone baked into the predicate, which is worth saying
     * in {@code note}.
     */
    record Extend() implements WinchGoal {}

    /**
     * Drive the winch in at the configured {@code retractSpeed} until
     * {@code isFullyRetracted()} reports true.
     *
     * <p>The mirror of {@link Extend}, and the goal that actually pulls the robot off the
     * floor on a climber. Note that {@code retractSpeed} is configured as a negative duty
     * cycle, so a team that types a positive number into {@code retractSpeed(...)} gets a
     * retract goal that extends; a binding's {@code validate} is the right place to catch
     * that, since it can see the {@code Config} and this goal cannot.
     */
    record Retract() implements WinchGoal {}

    /**
     * Hold the winch at zero output — the safe resting goal.
     *
     * <p>Zero output is not the same as zero motion. The mechanism forces brake mode on
     * every winch motor ({@code brakeMode(true)}, commented "always brake for safety"), so
     * a stopped climber is held by the motor's short-circuit braking and by whatever
     * ratchet or worm gearing the team has, not by a controller. On a back-drivable winch
     * carrying robot weight this goal will still sag, and no amount of state-machine
     * plumbing changes that.
     *
     * <p>Arrival is immediate and unconditional: there is nothing to wait for, so a binding
     * should return {@code true} from {@code atGoal} on the first loop. Because nothing is
     * sensed, {@code observable(goal)} should be {@code false} — the state machine reports
     * that the command was issued, not that the winch is motionless.
     */
    record Stop() implements WinchGoal {}

    /**
     * Drive the winch at an explicit duty cycle for a fixed dwell, then stop.
     *
     * <p>This is the escape hatch for the motions the two limit predicates cannot describe:
     * a partial deploy, a pre-tension pull before a climb, a nudge to unseat a ratchet.
     * Arrival is a <b>settle timer</b> and nothing more — {@code atGoal} is
     * {@code secondsSinceApplied >= seconds()}, evaluated against the parameter the engine
     * supplies rather than {@code Timer.getFPGATimestamp()}, and a binding must report
     * {@code observable(goal) == false} so that nobody reading a log later mistakes an
     * elapsed stopwatch for a sensed position.
     *
     * <p>Because the timer says nothing about where the winch actually went, this goal can
     * and will run the mechanism past a limit if {@code seconds} is generous and the travel
     * is short. A binding should still fold the limit predicates into {@code atGoal} as an
     * early out — {@code dutyCycle > 0 && isFullyExtended()}, and the retract mirror — so
     * that the dwell is an upper bound on driving rather than a promise to keep driving.
     *
     * @param dutyCycle the percent output to command, nominally in {@code [-1, 1]}, where
     *                  positive is the extend direction. Values outside that range are
     *                  <b>deliberately left intact</b> rather than clamped here: a binding's
     *                  build-time {@code validate} can report "duty cycle 1.5 is outside
     *                  [-1, 1]" by name, with the goal's key attached, all at once on a
     *                  laptop, whereas silently clamping to 1.0 in this constructor would
     *                  destroy the evidence and let a typo ship. Only a non-finite value is
     *                  normalised, to {@code 0.0}, because {@code NaN} would propagate
     *                  straight into a motor output request.
     * @param seconds   how long to drive before the goal counts as reached. Negative and
     *                  non-finite values are normalised to {@code 0.0}, since a negative
     *                  dwell has no meaning and clamping is strictly safer than the
     *                  alternative of never arriving. Be aware that a dwell of {@code 0.0}
     *                  makes the goal arrive on its first loop, at which point the hold
     *                  command stops the motor — so a zero-second speed goal is an elaborate
     *                  no-op, and a binding's {@code validate} should flag it as such.
     */
    record Speed(double dutyCycle, double seconds) implements WinchGoal {

        /**
         * Normalises the two values that would otherwise poison arithmetic downstream,
         * while preserving everything a build-time check can diagnose better than a
         * constructor can.
         *
         * <p>Nothing here throws. A goal that throws from its constructor fails at the
         * first line of {@code RobotContainer} with a stack trace pointing at a record,
         * which tells a student nothing; the same mistake routed through
         * {@code Binding.validate} arrives as a named problem in an aggregated
         * {@code StateMachineConfigException} alongside every other configuration error in
         * the state table.
         */
        public Speed {
            if (!Double.isFinite(dutyCycle)) {
                dutyCycle = 0.0;
            }
            if (!Double.isFinite(seconds) || seconds < 0.0) {
                seconds = 0.0;
            }
        }
    }

    /**
     * The canonical {@link Extend} instance.
     *
     * <p>{@code Extend} has no components, so every instance is {@code equals} to every
     * other one and allocating a fresh record per call would be pure garbage. Sharing one
     * is safe because records are immutable.
     */
    Extend EXTEND = new Extend();

    /** The canonical {@link Retract} instance; see {@link #EXTEND} for why it is shared. */
    Retract RETRACT = new Retract();

    /** The canonical {@link Stop} instance; see {@link #EXTEND} for why it is shared. */
    Stop STOP = new Stop();

    /**
     * Drive the winch out at its configured extend speed until the fully-extended limit
     * reports true.
     *
     * <p>Prefer this over {@code new Extend()} at state-table call sites: it reads as prose
     * in a builder chain and it hands back the shared instance instead of allocating.
     *
     * @return the shared {@link Extend} goal
     */
    static WinchGoal extend() {
        return EXTEND;
    }

    /**
     * Drive the winch in at its configured retract speed until the fully-retracted limit
     * reports true.
     *
     * <p>Remember that {@code retractSpeed} is configured negative; this factory does not
     * and cannot check that, because a goal has no view of the mechanism's {@code Config}.
     *
     * @return the shared {@link Retract} goal
     */
    static WinchGoal retract() {
        return RETRACT;
    }

    /**
     * Command zero output and consider the winch immediately arrived.
     *
     * <p>This is the goal a winch should hold in every state that is not actively climbing
     * or deploying. Leaving a winch bound to {@link #extend()} in an idle state is the
     * single most common way to cook a climber motor over a match.
     *
     * @return the shared {@link Stop} goal
     */
    static WinchGoal stop() {
        return STOP;
    }

    /**
     * Drive the winch at an explicit duty cycle for a fixed dwell.
     *
     * <p>Reach for this only when neither limit predicate describes the motion you want.
     * Arrival is a stopwatch, not a measurement, with all the caveats spelled out on
     * {@link Speed}.
     *
     * @param dutyCycle percent output, nominally {@code [-1, 1]}, positive to extend.
     *                  Out-of-range values survive to build-time validation on purpose;
     *                  only non-finite values are normalised to {@code 0.0}.
     * @param seconds   dwell before the goal counts as reached. Negative and non-finite
     *                  values are normalised to {@code 0.0}.
     * @return a {@link Speed} goal carrying the normalised values
     */
    static WinchGoal speed(double dutyCycle, double seconds) {
        return new Speed(dutyCycle, seconds);
    }
}
