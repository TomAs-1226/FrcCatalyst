package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.FlywheelMechanism;
import frc.lib.catalyst.statemachine.goals.FlywheelGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;

import java.util.Set;
import java.util.function.Consumer;

/**
 * Binds a {@link FlywheelMechanism} to the state machine as an {@link Actuator} over
 * {@link FlywheelGoal}.
 *
 * <p>A flywheel is the simplest mechanism in the library to actuate and one of the most
 * treacherous to arrival-check, because {@link FlywheelMechanism#atSpeed()} is not the
 * general-purpose predicate its name suggests. Two properties of it shape almost everything
 * below, and both are verified against the mechanism source rather than inferred from its
 * javadoc.
 *
 * <h2>Why a stopped flywheel needs its own arrival branch</h2>
 *
 * <p>{@code atSpeed()} opens with {@code if (primarySetpointRPS == 0) return false;}. A flywheel
 * that has been commanded to stop therefore reports "not at speed" for the rest of the match, and
 * a state whose only gating binding is a spun-down shooter would never be reported as reached —
 * the machine would sit in transition forever. Arrival for {@link FlywheelGoal#isIdle()} is
 * consequently tested directly against the measured velocity and the goal's own
 * {@link FlywheelGoal#idleToleranceRPS()}, which is exactly the shape the goal type was designed
 * for. The tolerance is not decoration: a coasting flywheel takes seconds to wind down and never
 * settles at a true {@code 0.0}, so a zero-width band would be as unreachable as {@code atSpeed()}
 * itself.
 *
 * <h2>Why the non-idle branch also consults the mechanism's setpoint</h2>
 *
 * <p>{@code atSpeed()} compares the live velocity against the mechanism's <em>current</em>
 * setpoint field, not against any goal handed to it. Used bare it would answer "is this flywheel
 * at whatever speed it happens to be chasing", which is not the same question as "is this
 * flywheel at the speed <em>this goal</em> asks for". That distinction is invisible for the goal
 * the machine is currently applying — the setpoint is that goal's speed, so the two questions
 * coincide — but {@code Binding.atGoal} is contractually called for the goals of states the
 * machine is <b>not</b> in, and there the bare form is wrong in the worst direction: it reports
 * arrival at a 20 RPS goal while the shooter is holding a 60 RPS shot, so
 * {@code isAt(SOME_OTHER_STATE)} would read true for every speed goal on the robot at once.
 *
 * <p>A fixed goal therefore additionally requires the mechanism to actually be chasing that
 * goal's primary speed, read live from {@link FlywheelMechanism#getSetpoint()}. This keeps
 * {@code atGoal} a pure function of live state and the goal argument, adds no field and no
 * memory of "what we last applied", and cannot change the answer for the goal the machine is
 * genuinely pursuing. A tracked goal skips the check, because its setpoint is whatever its
 * supplier last produced and comparing that to anything fixed is meaningless.
 *
 * <p>The one visible consequence is a single loop of latency: {@code spinUp} writes the setpoint
 * in its {@code run} body, so the setpoint matches the goal from the first {@code execute} rather
 * than from {@code initialize}. Twenty milliseconds against a spin-up measured in seconds.
 *
 * <h2>Tolerance is reported, not read</h2>
 *
 * <p>{@code FlywheelMechanism.Config.velocityTolerance} is package-private with no accessor, so
 * this binding cannot see the band {@code atSpeed()} actually applies. {@link #tolerance} exists
 * only to label a log, never to decide arrival — that decision stays inside the mechanism where
 * the real number lives — so the mirror supplied to the constructor being stale costs a
 * cosmetically wrong number on a dashboard and nothing else. Pass the same value given to
 * {@code Config.Builder.velocityTolerance} when you care about that number being right.
 *
 * @since 1.2.0
 */
public final class FlywheelBinding implements Actuator<FlywheelGoal> {

    /**
     * Tolerance reported by {@link #tolerance} when the caller supplies none, matching the
     * default in {@code FlywheelMechanism.Config.Builder}.
     *
     * <p>Kept in sync by hand because the mechanism's tolerance field is package-private. It is a
     * logging label only; see the type javadoc.
     */
    public static final double DEFAULT_VELOCITY_TOLERANCE_RPS = 3.0;

    /**
     * Width of the band within which the mechanism's live setpoint is considered to be the
     * goal's setpoint.
     *
     * <p>{@code spinUp} writes the goal's {@code double} through unmodified, so exact equality
     * would in fact hold; this exists so that a future setpoint path which rounds, ramps or
     * slews by a hair does not silently strand the machine in transition.
     */
    private static final double SETPOINT_MATCH_EPSILON_RPS = 1e-6;

    private final FlywheelMechanism mechanism;
    private final String key;
    private final double velocityToleranceRPS;

    /**
     * Creates a binding reporting {@link #DEFAULT_VELOCITY_TOLERANCE_RPS} as its tolerance.
     *
     * <p>Use {@link #FlywheelBinding(FlywheelMechanism, String, double)} when the mechanism was
     * configured with a non-default {@code velocityTolerance} and you want logs to say so.
     *
     * @param mechanism the flywheel this binding drives; must not be {@code null}
     * @param key       stable, unique, log-safe key; must not be {@code null} or blank
     */
    public FlywheelBinding(FlywheelMechanism mechanism, String key) {
        this(mechanism, key, DEFAULT_VELOCITY_TOLERANCE_RPS);
    }

    /**
     * Creates a binding that reports {@code velocityToleranceRPS} as the at-speed band.
     *
     * <p>This value is never used to decide arrival — {@code atSpeed()} applies the mechanism's
     * own configured tolerance internally and this binding has no way to read it. Supplying a
     * wrong number here therefore mislabels a log and changes no behaviour whatsoever.
     *
     * @param mechanism            the flywheel this binding drives; must not be {@code null}
     * @param key                  stable, unique, log-safe key; must not be {@code null} or blank
     * @param velocityToleranceRPS the band {@code atSpeed()} applies, for logging only. A
     *                             non-positive or non-finite value is replaced with
     *                             {@link #DEFAULT_VELOCITY_TOLERANCE_RPS}
     * @throws IllegalArgumentException if {@code mechanism} is {@code null} or {@code key} is
     *                                  {@code null} or blank. Both are build-time programming
     *                                  errors that would otherwise surface as a null pointer
     *                                  somewhere inside the scheduler at 50 Hz
     */
    public FlywheelBinding(FlywheelMechanism mechanism, String key, double velocityToleranceRPS) {
        if (mechanism == null) {
            throw new IllegalArgumentException("FlywheelBinding requires a mechanism");
        }
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException(
                    "FlywheelBinding requires a non-blank key (mechanism "
                            + mechanism.getMechanismName() + ")");
        }
        this.mechanism = mechanism;
        this.key = key;
        this.velocityToleranceRPS =
                (velocityToleranceRPS > 0.0 && Double.isFinite(velocityToleranceRPS))
                        ? velocityToleranceRPS
                        : DEFAULT_VELOCITY_TOLERANCE_RPS;
    }

    /** {@inheritDoc} */
    @Override
    public String key() {
        return key;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Matches the kind {@code FlywheelMechanism.describe()} publishes, so the state machine's
     * binding telemetry and the simulation dashboard agree on how to draw this mechanism.
     */
    @Override
    public String kind() {
        return "flywheel";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Rotations per second throughout. {@link FlywheelGoal} converts RPM at its own boundary
     * precisely so that nothing downstream of it — including this binding — ever has to.
     */
    @Override
    public String unit() {
        return "rps";
    }

    /**
     * Builds a fresh command driving the flywheel toward {@code goal}.
     *
     * <p>Four cases, tested in an order that matters only in that a tracked goal is checked
     * first: {@link FlywheelGoal#isIdle()} is false whenever a supplier is attached, so the
     * remaining branches cannot steal a tracked goal from the first.
     *
     * <ul>
     *   <li>A tracked goal becomes {@link FlywheelMechanism#track}, which re-reads the supplier
     *       every loop — the shoot-on-the-fly case, where the required speed is a function of
     *       range and is not known when the state machine is declared.</li>
     *   <li>An idle goal becomes {@link FlywheelMechanism#stopCommand()}. That command is a
     *       {@code runOnce} and so ends immediately, which is permitted because its effect is
     *       persistent: the motors are stopped and the setpoints zeroed, and nothing re-commands
     *       them. Arrival then follows on its own as the wheel coasts into the idle band, which
     *       is why {@link #reassertPeriodLoops()} stays at zero — re-issuing a stop to an
     *       already-stopped motor would achieve nothing but log noise.</li>
     *   <li>A goal whose second wheel differs becomes the two-argument
     *       {@link FlywheelMechanism#spinUp(double, double)}, for backspin or topspin. On a
     *       mechanism configured without a second motor that overload quietly delegates to the
     *       single-wheel one; {@link #validate} turns that silent degradation into a build
     *       failure rather than letting the robot shoot a different shot than the one asked for.</li>
     *   <li>Anything else becomes the single-argument
     *       {@link FlywheelMechanism#spinUp(double)}, which commands both wheels alike.</li>
     * </ul>
     *
     * <p>Every branch constructs a new command object, as the hosted-command contract requires.
     * Nothing here can throw: the supplier attached to a tracked goal is invoked inside the
     * command's {@code run} body by the scheduler, never while the command is being built, and a
     * {@code null} goal degrades to a stop rather than to a {@code NullPointerException}
     * propagating out of {@code CommandScheduler.run()} and taking the robot loop with it.
     *
     * @param goal the goal to pursue
     * @return a fresh command; never {@code null}
     */
    @Override
    public Command pursueCommand(FlywheelGoal goal) {
        if (goal == null) {
            return mechanism.stopCommand();
        }
        if (goal.tracked() != null) {
            return mechanism.track(goal.tracked());
        }
        if (goal.isIdle()) {
            return mechanism.stopCommand();
        }
        // Dispatch on whether the two wheels are asked for DIFFERENT speeds, not on the secondary
        // being non-zero: FlywheelMechanism.spinUp(double) drives BOTH wheels to the primary speed, so
        // rps(50, 0) — "primary at 50, secondary stopped" — must reach the two-argument overload, or
        // the second wheel would spin at 50 instead of stopping. rps(50) folds to rps(50, 50), which
        // correctly takes the single-argument path.
        if (goal.secondaryRPS() != goal.primaryRPS()) {
            return mechanism.spinUp(goal.primaryRPS(), goal.secondaryRPS());
        }
        return mechanism.spinUp(goal.primaryRPS());
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code null}: the pursue command keeps running after arrival, which is what a
     * flywheel wants. {@code spinUp} and {@code track} are closed-loop velocity commands, so
     * holding them costs nothing and stopping them would spin the wheel down the instant it
     * reached speed — the opposite of the intent. This is the closed-loop case the
     * {@link Actuator#holdCommand} contract describes, not the open-loop winch case that needs a
     * distinct hold.
     *
     * @param goal the goal that has been reached
     * @return {@code null}, meaning keep pursuing
     */
    @Override
    public Command holdCommand(FlywheelGoal goal) {
        return null;
    }

    /**
     * Has the flywheel reached {@code goal}?
     *
     * <p>Three branches, for the three reasons given at length on the type:
     *
     * <ul>
     *   <li><b>Idle</b> is measured directly, because {@code atSpeed()} is hard-false at a zero
     *       setpoint and would leave a spun-down shooter permanently unarrived.</li>
     *   <li><b>Tracked</b> defers to {@code atSpeed()} unqualified. Its setpoint is whatever the
     *       supplier last produced, so there is no fixed number to corroborate it against.</li>
     *   <li><b>Fixed</b> requires {@code atSpeed()} <em>and</em> that the mechanism is chasing
     *       this goal's primary speed, so that the answer is about this goal rather than about
     *       whichever goal happens to own the flywheel right now.</li>
     * </ul>
     *
     * <p>{@code secondsSinceApplied} is deliberately unused. Every branch here is a genuine
     * sensor reading, so there is no settle timer to run and {@link #observable} is
     * correspondingly always {@code true}. Reading a clock would only paper over an arrival test
     * that already works.
     *
     * <p>Pure, allocation-free, and incapable of throwing for any goal including {@code null}.
     *
     * @param goal                the goal to test against
     * @param secondsSinceApplied unused; a flywheel senses its own arrival
     * @return {@code true} when the flywheel is physically at this goal
     */
    @Override
    public boolean atGoal(FlywheelGoal goal, double secondsSinceApplied) {
        if (goal == null) {
            return false;
        }
        if (goal.isIdle()) {
            return Math.abs(mechanism.getVelocity()) <= goal.idleToleranceRPS();
        }
        if (goal.tracked() != null) {
            return mechanism.atSpeed();
        }
        return mechanism.atSpeed() && pursuingSetpointOf(goal);
    }

    /**
     * Is the mechanism's live setpoint this goal's primary setpoint?
     *
     * <p>Reads the mechanism's public {@link FlywheelMechanism#getSetpoint()} rather than any
     * remembered state of this binding's own, which is what keeps {@link #atGoal} a measurement
     * rather than a latch.
     *
     * @param goal the fixed goal being tested
     * @return {@code true} when the flywheel is chasing {@code goal}'s primary speed
     */
    private boolean pursuingSetpointOf(FlywheelGoal goal) {
        return Math.abs(mechanism.getSetpoint() - goal.primaryRPS()) <= SETPOINT_MATCH_EPSILON_RPS;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Primary wheel velocity in rotations per second. The secondary wheel is reported through
     * the mechanism's own telemetry rather than here, because a binding publishes one number and
     * the primary is the one every arrival branch is expressed in.
     */
    @Override
    public double measured() {
        return mechanism.getVelocity();
    }

    /**
     * Signed error toward {@code goal}, in rotations per second — positive when the wheel is
     * spinning faster than asked.
     *
     * <p>For an idle or fixed goal the reference is the goal's own primary speed. For a tracked
     * goal it is the mechanism's live setpoint instead, because a tracked goal stores
     * {@code primaryRPS == 0} — its speed lives in the supplier — and differencing against that
     * zero would report the raw velocity as if it were error, showing a perfectly on-target
     * shot-on-the-fly as maximally wrong in every log for the whole match.
     *
     * @param goal the goal to measure error against
     * @return signed error in rotations per second, or {@code NaN} for a {@code null} goal
     */
    @Override
    public double error(FlywheelGoal goal) {
        if (goal == null) {
            return Double.NaN;
        }
        double reference = (goal.tracked() != null) ? mechanism.getSetpoint() : goal.primaryRPS();
        return mechanism.getVelocity() - reference;
    }

    /**
     * Tolerance band for {@code goal}, in rotations per second.
     *
     * <p>An idle goal carries its own band and reports it exactly. Everything else reports the
     * constructor's mirror of the mechanism's configured velocity tolerance — see the type
     * javadoc for why that is a mirror and not a reading, and why being wrong here is harmless.
     *
     * @param goal the goal whose band to report
     * @return the tolerance in rotations per second, or {@code NaN} for a {@code null} goal
     */
    @Override
    public double tolerance(FlywheelGoal goal) {
        if (goal == null) {
            return Double.NaN;
        }
        return goal.isIdle() ? goal.idleToleranceRPS() : velocityToleranceRPS;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code true}. Every branch of {@link #atGoal} is a comparison against a live
     * velocity signal, so arrival here is genuinely sensed and no branch degrades to a settle
     * timer. A flywheel is one of the few mechanisms in the library that can say that without
     * qualification.
     *
     * @param goal the goal being pursued
     * @return {@code true}, always
     */
    @Override
    public boolean observable(FlywheelGoal goal) {
        return true;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Delegates to {@link FlywheelGoal#label()}, which is where the cardinality problem was
     * already solved. A fixed goal names its own setpoint, which is safe because a fixed goal's
     * speeds never change and the set of emitted labels is bounded by the number of declared
     * states. A tracked goal carries a caller-supplied name — {@code "SOTF"}, {@code "PodiumRange"}
     * — precisely so that nothing is tempted to interpolate its live supplier output into a
     * string that is edge-detected into the log and would otherwise write a new value every loop
     * for the whole match. Use {@link #detail} for the live number.
     *
     * @param goal the goal to name
     * @return a stable, low-cardinality label
     */
    @Override
    public String label(FlywheelGoal goal) {
        return (goal == null) ? "None" : goal.label();
    }

    /**
     * Human-readable detail, free to interpolate live values because it is never edge-detected.
     *
     * <p>Reports measured against target so that a log reader can see a spin-up in progress
     * rather than only its label. For a tracked goal the target shown is the mechanism's live
     * setpoint — what the supplier last actually produced — which is the number worth seeing and
     * the one the label is forbidden from carrying.
     *
     * @param goal the goal to describe
     * @return a description of where the flywheel is relative to this goal
     */
    @Override
    public String detail(FlywheelGoal goal) {
        if (goal == null) {
            return "None";
        }
        double velocity = mechanism.getVelocity();
        if (goal.isIdle()) {
            return String.format("%s: %.1f rps (idle band %.1f)",
                    goal.label(), velocity, goal.idleToleranceRPS());
        }
        if (goal.tracked() != null) {
            return String.format("%s: %.1f -> %.1f rps (tracked)",
                    goal.label(), velocity, mechanism.getSetpoint());
        }
        if (goal.secondaryRPS() != goal.primaryRPS()) {
            return String.format("%s: %.1f/%.1f -> %.1f/%.1f rps",
                    goal.label(), velocity, mechanism.getSecondaryVelocity(),
                    goal.primaryRPS(), goal.secondaryRPS());
        }
        return String.format("%s: %.1f -> %.1f rps", goal.label(), velocity, goal.primaryRPS());
    }

    /**
     * Runtime note explaining why this binding is not arriving, surfaced in {@code BlockerDetail}.
     *
     * <p>Each case below is a way a flywheel can look stuck for a reason that is not obvious from
     * a velocity trace, and every one of them has cost somebody a match:
     *
     * <ul>
     *   <li>A spin-up simply still in progress — reported with the gap, so it is visibly a wait
     *       and not a fault.</li>
     *   <li>An idle goal still coasting. Same idea, and the usual answer to "why did the machine
     *       hang for four seconds leaving the shoot state".</li>
     *   <li>A fixed goal that {@code atSpeed()} likes but that this binding rejects because the
     *       mechanism is chasing some other speed. This is the state machine telling you another
     *       state owns the shooter, which reads as an inexplicable false without the note.</li>
     *   <li>A tracked supplier that has returned zero, which drives {@code atSpeed()} into its
     *       hard-false branch and makes the goal unreachable for as long as it keeps doing so.</li>
     *   <li>A dual-speed goal on a single-motor mechanism. {@link #validate} already fails the
     *       build for this, so seeing it at runtime means the binding was constructed outside the
     *       builder entirely.</li>
     * </ul>
     *
     * @param goal the goal being pursued
     * @return an explanation, or {@code ""} when there is nothing to say
     */
    @Override
    public String note(FlywheelGoal goal) {
        if (goal == null) {
            return "";
        }
        double velocity = mechanism.getVelocity();

        if (goal.isIdle()) {
            double excess = Math.abs(velocity) - goal.idleToleranceRPS();
            if (excess > 0.0) {
                return String.format("coasting down: %.1f rps, %.1f above the %.1f rps idle band",
                        Math.abs(velocity), excess, goal.idleToleranceRPS());
            }
            return "";
        }

        if (goal.tracked() != null) {
            if (mechanism.getSetpoint() == 0.0) {
                return "tracked supplier is returning 0 rps; atSpeed() is hard-false at a zero "
                        + "setpoint, so this goal cannot be reached until the supplier returns "
                        + "a non-zero speed";
            }
            return mechanism.atSpeed()
                    ? ""
                    : String.format("tracking %.1f rps, at %.1f rps",
                            mechanism.getSetpoint(), velocity);
        }

        if (goal.secondaryRPS() != goal.primaryRPS() && mechanism.getSecondaryMotor() == null) {
            return "goal asks for a distinct secondary speed but this flywheel has no second "
                    + "motor; spinUp delegates to the single-wheel overload and the secondary "
                    + "speed is discarded";
        }

        if (!pursuingSetpointOf(goal)) {
            return String.format(
                    "flywheel is chasing %.1f rps, not this goal's %.1f rps - another state owns "
                            + "the mechanism", mechanism.getSetpoint(), goal.primaryRPS());
        }

        if (!mechanism.atSpeed()) {
            return String.format("spinning up: %.1f of %.1f rps", velocity, goal.primaryRPS());
        }
        return "";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code true}. A flywheel has no zero and no homing sequence — its only sensed
     * quantity is velocity, which is meaningful from power-on — so gating a state on this
     * binding being zeroed would reject states for a condition that can never be false.
     *
     * @return {@code true}, always
     */
    @Override
    public boolean zeroed() {
        return true;
    }

    /**
     * Build-time self-check for one goal.
     *
     * <p>Three structurally impossible configurations are caught here, on a laptop, all at once,
     * rather than one exception per deploy cycle in a pit:
     *
     * <ul>
     *   <li><b>A distinct secondary speed on a single-motor flywheel.</b>
     *       {@link FlywheelMechanism#spinUp(double, double)} checks for a null secondary motor
     *       and delegates to the single-wheel overload, so the requested spin is discarded in
     *       silence and the robot shoots a different shot than the one declared — a failure that
     *       looks like a tuning problem and gets chased for a whole regional. Reporting it turns
     *       silent degradation into a build failure.</li>
     *   <li><b>A non-idle goal whose primary speed is zero.</b> Such a goal reaches
     *       {@code atSpeed()}, which is hard-false at a zero primary setpoint, so it can never
     *       arrive. It comes from asking for a secondary-only spin, and the fix is either a
     *       non-zero primary or {@link FlywheelGoal#idle()}.</li>
     *   <li><b>A non-finite speed.</b> The goal's canonical constructor deliberately leaves these
     *       intact so that they can be reported here, with the goal's name attached, instead of
     *       throwing somewhere inside a velocity request.</li>
     * </ul>
     *
     * <p>The supplier of a tracked goal is deliberately not invoked. Nothing guarantees it is
     * safe to call at build time — the usual one reads a pose estimator or a shot solver that
     * does not exist yet — and a validator that crashes the build it was written to protect is
     * worse than one that checks less.
     *
     * @param goal     the goal to check
     * @param problems sink for problem descriptions; the builder aggregates them all
     */
    @Override
    public void validate(FlywheelGoal goal, Consumer<String> problems) {
        if (goal == null) {
            problems.accept(key + ": null goal");
            return;
        }

        if (goal.secondaryRPS() != 0.0
                && goal.secondaryRPS() != goal.primaryRPS()
                && mechanism.getSecondaryMotor() == null) {
            problems.accept(String.format(
                    "%s: goal '%s' asks for a secondary speed of %.2f rps but mechanism '%s' is "
                            + "configured with no second motor. spinUp(primary, secondary) "
                            + "delegates to the single-wheel overload in that case, so the "
                            + "secondary speed would be discarded silently. Add "
                            + "Config.Builder.secondMotor(canId), or use FlywheelGoal.rps(rps).",
                    key, goal.label(), goal.secondaryRPS(), mechanism.getMechanismName()));
        }

        if (goal.tracked() == null && !goal.isIdle() && goal.primaryRPS() == 0.0) {
            problems.accept(String.format(
                    "%s: goal '%s' has a primary speed of 0 rps but is not the idle goal (its "
                            + "secondary is %.2f rps). Arrival for a non-idle goal runs through "
                            + "atSpeed(), which returns false whenever the primary setpoint is 0, "
                            + "so this goal can never be reached. Give the primary a non-zero "
                            + "speed, or use FlywheelGoal.idle().",
                    key, goal.label(), goal.secondaryRPS()));
        }

        if (!Double.isFinite(goal.primaryRPS())) {
            problems.accept(String.format(
                    "%s: goal '%s' has a non-finite primary speed (%s)",
                    key, goal.label(), goal.primaryRPS()));
        }
        if (!Double.isFinite(goal.secondaryRPS())) {
            problems.accept(String.format(
                    "%s: goal '%s' has a non-finite secondary speed (%s)",
                    key, goal.label(), goal.secondaryRPS()));
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>Exactly the one flywheel. Every Catalyst mechanism extends {@code SubsystemBase}, so the
     * mechanism is its own requirement and the commands returned by {@link #pursueCommand} —
     * built through the mechanism's own {@code run} and {@code runOnce} factories — require
     * precisely this set and nothing else.
     */
    @Override
    public Set<Subsystem> requirements() {
        return Set.of(mechanism);
    }

    /**
     * {@inheritDoc}
     *
     * <p>Zero. Only one pursue command here ends on its own — {@code stopCommand()}, for an idle
     * goal — and its effect is persistent: the motors are stopped and stay stopped, and arrival
     * follows as the wheel coasts into the idle band. Re-initialising it every half second would
     * re-stop an already-stopped motor and log a state change for it each time. The pneumatic
     * binding needs a non-zero period because its command can be silently refused; nothing
     * refuses a flywheel stop.
     *
     * @return {@code 0}, disabling reassertion
     */
    @Override
    public int reassertPeriodLoops() {
        return 0;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Nothing to release. This binding owns no resources of its own and holds no state to
     * unwind; the pursue command's {@code finallyDo} already stops both motors and zeroes both
     * setpoints when the machine stops hosting it, which is the entirety of what releasing a
     * flywheel means.
     */
    @Override
    public void release() {
        // Intentionally empty; the pursue command's finallyDo performs the whole teardown.
    }
}
