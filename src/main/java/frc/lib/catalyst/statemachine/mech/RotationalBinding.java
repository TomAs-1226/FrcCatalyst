package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.MechanismView;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.lib.catalyst.statemachine.goals.RotationalGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Consumer;

/**
 * Drives a {@link RotationalMechanism} — an arm, a wrist, a hood — to the angle named by a
 * {@link RotationalGoal}, and reports arrival from the encoder.
 *
 * <h2>Why the pursue command is a composition</h2>
 * {@code RotationalMechanism.goTo(double)} is a {@code runOnce}: it latches a Motion Magic target
 * into the mechanism's {@code setpointDegrees} field and ends on the same tick. On its own that is
 * useless to a hosted actuator, because {@link frc.lib.catalyst.statemachine.robot.GoalRunner} would
 * see the command finish while the arm is still halfway through its travel. {@code holdPosition()},
 * meanwhile, re-drives Motion Magic to that same mutable field every loop and never ends. Sequencing
 * them — {@code goTo(x).andThen(holdPosition())} — therefore produces exactly the shape the
 * {@link Actuator} contract wants: one tick that commands the move, then an unending closed-loop hold
 * on the very setpoint that tick installed. This is a genuine move-then-hold, not a no-op, precisely
 * because {@code holdPosition} reads the field rather than capturing an angle at its own
 * initialisation.
 *
 * <p>Because that composition never ends, {@link #holdCommand(RotationalGoal)} returns {@code null}
 * (keep pursuing) and {@link #reassertPeriodLoops()} keeps its default of {@code 0}: there is no
 * finished-but-not-arrived case to retry. And because Motion Magic keeps holding after the state
 * machine lets go, {@link #release()} is deliberately left as the inherited no-op — a released arm
 * that stopped holding would fall.
 *
 * <h2>Resolve once, at build time</h2>
 * A goal names its target either as an explicit angle or as a preset that only the mechanism's
 * configuration knows. Both forms are also subject to the mechanism's soft limits, since
 * {@code goTo} silently clamps into {@code [minAngle, maxAngle]}. Both the preset lookup and that
 * clamp happen once, in {@link #validate(RotationalGoal, Consumer)}, and the answer is cached in
 * {@link #resolvedDegrees}.
 *
 * <p>Resolving once is not a micro-optimisation, it is a correctness requirement. If
 * {@link #atGoal} tested the <em>raw</em> angle while {@code goTo} drove the <em>clamped</em> one,
 * an out-of-range goal would leave the mechanism parked against its soft limit reporting "not
 * arrived" forever, and the state machine would sit in a transition for the rest of the match. So
 * arrival is judged against the same clamped number that was actually commanded, and the fact that
 * clamping occurred is reported as a build-time problem by {@code validate} and surfaced at runtime
 * by {@link #note(RotationalGoal)}. The out-of-range goal gets fixed on a laptop instead of hanging
 * a robot on a field.
 *
 * <p>A goal that was never validated still works: {@link #resolveDegrees} falls back to computing
 * the same value on demand. The fallback exists so that a hand-assembled binding used in a test, or
 * a goal that reached the engine by some path that skipped the builder's validation sweep, degrades
 * to "slightly more work per call" rather than to a null-pointer exception inside the command
 * scheduler.
 *
 * @since 1.2.0
 */
public final class RotationalBinding implements Actuator<RotationalGoal> {

    /** The mechanism this binding owns and drives. */
    private final RotationalMechanism mechanism;

    /** Stable telemetry key; becomes {@code Bindings/<key>/...}. */
    private final String key;

    /** Pre-built requirement set, so {@link #requirements()} allocates nothing per call. */
    private final Set<Subsystem> requirements;

    /**
     * Goal to the angle actually commanded for it: preset resolved, soft limits applied.
     *
     * <p>Populated by {@link #validate(RotationalGoal, Consumer)} and read by
     * {@link #pursueCommand(RotationalGoal)} and {@link #atGoal(RotationalGoal, double)}. Keyed by
     * the goal record, which is safe because {@code RotationalGoal} has value-based equality — the
     * same reason the engine can compare goals with {@link Objects#equals} at all.
     *
     * <p>This is a memo of a build-time computation, not a record of what the binding last did.
     * {@code atGoal} stays a pure function of live sensors, the goal handed to it, and elapsed time,
     * which is what lets {@code StateMachineCore.isAt(...)} ask about states the machine is not in.
     */
    private final Map<RotationalGoal, Double> resolvedDegrees = new HashMap<>();

    /** Lower soft limit in degrees, or {@code NaN} when the mechanism did not report one. */
    private double minDegrees = Double.NaN;

    /** Upper soft limit in degrees, or {@code NaN} when the mechanism did not report one. */
    private double maxDegrees = Double.NaN;

    /** Whether {@link #captureBounds()} has already run; the bounds are configuration constants. */
    private boolean boundsCaptured = false;

    /**
     * Wrap a rotational mechanism as a state-machine actuator.
     *
     * <p>Nothing is read from the mechanism here. Soft limits are captured lazily on first use
     * instead, because a binding is routinely constructed during robot construction, before the
     * motor has produced its first status frame, and because {@code describe()} allocates a
     * {@link MechanismView} that there is no reason to build during field construction.
     *
     * @param mechanism the mechanism to drive; becomes this binding's sole requirement
     * @param key       stable, unique, log-safe telemetry key
     * @throws NullPointerException if either argument is {@code null}
     */
    public RotationalBinding(RotationalMechanism mechanism, String key) {
        this.mechanism = Objects.requireNonNull(mechanism, "RotationalBinding: mechanism must not be null");
        this.key = Objects.requireNonNull(key, "RotationalBinding: key must not be null");
        this.requirements = Set.of(mechanism);
    }

    // ---------------------------------------------------------------- identity

    /** {@inheritDoc} */
    @Override
    public String key() {
        return key;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Matches the kind string {@code RotationalMechanism.describe()} publishes, so a dashboard
     * widget chosen from the binding and one chosen from the mechanism agree.
     */
    @Override
    public String kind() {
        return "rotational";
    }

    /** {@inheritDoc} Angles throughout this binding are mechanism-frame degrees. */
    @Override
    public String unit() {
        return "deg";
    }

    /** Subsystems this actuator owns: exactly the one mechanism it wraps. */
    @Override
    public Set<Subsystem> requirements() {
        return requirements;
    }

    // ---------------------------------------------------------------- actuation

    /**
     * {@inheritDoc}
     *
     * <p>Returns {@code goTo(resolved).andThen(holdPosition())}, freshly built on every call as the
     * contract demands — {@code goTo} and {@code holdPosition} are both command factories, and
     * {@code andThen} wraps them in a new sequential group, so no instance is ever reused across
     * calls or shared between groups.
     *
     * <p>When the target cannot be resolved — an unknown preset, or a goal carrying neither an angle
     * nor a preset name, both of which {@code validate} reports at build time — this returns a bare
     * {@code holdPosition()} instead. That freezes the mechanism at whatever setpoint it already had
     * rather than commanding a {@code NaN} into Motion Magic, and it pairs with
     * {@link #atGoal(RotationalGoal, double)} returning {@code false}, so the state machine reports
     * a stuck transition and {@link #note(RotationalGoal)} says why.
     *
     * <p>This method never throws. An exception escaping here would propagate out of
     * {@code CommandScheduler.run()} and take the whole robot loop down with it, so the entire body
     * is guarded and falls back to the same benign hold.
     */
    @Override
    public Command pursueCommand(RotationalGoal goal) {
        try {
            double target = resolveDegrees(goal);
            if (Double.isNaN(target)) {
                return mechanism.holdPosition();
            }
            return mechanism.goTo(target).andThen(mechanism.holdPosition());
        } catch (RuntimeException e) {
            return mechanism.holdPosition();
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>{@code null}: keep running the pursue command. Its trailing {@code holdPosition()} is
     * already a closed-loop hold that re-drives Motion Magic to the commanded setpoint every loop,
     * so swapping in a separate hold command on arrival would replace a working closed loop with an
     * identical one and give a gravity-loaded arm a chance to sag during the handover.
     */
    @Override
    public Command holdCommand(RotationalGoal goal) {
        return null;
    }

    // ---------------------------------------------------------------- measurement

    /**
     * {@inheritDoc}
     *
     * <p>Arrival is the encoder's answer, tested against the clamped target with the effective
     * tolerance — never the mechanism's own {@code atSetpoint}, which would report against whatever
     * setpoint happens to be latched rather than against the goal being asked about. That
     * distinction matters because the engine calls this for goals of states the machine is not
     * currently in.
     *
     * <p>{@code secondsSinceApplied} is unused: a rotational mechanism senses its own position, so
     * arrival is a measurement and never a settle timer.
     *
     * <p>Returns {@code false} rather than throwing when the target cannot be resolved, for the same
     * loop-safety reason as {@link #pursueCommand(RotationalGoal)}.
     */
    @Override
    public boolean atGoal(RotationalGoal goal, double secondsSinceApplied) {
        try {
            double target = resolveDegrees(goal);
            if (Double.isNaN(target)) {
                return false;
            }
            double band = effectiveTolerance(goal);
            if (Double.isNaN(band)) {
                return false;
            }
            return mechanism.atAngle(target, band);
        } catch (RuntimeException e) {
            return false;
        }
    }

    /** {@inheritDoc} Live angle in degrees, straight off the encoder. */
    @Override
    public double measured() {
        return mechanism.getAngle();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Signed so that a positive error means the mechanism must rotate <em>toward increasing
     * degrees</em> to reach the goal: {@code target - measured}. Measured against the clamped
     * target, matching {@link #atGoal}, so a log never shows a healthy error next to a failing
     * arrival test.
     */
    @Override
    public double error(RotationalGoal goal) {
        double target = resolveDegrees(goal);
        if (Double.isNaN(target)) {
            return Double.NaN;
        }
        return target - mechanism.getAngle();
    }

    /**
     * {@inheritDoc}
     *
     * <p>The goal's own tolerance when it named one, otherwise the mechanism's configured
     * {@code toleranceDegrees}. Routed through {@link RotationalGoal#toleranceOr(double)} so that the
     * number logged here is provably the number {@link #atGoal} decided with, and so the state
     * machine agrees with {@code RotationalMechanism.atPosition(...)} and with any trigger a team
     * built by hand.
     */
    @Override
    public double tolerance(RotationalGoal goal) {
        return effectiveTolerance(goal);
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code true}. An arm's arrival is read from its encoder, so this binding never
     * degrades into a settle timer the way the claw's {@code OPEN} goal must. A goal whose target
     * failed to resolve is still not a timer — it simply never reports arrival, which
     * {@link #note(RotationalGoal)} explains.
     */
    @Override
    public boolean observable(RotationalGoal goal) {
        return true;
    }

    /** {@inheritDoc} Homing state of the mechanism, from its hard stop or an explicit {@code zero()}. */
    @Override
    public boolean zeroed() {
        return mechanism.hasBeenZeroed();
    }

    // ---------------------------------------------------------------- description

    /**
     * {@inheritDoc}
     *
     * <p>A preset goal labels as its preset name, which is both the lowest-cardinality and the most
     * stable choice: retuning {@code SCORE} from 100 to 104 degrees must not rewrite the logged
     * label. An explicit-angle goal labels as its declared angle, which is a compile-time constant
     * of that goal and so cannot churn either. Neither form interpolates a live value, so the
     * edge-detected log string this becomes is written once per state entry rather than at 50 Hz.
     *
     * <p>Note that the label reports the angle the goal <em>asked</em> for, not the clamped angle
     * that was commanded. The two differ only for a misconfigured goal, and in that case the number
     * a reader wants to see in the log is the one written in the state machine.
     */
    @Override
    public String label(RotationalGoal goal) {
        if (goal == null) {
            return "none";
        }
        if (goal.isPreset()) {
            return goal.preset();
        }
        if (Double.isNaN(goal.degrees())) {
            return "unspecified";
        }
        return format(goal.degrees()) + "deg";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Free to interpolate, and does: the resolved target, the live angle, and the band being
     * tested. This is the line that answers "why does the log say this arm has not arrived" without
     * anyone having to cross-reference three separate telemetry keys.
     */
    @Override
    public String detail(RotationalGoal goal) {
        if (goal == null) {
            return "none";
        }
        double target = resolveDegrees(goal);
        double band = effectiveTolerance(goal);
        if (Double.isNaN(target)) {
            return label(goal) + " -> unresolved (at " + format(mechanism.getAngle()) + "deg)";
        }
        return label(goal) + " -> " + format(target) + "deg"
                + " (at " + format(mechanism.getAngle()) + "deg"
                + ", err " + format(target - mechanism.getAngle()) + "deg"
                + ", tol " + format(band) + "deg)";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Reports the conditions under which this binding will never report arrival, which is exactly
     * what a reader staring at a stuck transition needs: an unresolvable preset, an unsatisfiable
     * tolerance band, a target the soft limits moved, or an encoder nobody has zeroed. Returns
     * {@code ""} in the ordinary case, and allocates nothing to do so.
     */
    @Override
    public String note(RotationalGoal goal) {
        if (goal == null) {
            return "goal is null";
        }
        if (goal.isPreset() && !mechanism.getNamedPositions().containsKey(goal.preset())) {
            return "preset '" + goal.preset() + "' is not configured on "
                    + mechanism.getMechanismName() + "; known presets: "
                    + mechanism.getNamedPositions().keySet();
        }
        if (!goal.isPreset() && Double.isNaN(goal.degrees())) {
            return "goal names neither an angle nor a preset";
        }
        double band = effectiveTolerance(goal);
        if (Double.isNaN(band) || band <= 0.0) {
            return "arrival band is " + format(band) + " deg, which no position can satisfy";
        }
        double requested = requestedDegrees(goal);
        double target = resolveDegrees(goal);
        if (!Double.isNaN(requested) && !Double.isNaN(target) && requested != target) {
            return "requested " + format(requested) + " deg was clamped to " + format(target)
                    + " deg by the soft limits; arrival is judged at the clamped angle";
        }
        if (!mechanism.hasBeenZeroed()) {
            return "mechanism has not been zeroed; the angle this arrival test reads is not trusted";
        }
        return "";
    }

    // ---------------------------------------------------------------- validation

    /**
     * {@inheritDoc}
     *
     * <p>This is both the self-check and the point at which every goal's target is resolved and
     * cached. Running the preset lookup and the soft-limit clamp here means they happen once per
     * distinct goal on a laptop, and it means an unknown preset name is reported as a configuration
     * error alongside the names the mechanism actually knows — strictly more useful than the
     * {@code IllegalArgumentException} that {@code RotationalMechanism.goTo(String)} would have
     * thrown from inside a command factory during a match.
     *
     * <p>Four things are checked: that the goal names a target at all, that a named preset exists,
     * that the target lies within the soft limits, and that the arrival band is satisfiable. A
     * clamped target is still cached, because reporting the problem and then also hanging the state
     * machine would be two failures where one is enough.
     */
    @Override
    public void validate(RotationalGoal goal, Consumer<String> problems) {
        if (goal == null) {
            problems.accept(key + ": goal is null");
            return;
        }

        double requested;
        if (goal.isPreset()) {
            Double preset = mechanism.getNamedPositions().get(goal.preset());
            if (preset == null) {
                problems.accept(key + ": unknown preset '" + goal.preset() + "' on "
                        + mechanism.getMechanismName() + "; configured presets are "
                        + mechanism.getNamedPositions().keySet());
                return;
            }
            requested = preset;
        } else {
            requested = goal.degrees();
            if (Double.isNaN(requested)) {
                problems.accept(key + ": goal names neither an angle nor a preset");
                return;
            }
        }

        captureBounds();
        double target = clampToBounds(requested);
        if (requested != target) {
            problems.accept(key + ": " + label(goal) + " targets " + format(requested)
                    + " deg, outside the configured range [" + format(minDegrees) + ", "
                    + format(maxDegrees) + "] deg; it would be clamped to " + format(target) + " deg");
        }

        double band = goal.toleranceOr(mechanism.getTolerance());
        if (Double.isNaN(band) || band <= 0.0) {
            problems.accept(key + ": " + label(goal) + " has an arrival band of " + format(band)
                    + " deg; a non-positive band can never be satisfied and would stall the"
                    + " state machine at this transition");
        }

        resolvedDegrees.put(goal, target);
    }

    // ---------------------------------------------------------------- internals

    /**
     * The angle actually commanded for {@code goal}: cached from {@link #validate} when available,
     * recomputed on demand otherwise, {@code NaN} when the goal cannot be resolved at all.
     */
    private double resolveDegrees(RotationalGoal goal) {
        if (goal == null) {
            return Double.NaN;
        }
        Double cached = resolvedDegrees.get(goal);
        if (cached != null) {
            return cached;
        }
        double requested = requestedDegrees(goal);
        if (Double.isNaN(requested)) {
            return Double.NaN;
        }
        return clampToBounds(requested);
    }

    /**
     * The angle {@code goal} asks for before soft limits are applied: its explicit degrees, or the
     * angle its preset names. {@code NaN} when the preset is unknown or the goal names no target.
     */
    private double requestedDegrees(RotationalGoal goal) {
        if (goal.isPreset()) {
            Double preset = mechanism.getNamedPositions().get(goal.preset());
            return preset == null ? Double.NaN : preset;
        }
        return goal.degrees();
    }

    /** The arrival band for {@code goal}, deferring to the mechanism's configured tolerance. */
    private double effectiveTolerance(RotationalGoal goal) {
        if (goal == null) {
            return Double.NaN;
        }
        return goal.toleranceOr(mechanism.getTolerance());
    }

    /**
     * Clamp into the mechanism's soft limits, mirroring what {@code goTo} does internally so that
     * the angle this binding tests arrival against is the angle Motion Magic was actually given. A
     * bound the mechanism did not report is simply not applied.
     */
    private double clampToBounds(double degrees) {
        captureBounds();
        double clamped = degrees;
        if (!Double.isNaN(minDegrees) && clamped < minDegrees) {
            clamped = minDegrees;
        }
        if (!Double.isNaN(maxDegrees) && clamped > maxDegrees) {
            clamped = maxDegrees;
        }
        return clamped;
    }

    /**
     * Read the soft limits out of {@code describe()} once and remember them.
     *
     * <p>The limits come from the mechanism's immutable {@code Config}, so memoising them is not
     * mutable state in any sense {@code atGoal}'s purity contract cares about — the value is the
     * same on every call, it is merely computed on the first one. Guarded because {@code describe()}
     * touches the motor, and a binding must not be the reason a loop dies.
     */
    private void captureBounds() {
        if (boundsCaptured) {
            return;
        }
        boundsCaptured = true;
        try {
            MechanismView view = mechanism.describe();
            minDegrees = view.min();
            maxDegrees = view.max();
        } catch (RuntimeException e) {
            minDegrees = Double.NaN;
            maxDegrees = Double.NaN;
        }
    }

    /** One-decimal formatting for logs; {@code NaN} and infinities print as themselves. */
    private static String format(double degrees) {
        if (Double.isNaN(degrees) || Double.isInfinite(degrees)) {
            return String.valueOf(degrees);
        }
        return String.format("%.1f", degrees);
    }
}
