package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.LinearMechanism;
import frc.lib.catalyst.mechanisms.MechanismView;
import frc.lib.catalyst.statemachine.goals.LinearGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Consumer;

/**
 * Drives a {@link LinearMechanism} — an elevator, a telescoping arm, a linear slide — from a
 * {@link LinearGoal}.
 *
 * <h2>Pursuing a position</h2>
 *
 * <p>The pursue command is {@code goTo(target).andThen(holdPosition())}. That composition looks
 * suspicious at first glance, because {@link LinearMechanism#goTo(double)} is a {@code runOnce} that
 * ends after a single tick — so {@code andThen} runs before the mechanism has visibly moved. It is
 * nevertheless the correct thing to run here, for a reason specific to how
 * {@link LinearMechanism#holdPosition()} is written: {@code holdPosition} re-drives Motion Magic to the
 * mechanism's mutable {@code setpointMeters} <em>field</em> every loop, not to a target captured when
 * the command was created. Since {@code goTo} has already written the clamped target into that field,
 * the hold phase keeps commanding the position the {@code goTo} asked for. The mechanism genuinely
 * travels and then genuinely holds, and the composed command never ends — which is exactly what a
 * hosted pursue command must do, because arrival is the state machine's decision via
 * {@link #atGoal(LinearGoal, double)} and never a command finishing.
 *
 * <p>For the same reason {@link #holdCommand(LinearGoal)} stays {@code null}. There is nothing to swap
 * to on arrival: the closed loop that got the mechanism there is the same closed loop that should hold
 * it against gravity afterwards. Swapping to anything else would drop the elevator.
 *
 * <h2>Resolution happens exactly once, at build time</h2>
 *
 * <p>Two transformations stand between the goal a human wrote and the number the motor is given: a
 * preset key has to be looked up in {@link LinearMechanism#getNamedPositions()}, and the result has to
 * be clamped into the mechanism's configured travel. Both happen in {@link #validate} and the answer is
 * cached in {@link #targets}, keyed by the goal.
 *
 * <p>Doing this once, in one place, is not merely an optimisation. {@code LinearMechanism.goTo} clamps
 * its argument to the soft limits internally before commanding it, so a binding that passed the raw
 * target to {@code goTo} but tested arrival against that same raw target would be commanding one
 * position and measuring against a different one. An out-of-range goal would then hang forever: the
 * mechanism sits at the end of its travel, perfectly still, while {@code atGoal} keeps reporting false
 * because it is comparing against a position the mechanism is physically incapable of reaching. Caching
 * the <em>clamped</em> value and using that same number for both pursuit and arrival removes that
 * failure mode entirely. The build-time range check still reports the goal as a configuration problem,
 * so the clamp is a safety net rather than a silent fix.
 *
 * <p>The cache also keeps {@link #atGoal} free of map lookups and string hashing, which matters because
 * the engine calls it every loop for every binding — including bindings belonging to states the machine
 * is not in.
 *
 * <h2>Nothing here throws at runtime</h2>
 *
 * <p>{@link #pursueCommand} and {@link #atGoal} run inside {@code CommandScheduler.run()}. An exception
 * escaping either one propagates out of the scheduler and takes the whole robot loop with it, turning a
 * misspelled preset into a dead robot. So every runtime path degrades instead: an unresolvable goal
 * pursues {@link LinearMechanism#holdPosition()} (which leaves the mechanism where it already is) and
 * reports {@code atGoal == false} forever, which surfaces as a state that visibly refuses to complete
 * and a {@link #note} that says why. Note deliberately that {@link LinearMechanism#goTo(String)} — the
 * preset overload — is never called from this class at all: it throws
 * {@code IllegalArgumentException} on an unknown key, from inside a command factory, during a match.
 *
 * @since 1.2.0
 */
public final class LinearBinding implements Actuator<LinearGoal> {

    /** The mechanism this binding owns. Also its sole requirement. */
    private final LinearMechanism mechanism;

    /** Stable telemetry key; becomes {@code Bindings/<key>/...}. */
    private final String key;

    /**
     * Lower end of configured travel, read once from {@link LinearMechanism#describe()} at
     * construction. Cached because {@code describe()} allocates a {@link MechanismView} and this value
     * is needed on paths that must stay allocation-free; the underlying range comes from the immutable
     * mechanism {@code Config} and cannot change afterwards.
     */
    private final double minMeters;

    /** Upper end of configured travel. See {@link #minMeters}. */
    private final double maxMeters;

    /**
     * Goal to fully-resolved, clamped target position in metres.
     *
     * <p>Populated by {@link #validate} at build time and read by {@link #pursueCommand} and
     * {@link #atGoal}. A goal that could not be resolved at all maps to {@link Double#NaN} rather than
     * being absent, so the failed lookup is not retried on every loop.
     *
     * <p>{@link LinearGoal} is a record, so its {@code hashCode} is value-based and a goal constructed
     * in a state table hashes equal to the copy the engine hands back here.
     */
    private final Map<LinearGoal, Double> targets = new HashMap<>();

    /**
     * Wrap a linear mechanism as a state-machine actuator.
     *
     * @param mechanism the mechanism to drive; becomes this binding's only requirement
     * @param key       stable, unique, log-safe telemetry key for this binding
     * @throws NullPointerException if either argument is {@code null}. This is a construction-time
     *                              programming error, not a runtime condition, so throwing here is
     *                              safe — nothing in the command scheduler is running yet.
     */
    public LinearBinding(LinearMechanism mechanism, String key) {
        this.mechanism = Objects.requireNonNull(mechanism, "mechanism");
        this.key = Objects.requireNonNull(key, "key");

        MechanismView view = mechanism.describe();
        this.minMeters = view.min();
        this.maxMeters = view.max();
    }

    // ------------------------------------------------------------------
    // Identity
    // ------------------------------------------------------------------

    /** {@inheritDoc} */
    @Override
    public String key() {
        return key;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Matches the {@code "linear"} kind {@link LinearMechanism#describe()} publishes, so a dashboard
     * can render binding telemetry with the same widget it uses for the mechanism itself.
     */
    @Override
    public String kind() {
        return "linear";
    }

    /** {@inheritDoc} Positions are metres throughout, in the mechanism's own frame. */
    @Override
    public String unit() {
        return "m";
    }

    /** The mechanism itself, and nothing else — every Catalyst mechanism extends {@code SubsystemBase}. */
    @Override
    public Set<Subsystem> requirements() {
        return Set.of(mechanism);
    }

    // ------------------------------------------------------------------
    // Actuation
    // ------------------------------------------------------------------

    /**
     * A fresh {@code goTo(target).andThen(holdPosition())} on every call.
     *
     * <p>Both halves are command factories that allocate a new instance per invocation, and
     * {@code andThen} wraps them in a new sequential group, so the fresh-instance contract is met
     * without any explicit defensive copying.
     *
     * <p>The target is the clamped value cached by {@link #validate}. When the goal was never validated
     * the cache resolves it lazily on this call; when it cannot be resolved at all this degrades to a
     * bare {@link LinearMechanism#holdPosition()}, which holds the last commanded setpoint rather than
     * commanding a {@code NaN} position into Motion Magic.
     *
     * @param goal the goal to pursue
     * @return a never-ending command that travels to the resolved position and then holds it
     */
    @Override
    public Command pursueCommand(LinearGoal goal) {
        double target = targetMeters(goal);
        if (Double.isNaN(target)) {
            return mechanism.holdPosition();
        }
        return mechanism.goTo(target).andThen(mechanism.holdPosition());
    }

    /**
     * {@inheritDoc}
     *
     * <p>{@code null}, meaning "keep pursuing". Motion Magic holding a position <em>is</em> the correct
     * post-arrival behaviour for an elevator; there is no open-loop drive here that would keep pushing
     * into a hard stop after arrival, which is the case {@code holdCommand} exists for.
     */
    @Override
    public Command holdCommand(LinearGoal goal) {
        return null;
    }

    // ------------------------------------------------------------------
    // Measurement
    // ------------------------------------------------------------------

    /**
     * Arrival, measured against the same clamped target {@link #pursueCommand} commands.
     *
     * <p>Pure in the sense the interface requires: it reads the live encoder and the supplied goal and
     * nothing else. In particular it consults no "last applied goal" field, because the engine calls
     * this for goals of states the machine is not currently in — that is what makes {@code isAt} a
     * measurement rather than a latch.
     *
     * <p>{@code secondsSinceApplied} is unused. A linear mechanism's position is directly sensed, so
     * arrival is a real measurement and never a settle timer.
     *
     * @param goal                the goal to test against
     * @param secondsSinceApplied unused; see above
     * @return {@code true} when the encoder reads within tolerance of the resolved target
     */
    @Override
    public boolean atGoal(LinearGoal goal, double secondsSinceApplied) {
        double target = targetMeters(goal);
        if (Double.isNaN(target)) {
            return false;
        }
        return mechanism.atPosition(target, tolerance(goal));
    }

    /** {@inheritDoc} Live carriage position in metres. */
    @Override
    public double measured() {
        return mechanism.getPosition();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Signed so the sign is readable in a log: positive means the mechanism is above the target and
     * must come down. {@code NaN} for a goal that never resolved.
     */
    @Override
    public double error(LinearGoal goal) {
        return mechanism.getPosition() - targetMeters(goal);
    }

    /**
     * {@inheritDoc}
     *
     * <p>The goal's own tolerance when it carries one, otherwise the mechanism's configured
     * {@link LinearMechanism#getPositionTolerance()}. Deferring is the normal case and keeps one
     * tolerance per mechanism instead of a different one per state.
     *
     * <p>This resolution matters for correctness, not just for logging: {@code LinearGoal} uses
     * {@code NaN} as its "defer to the mechanism" sentinel, and {@code LinearMechanism.atPosition}
     * tests {@code abs(error) < tolerance}, a comparison that is false for every value when the
     * tolerance is {@code NaN}. Handing the raw component to {@code atPosition} would therefore produce
     * a mechanism that arrives perfectly and reports arrival never.
     */
    @Override
    public double tolerance(LinearGoal goal) {
        return goal.hasExplicitTolerance()
                ? goal.toleranceMeters()
                : mechanism.getPositionTolerance();
    }

    /**
     * {@inheritDoc}
     *
     * <p>{@code true} for any goal this binding can actually resolve: the position is read from an
     * encoder, so arrival is sensed rather than assumed after a delay. A goal whose target never
     * resolved reports {@code false}, because for that goal {@link #atGoal} is a constant rather than a
     * measurement and a log reader should not mistake it for one.
     */
    @Override
    public boolean observable(LinearGoal goal) {
        return !Double.isNaN(targetMeters(goal));
    }

    /**
     * {@inheritDoc}
     *
     * <p>The preset key when the goal has one — {@code "L4"} tells a student which scoring position
     * failed, where {@code 1.37} tells them nothing. Numeric goals fall back to their target formatted
     * to millimetre precision.
     *
     * <p>Low cardinality is preserved because every value interpolated here is fixed for the life of
     * the goal: the preset key is a string literal from the state table, and the resolved target is
     * computed once at build time. No live sensor value appears, so the logged label is edge-detected
     * into a handful of writes per match rather than fifty per second.
     */
    @Override
    public String label(LinearGoal goal) {
        if (goal.isPreset()) {
            return goal.preset();
        }
        double target = targetMeters(goal);
        return Double.isNaN(target) ? "unresolved" : String.format("%.3fm", target);
    }

    /**
     * {@inheritDoc}
     *
     * <p>Free to interpolate live values, and does: the resolved target, where the mechanism actually
     * is, and the band it has to land in. This is the line that answers "why is this state not
     * finishing" without opening a plot.
     */
    @Override
    public String detail(LinearGoal goal) {
        double target = targetMeters(goal);
        if (Double.isNaN(target)) {
            return label(goal) + " (unresolved target)";
        }
        return String.format("%s -> %.3fm (now %.3fm, tol %.3fm)",
                label(goal), target, mechanism.getPosition(), tolerance(goal));
    }

    /**
     * {@inheritDoc}
     *
     * <p>Reports only conditions that explain a stuck or surprising state, in priority order: a target
     * that never resolved, an unhomed encoder, a target the configured travel forced a clamp on, and a
     * limit switch that is currently holding the mechanism at an end of travel. Anything else returns
     * {@code ""} so a healthy binding adds no noise to {@code BlockerDetail}.
     */
    @Override
    public String note(LinearGoal goal) {
        double target = targetMeters(goal);

        if (Double.isNaN(target)) {
            return goal.isPreset()
                    ? "preset '" + goal.preset() + "' is not configured on " + mechanism.getMechanismName()
                    : "goal names neither a position nor a preset";
        }

        if (!mechanism.hasBeenZeroed()) {
            return "not zeroed — position is relative to wherever the mechanism booted";
        }

        double requested = requestedMeters(goal);
        if (!Double.isNaN(requested) && requested != target) {
            return String.format("target %.3fm clamped to %.3fm by travel [%.3fm, %.3fm]",
                    requested, target, minMeters, maxMeters);
        }

        if (mechanism.isReverseLimitPressed()) {
            return "reverse limit switch pressed — mechanism is at the bottom of travel";
        }
        if (mechanism.isForwardLimitPressed()) {
            return "forward limit switch pressed — mechanism is at the top of travel";
        }

        return "";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Delegates to {@link LinearMechanism#hasBeenZeroed()}. A linear mechanism's position is
     * meaningless in absolute terms until it has been homed, so a state gated on this binding is
     * rejected with {@code NOT_ZEROED} rather than being driven to a number that means nothing.
     */
    @Override
    public boolean zeroed() {
        return mechanism.hasBeenZeroed();
    }

    // ------------------------------------------------------------------
    // Build-time validation
    // ------------------------------------------------------------------

    /**
     * Resolve this goal once and report anything wrong with it.
     *
     * <p>Three problems are detectable here, and all three are reported rather than thrown so the
     * builder can aggregate every bad goal in the whole superstructure into one exception on a laptop:
     *
     * <ul>
     *   <li>a goal that names neither a numeric position nor a preset;</li>
     *   <li>a preset key the mechanism has no named position for — reported together with the sorted
     *       list of keys that <em>do</em> exist, which is what turns a build failure into a fix;</li>
     *   <li>a target outside the mechanism's configured travel — reported with both bounds, since
     *       "1.40m is out of range" is only half an error message.</li>
     * </ul>
     *
     * <p>Whatever happens, the clamped target is cached before returning, so the runtime path never has
     * to repeat this work or make a decision of its own.
     *
     * @param goal     the goal to check and resolve
     * @param problems sink for human-readable problem descriptions
     */
    @Override
    public void validate(LinearGoal goal, Consumer<String> problems) {
        double requested = Double.NaN;

        if (goal.isPreset()) {
            Map<String, Double> presets = mechanism.getNamedPositions();
            Double found = presets.get(goal.preset());
            if (found == null) {
                problems.accept(key + ": unknown position preset '" + goal.preset() + "' on "
                        + mechanism.getMechanismName() + ". Configured presets: "
                        + new TreeSet<>(presets.keySet()));
            } else {
                requested = found;
            }
        } else if (goal.isResolved()) {
            requested = goal.meters();
        } else {
            problems.accept(key + ": goal names neither a numeric position nor a preset");
        }

        if (!Double.isNaN(requested) && (requested < minMeters || requested > maxMeters)) {
            problems.accept(String.format(
                    "%s: target %.3fm is outside the configured travel of %s, which is [%.3fm, %.3fm]. "
                            + "It will be clamped to the nearer bound.",
                    key, requested, mechanism.getMechanismName(), minMeters, maxMeters));
        }

        targets.put(goal, clamp(requested));
    }

    // ------------------------------------------------------------------
    // Internals
    // ------------------------------------------------------------------

    /**
     * The clamped target for a goal, or {@code NaN} when it cannot be resolved.
     *
     * <p>Normally a straight cache read: {@link #validate} ran at build time for every goal the state
     * machine holds. The lazy branch covers a binding used outside the builder — a unit test, or a goal
     * reached through an override path — and is deliberately kept here rather than left as a
     * {@code NaN} so those callers get working behaviour instead of a mechanism that never moves. It
     * runs at most once per distinct goal, and its result (including the {@code NaN} of a failed
     * resolution) is cached so no lookup repeats on later loops.
     */
    private double targetMeters(LinearGoal goal) {
        Double cached = targets.get(goal);
        if (cached != null) {
            return cached;
        }
        double resolved = clamp(requestedMeters(goal));
        targets.put(goal, resolved);
        return resolved;
    }

    /**
     * The target a goal asks for, before clamping: its own {@code meters} when numeric, the mapped
     * value when it is a preset the mechanism knows, {@code NaN} otherwise. Never throws and never
     * reports a problem — {@link #validate} is where problems are reported.
     */
    private double requestedMeters(LinearGoal goal) {
        if (goal.isPreset()) {
            Double found = mechanism.getNamedPositions().get(goal.preset());
            return found == null ? Double.NaN : found;
        }
        return goal.meters();
    }

    /**
     * Clamp a target into the configured travel, passing {@code NaN} through untouched so an
     * unresolvable goal stays recognisably unresolvable rather than silently becoming a bound.
     */
    private double clamp(double meters) {
        return Double.isNaN(meters) ? Double.NaN : MathUtil.clamp(meters, minMeters, maxMeters);
    }
}
