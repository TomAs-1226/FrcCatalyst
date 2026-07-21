package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.DifferentialWristMechanism;
import frc.lib.catalyst.statemachine.goals.WristGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;

import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * Binds a {@link DifferentialWristMechanism} to the state machine as a two-axis pose actuator.
 *
 * <p>A differential wrist is the most sensor-honest of the Catalyst mechanisms and also the most
 * quietly misleading one. Both axes are derived from real encoders — pitch from the sum of the two
 * rotor positions and roll from their difference — so arrival here is genuinely measured rather
 * than timed, and {@link #observable} is always {@code true}. What is misleading is the
 * mechanism's own {@code atSetpoint()}, and understanding why is the whole reason this class is
 * more than a two-line wrapper.
 *
 * <p><b>Why arrival is not simply {@code atSetpoint()}.</b> The mechanism's
 * {@code applyTargets} clamps the requested pitch and roll into the configured
 * {@code pitchRange}/{@code rollRange} before storing them as the setpoints it will later compare
 * against. Ask a wrist limited to a roll of 180 degrees for a roll of 200 and it will drive to
 * 180, settle there, and report {@code atSetpoint() == true} while sitting twenty degrees away
 * from what was actually requested. A state machine that trusted that would advance to the next
 * state with the game piece pointed the wrong way. Arrival therefore tests the raw axes against
 * the values carried on the {@link WristGoal} itself, and keeps {@code atSetpoint()} only as an
 * additional conjunct so that the mechanism's own configured band still has a vote. An
 * out-of-range goal consequently never reports arrival, which is the correct outcome: the machine
 * blocks, {@link #note} says which axis is pinned against which limit, and
 * {@link #validate} has already reported the same thing at build time on a laptop.
 *
 * <p><b>One consequence worth knowing before reading a log.</b> Because {@code atSetpoint()}
 * compares against the mechanism's last-applied setpoint fields, this binding's {@code atGoal}
 * reflects mechanism state that belongs to whichever goal was most recently commanded. Querying
 * {@code isAt} for a pose the machine is not currently pursuing can therefore answer {@code false}
 * even when the wrist is physically parked inside that pose's band, because the mechanism's
 * internal setpoint still names the other pose. This binding holds no last-applied field of its
 * own and is pure with respect to its own state; the residue lives inside the mechanism. The
 * conjunct is mandated by the binding specification and is the conservative direction to be wrong
 * in — it produces a spurious "not there yet", never a spurious "arrived".
 *
 * <p><b>Tolerance.</b> The arrival band comes from {@link WristGoal#toleranceDegrees()} rather
 * than the mechanism, whose {@code Config.toleranceDegrees} is package-private with no accessor.
 * Because {@code atSetpoint()} is also a conjunct, the band that actually governs arrival is the
 * tighter of the goal's band and the mechanism's configured one. A goal asking for a band looser
 * than the mechanism's config will not get it; a goal asking for a tighter band will.
 *
 * @since 1.2.0
 */
public final class WristBinding implements Actuator<WristGoal> {

    /** The wrapped mechanism. Also the sole subsystem requirement of every command produced here. */
    private final DifferentialWristMechanism mechanism;

    /** Stable telemetry key; see {@link #key()}. */
    private final String key;

    /**
     * Reports whether the wrist has been homed.
     *
     * <p>{@code DifferentialWristMechanism} tracks a {@code hasBeenZeroed} flag internally and
     * publishes it in its inputs, but unlike {@code LinearMechanism} and
     * {@code RotationalMechanism} it exposes no public accessor for it, so this binding cannot
     * read it. Teams that home their wrist through their own routine pass a supplier to the
     * three-argument constructor; everyone else gets the optimistic default described on
     * {@link #zeroed()}.
     */
    private final BooleanSupplier zeroedSupplier;

    /** Pre-built requirement set, so {@link #requirements()} allocates nothing per loop. */
    private final Set<Subsystem> requirements;

    /**
     * Per-goal clamp displacement in degrees, resolved once by {@link #validate} and read by
     * {@link #note}.
     *
     * <p>The value is the larger of the two axes' distances between what the goal asks for and
     * what {@code applyTargets} will actually clamp it to, so {@code 0.0} means the pose is
     * reachable and anything positive means the pose can never report arrival. Resolving it at
     * build time keeps the runtime path free of the range arithmetic, and a goal that was never
     * validated falls back to {@link #clampDisplacement} computed live rather than failing.
     */
    private final Map<WristGoal, Double> clampDisplacementByGoal = new ConcurrentHashMap<>();

    /**
     * Per-goal label strings, resolved on first use.
     *
     * <p>{@link #label} is edge-detected by the logging layer, which means it can be called every
     * loop. Formatting a fresh string each time would hand the garbage collector two objects per
     * binding per loop for no benefit, since a {@link WristGoal} is immutable and its label can
     * never change.
     */
    private final Map<WristGoal, String> labelByGoal = new ConcurrentHashMap<>();

    /**
     * Binds a wrist that reports no homing state of its own.
     *
     * @param mechanism the wrist to drive; must not be {@code null}
     * @param key       stable, unique, log-safe telemetry key; must not be blank
     */
    public WristBinding(DifferentialWristMechanism mechanism, String key) {
        this(mechanism, key, () -> true);
    }

    /**
     * Binds a wrist together with an external notion of whether it has been homed.
     *
     * <p>Use this overload when the robot homes its wrist against a hard stop or an absolute
     * encoder outside the mechanism, so that states depending on the wrist are rejected with
     * {@code NOT_ZEROED} instead of slewing an unreferenced mechanism into a limit.
     *
     * @param mechanism      the wrist to drive; must not be {@code null}
     * @param key            stable, unique, log-safe telemetry key; must not be blank
     * @param zeroedSupplier reports whether the wrist's encoders have been referenced; must not be
     *                       {@code null} and must be cheap, as it is polled every loop
     */
    public WristBinding(DifferentialWristMechanism mechanism, String key,
                        BooleanSupplier zeroedSupplier) {
        this.mechanism = Objects.requireNonNull(mechanism, "mechanism");
        this.key = Objects.requireNonNull(key, "key");
        this.zeroedSupplier = Objects.requireNonNull(zeroedSupplier, "zeroedSupplier");
        if (key.isBlank()) {
            throw new IllegalArgumentException("WristBinding key must not be blank");
        }
        this.requirements = Set.of(mechanism);
    }

    @Override
    public String key() {
        return key;
    }

    /**
     * {@code "diffwrist"}, matching the kind string the mechanism itself publishes from
     * {@code describe()}, so a dashboard renders the binding and the mechanism with the same
     * widget.
     */
    @Override
    public String kind() {
        return "diffwrist";
    }

    /** Degrees. Both wrist axes are angular and the mechanism reports both in degrees. */
    @Override
    public String unit() {
        return "deg";
    }

    /**
     * Live pitch in degrees.
     *
     * <p>A single scalar cannot describe a two-axis pose, and pitch is the axis reported as the
     * primary value by the mechanism's own {@code describe()}, so the two agree. Roll is not lost:
     * it appears in {@link #detail} and drives {@link #error} whenever it is the worse of the two.
     */
    @Override
    public double measured() {
        return mechanism.getPitch();
    }

    /**
     * Drives to the pose and then holds it.
     *
     * <p>{@code goTo} is a {@code runOnce} that latches one differential Motion Magic request and
     * ends immediately; on its own it would leave the state machine hosting a finished command
     * while the wrist was still travelling. {@code holdPosition} then runs forever, re-applying
     * the mechanism's stored setpoint fields every loop, which both keeps the composition from
     * ending and re-asserts the request if it is ever lost. The composition is rebuilt from
     * scratch here so that every call returns the fresh instance the hosting contract requires.
     *
     * <p>Never throws. The two-argument {@code goTo} performs no preset lookup and so has no
     * failure mode of its own, but a {@code null} goal — which can only reach this method through
     * a mis-wired machine — degrades to a plain hold rather than an exception that would propagate
     * out of {@code CommandScheduler.run()} and take the robot loop down with it.
     */
    @Override
    public Command pursueCommand(WristGoal goal) {
        if (goal == null || !Double.isFinite(goal.pitchDegrees()) || !Double.isFinite(goal.rollDegrees())) {
            return mechanism.holdPosition();
        }
        return mechanism.goTo(goal.pitchDegrees(), goal.rollDegrees())
                .andThen(mechanism.holdPosition());
    }

    /**
     * {@code null}, so the pursue command keeps running after arrival.
     *
     * <p>This is the correct answer for a closed-loop mechanism. {@code holdPosition} is already
     * the tail of the pursue composition and already re-drives Motion Magic to the setpoint every
     * loop, so there is nothing a separate hold command would add. Handing back a stop command
     * instead would let a loaded wrist sag out of its own arrival band under gravity and make the
     * state machine oscillate between arrived and not.
     */
    @Override
    public Command holdCommand(WristGoal goal) {
        return null;
    }

    @Override
    public Set<Subsystem> requirements() {
        return requirements;
    }

    /**
     * {@code 0} — no re-assertion needed.
     *
     * <p>The pursue command's {@code holdPosition} tail never ends and re-sends the control
     * request on every loop, so the failure mode re-assertion exists to cover — a command that
     * finishes having silently failed to change anything, as the pneumatic binding suffers under
     * low pressure — cannot occur here.
     */
    @Override
    public int reassertPeriodLoops() {
        return 0;
    }

    /**
     * True when both axes are measured inside the goal's band.
     *
     * <p>Arrival is decided purely from {@code getPitch()} and {@code getRoll()} against the goal's
     * own values. The mechanism's {@code atSetpoint()} is deliberately <b>not</b> a conjunct: it
     * compares the live axes against the setpoints of whatever pose was most recently commanded, so
     * asking "is the wrist at pose A?" while it is being driven toward pose B would fold B's progress
     * into the answer — a measurement must depend only on where the wrist physically is, not on what
     * it was last told. It is also the conjunct that was originally added to defeat the clamped-setpoint
     * lie, but the raw axis checks below already do that: an out-of-range goal simply never satisfies
     * them, which is a truthful negative.
     *
     * <p>{@code secondsSinceApplied} is deliberately unused. Both axes are encoder-derived, so
     * arrival is measured rather than assumed, and a settle timer on top would only delay a transition
     * the sensors have already justified.
     *
     * <p>Pure, allocation-free and non-throwing: it reads only live sensors and the goal, and a
     * {@code null} goal answers {@code false} rather than raising out of the scheduler.
     */
    @Override
    public boolean atGoal(WristGoal goal, double secondsSinceApplied) {
        if (goal == null) {
            return false;
        }
        double tolerance = goal.toleranceDegrees();
        return Math.abs(mechanism.getPitch() - goal.pitchDegrees()) < tolerance
                && Math.abs(mechanism.getRoll() - goal.rollDegrees()) < tolerance;
    }

    /**
     * Signed error on whichever axis is further from its target, in degrees.
     *
     * <p>Reporting the worse axis rather than a per-axis pair or a Euclidean blend keeps the
     * logged number meaningful: it is the one that decides arrival, because {@link #atGoal}
     * requires both axes to be inside the band and so is gated by the laggard. The sign is
     * preserved so a log shows which way the wrist still has to travel. Ties resolve to pitch,
     * which only matters when the two are exactly equal.
     *
     * @param goal the goal to measure against
     * @return the larger-magnitude of the pitch and roll errors, or {@code NaN} for a null goal
     */
    @Override
    public double error(WristGoal goal) {
        if (goal == null) {
            return Double.NaN;
        }
        double pitchError = goal.pitchDegrees() - mechanism.getPitch();
        double rollError = goal.rollDegrees() - mechanism.getRoll();
        return Math.abs(rollError) > Math.abs(pitchError) ? rollError : pitchError;
    }

    /**
     * The goal's own arrival band in degrees, applied to both axes.
     *
     * <p>Note that this is the band this binding enforces, not necessarily the one that decides
     * arrival on its own: {@link #atGoal} also requires the mechanism's {@code atSetpoint()}, so
     * the effective band is the tighter of this and the mechanism's configured
     * {@code toleranceDegrees}, which is package-private and cannot be read from here to report a
     * combined figure.
     */
    @Override
    public double tolerance(WristGoal goal) {
        return goal == null ? Double.NaN : goal.toleranceDegrees();
    }

    /**
     * Always {@code true} — wrist arrival is genuinely sensed.
     *
     * <p>Pitch and roll are both derived from live rotor encoders through the mechanism's sum and
     * difference conversions, so {@link #atGoal} is a measurement and never a settle timer. This
     * holds even for an out-of-range goal: such a goal simply never arrives, which is a truthful
     * negative measurement rather than an unobservable one.
     */
    @Override
    public boolean observable(WristGoal goal) {
        return true;
    }

    /**
     * Low-cardinality label of the form {@code "p12.0/r90.0"}.
     *
     * <p>The interpolated numbers are the goal's own immutable record components, never a live
     * sensor reading, so the string is fixed for the lifetime of the goal and the set of distinct
     * labels is bounded by the number of wrist poses the superstructure declares. The result is
     * memoised because this is called on a path that runs every loop.
     */
    @Override
    public String label(WristGoal goal) {
        if (goal == null) {
            return "none";
        }
        String cached = labelByGoal.get(goal);
        if (cached != null) {
            return cached;
        }
        String built = String.format("p%.1f/r%.1f", goal.pitchDegrees(), goal.rollDegrees());
        labelByGoal.put(goal, built);
        return built;
    }

    /**
     * Human-readable detail interpolating live axis positions, the goal, and the setpoints the
     * mechanism actually latched.
     *
     * <p>Showing the mechanism's own setpoints alongside the goal is the fastest way to see a
     * clamp from a log: when they disagree, the range configuration and not the controller is what
     * is holding the state machine.
     */
    @Override
    public String detail(WristGoal goal) {
        if (goal == null) {
            return "no goal";
        }
        return String.format(
                "pitch %.1f -> %.1f (set %.1f), roll %.1f -> %.1f (set %.1f), tol %.1f deg",
                mechanism.getPitch(), goal.pitchDegrees(), mechanism.getPitchSetpoint(),
                mechanism.getRoll(), goal.rollDegrees(), mechanism.getRollSetpoint(),
                goal.toleranceDegrees());
    }

    /**
     * Names what is keeping the wrist from the goal, in priority order.
     *
     * <p>An unreachable pose is reported first, because no amount of waiting fixes it and the
     * displacement resolved by {@link #validate} says exactly how far out of range it is. Otherwise
     * the note names the lagging axis — the one {@link #error} reports — since on a differential
     * wrist the two axes are mechanically coupled and knowing which one is short tells you whether
     * to look at the average gains or the differential ones. An empty string means both axes are
     * inside the band and there is nothing to say.
     */
    @Override
    public String note(WristGoal goal) {
        if (goal == null) {
            return "no goal";
        }
        if (!zeroedSupplier.getAsBoolean()) {
            return "wrist not zeroed; axis positions are not referenced to anything";
        }

        Double resolved = clampDisplacementByGoal.get(goal);
        double displacement = resolved != null ? resolved : clampDisplacement(goal);
        if (displacement > 0.0) {
            return String.format(
                    "unreachable: goal p=%.1f r=%.1f clamps by up to %.1f deg into ranges "
                            + "pitch [%.1f, %.1f] roll [%.1f, %.1f]; this goal can never arrive",
                    goal.pitchDegrees(), goal.rollDegrees(), displacement,
                    mechanism.getMinPitch(), mechanism.getMaxPitch(),
                    mechanism.getMinRoll(), mechanism.getMaxRoll());
        }

        double pitchError = goal.pitchDegrees() - mechanism.getPitch();
        double rollError = goal.rollDegrees() - mechanism.getRoll();
        double tolerance = goal.toleranceDegrees();
        boolean pitchOut = Math.abs(pitchError) >= tolerance;
        boolean rollOut = Math.abs(rollError) >= tolerance;

        if (pitchOut && rollOut) {
            return String.format("both axes lagging: pitch off %.1f deg, roll off %.1f deg", pitchError, rollError);
        }
        if (pitchOut) {
            return String.format("pitch lagging by %.1f deg (roll inside band)", pitchError);
        }
        if (rollOut) {
            return String.format("roll lagging by %.1f deg (pitch inside band)", rollError);
        }
        if (!mechanism.atSetpoint()) {
            return "both axes inside the goal band but the mechanism's own tighter band "
                    + "has not been met yet";
        }
        return "";
    }

    /**
     * Whether the wrist has been homed.
     *
     * <p>Defaults to {@code true} for the two-argument constructor. That is optimistic, and it is
     * optimistic because it has to be: the mechanism keeps a {@code hasBeenZeroed} flag but
     * exposes no accessor for it, so a binding built without an external supplier has no way to
     * know. A wrist driven from a genuinely unreferenced encoder will happily slew into a hard
     * stop, so teams whose wrist needs homing should use the three-argument constructor and pass
     * their own homing state rather than relying on this default.
     */
    @Override
    public boolean zeroed() {
        return zeroedSupplier.getAsBoolean();
    }

    /**
     * Build-time check that the pose is finite and inside both axis ranges, and resolution of the
     * clamp displacement {@link #note} reports at runtime.
     *
     * <p>An out-of-range pose is an error rather than a warning because of how the mechanism
     * fails: {@code applyTargets} clamps silently, the wrist drives to the limit, and
     * {@code atSetpoint()} then reports success from a pose the robot never asked for. This
     * binding's arrival test refuses to be fooled by that, which converts a silently wrong pose
     * into a state machine that blocks forever — better, but still a match lost. Catching it here
     * turns it into a message on a laptop, alongside every other configuration problem, before the
     * code is ever deployed.
     *
     * @param goal     the goal to check
     * @param problems sink for problem descriptions; the builder aggregates all of them
     */
    @Override
    public void validate(WristGoal goal, Consumer<String> problems) {
        if (goal == null) {
            problems.accept(key + ": null wrist goal");
            return;
        }

        boolean finite = true;
        if (!Double.isFinite(goal.pitchDegrees())) {
            problems.accept(key + ": pitch target is not a finite number (" + goal.pitchDegrees() + ")");
            finite = false;
        }
        if (!Double.isFinite(goal.rollDegrees())) {
            problems.accept(key + ": roll target is not a finite number (" + goal.rollDegrees() + ")");
            finite = false;
        }
        if (!finite) {
            return;
        }

        double minPitch = mechanism.getMinPitch();
        double maxPitch = mechanism.getMaxPitch();
        double minRoll = mechanism.getMinRoll();
        double maxRoll = mechanism.getMaxRoll();

        if (goal.pitchDegrees() < minPitch || goal.pitchDegrees() > maxPitch) {
            problems.accept(String.format(
                    "%s: pitch target %.1f deg is outside the configured range [%.1f, %.1f]; "
                            + "the mechanism would clamp and never report arrival",
                    key, goal.pitchDegrees(), minPitch, maxPitch));
        }
        if (goal.rollDegrees() < minRoll || goal.rollDegrees() > maxRoll) {
            problems.accept(String.format(
                    "%s: roll target %.1f deg is outside the configured range [%.1f, %.1f]; "
                            + "the mechanism would clamp and never report arrival",
                    key, goal.rollDegrees(), minRoll, maxRoll));
        }

        clampDisplacementByGoal.put(goal, clampDisplacement(goal));
        // Warm the label cache here too, so the first loop of a match does no string formatting.
        label(goal);
    }

    /**
     * Does nothing on purpose.
     *
     * <p>Releasing a wrist does not mean letting go of it. The motors are in brake mode and the
     * mechanism holds no latched state that needs unwinding, so the useful behaviour on release is
     * for the wrist to stay where the last goal put it. Neutralling the motors here would drop a
     * loaded wrist under gravity the instant a state was released, which is exactly the wrong
     * thing to do mid-match.
     */
    @Override
    public void release() {
        // Intentionally empty; see javadoc.
    }

    /**
     * How far, in degrees, the worse axis of {@code goal} would be moved by the mechanism's range
     * clamping. Zero means the pose is reachable as asked.
     */
    private double clampDisplacement(WristGoal goal) {
        double pitchClamped = clamp(goal.pitchDegrees(), mechanism.getMinPitch(), mechanism.getMaxPitch());
        double rollClamped = clamp(goal.rollDegrees(), mechanism.getMinRoll(), mechanism.getMaxRoll());
        return Math.max(Math.abs(goal.pitchDegrees() - pitchClamped),
                Math.abs(goal.rollDegrees() - rollClamped));
    }

    /**
     * Clamps {@code value} into {@code [min, max]}, mirroring what the mechanism's
     * {@code applyTargets} does with {@code MathUtil.clamp}. Kept local so this class stays free of
     * a WPILib math import it would otherwise need for one line.
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
