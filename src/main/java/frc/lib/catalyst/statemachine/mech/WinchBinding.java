package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.WinchMechanism;
import frc.lib.catalyst.statemachine.goals.WinchGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;

import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;

/**
 * The {@link Actuator} that drives a {@link WinchMechanism} from a {@link WinchGoal}.
 *
 * <p>This is the least sensed of the nine Catalyst bindings, and almost everything unusual
 * about it follows from two properties of the mechanism it wraps. First,
 * {@code WinchMechanism.extend()} and {@code retract()} are bare {@code run(...)} commands
 * with no {@code isFinished}, so they never end on their own; a {@code GoalRunner} that hosts
 * one will keep feeding the configured duty cycle into a hard stop for the rest of the match
 * unless something takes the command away. That is why {@link #holdCommand(WinchGoal)} is
 * non-null for every goal that drives the motor — it is not an optimisation, it is the only
 * thing that stops a climber stalling at full current once it has arrived. Second, when the
 * mechanism is configured with {@code spoolRadius <= 0} it runs in <em>rotation mode</em>, and
 * in that mode its constructor never applies motor soft limits at all, so the firmware will
 * not catch an overrun either.
 *
 * <p>There is no positional goal because the mechanism exposes no position control: every one
 * of its command factories writes a percent output and none of them accepts a target. See
 * {@link WinchGoal} for the full argument; the short version is that a positional winch goal
 * could only be serviced by an untuned bang-bang loop hidden inside this class.
 *
 * <p><b>Arrival is only as good as the predicates.</b> {@code isFullyExtended()} and
 * {@code isFullyRetracted()} compare the raw position against the configured travel range with
 * a hard-coded {@value #LIMIT_EPSILON} dead zone, expressed in metres in linear mode and in
 * <em>rotations</em> in rotation mode. {@link #validate(WinchGoal, Consumer)} says so out loud
 * rather than letting a team discover it by reading the mechanism source.
 *
 * <p>{@link #release()} is deliberately not overridden. It returns {@code void} and so cannot
 * schedule anything, and it does not need to: every command this binding hands out carries a
 * {@code finallyDo} that stops the motors, so relinquishing the binding already leaves the
 * winch at zero output and held by the brake mode the mechanism forces on unconditionally.
 *
 * @since 1.2.0
 */
public final class WinchBinding implements Actuator<WinchGoal> {

    /**
     * The dead zone baked into {@code WinchMechanism.isFullyExtended()} and
     * {@code isFullyRetracted()}, duplicated here because the mechanism hard-codes the literal
     * and exposes no accessor for it. Reported from {@link #tolerance(WinchGoal)} so a log
     * reader can see how coarse the two limit predicates really are. Units follow the
     * mechanism's mode: metres when a spool radius is configured, rotations otherwise.
     */
    public static final double LIMIT_EPSILON = 0.01;

    /**
     * Prefix marking a problem that should not block a build.
     *
     * <p>{@link frc.lib.catalyst.statemachine.Binding#validate} offers a single
     * {@code Consumer<String>} and the builder funnels everything it receives into
     * {@link frc.lib.catalyst.statemachine.StateMachineConfigException}, which is fatal — there
     * is no second channel for the advisory half of
     * {@link frc.lib.catalyst.statemachine.ValidationReport}. Until the builder grows one, this
     * binding tags advisory findings with this prefix so the builder can route them into
     * {@code warnings} with a {@code startsWith} test instead of failing a build over a
     * configuration that is unusual but legal.
     */
    public static final String WARNING_PREFIX = "WARNING: ";

    private final WinchMechanism mechanism;
    private final String key;
    private final Set<Subsystem> requirements;

    /**
     * True when the wrapped mechanism runs in rotation mode ({@code spoolRadius <= 0}), which
     * means no motor soft limits and travel measured in rotations rather than metres.
     *
     * <p>Resolved once at construction because it is a function of an immutable {@code Config}
     * field. {@code Config.spoolRadius} is package-private with no accessor, so the only public
     * window onto it is {@code describe().unit()}, which the mechanism sets to {@code "rot"} in
     * rotation mode and {@code "m"} otherwise.
     */
    private final boolean rotationMode;

    /**
     * Duty cycles resolved once at validation time, keyed by the goal that produced them.
     *
     * <p>{@link WinchGoal.Speed} deliberately preserves an out-of-range duty cycle so that
     * {@link #validate(WinchGoal, Consumer)} can name it in a build-time problem instead of
     * silently clamping the evidence away. Something still has to clamp it before it reaches a
     * motor, and doing that work per loop inside {@link #pursueCommand(WinchGoal)} would be
     * waste, so validation clamps once and stores the result here.
     *
     * <p>Concurrent because validation runs on the thread that builds the superstructure while
     * reads happen on the scheduler thread. Reads fall back to clamping inline, so a goal that
     * never passed through validation still actuates correctly — it simply pays for the clamp
     * every time.
     */
    private final Map<WinchGoal, Double> resolvedDutyCycles = new ConcurrentHashMap<>();

    /**
     * Wraps a winch mechanism as a state machine actuator.
     *
     * @param mechanism the winch to drive; becomes this binding's sole requirement
     * @param key       stable, unique, log-safe key; becomes {@code Bindings/<key>/...} in
     *                  telemetry, so it should be a mechanism-scoped name like
     *                  {@code "climber"} rather than anything derived from a state
     * @throws NullPointerException if either argument is {@code null}. This is the one place
     *                              this class throws: a null here is a wiring mistake that
     *                              should surface at construction in {@code RobotContainer},
     *                              not as a mystery {@code NullPointerException} out of the
     *                              command scheduler in a match.
     */
    public WinchBinding(WinchMechanism mechanism, String key) {
        this.mechanism = Objects.requireNonNull(mechanism, "mechanism");
        this.key = Objects.requireNonNull(key, "key");
        this.requirements = Set.of(mechanism);
        this.rotationMode = resolveRotationMode(mechanism);
    }

    /**
     * Reads the mechanism's mode off its {@code MechanismView}, treating any failure as linear
     * mode.
     *
     * <p>The introspection is a read of an immutable config field plus a status signal and has
     * no realistic failure mode on a robot, where bindings are constructed after the HAL is up.
     * It is guarded anyway because a constructor that throws takes {@code robotInit} with it,
     * and a binding that cannot answer this question is still a perfectly good binding — the
     * only casualty is the rotation-mode advisory in {@link #validate(WinchGoal, Consumer)}.
     */
    private static boolean resolveRotationMode(WinchMechanism mechanism) {
        try {
            return "rot".equals(mechanism.describe().unit());
        } catch (RuntimeException e) {
            return false;
        }
    }

    /** {@inheritDoc} */
    @Override
    public String key() {
        return key;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Matches the {@code kind} the mechanism reports from its own {@code describe()}, so the
     * simulation dashboard picks the same widget whether it is reading the mechanism directly
     * or reading this binding's telemetry.
     */
    @Override
    public String kind() {
        return "winch";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Mirrors {@code WinchMechanism.describe()} exactly: {@code "m"} when a spool radius is
     * configured, {@code "rot"} when the mechanism is in rotation mode. Reporting a fixed
     * {@code "m"} would be a lie in rotation mode, where {@link #measured()} returns spool
     * rotations and a log axis labelled metres would be off by a factor of the circumference.
     */
    @Override
    public String unit() {
        return rotationMode ? "rot" : "m";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Position in the unit reported by {@link #unit()}. A winch has no setpoint to compare
     * this against; it is the raw travel reading and nothing more.
     */
    @Override
    public double measured() {
        return mechanism.getPosition();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Drives the winch open-loop and never waits for arrival — deciding that is
     * {@link #atGoal(WinchGoal, double)}'s job. Every branch returns a freshly built command,
     * because each factory on {@code WinchMechanism} constructs one on every call and a hosted
     * command is initialised more than once over a match.
     *
     * <p>{@code Stop} and an unrecognised or {@code null} goal both fall through to
     * {@code runAtSpeed(0)}. That shared fallback is why this method has no {@code throw} in
     * it: an exception raised here would propagate out of {@code CommandScheduler.run()} and
     * take the whole robot loop down, and zero output is the correct thing to do when the
     * binding does not understand what it was asked for.
     */
    @Override
    public Command pursueCommand(WinchGoal goal) {
        if (goal instanceof WinchGoal.Extend) {
            return mechanism.extend();
        }
        if (goal instanceof WinchGoal.Retract) {
            return mechanism.retract();
        }
        if (goal instanceof WinchGoal.Speed speed) {
            return mechanism.runAtSpeed(dutyCycleFor(speed));
        }
        return mechanism.runAtSpeed(0.0);
    }

    /**
     * {@inheritDoc}
     *
     * <p>Non-null for every goal that drives the motor, and that is <b>mandatory</b> rather
     * than stylistic. {@code extend()} and {@code retract()} never finish, so without a hold to
     * swap in, arrival would change nothing and the winch would keep driving into its hard stop
     * — with no soft limits at all in rotation mode. {@code Speed} needs the same treatment for
     * the same reason: its dwell is an upper bound on driving, not a command that ends.
     *
     * <p>{@code Stop} returns {@code null}, which keeps the pursue command running. That is not
     * the closed-loop {@code null} the interface describes for {@code holdPosition()}; here the
     * pursue command is already {@code runAtSpeed(0)}, so swapping in an identical command
     * would only churn the scheduler.
     */
    @Override
    public Command holdCommand(WinchGoal goal) {
        if (goal instanceof WinchGoal.Extend
                || goal instanceof WinchGoal.Retract
                || goal instanceof WinchGoal.Speed) {
            return mechanism.runAtSpeed(0.0);
        }
        return null;
    }

    /**
     * {@inheritDoc}
     *
     * <p>A pure read of the two limit predicates, the goal, and the elapsed-time parameter —
     * no field records what was last applied, which is what lets {@code isAt} interrogate this
     * binding about goals belonging to states the machine is not in.
     *
     * <p>{@code Extend} and {@code Retract} are genuinely sensed, to within
     * {@value #LIMIT_EPSILON}. {@code Stop} arrives on its first loop because there is nothing
     * to wait for. {@code Speed} is a settle timer, but it also folds in whichever limit
     * predicate faces the direction of travel, so that a generous dwell on a short travel stops
     * driving at the limit instead of grinding there until the stopwatch expires — the dwell
     * bounds how long the winch drives, it does not promise to keep driving that long.
     *
     * <p>An unrecognised or {@code null} goal reports arrived. Reporting "not arrived" would
     * park the machine forever on a goal nothing can satisfy, and throwing would kill the robot
     * loop from inside the scheduler.
     */
    @Override
    public boolean atGoal(WinchGoal goal, double secondsSinceApplied) {
        if (goal instanceof WinchGoal.Extend) {
            return mechanism.isFullyExtended();
        }
        if (goal instanceof WinchGoal.Retract) {
            return mechanism.isFullyRetracted();
        }
        if (goal instanceof WinchGoal.Speed speed) {
            double duty = dutyCycleFor(speed);
            if (duty > 0.0 && mechanism.isFullyExtended()) {
                return true;
            }
            if (duty < 0.0 && mechanism.isFullyRetracted()) {
                return true;
            }
            return secondsSinceApplied >= speed.seconds();
        }
        return true;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Signed distance still to travel, in the unit reported by {@link #unit()}: positive
     * toward extension, negative toward retraction. Only the two limit goals have a target
     * position to measure against — a {@code Speed} goal's target is a duration and a
     * {@code Stop} goal has no target at all, so both report {@code NaN} rather than inventing
     * a number for a dashboard to plot.
     */
    @Override
    public double error(WinchGoal goal) {
        if (goal instanceof WinchGoal.Extend) {
            return mechanism.getMaxPosition() - mechanism.getPosition();
        }
        if (goal instanceof WinchGoal.Retract) {
            return mechanism.getMinPosition() - mechanism.getPosition();
        }
        return Double.NaN;
    }

    /**
     * {@inheritDoc}
     *
     * <p>The limit goals are accurate only to the {@value #LIMIT_EPSILON} dead zone the
     * mechanism hard-codes into its predicates; there is no configurable tolerance to consult.
     * The timed and stopped goals have no position band and report {@code NaN}.
     */
    @Override
    public double tolerance(WinchGoal goal) {
        if (goal instanceof WinchGoal.Extend || goal instanceof WinchGoal.Retract) {
            return LIMIT_EPSILON;
        }
        return Double.NaN;
    }

    /**
     * {@inheritDoc}
     *
     * <p>True only for the two limit goals, whose arrival is read off the encoder. A
     * {@code Speed} goal's arrival is a stopwatch and a {@code Stop} goal's is unconditional —
     * in both cases the state machine is reporting that a command was issued, not that the
     * winch is anywhere in particular, and saying so here keeps anyone reading a log later from
     * mistaking one for the other.
     */
    @Override
    public boolean observable(WinchGoal goal) {
        return goal instanceof WinchGoal.Extend || goal instanceof WinchGoal.Retract;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Low cardinality: one string per distinct goal in the state table. The {@code Speed}
     * variant interpolates its duty cycle, which is safe precisely because that value is an
     * immutable record component fixed when the state table was written — it is not a live
     * sensor reading or a supplier, so this string is constant for the life of the goal and the
     * edge-detected log it feeds writes once rather than at 50 Hz. The dwell is left out to
     * keep the label short; {@link #detail(WinchGoal)} carries it.
     */
    @Override
    public String label(WinchGoal goal) {
        if (goal instanceof WinchGoal.Extend) {
            return "Extend";
        }
        if (goal instanceof WinchGoal.Retract) {
            return "Retract";
        }
        if (goal instanceof WinchGoal.Stop) {
            return "Stop";
        }
        if (goal instanceof WinchGoal.Speed speed) {
            return String.format(Locale.ROOT, "Speed(%.0f%%)", clampDutyCycle(speed.dutyCycle()) * 100.0);
        }
        return "None";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Free to interpolate live values, and does: each branch appends the current position so
     * a blocked transition reads as "how far from the limit is it actually sitting" instead of
     * leaving the reader to correlate two log entries by timestamp.
     */
    @Override
    public String detail(WinchGoal goal) {
        String unit = unit();
        double position = mechanism.getPosition();
        if (goal instanceof WinchGoal.Extend) {
            return String.format(Locale.ROOT, "Extend to %.3f%s (at %.3f%s)",
                    mechanism.getMaxPosition(), unit, position, unit);
        }
        if (goal instanceof WinchGoal.Retract) {
            return String.format(Locale.ROOT, "Retract to %.3f%s (at %.3f%s)",
                    mechanism.getMinPosition(), unit, position, unit);
        }
        if (goal instanceof WinchGoal.Stop) {
            return String.format(Locale.ROOT, "Stop (at %.3f%s)", position, unit);
        }
        if (goal instanceof WinchGoal.Speed speed) {
            return String.format(Locale.ROOT, "Drive %.0f%% for %.2fs (at %.3f%s)",
                    clampDutyCycle(speed.dutyCycle()) * 100.0, speed.seconds(), position, unit);
        }
        return "No winch goal";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Surfaces the two things that most often explain a winch behaving unlike its state
     * table reads. In rotation mode the note leads with the missing soft limits, because a team
     * debugging a climber that overran needs to know the firmware was never going to stop it.
     * Otherwise the note explains the current blocker: a limit goal that has not reached its
     * predicate, or a timed goal that is still dwelling.
     */
    @Override
    public String note(WinchGoal goal) {
        if (rotationMode) {
            return "Rotation mode (spoolRadius <= 0): no motor soft limits are applied, and travel "
                    + "and the " + LIMIT_EPSILON + " limit dead zone are in rotations, not metres.";
        }
        if (goal instanceof WinchGoal.Extend && !mechanism.isFullyExtended()) {
            return String.format(Locale.ROOT,
                    "Still %.3fm short of the extend limit (dead zone %.2fm).",
                    mechanism.getMaxPosition() - mechanism.getPosition(), LIMIT_EPSILON);
        }
        if (goal instanceof WinchGoal.Retract && !mechanism.isFullyRetracted()) {
            return String.format(Locale.ROOT,
                    "Still %.3fm above the retract limit (dead zone %.2fm).",
                    mechanism.getPosition() - mechanism.getMinPosition(), LIMIT_EPSILON);
        }
        if (goal instanceof WinchGoal.Speed) {
            return "Arrival is a dwell timer bounded by the limit predicates, not a sensed position.";
        }
        return "";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code true}, and not because a winch has no zero concept — it has one, and
     * {@code WinchMechanism.zero()} sets it. The mechanism keeps {@code hasBeenZeroed} in a
     * private field and publishes it only through its logged inputs; there is no public getter
     * to query, so this binding cannot honestly gate a state on homing. Reporting {@code false}
     * would reject every state that touches the winch forever, so it reports {@code true} and
     * this javadoc records the gap. Adding {@code public boolean hasBeenZeroed()} to
     * {@code WinchMechanism} is the additive fix.
     */
    @Override
    public boolean zeroed() {
        return true;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Exactly the one winch subsystem, allocated once at construction.
     */
    @Override
    public Set<Subsystem> requirements() {
        return requirements;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Zero, which disables reassertion. Reassertion exists for the pneumatic binding, whose
     * pursue command can finish having silently done nothing. Every command this binding
     * returns is a {@code run(...)} that never finishes, so there is no "finished early without
     * arriving" case to retry — re-initialising would only interrupt a drive that is already
     * happening.
     */
    @Override
    public int reassertPeriodLoops() {
        return 0;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Runs on a laptop at build time, touching no hardware beyond the config values the
     * mechanism already exposes, and reports every problem it finds rather than the first.
     * Advisory findings are prefixed with {@value #WARNING_PREFIX}; see that constant for why
     * the distinction has to be carried in the string.
     *
     * <p>This is also where {@link WinchGoal.Speed} duty cycles are clamped and cached, so the
     * out-of-range value survives long enough to be named in a problem message but never
     * reaches a motor.
     *
     * <p><b>What this cannot check.</b> {@code WinchGoal} suggests validation should catch a
     * {@code retractSpeed} typed as a positive number, which would make a retract goal extend.
     * {@code Config.extendSpeed} and {@code Config.retractSpeed} are package-private with no
     * accessors, so from outside {@code frc.lib.catalyst.mechanisms} that check is impossible
     * today. It becomes possible the moment {@code WinchMechanism} gains
     * {@code getExtendSpeed()} and {@code getRetractSpeed()}.
     */
    @Override
    public void validate(WinchGoal goal, Consumer<String> problems) {
        if (problems == null) {
            return;
        }
        if (goal == null) {
            problems.accept(key + ": null winch goal");
            return;
        }

        if (rotationMode) {
            problems.accept(WARNING_PREFIX + key + ": winch is in rotation mode (spoolRadius <= 0). "
                    + "No motor soft limits are configured, so the " + LIMIT_EPSILON
                    + " extend/retract dead zone is in rotations rather than metres and the limit "
                    + "predicates are the only thing bounding travel.");
        }

        double min = mechanism.getMinPosition();
        double max = mechanism.getMaxPosition();
        if (!(min < max)) {
            problems.accept(key + ": travel range is empty (min " + min + " >= max " + max
                    + "), so isFullyExtended() and isFullyRetracted() are both permanently true "
                    + "and no limit goal can ever mean anything.");
        }

        if (goal instanceof WinchGoal.Speed speed) {
            double raw = speed.dutyCycle();
            double clamped = clampDutyCycle(raw);
            resolvedDutyCycles.put(speed, clamped);

            if (raw != clamped) {
                problems.accept(key + ": duty cycle " + raw + " is outside [-1, 1]; it will be "
                        + "clamped to " + clamped + " before it reaches the motor.");
            }
            if (clamped == 0.0) {
                problems.accept(WARNING_PREFIX + key + ": Speed goal commands zero duty cycle, "
                        + "which is a slower way of writing WinchGoal.stop().");
            }
            if (speed.seconds() == 0.0) {
                problems.accept(WARNING_PREFIX + key + ": Speed goal has a zero-second dwell, so it "
                        + "arrives on its first loop and the hold command stops the motor "
                        + "immediately — the goal is a no-op.");
            }
        }
    }

    /**
     * The duty cycle to actually command for {@code goal}, preferring the value resolved at
     * validation time and clamping inline when the goal never passed through validation.
     */
    private double dutyCycleFor(WinchGoal.Speed goal) {
        Double resolved = resolvedDutyCycles.get(goal);
        return resolved != null ? resolved : clampDutyCycle(goal.dutyCycle());
    }

    /**
     * Clamps a duty cycle into the {@code [-1, 1]} a percent-output request accepts, mapping a
     * non-finite value to zero.
     *
     * <p>{@code WinchGoal.Speed} already normalises {@code NaN} and infinity in its compact
     * constructor; the check is repeated here so this method is safe on any {@code double},
     * including one that reached it some other way.
     */
    private static double clampDutyCycle(double dutyCycle) {
        if (!Double.isFinite(dutyCycle)) {
            return 0.0;
        }
        return MathUtil.clamp(dutyCycle, -1.0, 1.0);
    }
}
