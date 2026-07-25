package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.ClawMechanism;
import frc.lib.catalyst.statemachine.goals.ClawGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;

import java.util.Objects;
import java.util.Set;
import java.util.function.Consumer;

/**
 * The {@link Actuator} that drives a {@link ClawMechanism} from a {@link ClawGoal}.
 *
 * <p>This binding is thin on purpose. It owns no state of its own beyond the mechanism it
 * wraps and the telemetry key it publishes under, which is what lets {@link #atGoal} stay the
 * pure function of live sensors, goal and elapsed time that {@code Binding} demands: the state
 * machine asks this binding about goals belonging to states the robot is <em>not</em> in, so a
 * remembered "last applied goal" field would turn a measurement into a latch and quietly break
 * {@code isAt}.
 *
 * <h2>Why four of the six goals are timers rather than sensors</h2>
 *
 * <p>Three separate properties of {@code ClawMechanism} conspire to leave most claw goals with
 * nothing at all to sense, and each of them has an obvious-looking implementation that does not
 * work. They are worth stating here in full, because the correct code looks lazy and the
 * incorrect code looks careful.
 *
 * <p>The first and worst is {@link ClawMechanism#open()}. It sets its grip state to
 * {@code "OPENING"} inside the command body and to {@code "OPEN"} only inside {@code finallyDo}.
 * The state machine <em>hosts</em> a pursue command rather than scheduling it — it calls
 * {@code initialize}, {@code execute} and {@code isFinished} directly and never lets the command
 * finish on the happy path — so that {@code finallyDo} does not run while the claw is being
 * driven open. The string {@code "OPEN"} is therefore unreachable, and a binding that tested
 * {@code "OPEN".equals(mechanism.getGripState())} would wait for it forever on a claw that had
 * been physically wide open for ten seconds, stalling every state queued behind it. Open arrival
 * is a settle timer, and {@link #observable(ClawGoal)} reports {@code false} for it so that
 * nobody reading a log mistakes that timer for a sensor.
 *
 * <p>The second is that {@link ClawMechanism#close()} never calls the mechanism's stall-detection
 * update; only {@link ClawMechanism#closeUntilGripped()} does. A plain close consequently cannot
 * ever raise {@link ClawMechanism#hasPiece()} through the stall path, so it has no arrival signal
 * either and likewise dwells. This is exactly why {@link ClawGoal.Grip} exists as its own goal
 * rather than as a flag on {@link ClawGoal.Close}, and why reaching for {@code Close} when you
 * meant {@code Grip} fails so late and so quietly.
 *
 * <p>The third is that {@link ClawMechanism#runAtVoltage(double)} does not touch grip-state
 * tracking whatsoever, which leaves {@link ClawGoal.Volts} in the same position.
 *
 * <p>{@link ClawGoal.Grip} is the one goal here with an honest sensed arrival: it runs
 * {@code closeUntilGripped()}, so either the beam break or the stall latch can raise
 * {@code hasPiece()}, and {@link #observable(ClawGoal)} reports {@code true} for it alone.
 *
 * <h2>Grip timeouts belong to the engine, not to this binding</h2>
 *
 * <p>{@link ClawGoal.Grip#maxSeconds()} is deliberately <b>not</b> consulted by {@link #atGoal}.
 * Giving up on a goal is the state machine's job — a hop that blows its deadline is reported as
 * such and routed through the configured {@code FaultPolicy}, where it is visible. If this
 * binding instead returned {@code true} once {@code secondsSinceApplied} passed the timeout, a
 * grip that closed on empty air would be indistinguishable in every log and every
 * {@code isAt} query from a grip that actually captured a piece, and the superstructure would
 * advance to a scoring state holding nothing with no fault raised anywhere. The timeout is
 * surfaced through {@link #detail(ClawGoal)} and {@link #note(ClawGoal)} so it can still be read
 * off a dashboard, and a routine that cares about possession should test
 * {@code hasPiece()} itself rather than infer it from having left the grip state.
 *
 * @see ClawGoal
 * @see Actuator
 * @since 1.2.0
 */
public final class ClawBinding implements Actuator<ClawGoal> {

    /**
     * Largest magnitude a {@link ClawGoal.Volts} setpoint may name, in volts.
     *
     * <p>Anything beyond this is not a stronger command, it is a typo: the motor controller
     * saturates at the battery rail regardless, so a request for 40 V produces exactly the same
     * physical result as a request for 12 V while reading in the log like something that was
     * tuned. {@link #validate} rejects it at build time rather than letting it look deliberate.
     */
    private static final double MAX_ABS_VOLTS = 12.0;

    /**
     * Dwell beyond which a settle time is treated as a units mistake, in seconds.
     *
     * <p>A motor-driven claw travels end to end in a few tenths of a second. A dwell of five
     * seconds is not a conservative tuning choice, it is almost always milliseconds that were
     * typed as seconds, and it costs a third of an autonomous routine.
     */
    private static final double IMPLAUSIBLE_SETTLE_SECONDS = 5.0;

    /**
     * Grip timeout beyond which the bound is treated as no bound at all, in seconds.
     *
     * <p>Fifteen seconds outlasts an entire autonomous period. A timeout that long defeats the
     * point of {@link ClawGoal#grip(double)} demanding one.
     */
    private static final double IMPLAUSIBLE_GRIP_TIMEOUT_SECONDS = 15.0;

    /** The mechanism this binding drives. Never {@code null}. */
    private final ClawMechanism mechanism;

    /** Stable telemetry key; see {@link #key()}. Never {@code null}. */
    private final String key;

    /**
     * Pre-built requirement set.
     *
     * <p>Built once in the constructor rather than per call because {@link #requirements()} is
     * consulted on hot paths, and the state machine is expected to be allocation-free in steady
     * state so that garbage collection never contributes to a loop overrun.
     */
    private final Set<Subsystem> requirements;

    /**
     * Wraps a claw mechanism as a state-machine actuator.
     *
     * <p>There is no settle-time or timeout parameter here, and that is intentional: every
     * duration this binding needs travels on the goal itself, so two states can drive the same
     * physical claw with different dwells without needing two bindings. See {@link ClawGoal}.
     *
     * @param mechanism the claw to drive; must not be {@code null}
     * @param key       stable, unique, log-safe telemetry key — this becomes
     *                  {@code Bindings/<key>/...} in the published telemetry, so it should be
     *                  chosen once and then left alone, since renaming it orphans the history in
     *                  every log a team has already recorded
     * @throws NullPointerException if either argument is {@code null}. Throwing here is safe and
     *                              desirable: construction happens at robot init, long before the
     *                              scheduler is running, so a mistake surfaces as a clean stack
     *                              trace instead of as a mechanism that silently never moves.
     */
    public ClawBinding(ClawMechanism mechanism, String key) {
        this.mechanism = Objects.requireNonNull(mechanism, "mechanism");
        this.key = Objects.requireNonNull(key, "key");
        this.requirements = Set.<Subsystem>of(mechanism);
    }

    /** {@inheritDoc} */
    @Override
    public String key() {
        return key;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Matches the kind string {@link ClawMechanism#describe()} publishes, so a dashboard
     * widget keyed on mechanism kind renders this binding with the same gauge as the mechanism
     * it wraps.
     */
    @Override
    public String kind() {
        return "claw";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Empty, matching the binding specification for this mechanism. Note the wrinkle this
     * leaves behind: {@link #measured()} reports stator current, which is in amps, so the
     * published unit understates what the number is. The alternative — returning {@code "A"} —
     * was not taken because the unit string is part of the agreed telemetry surface for the claw
     * binding and changing it unilaterally would desynchronise this file from the others.
     */
    @Override
    public String unit() {
        return "";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Only {@link ClawGoal.Grip} consults a sensor. Everything else is a dwell or is
     * immediate, for the reasons set out on the type documentation.
     *
     * <p>This method never throws and never allocates. An unrecognised or {@code null} goal
     * reports "not arrived" rather than raising: an exception escaping here would propagate out
     * of {@code CommandScheduler.run()} and take the entire robot loop down mid-match, whereas a
     * goal that never reports arrival blows its hop deadline and is handled by the fault policy
     * exactly like any other mechanism that failed to get where it was told to go.
     *
     * @param goal                the goal to test; a {@code null} or unrecognised goal is
     *                            reported as not arrived
     * @param secondsSinceApplied elapsed time supplied by the engine — the only sanctioned clock
     *                            here. {@code Timer.getFPGATimestamp()} is never called from this
     *                            file, which is what keeps the binding unit-testable off-robot
     *                            and keeps disabled-mode accounting correct.
     */
    @Override
    public boolean atGoal(ClawGoal goal, double secondsSinceApplied) {
        if (goal instanceof ClawGoal.Grip) {
            // The one real measurement in this file: beam break or latched stall.
            return mechanism.hasPiece();
        }
        if (goal instanceof ClawGoal.Hold || goal instanceof ClawGoal.Idle) {
            // Applying a voltage, or removing one, has no travel to wait on.
            return true;
        }
        if (goal instanceof ClawGoal.Open open) {
            // Settle timer, not a sensor: "OPEN" is published only from an unreachable finallyDo.
            return secondsSinceApplied >= open.settleSeconds();
        }
        if (goal instanceof ClawGoal.Close close) {
            // Settle timer: close() never runs stall detection, so hasPiece() cannot latch here.
            return secondsSinceApplied >= close.settleSeconds();
        }
        if (goal instanceof ClawGoal.Volts volts) {
            // Settle timer: runAtVoltage() does not touch grip-state tracking at all.
            return secondsSinceApplied >= volts.settleSeconds();
        }
        return false;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Stator current in amps, which is the only continuously varying quantity a
     * {@code ClawMechanism} exposes. It is genuinely informative for this mechanism even though
     * no goal is expressed in it: current is what rises as the claw bites down on a piece, so a
     * log of it is how a team distinguishes a grip that closed on the piece from one that closed
     * on air, and how they pick a stall threshold in the first place.
     */
    @Override
    public double measured() {
        return mechanism.getCurrent();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code NaN}. No claw goal names a target in the units of {@link #measured()} —
     * not even {@link ClawGoal.Volts}, which names a voltage while the measurement is a current
     * — so any number returned here would be a subtraction between unrelated quantities. The
     * remaining dwell would be the one meaningful "distance to go", but it cannot be computed:
     * this method is not given {@code secondsSinceApplied}, and reading a clock to recover it
     * would reintroduce exactly the hidden time dependence the {@code Binding} contract forbids.
     */
    @Override
    public double error(ClawGoal goal) {
        return Double.NaN;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code NaN}, for the same reason {@link #error(ClawGoal)} is. Returning a goal's
     * settle time here was considered and rejected: it would put a duration in a field that a
     * log reader will compare against a current in amps, which is worse than publishing nothing.
     * The dwells are reported in {@link #detail(ClawGoal)}, where their units are written out.
     */
    @Override
    public double tolerance(ClawGoal goal) {
        return Double.NaN;
    }

    /**
     * {@inheritDoc}
     *
     * <p>{@code true} only for {@link ClawGoal.Grip}, whose arrival is a real beam-break or
     * stall reading. Every other goal is a dwell or an immediate arrival, and says so, so that
     * a log reader debugging a dropped piece can see at a glance which "arrived" markers in the
     * trace were measurements and which were just time passing.
     *
     * <p>{@link ClawGoal.Hold} and {@link ClawGoal.Idle} report {@code false} as well. They are
     * not sensed either — they are unconditionally true — and calling that observable would
     * dress up an assumption as a reading.
     */
    @Override
    public boolean observable(ClawGoal goal) {
        return goal instanceof ClawGoal.Grip;
    }

    /**
     * {@inheritDoc}
     *
     * <p>One of six fixed strings, with no interpolation of any kind. This value is logged and
     * edge-detected, so folding a settle time or a voltage into it would make two states that
     * differ only in dwell look like different goals in a trace, and would defeat the change
     * detection that keeps the label from being rewritten fifty times a second. The numbers live
     * in {@link #detail(ClawGoal)}, which is never edge-detected.
     */
    @Override
    public String label(ClawGoal goal) {
        if (goal instanceof ClawGoal.Close) {
            return "Close";
        }
        if (goal instanceof ClawGoal.Grip) {
            return "Grip";
        }
        if (goal instanceof ClawGoal.Hold) {
            return "Hold";
        }
        if (goal instanceof ClawGoal.Open) {
            return "Open";
        }
        if (goal instanceof ClawGoal.Idle) {
            return "Idle";
        }
        if (goal instanceof ClawGoal.Volts) {
            return "Volts";
        }
        return "Unknown";
    }

    /**
     * {@inheritDoc}
     *
     * <p>The label plus the number the goal carries, and for the two goals that are most often
     * confused with each other, a word about how arrival is decided. This string is written for
     * somebody staring at a dashboard wondering why the claw is or is not advancing, so it
     * spends characters on the distinction between a dwell and a measurement.
     */
    @Override
    public String detail(ClawGoal goal) {
        if (goal instanceof ClawGoal.Close close) {
            return String.format("Close (dwell %.2f s, unsensed)", close.settleSeconds());
        }
        if (goal instanceof ClawGoal.Grip grip) {
            return String.format("Grip (sensed; hop deadline governs give-up, goal timeout %.2f s)",
                    grip.maxSeconds());
        }
        if (goal instanceof ClawGoal.Hold) {
            return "Hold (passive hold voltage, arrival immediate)";
        }
        if (goal instanceof ClawGoal.Open open) {
            return String.format("Open (dwell %.2f s, unsensed)", open.settleSeconds());
        }
        if (goal instanceof ClawGoal.Idle) {
            return "Idle (no output, arrival immediate)";
        }
        if (goal instanceof ClawGoal.Volts volts) {
            return String.format("Volts (%.2f V, dwell %.2f s, unsensed)",
                    volts.volts(), volts.settleSeconds());
        }
        return "Unknown goal";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always reports the live grip state and possession flag, since between them they explain
     * nearly every claw question somebody asks of a log, and then appends whichever caveat
     * applies to the goal being blocked on.
     *
     * <p>The {@code Open} caveat is the one that matters most. Somebody debugging a stuck open
     * will see {@code grip=OPENING} and reasonably conclude the claw has not finished opening;
     * the note tells them that {@code OPENING} is the terminal observable value here and that the
     * only thing they can be waiting on is the dwell.
     */
    @Override
    public String note(ClawGoal goal) {
        String live = "grip=" + mechanism.getGripState() + ", hasPiece=" + mechanism.hasPiece();
        if (goal instanceof ClawGoal.Open) {
            return live + "; arrival is a dwell — the mechanism reports OPENING while driven and "
                    + "only reports OPEN from a finallyDo the hosted command never reaches";
        }
        if (goal instanceof ClawGoal.Close) {
            return live + "; arrival is a dwell — close() does not run stall detection, so "
                    + "hasPiece cannot latch under this goal. Use Grip to capture a piece.";
        }
        if (goal instanceof ClawGoal.Grip) {
            return live + "; arrival is sensed via beam break or stall latch. Running out of time "
                    + "here is a blown hop deadline, not an arrival — this binding never reports "
                    + "a grip that failed as a grip that succeeded.";
        }
        if (goal instanceof ClawGoal.Volts) {
            return live + "; arrival is a dwell — runAtVoltage does not update grip state, so the "
                    + "reported grip state is left over from whatever ran before.";
        }
        return live;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code true}. A motor-driven claw has no homing routine and no zero reference —
     * it has no position sensor to zero — so there is no state in which withholding goals from
     * it would be the safe thing to do.
     */
    @Override
    public boolean zeroed() {
        return true;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Fresh instance on every call, as the contract requires: each of these mechanism
     * factories builds a new command object, so nothing is shared between hops.
     *
     * <p>The mapping is one line per goal. {@link ClawGoal.Grip} maps to
     * {@code closeUntilGripped()} rather than {@code close()} specifically because it is the only
     * factory that runs the stall-detection update and the only one that drops to the passive
     * hold voltage once a piece is captured, which is what stops the motor cooking itself
     * squeezing a piece it already has.
     *
     * <p>This method never throws. A {@code null} or unrecognised goal falls through to
     * {@code stopCommand()} — a claw that is not being driven is the safe failure, and an
     * exception raised here would escape {@code CommandScheduler.run()} and end the match.
     */
    @Override
    public Command pursueCommand(ClawGoal goal) {
        if (goal instanceof ClawGoal.Close) {
            return mechanism.close();
        }
        if (goal instanceof ClawGoal.Grip) {
            return mechanism.closeUntilGripped();
        }
        if (goal instanceof ClawGoal.Hold) {
            return mechanism.hold();
        }
        if (goal instanceof ClawGoal.Open) {
            return mechanism.open();
        }
        if (goal instanceof ClawGoal.Volts volts) {
            return mechanism.runAtVoltage(volts.volts());
        }
        // ClawGoal.Idle, and the defensive null / unknown case.
        return mechanism.stopCommand();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code null}: every claw goal wants its pursue command to keep running after
     * arrival, which is unusual enough among the Catalyst bindings to be worth justifying.
     *
     * <p>A gripper holds by continuing to apply voltage. The mechanism's factories stop the motor
     * in {@code finallyDo}, so swapping to a hold command after arrival would end the pursue
     * command, drop the output to zero, and open the claw under spring or gravity with the piece
     * in it. This is the opposite of the winch case, where the pursue command must be replaced
     * after arrival precisely because continuing to drive would run the mechanism into a hard
     * stop. A claw has no hard stop to damage — it is designed to stall against the piece — and
     * {@code closeUntilGripped()} manages the thermal risk itself by dropping to the passive hold
     * voltage the moment {@code hasPiece()} raises.
     *
     * <p>{@link ClawGoal.Idle} returns {@code null} too, but for a different reason: its pursue
     * command is a {@code runOnce} that has already finished, and its effect — a stopped motor —
     * is persistent, so there is nothing left to keep running.
     */
    @Override
    public Command holdCommand(ClawGoal goal) {
        return null;
    }

    /** {@inheritDoc} */
    @Override
    public Set<Subsystem> requirements() {
        return requirements;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Zero — no reassertion. Five of the six goals run commands that never finish, so there is
     * nothing to re-initialise. The sixth, {@link ClawGoal.Idle}, finishes immediately but leaves
     * a persistent effect and reports arrival unconditionally, so the reassert path is never
     * reached for it either.
     *
     * <p>Contrast the pneumatic binding, which does need this: an actuation refused for low
     * pressure leaves the mechanism in the wrong state while its command still ends normally, so
     * without a retry the state change simply never happens. A claw has no equivalent silent
     * refusal — voltage is applied or the motor is faulted, and a faulted motor is reported by
     * the health checks rather than papered over by retrying.
     */
    @Override
    public int reassertPeriodLoops() {
        return 0;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Checks what can actually be checked from outside the mechanisms package, which is less
     * than one would like — see the limitation noted below.
     *
     * <p>The dwells carried by {@link ClawGoal.Close}, {@link ClawGoal.Open} and
     * {@link ClawGoal.Volts} are already normalised to something finite and non-negative by their
     * compact constructors, and {@link ClawGoal.Grip}'s timeout to something positive and finite,
     * so this method does not re-check for {@code NaN} on those. What it does check is the class
     * of mistake normalisation cannot see: a value that is representable, and therefore silently
     * accepted, but is not a value anybody meant to type.
     *
     * <p><b>Known gap.</b> The most valuable check for this mechanism is unavailable.
     * {@link ClawGoal.Grip} is only meaningful when the mechanism has at least one piece-detection
     * signal configured — a stall threshold or a beam-break port — and without either,
     * {@code hasPiece()} can never become true and the grip will blow its deadline every single
     * time. Both settings live in {@code ClawMechanism.Config} as package-private fields with no
     * accessor, so this binding cannot see them and the mistake survives until the robot is on
     * carpet. Adding {@code ClawMechanism.hasPieceDetection()} would let this method catch it on
     * a laptop instead.
     *
     * @param goal     the goal to check
     * @param problems sink for complaints; the builder aggregates every string reported by every
     *                 binding into one exception, so reporting several problems for one goal is
     *                 both allowed and preferred over stopping at the first
     */
    @Override
    public void validate(ClawGoal goal, Consumer<String> problems) {
        if (goal == null) {
            problems.accept(key + ": null goal");
            return;
        }

        if (goal instanceof ClawGoal.Volts volts) {
            double v = volts.volts();
            if (Double.isNaN(v) || Double.isInfinite(v)) {
                problems.accept(key + ": Volts goal has a non-finite voltage (" + v + ")");
            } else if (Math.abs(v) > MAX_ABS_VOLTS) {
                problems.accept(String.format(
                        "%s: Volts goal requests %.2f V, beyond the %.1f V rail — the controller "
                        + "saturates, so this behaves identically to %.1f V while reading like a "
                        + "tuned value",
                        key, v, MAX_ABS_VOLTS, Math.copySign(MAX_ABS_VOLTS, v)));
            }
            checkSettle(volts.settleSeconds(), "Volts", problems);
            return;
        }

        if (goal instanceof ClawGoal.Close close) {
            checkSettle(close.settleSeconds(), "Close", problems);
            return;
        }

        if (goal instanceof ClawGoal.Open open) {
            checkSettle(open.settleSeconds(), "Open", problems);
            return;
        }

        if (goal instanceof ClawGoal.Grip grip) {
            if (grip.maxSeconds() > IMPLAUSIBLE_GRIP_TIMEOUT_SECONDS) {
                problems.accept(String.format(
                        "%s: Grip timeout of %.2f s outlasts an entire autonomous period, which "
                        + "defeats the point of naming one — a grip that missed will occupy the "
                        + "superstructure for the rest of the routine",
                        key, grip.maxSeconds()));
            }
            return;
        }

        if (goal instanceof ClawGoal.Hold || goal instanceof ClawGoal.Idle) {
            return;
        }

        problems.accept(key + ": unrecognised ClawGoal subtype " + goal.getClass().getName()
                + " — this binding's dispatch chains need a branch for it");
    }

    /**
     * Flags a settle time that is long enough to be a units mistake.
     *
     * <p>Only the upper bound is worth checking. A dwell of zero is explicitly honoured by
     * {@link ClawGoal} as "arrive the instant this is applied", which is a coherent request on a
     * fast claw, and rejecting it here would fail the build for a deliberate choice.
     *
     * @param settleSeconds the dwell to check, already normalised finite and non-negative
     * @param goalName      the goal's label, for the message
     * @param problems      sink for complaints
     */
    private void checkSettle(double settleSeconds, String goalName, Consumer<String> problems) {
        if (settleSeconds > IMPLAUSIBLE_SETTLE_SECONDS) {
            problems.accept(String.format(
                    "%s: %s goal dwells %.2f s, far longer than a claw takes to travel end to "
                    + "end — check for milliseconds typed as seconds",
                    key, goalName, settleSeconds));
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>Nothing to do. This binding holds no resources and caches no state, and stopping the
     * motor is already handled by the {@code finallyDo} on whichever mechanism command was
     * hosted, which runs when the engine ends it. Duplicating that stop here would be harmless
     * but would misrepresent where the responsibility lives.
     */
    @Override
    public void release() {
        // Intentionally empty; see javadoc.
    }
}
