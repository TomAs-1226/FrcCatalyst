package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.PneumaticMechanism;
import frc.lib.catalyst.statemachine.goals.PneumaticGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;

import java.util.Objects;
import java.util.Set;
import java.util.function.Consumer;

/**
 * The {@link frc.lib.catalyst.statemachine.robot.Actuator} implementation for
 * {@link PneumaticMechanism} — a solenoid driven to a commanded state, with arrival decided by a
 * settle timer rather than by any sensor.
 *
 * <p>Every other Catalyst binding closes a loop around a real measurement: an elevator knows where
 * its carriage is, a flywheel knows how fast it is spinning. A pneumatic actuator knows nothing at
 * all. There is no rod-position sensor in the FRC pneumatics ecosystem, so the only fact available
 * to this binding is which coil the mechanism believes it energised. Arrival is therefore defined
 * as "the mechanism reports the state we asked for, and enough time has passed for the piston to
 * have physically travelled" — and {@link #observable(PneumaticGoal)} returns {@code false} for
 * every goal so that anyone reading a log a week later can see at a glance that the arrival flag
 * was a stopwatch and not a limit switch.
 *
 * <p>The awkward behaviour this binding exists to paper over lives in
 * {@code PneumaticMechanism.applyState}. When the mechanism was built with
 * {@code requirePressureAbove(psi)} and measured pressure is under that threshold, a
 * {@code FORWARD} actuation is refused: the mechanism raises an {@code AlertManager} warning,
 * returns early <em>without</em> updating {@code getState()}, and the {@code runOnce} command that
 * requested it nevertheless ends completely normally. Nothing in the command layer distinguishes
 * that from success. Two decisions here cover it. Arrival is tested against
 * {@code getState() == goal.state()} rather than against the command having finished, so a refusal
 * reads as "not arrived"; and {@link #reassertPeriodLoops()} is non-zero, so the runner re-runs the
 * actuation roughly twice a second and the piston fires by itself the moment the compressor catches
 * up. The same re-assertion incidentally repairs a solenoid that some operator-bound
 * {@code toggle()} moved out from under the state machine.
 *
 * <p><b>The pressure threshold is not readable from outside the mechanisms package.</b>
 * {@code PneumaticMechanism.Config.minPressurePSI} is a package-private field and the mechanism
 * exposes no {@code getConfig()}, so this binding cannot discover on its own whether pressure
 * gating is configured or what the threshold is. Teams that want
 * {@link #note(PneumaticGoal)} to name the number must mirror it into
 * {@link #PneumaticBinding(PneumaticMechanism, String, double)}; the two-argument constructor still
 * produces a useful note, it just cannot quote a threshold it was never told.
 *
 * @since 1.2.0
 */
public final class PneumaticBinding implements Actuator<PneumaticGoal> {

    /** The wrapped mechanism. Never null; also the single element of {@link #requirements()}. */
    private final PneumaticMechanism mechanism;

    /** Stable telemetry key, used verbatim as {@code Bindings/<key>/...}. Never null. */
    private final String key;

    /**
     * The team's mirror of {@code requirePressureAbove(psi)}, or {@link Double#NaN} when the
     * caller did not supply one. Used only to phrase {@link #note(PneumaticGoal)}; it never gates
     * actuation here, because the real gate lives inside the mechanism and this value is only ever
     * a copy that could drift out of date.
     */
    private final double minPressurePSI;

    /**
     * Wrap a mechanism whose pressure threshold is unknown to the caller.
     *
     * <p>This is the right constructor for a mechanism built without
     * {@code requirePressureAbove(psi)}, and an acceptable one for a mechanism built with it —
     * {@link #note(PneumaticGoal)} degrades to describing the symptom and the measured pressure
     * without quoting a minimum.
     *
     * @param mechanism the pneumatic mechanism to drive; must not be null
     * @param key       stable, unique, log-safe telemetry key; must not be null
     */
    public PneumaticBinding(PneumaticMechanism mechanism, String key) {
        this(mechanism, key, Double.NaN);
    }

    /**
     * Wrap a mechanism and mirror its configured pressure threshold for diagnostics.
     *
     * <p>Pass the same number that was given to {@code Config.Builder.requirePressureAbove(psi)}.
     * It is used for nothing but the wording of {@link #note(PneumaticGoal)}, so a stale or absent
     * value costs a less specific blocker message and nothing else — this binding never makes an
     * actuation decision from it.
     *
     * @param mechanism      the pneumatic mechanism to drive; must not be null
     * @param key            stable, unique, log-safe telemetry key; must not be null
     * @param minPressurePSI the mirrored {@code requirePressureAbove} threshold in psi, or
     *                       {@link Double#NaN} when there is none or it is not known here
     */
    public PneumaticBinding(PneumaticMechanism mechanism, String key, double minPressurePSI) {
        this.mechanism = Objects.requireNonNull(mechanism, "PneumaticBinding.mechanism must not be null");
        this.key = Objects.requireNonNull(key, "PneumaticBinding.key must not be null");
        this.minPressurePSI = minPressurePSI;
    }

    /** {@inheritDoc} */
    @Override
    public String key() {
        return key;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Matches the {@code "pneumatic"} kind that {@code PneumaticMechanism.describe()} publishes,
     * so a dashboard can render the binding and the mechanism with the same widget.
     */
    @Override
    public String kind() {
        return "pneumatic";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Psi, because the only continuous number a pneumatic actuator has is tank pressure. Note
     * that this describes {@link #measured()}, which is a property of the air system rather than of
     * the piston — there is no unit in which this mechanism's position could be reported.
     */
    @Override
    public String unit() {
        return "psi";
    }

    /**
     * Live tank pressure in psi, or {@link Double#NaN} when no analog sensor is available.
     *
     * <p>{@code PneumaticMechanism.getPressure()} returns {@code -1} both when no {@code Compressor}
     * was attached and, on a CTRE PCM, when there is no analog channel to read. That sentinel is
     * translated to {@code NaN} here so it is not logged and plotted as a real reading of minus one
     * psi.
     *
     * @return tank pressure in psi, or {@code NaN} when unavailable
     */
    @Override
    public double measured() {
        double psi = mechanism.getPressure();
        return psi < 0.0 ? Double.NaN : psi;
    }

    /**
     * Has the solenoid been commanded to {@code goal}'s state, and has the piston had time to get
     * there?
     *
     * <p>Both halves matter. The state comparison is what catches a refused actuation: a
     * {@code FORWARD} the mechanism declined for low pressure leaves {@code getState()} unchanged,
     * so this stays false and {@link #reassertPeriodLoops()} keeps trying. The elapsed-time
     * comparison is the stand-in for travel, since nothing on the robot can report that the rod
     * moved.
     *
     * <p>This is a pure function of the mechanism's commanded state, the goal, and the supplied
     * elapsed time — it holds no memory of what was last applied, which is what lets
     * {@code StateMachineCore.isAt} ask it about states the machine is not in. One consequence
     * follows from that and is worth knowing when reading a log: {@code secondsSinceApplied} is
     * {@code 0.0} for a goal that is not currently applied, so a query about an unapplied goal with
     * a non-zero settle time answers false even when the solenoid happens to be sitting in exactly
     * that state. Arrival by stopwatch is only meaningful while the stopwatch is running.
     *
     * @param goal                the goal to test; a null goal is never reached
     * @param secondsSinceApplied seconds since this goal was first applied, or {@code 0.0}
     * @return {@code true} when the commanded state matches and the settle time has elapsed
     */
    @Override
    public boolean atGoal(PneumaticGoal goal, double secondsSinceApplied) {
        if (goal == null) {
            return false;
        }
        return mechanism.getState() == goal.state() && secondsSinceApplied >= goal.settleSeconds();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@link Double#NaN}. Error is the distance still to travel toward the goal, and this
     * mechanism has no coordinate in which to express one — the rod is either where the coil put it
     * or it is not, and nothing measures which. Tank pressure is reported by {@link #measured()},
     * but it is not an error toward the goal and is deliberately not returned here, where a
     * dashboard would plot it against a setpoint that does not exist.
     */
    @Override
    public double error(PneumaticGoal goal) {
        return Double.NaN;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@link Double#NaN}, for the same reason as {@link #error(PneumaticGoal)}: with no
     * measured position there is no band around it. The goal's tolerance for reality is
     * {@code settleSeconds}, which is a duration rather than a distance and so cannot be returned
     * through this method — it is surfaced in {@link #detail(PneumaticGoal)} instead.
     */
    @Override
    public double tolerance(PneumaticGoal goal) {
        return Double.NaN;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code false}. This is the honest half of the whole class: arrival here is a settle
     * timer wearing a sensor's clothes, and publishing that fact under
     * {@code Bindings/<key>/Observable} is what stops a future reader of a log from concluding that
     * a piston was confirmed extended when all that was confirmed is that a quarter second passed.
     */
    @Override
    public boolean observable(PneumaticGoal goal) {
        return false;
    }

    /**
     * {@inheritDoc}
     *
     * <p>The bare enum constant name — {@code FORWARD}, {@code REVERSE} or {@code OFF}. Three
     * possible values across the whole life of the robot, with no live number interpolated, which
     * is what the low-cardinality contract asks for: this string is edge-detected before it is
     * logged, so a label carrying the settle time or the current pressure would write at loop rate
     * forever.
     */
    @Override
    public String label(PneumaticGoal goal) {
        return goal == null ? "none" : goal.state().name();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Free to interpolate, and does: the commanded state, whether the mechanism has actually
     * taken it, the settle time being waited out, and tank pressure when a sensor is present. When
     * the goal and the mechanism disagree this reads as "FORWARD commanded, solenoid still OFF at
     * 34 psi", which is usually enough on its own to diagnose a refused actuation from a log.
     */
    @Override
    public String detail(PneumaticGoal goal) {
        if (goal == null) {
            return "no goal";
        }
        double psi = mechanism.getPressure();
        String pressure = psi >= 0.0 ? String.format(" at %.0f psi", psi) : "";
        if (mechanism.getState() == goal.state()) {
            return String.format("%s commanded, %.2f s settle%s",
                    goal.state().name(), goal.settleSeconds(), pressure);
        }
        return String.format("%s commanded, solenoid still %s%s",
                goal.state().name(), mechanism.getState().name(), pressure);
    }

    /**
     * {@inheritDoc}
     *
     * <p>Speaks up in the two situations where this mechanism fails quietly.
     *
     * <p>The first is the refused actuation. A {@code FORWARD} goal that the mechanism has not
     * taken, on a robot with a pressure sensor, is very probably a refusal by the mechanism's own
     * pressure guard. When the threshold was mirrored into this binding at construction the note
     * quotes it — {@code "solenoid refused: 42 psi < 60 psi minimum"}. When it was not, the note
     * says the same thing without the number and says why it cannot: the threshold lives in a
     * package-private {@code Config} field with no accessor, so a binding outside the mechanisms
     * package genuinely cannot read it.
     *
     * <p>The second is a {@code REVERSE} goal on a single solenoid, which is unreachable by
     * construction. {@link #validate(PneumaticGoal, Consumer)} rejects that at build time, so this
     * note only ever appears if a goal reached the runtime without passing through the builder.
     */
    @Override
    public String note(PneumaticGoal goal) {
        if (goal == null) {
            return "";
        }
        if (goal.state() == PneumaticMechanism.State.REVERSE && mechanism.getDoubleSolenoid() == null) {
            return "REVERSE is unreachable on a single solenoid; retract() commands OFF instead";
        }
        if (goal.state() == PneumaticMechanism.State.FORWARD
                && mechanism.getState() != PneumaticMechanism.State.FORWARD) {
            double psi = mechanism.getPressure();
            if (psi < 0.0) {
                return "";
            }
            if (!Double.isNaN(minPressurePSI)) {
                if (psi < minPressurePSI) {
                    return String.format("solenoid refused: %.0f psi < %.0f psi minimum", psi, minPressurePSI);
                }
                return "";
            }
            return String.format(
                    "solenoid refused at %.0f psi (threshold unreadable from outside the mechanisms package)",
                    psi);
        }
        return "";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code true}. A solenoid has no home position and nothing to home against, so
     * gating a state on this binding being zeroed would never do anything but return true.
     */
    @Override
    public boolean zeroed() {
        return true;
    }

    /**
     * {@inheritDoc}
     *
     * <p>One structural problem is worth catching here, and it is worth catching on a laptop rather
     * than in a pit: a {@code REVERSE} goal aimed at a single-solenoid mechanism. A single solenoid
     * has one coil and two electrical states, and {@code PneumaticMechanism.retract()} maps to
     * {@code OFF} rather than {@code REVERSE} when the mechanism was built with
     * {@code singleSolenoid(...)}. So {@code getState()} can never become {@code REVERSE},
     * {@link #atGoal} can never become true, and the state owning that goal would simply hang
     * forever with no error anywhere. The fix at the call site is to use {@code PneumaticGoal.off()}
     * instead, which is what retracting means on this wiring.
     *
     * <p>Nothing else is reported. In particular a {@code FORWARD} goal on a mechanism with a
     * pressure guard is not a build-time problem — it is legal, expected, and handled at runtime by
     * {@link #reassertPeriodLoops()} — and the settle time cannot be sanity-checked against
     * anything, since the only honest bound on a piston's travel time is the piston.
     */
    @Override
    public void validate(PneumaticGoal goal, Consumer<String> problems) {
        if (goal == null) {
            problems.accept(key + ": null PneumaticGoal");
            return;
        }
        if (goal.state() == PneumaticMechanism.State.REVERSE && mechanism.getDoubleSolenoid() == null) {
            problems.accept(key + ": REVERSE goal on a single-solenoid mechanism is unreachable — "
                    + "retract() commands OFF on a single solenoid, so getState() never becomes REVERSE "
                    + "and this goal can never be reached. Use PneumaticGoal.off() instead.");
        }
    }

    /**
     * A fresh {@code runOnce} that commands the goal's solenoid state.
     *
     * <p>{@code extend()}, {@code retract()} and {@code off()} each build a new command every call,
     * so the fresh-instance contract is satisfied by construction. Each ends immediately after
     * latching the solenoid, which is a persistent effect and therefore allowed to end — and when
     * the latch was refused for low pressure, the runner's re-assertion re-initialises it. Note that
     * {@code retract()} is deliberately used for {@code REVERSE} rather than the double solenoid
     * being driven directly: on a double solenoid it energises the reverse channel, which is exactly
     * the goal, and on a single solenoid it commands {@code OFF}, which is the case
     * {@link #validate(PneumaticGoal, Consumer)} has already rejected at build time.
     *
     * <p>This method never throws. A null goal yields a do-nothing command rather than an exception,
     * because an exception raised here would propagate out of {@code CommandScheduler.run()} and
     * take the entire robot loop down with it.
     *
     * @param goal the goal to pursue
     * @return a fresh command that commands the solenoid, never null
     */
    @Override
    public Command pursueCommand(PneumaticGoal goal) {
        if (goal == null) {
            return Commands.none().withName(key + ".NoGoal");
        }
        switch (goal.state()) {
            case FORWARD:
                return mechanism.extend();
            case REVERSE:
                return mechanism.retract();
            case OFF:
            default:
                return mechanism.off();
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always null, meaning "keep pursuing". There is nothing to hold: a solenoid is a latch, and
     * once {@code applyState} has driven it the coil stays where it was put with no further command
     * involvement. Returning null also keeps the re-assertion path alive, which is the point — a
     * hold command would replace the pursue command and with it the only mechanism this binding has
     * for retrying an actuation the mechanism refused.
     *
     * <p>This is the opposite of the winch binding, where a non-null hold is mandatory because an
     * arrived winch that keeps being driven climbs into its own hard stop. A solenoid draws its
     * holding current whether or not anything is scheduled, so there is no equivalent hazard.
     */
    @Override
    public Command holdCommand(PneumaticGoal goal) {
        return null;
    }

    /** {@inheritDoc} */
    @Override
    public Set<Subsystem> requirements() {
        return Set.of(mechanism);
    }

    /**
     * Re-run the actuation every {@value PneumaticGoal#DEFAULT_REASSERT_PERIOD_LOOPS} loops,
     * roughly twice a second at 50 Hz, for as long as the goal has not been reached.
     *
     * <p>This is the single most important line in the class. Without it, a {@code FORWARD}
     * commanded while the tank sits below {@code requirePressureAbove(psi)} is dropped on the floor:
     * the mechanism warns, returns early without changing state, the {@code runOnce} ends normally,
     * and the owning state waits on an actuation nobody is ever going to attempt again. With it, the
     * piston fires on its own within half a second of the compressor recovering.
     *
     * <p>The value is a constant because the interface method takes no goal — it is asked of the
     * binding, not of the goal in hand, so there is no way to honour a per-goal period from here.
     * {@link PneumaticGoal} nevertheless carries its own {@code reassertPeriodLoops} component, and
     * the engine is free to prefer that per-goal value wherever it has the applied goal available;
     * this method is the fallback for the places where it does not. The two agree by default, since
     * both are {@link PneumaticGoal#DEFAULT_REASSERT_PERIOD_LOOPS}.
     *
     * @return {@code 25} loops
     */
    @Override
    public int reassertPeriodLoops() {
        return PneumaticGoal.DEFAULT_REASSERT_PERIOD_LOOPS;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Deliberately does nothing. Releasing a binding means the state machine has stopped
     * managing this mechanism, not that the mechanism should be safed — and venting a solenoid on
     * release would drop a climber hook or release a shifter at the exact moment control changes
     * hands. The piston stays where it was left; a robot that wants it elsewhere should command a
     * goal that says so.
     */
    @Override
    public void release() {
        // Intentionally empty: see javadoc. A solenoid holds its state without supervision, and
        // de-energising here would be an unrequested motion at a control handover.
    }
}
