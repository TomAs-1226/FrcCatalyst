package frc.lib.catalyst.statemachine.robot;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Mechanism;
import frc.lib.catalyst.statemachine.Handle;
import frc.lib.catalyst.statemachine.StateMachineCore;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * The per-mechanism worker that turns the engine's decisions into motor output.
 *
 * <p>Installed as the <b>default command</b> of each bound mechanism's subsystem. That one choice
 * buys driver override for free: a driver's {@code whileTrue(elevator.jogUp(2))} interrupts this
 * runner on the elevator alone, the state machine notices and logs the loss of ownership, and when
 * the button is released WPILib re-schedules the default, {@link #initialize()} clears the applied
 * goal, and the machine's goal is re-applied on the very next loop. No policy flag, no reassertion
 * race, no fighting the driver.
 *
 * <p>Inner commands are <b>hosted</b>, not scheduled: this class calls
 * {@code initialize}/{@code execute}/{@code isFinished}/{@code end} directly. Those are public and
 * the scheduler does nothing else to a leaf command, so it is correct — but it is only correct with
 * the bookkeeping below, in particular never calling {@code execute()} after {@code isFinished()}
 * returned true (which is what makes hosting a sequential group blow up with an index error) and
 * never calling {@code end()} twice.
 *
 * @param <S> the machine's state enum
 * @param <G> the goal type of the bound mechanism
 * @since 1.2.0
 */
public final class GoalRunner<S extends Enum<S>, G> implements Command {

    /** Distinct goals whose commands are cached. Small, because a state machine has few goals per mechanism. */
    private static final int COMMAND_CACHE_SIZE = 4;

    private final StateMachineCore<S> core;
    private final Handle<G> handle;
    private final Actuator<G> actuator;
    private final DoubleSupplier clock;
    private final String key;
    /**
     * Read once, here, rather than every loop from {@link Actuator#reassertPeriodLoops()}. It is
     * configuration, not state, so re-reading buys nothing — and calling a user method every loop
     * from a default command's {@code execute()} is one more place an exception could escape into
     * the scheduler.
     */
    private final int reassertPeriod;

    private final Map<G, Command> pursueCache = boundedCache();
    private final Map<G, Command> holdCache = boundedCache();

    private final Set<Mechanism> required;
    private final String commandName;

    private G activeGoal;
    private Command inner;
    private boolean innerFinished;
    private boolean arrived;
    private int loopsSinceInit;
    private double appliedAtSeconds;

    GoalRunner(StateMachineCore<S> core, Handle<G> handle, Actuator<G> actuator, DoubleSupplier clock) {
        this.core = core;
        this.handle = handle;
        this.actuator = actuator;
        this.clock = clock;
        this.key = handle.key();
        int period;
        try {
            period = actuator.reassertPeriodLoops();
        } catch (RuntimeException ex) {
            period = 0;
        }
        this.reassertPeriod = Math.max(0, period);
        Set<Mechanism> declared = actuator.requirements();
        this.required = declared == null ? Set.of() : Set.copyOf(declared);
        this.commandName = "SM/" + core.name() + "/" + key;
    }

    @Override
    public String name() {
        return commandName;
    }

    @Override
    public Set<Mechanism> requirements() {
        return required;
    }

    /**
     * Commands v3 collapses initialize/execute/isFinished/end into one coroutine body.
     *
     * <p>This also retires the hazard the class comment above described. Under v2 this runner drove
     * the hosted command's {@code initialize}/{@code execute}/{@code isFinished}/{@code end} by
     * hand, and had to guarantee it never executed a finished command or ended one twice. In v3 the
     * hosted command is {@link Coroutine#fork(Command...) forked}: the scheduler owns its lifecycle,
     * and a forked child's requirements are covered by its parent, so forking one that reserves the
     * same mechanisms as this runner does not cancel this runner. The bookkeeping that made the
     * manual version correct is no longer needed, because the manual version is gone.
     */
    @Override
    public void run(Coroutine coroutine) {
        activeGoal = null;
        stopInner();
        arrived = false;
        loopsSinceInit = 0;
        core.noteOwned(key, true);

        while (true) {
            drive(coroutine);
            coroutine.yield();
        }
    }

    private void drive(Coroutine coroutine) {
        G want = core.activeGoalOf(handle);

        if (want == null) {
            // The active state released this mechanism, or the machine is disabled.
            stopInner();
            if (activeGoal != null) {
                safeRelease();
                activeGoal = null;
                arrived = false;
            }
            return;
        }

        if (!Objects.equals(want, activeGoal)) {
            stopInner();
            activeGoal = want;
            arrived = false;
            loopsSinceInit = 0;
            appliedAtSeconds = clock.getAsDouble();
            inner = cached(pursueCache, want, true);
            startInner(coroutine);
        }

        double since = clock.getAsDouble() - appliedAtSeconds;
        boolean atGoalNow = safeAtGoal(activeGoal, since);

        if (atGoalNow && !arrived) {
            arrived = true;
            Command hold = cached(holdCache, activeGoal, false);
            if (hold != null) {
                stopInner();
                inner = hold;
                startInner(coroutine);
            }
        } else if (!atGoalNow && arrived) {
            // Drifted back out of tolerance — go back to pursuing.
            arrived = false;
            Command hold = cached(holdCache, activeGoal, false);
            if (hold != null) {
                stopInner();
                inner = cached(pursueCache, activeGoal, true);
                startInner(coroutine);
            }
        }

        loopsSinceInit++;
        if (inner == null) return;

        // The scheduler owns the forked command now, so "finished" is a question we ask it.
        innerFinished = !Scheduler.getDefault().isScheduledOrRunning(inner);

        if (innerFinished && !atGoalNow && reassertPeriod > 0
                && loopsSinceInit % reassertPeriod == 0) {
            // The command completed but the mechanism never got there — a solenoid refused for low
            // pressure, say. Re-fire it rather than sitting silently on an actuation that never happened.
            startInner(coroutine);
        }
    }

    @Override
    public void onCancel() {
        stopInner();
        safeRelease();
        activeGoal = null;
        arrived = false;
        core.noteOwned(key, false);
    }

    /** The binding key this runner drives. */
    public String bindingKey() {
        return key;
    }

    private void stopInner() {
        if (inner != null && !innerFinished) {
            try {
                Scheduler.getDefault().cancel(inner);
            } catch (RuntimeException ex) {
                report("cancel", ex);
            }
        }
        inner = null;
        innerFinished = false;
    }

    /**
     * Fork {@link #inner} as a child of this runner, guarded. A fork that fails or throws drops the
     * command rather than leaving it half-started, and the exception is swallowed so it can never
     * escape into the scheduler.
     */
    private void startInner(Coroutine coroutine) {
        if (inner == null) {
            innerFinished = false;
            return;
        }
        try {
            // fork() returns void in the released alpha-6; the snapshot returns a result that
            // says whether the scheduler accepted the command. Without it there is nothing to check,
            // so a refusal is not detectable here and the command simply does not run.
            coroutine.fork(inner);
            innerFinished = false;
            if (false) {
                report("fork", new IllegalStateException("scheduler refused the command"));
                inner = null;
            }
        } catch (RuntimeException ex) {
            report("fork", ex);
            inner = null;
            innerFinished = false;
        }
    }

    private Command cached(Map<G, Command> cache, G goal, boolean pursue) {
        Command existing = cache.get(goal);
        if (existing != null) return existing;
        Command built;
        try {
            built = pursue ? actuator.pursueCommand(goal) : actuator.holdCommand(goal);
        } catch (RuntimeException ex) {
            report(pursue ? "pursueCommand" : "holdCommand", ex);
            return null;
        }
        if (built != null) cache.put(goal, built);
        return built;
    }

    private boolean safeAtGoal(G goal, double since) {
        try {
            return actuator.atGoal(goal, since);
        } catch (RuntimeException ex) {
            report("atGoal", ex);
            return false;
        }
    }

    private void safeRelease() {
        try {
            actuator.release();
        } catch (RuntimeException ex) {
            report("release", ex);
        }
    }

    private void report(String what, RuntimeException ex) {
        // Nothing here may propagate: an exception escaping a default command takes down
        // CommandScheduler.run(), and with it the entire robot loop.
        System.err.println("[Catalyst] binding '" + key + "' threw from " + what + "(): " + ex);
    }

    private static <K, V> Map<K, V> boundedCache() {
        return new LinkedHashMap<K, V>(8, 0.75f, true) {
            private static final long serialVersionUID = 1L;
            @Override
            protected boolean removeEldestEntry(Map.Entry<K, V> eldest) {
                return size() > COMMAND_CACHE_SIZE;
            }
        };
    }
}
