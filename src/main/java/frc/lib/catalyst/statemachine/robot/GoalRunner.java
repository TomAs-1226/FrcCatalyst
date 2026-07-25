package frc.lib.catalyst.statemachine.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
public final class GoalRunner<S extends Enum<S>, G> extends Command {

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
        Set<Subsystem> requirements = actuator.requirements();
        if (requirements != null) {
            for (Subsystem s : requirements) addRequirements(s);
        }
        setName("SM/" + core.name() + "/" + key);
    }

    @Override
    public void initialize() {
        activeGoal = null;
        stopInner();
        arrived = false;
        loopsSinceInit = 0;
        core.noteOwned(key, true);
    }

    @Override
    public void execute() {
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
            startInner();
        }

        double since = clock.getAsDouble() - appliedAtSeconds;
        boolean atGoalNow = safeAtGoal(activeGoal, since);

        if (atGoalNow && !arrived) {
            arrived = true;
            Command hold = cached(holdCache, activeGoal, false);
            if (hold != null) {
                stopInner();
                inner = hold;
                startInner();
            }
        } else if (!atGoalNow && arrived) {
            // Drifted back out of tolerance — go back to pursuing.
            arrived = false;
            Command hold = cached(holdCache, activeGoal, false);
            if (hold != null) {
                stopInner();
                inner = cached(pursueCache, activeGoal, true);
                startInner();
            }
        }

        loopsSinceInit++;
        if (inner == null) return;

        if (!innerFinished) {
            safeExecute();
            if (safeIsFinished()) {
                safeEnd(false);
                innerFinished = true;
            }
        } else if (!atGoalNow && reassertPeriod > 0 && loopsSinceInit % reassertPeriod == 0) {
            // The command completed but the mechanism never got there — a solenoid refused for low
            // pressure, say. Re-fire it rather than sitting silently on an actuation that never happened.
            startInner();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        stopInner();
        safeRelease();
        activeGoal = null;
        arrived = false;
        core.noteOwned(key, false);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    /** The binding key this runner drives. */
    public String bindingKey() {
        return key;
    }

    private void stopInner() {
        if (inner != null && !innerFinished) {
            try {
                inner.end(true);
            } catch (RuntimeException ex) {
                report("end", ex);
            }
        }
        inner = null;
        innerFinished = false;
    }

    /**
     * Initialise {@link #inner}, guarded. If the hosted command throws from {@code initialize()} it
     * is dropped rather than left half-started, and — like every other user-code call in this class —
     * the exception is swallowed so it can never escape into {@code CommandScheduler.run()}.
     */
    private void startInner() {
        if (inner == null) {
            innerFinished = false;
            return;
        }
        try {
            inner.initialize();
            innerFinished = false;
        } catch (RuntimeException ex) {
            report("initialize", ex);
            inner = null;
            innerFinished = false;
        }
    }

    private void safeExecute() {
        try {
            inner.execute();
        } catch (RuntimeException ex) {
            report("execute", ex);
            // A command that threw mid-run is not safe to keep driving; drop it and let the next
            // goal change or reassert rebuild a fresh one.
            inner = null;
            innerFinished = false;
        }
    }

    private boolean safeIsFinished() {
        if (inner == null) return false;
        try {
            return inner.isFinished();
        } catch (RuntimeException ex) {
            report("isFinished", ex);
            return true;   // treat a throwing predicate as finished, so we stop calling execute() on it
        }
    }

    private void safeEnd(boolean interrupted) {
        if (inner == null) return;
        try {
            inner.end(interrupted);
        } catch (RuntimeException ex) {
            report("end", ex);
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
