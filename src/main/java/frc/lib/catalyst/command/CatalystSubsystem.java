package frc.lib.catalyst.command;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.RobotBase;

/**
 * What Catalyst subsystems implement in place of the removed {@code SubsystemBase}.
 *
 * <p>Commands v3's {@link Mechanism} is an interface with every method defaulted, which makes it a
 * near drop-in for {@code SubsystemBase} — except for two things Catalyst relied on heavily:
 *
 * <ol>
 *   <li>{@code SubsystemBase} offered {@code run(Runnable)}, {@code runOnce(Runnable)} and
 *       {@code startEnd(...)} as command factories bound to {@code this}. {@code Mechanism} has
 *       {@code run(Consumer<Coroutine>)} instead — same name, different shape — so every existing
 *       call site would otherwise have to be rewritten. The overloads below restore the old shapes.
 *       They coexist with the inherited coroutine version because Java picks by lambda arity:
 *       {@code run(() -> …)} is the {@link Runnable}, {@code run(co -> …)} is the coroutine.</li>
 *   <li>{@code SubsystemBase} registered itself so {@code periodic()} was called every loop.
 *       {@code Mechanism} has no such hook — v3 uses {@link Scheduler#addPeriodic(Runnable)}.
 *       Implementors call {@link #registerPeriodic()} once, from their constructor.</li>
 * </ol>
 *
 * <p>The second point is a genuine behaviour change and the one worth being careful about: a
 * subsystem that defines {@code periodic()} but never calls {@code registerPeriodic()} will compile
 * and silently never tick. Catalyst's own subsystems all register in their constructors.
 *
 * @since 2.0.0
 */
public abstract class CatalystSubsystem extends Mechanism {

    /**
     * A mechanism with a generated name.
     *
     * <p>This is a class rather than an interface on this branch, and not by choice: Commands v3's
     * {@code Mechanism} is an interface in WPILib's development snapshot and a CLASS in the released
     * alpha-6, and an interface cannot extend a class. The released build is the one Systemcore OS
     * beta 13 will actually run, so this follows it.
     *
     * <p>For a team the difference is invisible: mechanisms extend {@code CatalystMechanism}, which
     * extends this, exactly as before.
     */
    protected CatalystSubsystem() {
        super();
    }

    /** A mechanism with an explicit name, as it appears in telemetry and the command list. */
    protected CatalystSubsystem(String name) {
        super(name);
    }

    /** Run {@code action} every loop until the command is cancelled. Requires this mechanism. */
    public CatalystCommand run(Runnable action) {
        return Commands.run(action, this);
    }

    /** Run {@code action} once, then finish. Requires this mechanism. */
    public CatalystCommand runOnce(Runnable action) {
        return Commands.runOnce(action, this);
    }

    /** Run {@code start} on entry and {@code end} on exit, holding this mechanism in between. */
    public CatalystCommand startEnd(Runnable start, Runnable end) {
        return Commands.startEnd(start, end, this);
    }

    /** Hold this mechanism, doing nothing, until cancelled. */
    public CatalystCommand idleCommand() {
        return Commands.idle(this);
    }

    /**
     * Called every scheduler loop, once {@link #registerPeriodic()} has been called.
     *
     * <p>Default is a no-op so subsystems that do not need it can ignore it entirely.
     */
    public void periodic() {}

    /**
     * Called every scheduler loop, but only when the robot program is running in simulation.
     *
     * <p>Same contract {@code SubsystemBase} had. Mechanism sim models update here so the real
     * {@link #periodic()} stays free of simulation-only work.
     */
    public void simulationPeriodic() {}

    /**
     * Register {@link #periodic()} — and {@link #simulationPeriodic()} when running in simulation —
     * with the default scheduler. Call once, from the constructor.
     *
     * <p>This replaces the automatic registration {@code SubsystemBase} performed in its own
     * constructor. It is explicit here because v3 has no constructor to hook.
     */
    public void registerPeriodic() {
        registerPeriodic(Scheduler.getDefault());
    }

    /**
     * As {@link #registerPeriodic()}, on a scheduler of your choosing.
     *
     * @param scheduler where the callbacks are registered
     */
    public void registerPeriodic(Scheduler scheduler) {
        warnIfThisTickWillBeReaped(scheduler);
        scheduler.addPeriodic(() -> guarded(this::periodic, "periodic"));
        if (RobotBase.isSimulation()) {
            scheduler.addPeriodic(() -> guarded(this::simulationPeriodic, "simulationPeriodic"));
        }
    }

    /**
     * Say so, loudly, when this registration is going to be silently thrown away later.
     *
     * <p><b>{@code addPeriodic} callbacks are scoped, and inactive scopes are deleted.</b> Read off
     * the released alpha-6 sources rather than inferred: {@code addPeriodic} delegates to
     * {@code sideload}, which stamps the callback with {@code BindingScope.createNarrowestScope},
     * and {@code Scheduler.runPeriodicSideloads} removes any callback whose scope has gone inactive
     * — without running it and without a word. The rule is exactly three cases:
     *
     * <pre>
     *   a command is currently running  -&gt; ForCommand  — dies when THAT command ends
     *   an opmode is selected (id != 0) -&gt; ForOpmode   — dies at the next opmode switch
     *   neither                         -&gt; Global      — never dies
     * </pre>
     *
     * <p>Constructing subsystems where they are meant to be constructed — in the robot's
     * constructor, before {@code startCompetition} — hits the third case, and everything is fine.
     * That is why this has never bitten. But a lazily built container, or a subsystem created
     * inside a command, hits one of the first two, and then {@code periodic()} runs perfectly until
     * the driver switches from autonomous to teleop and stops forever. The subsystem is still
     * there, its commands still schedule, and nothing anywhere reports a fault — which is about the
     * worst shape a bug can have on a competition field.
     *
     * <p>Catalyst cannot widen the scope; {@code BindingScope} is package-private to WPILib and
     * there is no public way to ask for a global one. What it can do is refuse to let it happen
     * quietly, so this checks the same two conditions WPILib checks and reports through the Driver
     * Station. A warning at construction is recoverable; a dead subsystem in the second match is
     * not.
     */
    private void warnIfThisTickWillBeReaped(Scheduler scheduler) {
        String scope;
        try {
            if (scheduler.currentCommand() != null) {
                scope = "a command that is running now";
            } else if (org.wpilib.driverstation.RobotState.getOpModeId() != 0) {
                scope = "the opmode selected right now";
            } else {
                return;   // Global scope. Nothing to say.
            }
        } catch (Throwable ignored) {
            return;       // No HAL, no opmode: a desktop test. Nothing to say there either.
        }

        DriverStationErrors.reportWarning(
                "[Catalyst] " + getName() + ".registerPeriodic() was called while " + scope
                        + " was active, so its periodic() is bound to that scope and WPILib will "
                        + "delete it when the scope ends — silently, with the subsystem still "
                        + "present and its commands still working. Construct subsystems in the "
                        + "robot constructor, before startCompetition(), where the scope is global.",
                false);
    }

    /**
     * Run a periodic callback, absorbing anything it throws.
     *
     * <p>Not defensive habit — without it, one unchecked exception out of any {@code periodic()}
     * ends the robot for the rest of the match.
     *
     * <p>v3 runs periodic callbacks as sideloaded coroutines. An exception escaping one destroys
     * that coroutine, and the scheduler re-mounts the dead continuation on every later
     * {@code run()}, so every subsequent loop throws before reaching a single command. Nothing
     * recovers it — not disabling, not re-enabling. From the driver's station the robot works and
     * then freezes completely at some arbitrary moment, with the log filling with
     * {@code IllegalStateException: Mounted!!!!} from inside WPILib, naming nothing the team wrote.
     *
     * <p>The moment is arbitrary because the trigger is ordinary: the first null vision frame, the
     * first CAN read on a device that has just browned out. A subsystem that reads a sensor which is
     * briefly absent is a normal subsystem.
     *
     * <p>This is one place v3 is strictly worse than v2, where the same exception recurred but the
     * scheduler carried on once the condition cleared. Catching here restores that.
     */
    private void guarded(Runnable body, String which) {
        try {
            body.run();
        } catch (RuntimeException | Error e) {
            // Reported, not swallowed. A subsystem quietly failing every loop is its own kind of
            // undiagnosable, so this goes where a team already looks for faults.
            DriverStationErrors.reportError(
                    getName() + "." + which + " threw: " + e, e.getStackTrace());
        }
    }
}
