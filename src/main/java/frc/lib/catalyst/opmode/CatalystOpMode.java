package frc.lib.catalyst.opmode;

import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.system.SystemCoreStatus;

import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.OpMode;

/**
 * A Catalyst-aware {@link OpMode}.
 *
 * <h2>What an OpMode is</h2>
 *
 * <p>WPILib 2027 replaced "autonomous is whatever {@code autonomousInit} does" with named modes the
 * <em>Driver Station lists and selects</em>. A routine is a class with an annotation on it:
 *
 * <pre>{@code
 * @Autonomous(name = "Three Piece Left", group = "Competition")
 * public class ThreePieceLeft extends CatalystOpMode {
 *     @Override
 *     public void onStart() {
 *         run(auto.threePiece());
 *     }
 * }
 * }</pre>
 *
 * <p>and it appears on the Driver Station under its group, with its description, ready to pick. No
 * chooser to publish, no dashboard to configure, and nothing to remember to update when a routine is
 * added — which is the failure the old way had: an auto written on Friday that nobody could select
 * on Saturday because the chooser was never touched.
 *
 * <h2>What this adds</h2>
 *
 * <p>An OpMode on its own does not run Catalyst. Its {@code periodic()} is called and nothing else
 * happens: the command scheduler does not tick, so no command runs, no default command runs, and no
 * subsystem periodic fires. A team's first OpMode therefore does nothing at all and gives no clue
 * why.
 *
 * <p>This runs the scheduler, publishes machine status, and records which OpMode is active so it
 * appears in logs and on Console beside everything else. Subclasses override {@link #onStart()},
 * {@link #onPeriodic()} and {@link #onEnd()} and never think about it.
 *
 * <p>Registering them is one line in the robot class:
 *
 * <pre>{@code
 * public class Robot extends OpModeRobot {
 *     public Robot() {
 *         addAnnotatedOpModeClasses(getClass().getPackage());   // finds every annotated OpMode
 *     }
 * }
 * }</pre>
 *
 * @since 2.0.0
 */
public abstract class CatalystOpMode implements OpMode {

    /** Whether the scheduler this OpMode drives should be the shared default. */
    private final boolean useDefaultScheduler;

    private Scheduler scheduler;
    private boolean started;

    /** An OpMode driving the default scheduler, which is what robot code wants. */
    protected CatalystOpMode() {
        this(true);
    }

    /**
     * @param useDefaultScheduler false to drive a scheduler of this OpMode's own, which is for tests
     *                            and for the rare mode that must not inherit the robot's commands
     */
    protected CatalystOpMode(boolean useDefaultScheduler) {
        this.useDefaultScheduler = useDefaultScheduler;
    }

    /** The scheduler this OpMode drives. */
    protected final Scheduler scheduler() {
        if (scheduler == null) {
            scheduler = useDefaultScheduler
                    ? Scheduler.getDefault()
                    : Scheduler.createIndependentScheduler();
        }
        return scheduler;
    }

    /**
     * Schedule a command on this OpMode's scheduler, and own it for the life of the mode.
     *
     * <p>Anything started here is cancelled when the mode ends. That was not true before, and the
     * gap is narrower than it sounds but real, because Commands v3 looks like it already covers
     * this. {@code Scheduler.schedule()} stamps every command with a {@code BindingScope.ForOpmode}
     * carrying the current opmode <em>id</em>, and reaps it when that id changes - so auto running
     * on into teleop is genuinely handled by WPILib, and so is a mode being deselected.
     *
     * <p>What the id does not change for is a disable. {@code RobotState.getOpModeId()} reflects the
     * mode <em>selected on the Driver Station</em>, and its own javadoc says it "does not mean the
     * robot is enabled". So on a disable, {@code OpModeRobot} ends the mode and forces a fresh
     * instance of the same one; that instance's {@code disabledPeriodic()} runs the scheduler, and
     * the command left over from auto keeps advancing through the entire disabled window. Timers
     * expire, sequential groups step forward, and nothing reveals it because the HAL is holding the
     * outputs neutral. Measured on the alpha-6 scheduler: ten further executions across ten disabled
     * ticks.
     *
     * <p>Nor does the id change when the same mode is enabled a second time - the pit routine of
     * running auto, disabling, and running it again. Then the leftover is still holding the
     * drivetrain when the new run starts.
     *
     * <p>A command scheduled directly on {@code scheduler()} is not tracked and not cancelled. That
     * is deliberate: it is the escape hatch for something meant to outlive the mode.
     */
    protected final void run(org.wpilib.command3.Command command) {
        owned.add(command);
        scheduler().schedule(command);
    }

    /**
     * The commands this mode started through {@link #run}, in the order they were started.
     *
     * <p>For a test or a dashboard. The list is a snapshot; mutating it does nothing.
     */
    public final java.util.List<org.wpilib.command3.Command> scheduledCommands() {
        return java.util.List.copyOf(owned);
    }

    /**
     * Cancel everything this mode started and forget it.
     *
     * <p>Cancelling a command that already finished is a documented no-op, so this is safe to call
     * more than once and safe to call on a mode that ended cleanly.
     */
    private void releaseOwned() {
        for (org.wpilib.command3.Command c : owned) {
            try {
                scheduler().cancel(c);
            } catch (Throwable ignored) {
                // One command refusing to die must not strand the rest.
            }
        }
        owned.clear();
    }

    /** Commands started through {@link #run}, owned until the mode ends. */
    private final java.util.List<org.wpilib.command3.Command> owned = new java.util.ArrayList<>();

    /** The name this OpMode is known by. Defaults to the class's simple name. */
    public String name() {
        return getClass().getSimpleName();
    }

    // --- what a subclass overrides -------------------------------------------

    /** Called once when the mode is selected and enabled. Schedule the routine here. */
    protected void onStart() {
    }

    /** Called every loop while the mode runs, after the scheduler has ticked. */
    protected void onPeriodic() {
    }

    /** Called once when the mode ends, whether it finished or the match did. */
    protected void onEnd() {
    }

    /** Called every loop while this mode is selected but the robot is disabled. */
    protected void onDisabledPeriodic() {
    }

    // --- the OpMode contract, wired to Catalyst -------------------------------

    @Override
    public final void start() {
        started = true;
        CatalystLog.log("OpMode/Active", name());
        CatalystLog.log("OpMode/Running", true);
        onStart();
    }

    @Override
    public final void periodic() {
        // Do nothing while disabled: disabledPeriodic() owns that window.
        //
        // periodic() is documented to be called only while enabled, and the shipped alpha-6
        // OpModeRobot does not honour that - it registers this callback when the opmode is selected,
        // with no enabled gate, so while disabled BOTH hooks fire and the scheduler ran twice per
        // robot loop. Everything that counts loops counted double: command timeouts expired at half
        // their stated duration, debounces halved, and onPeriodic() fired on disabled loops
        // alongside onDisabledPeriodic(), against its own javadoc.
        //
        // Gating here rather than in disabledPeriodic() because the disabled branch of loopFunc is
        // skipped entirely while enabled - verified, zero disabledPeriodic calls across an enabled
        // phase - so exactly one hook does the work in either state, in both wpilib builds.
        if (!org.wpilib.driverstation.RobotState.isEnabled()) {
            return;
        }
        // The scheduler first, so a subclass reading mechanism state in onPeriodic sees the state
        // this loop produced rather than the previous loop's.
        scheduler().run();
        SystemCoreStatus.getInstance().publish();
        frc.lib.catalyst.util.HealthMonitor.getInstance().update();
        onPeriodic();
    }

    @Override
    public final void disabledPeriodic() {
        // The scheduler still ticks while disabled. Commands that run in this window are v3's
        // business, and stopping the scheduler here would also stop the telemetry and the periodic
        // callbacks that a pit crew reads between matches.
        scheduler().run();
        SystemCoreStatus.getInstance().publish();
        // Health matters more between matches than during them - a pit crew reads it to decide
        // whether the robot goes back out. See the note in HealthMonitor on why this is called from
        // here rather than from a scheduler periodic.
        frc.lib.catalyst.util.HealthMonitor.getInstance().update();
        onDisabledPeriodic();
    }

    @Override
    public final void end() {
        // Guard against an end without a start: WPILib calls end() on the selected mode even when it
        // never ran, and a subclass releasing something it never acquired is a confusing crash.
        if (!started) {
            return;
        }
        started = false;
        try {
            onEnd();
        } finally {
            // In the finally, so a subclass whose onEnd throws still releases the drivetrain. A
            // leftover auto command holding mechanisms is worse than the exception that caused it.
            releaseOwned();
            CatalystLog.log("OpMode/Running", false);
        }
    }

    /**
     * Release anything still owned, even for a mode that never started.
     *
     * <p>WPILib calls {@code close()} on every opmode instance it discards, and there is one path
     * where it calls {@code close()} without {@code end()}: a mode deselected while the robot is
     * disabled. {@code end()} is guarded behind having run, {@code close()} is not — so this is the
     * only hook that covers an instance which scheduled something from
     * {@link #onDisabledPeriodic()} and was then thrown away.
     */
    @Override
    public void close() {
        try {
            releaseOwned();
        } finally {
            OpMode.super.close();
        }
    }
}
