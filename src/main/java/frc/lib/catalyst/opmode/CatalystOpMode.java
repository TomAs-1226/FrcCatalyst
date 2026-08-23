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

    /** Schedule a command on this OpMode's scheduler. The usual way to start a routine. */
    protected final void run(org.wpilib.command3.Command command) {
        scheduler().schedule(command);
    }

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
        // The scheduler first, so a subclass reading mechanism state in onPeriodic sees the state
        // this loop produced rather than the previous loop's.
        scheduler().run();
        SystemCoreStatus.getInstance().publish();
        onPeriodic();
    }

    @Override
    public final void disabledPeriodic() {
        // The scheduler still ticks while disabled. Commands that run in this window are v3's
        // business, and stopping the scheduler here would also stop the telemetry and the periodic
        // callbacks that a pit crew reads between matches.
        scheduler().run();
        SystemCoreStatus.getInstance().publish();
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
            CatalystLog.log("OpMode/Running", false);
        }
    }
}
