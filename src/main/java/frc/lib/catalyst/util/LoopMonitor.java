package frc.lib.catalyst.util;

import java.util.Locale;
import java.util.function.DoubleSupplier;

import org.wpilib.system.Timer;

import frc.lib.catalyst.logging.CatalystLog;

/**
 * Measures how long your robot loop actually takes and tells you when it runs over budget.
 *
 * <p>The robot program runs {@code robotPeriodic()} every 20&nbsp;ms (50&nbsp;Hz). When a loop takes
 * longer than that, the scheduler falls behind, controllers get stale, and driving feels laggy. This
 * is the "Loop time overrun" problem.
 *
 * <p><b>What changed on Systemcore, and what did not.</b> The 20&nbsp;ms budget is unchanged — it is
 * set by the control loop, not the hardware. What changed is everything around it, and mostly for
 * the better: Systemcore runs a fully preemptible kernel (Linux 6.12 {@code PREEMPT_RT}) on four
 * Cortex-A76 cores, and the robot program is started at real-time priority
 * ({@code LimitRTPRIO=50} in {@code robot.service}). Overruns should be rarer and, more usefully,
 * less random than they were on a roboRIO.
 *
 * <p>That cuts both ways when reading the numbers. A roboRIO overrun was usually "not enough CPU".
 * On Systemcore, with this much headroom, a sustained overrun more often means something is
 * blocking — a synchronous file write, a network call in a periodic method, a CAN read waiting on a
 * saturated bus. Check {@code CANRegistry.contentionWarnings()} and the Systemcore health checks
 * before assuming the robot simply ran out of compute; the machine will usually tell you it is idle
 * while your loop is late. {@code LoopMonitor} is the measurement tool: call {@link #record()}
 * once every loop and it tracks the last, rolling-average, and peak loop time, publishes them under
 * {@code Catalyst/Loop/<name>/...}, and raises a single warning through {@link AlertManager} when the
 * rolling average sits over the budget. It alerts on the average, not on one-off spikes, so a single
 * slow loop at startup does not cry wolf.
 *
 * <p>Typical use is one line in {@code robotPeriodic()}:
 *
 * <pre>{@code
 * private final LoopMonitor loop = new LoopMonitor();   // "Robot", 20 ms budget
 *
 * public void robotPeriodic() {
 *     Scheduler.getDefault().run();
 *     loop.record();   // measures the time since the last call
 * }
 * }</pre>
 *
 * <p>Then watch {@code Catalyst/Loop/Robot/AverageMs} in AdvantageScope. If it creeps toward 20, see
 * the loop-cost guidance in the logging docs, and read it next to {@code Systemcore/CpuPercent}
 * (see {@code SystemCoreStatus}) — a late loop on an idle machine is a blocking call, not a
 * shortage of compute, and the two want opposite fixes. The engine takes an injectable clock, so the statistics
 * are unit-tested with no HAL, no NetworkTables, and no robot.
 *
 * @since 1.3.3
 */
public class LoopMonitor {

    private final String name;
    private final double budgetSeconds;
    private final DoubleSupplier clock;
    private final MovingAverage average;
    private final String overBudgetMessage;

    private double lastTimestamp = Double.NaN;
    private double lastLoopSeconds = 0.0;
    private double maxLoopSeconds = 0.0;
    private long overCount = 0;
    private long totalCount = 0;

    /** Work-duration window, only fed when a team brackets its loop with begin/end. */
    private final MovingAverage workAverage;

    private double workBeganAt = Double.NaN;
    private double lastWorkSeconds = 0.0;
    private boolean measuringWork = false;

    private boolean loggingEnabled = true;
    private boolean alertsEnabled = true;
    private boolean alertActive = false;

    /** A monitor named {@code "Robot"} with the standard 20&nbsp;ms loop budget. */
    public LoopMonitor() {
        this("Robot", 0.020);
    }

    /**
     * A monitor with a custom name and budget, averaged over the last 50 loops.
     *
     * @param name          label used in telemetry keys and the alert (e.g. {@code "Robot"})
     * @param budgetSeconds the loop time you want to stay under, in seconds (20&nbsp;ms is {@code 0.020})
     */
    public LoopMonitor(String name, double budgetSeconds) {
        this(name, budgetSeconds, 50, Timer::getTimestamp);
    }

    /**
     * Full control, including the clock — the constructor tests use.
     *
     * @param name          label used in telemetry keys and the alert
     * @param budgetSeconds the loop time you want to stay under, in seconds
     * @param averageWindow how many recent loops the rolling average covers (must be positive)
     * @param clock         source of the current time in seconds; the robot uses
     *                      {@code Timer::getTimestamp}, a test passes its own
     */
    public LoopMonitor(String name, double budgetSeconds, int averageWindow, DoubleSupplier clock) {
        if (budgetSeconds <= 0) throw new IllegalArgumentException("budgetSeconds must be > 0");
        if (clock == null) throw new IllegalArgumentException("clock must not be null");
        this.name = name;
        this.budgetSeconds = budgetSeconds;
        this.clock = clock;
        this.average = new MovingAverage(averageWindow);
        this.workAverage = new MovingAverage(averageWindow);
        this.overBudgetMessage =
                String.format(Locale.ROOT, "Robot loop is averaging over its %.0f ms budget", budgetSeconds * 1000.0);
    }

    /**
     * How far over the period an interval has to run before it counts as an overrun.
     *
     * <p>This exists because {@link #record()} measures the loop <em>period</em> — the interval
     * between successive calls — and not the work done inside it. On a {@code TimedRobot} the
     * notifier is absolute-scheduled, so on a completely healthy robot that interval is the period
     * exactly, and comparing it directly against a budget equal to the period marks a perfectly
     * fine robot as over budget roughly half the time. The alert then latches and can never clear,
     * because clearing needs an interval <em>below</em> the period and the loop cannot run faster
     * than the notifier driving it.
     *
     * <p>A warning that is always on is worse than none: it gets filtered out, and takes the real
     * ones with it. So a period-based measurement only counts when it is meaningfully over — the
     * notifier is being missed, which is a genuine overrun.
     *
     * <p>None of this applies when a team brackets its loop with {@link #begin()} and {@link #end()}.
     * That measures real work, which is what the budget was always meant to be compared against, and
     * it is used directly.
     */
    private static final double OVERRUN_FACTOR = 1.2;

    /**
     * Mark the start of the work being measured. Optional; pairs with {@link #end()}.
     *
     * <p>Without this, {@link #record()} can only see how far apart the loops are, which says
     * nothing about how much of that was spent working. With it, the monitor knows the difference
     * between a loop that is computing too much and one that is blocked waiting — the distinction
     * that decides what to do about a late loop, and the one a period alone cannot make.
     *
     * <pre>{@code
     * public void robotPeriodic() {
     *     loop.begin();
     *     CommandScheduler.getInstance().run();
     *     loop.end();
     * }
     * }</pre>
     */
    public void begin() {
        workBeganAt = clock.getAsDouble();
    }

    /** Mark the end of the work begun by {@link #begin()}. */
    public void end() {
        if (Double.isNaN(workBeganAt)) {
            return;
        }
        double elapsed = clock.getAsDouble() - workBeganAt;
        workBeganAt = Double.NaN;
        if (elapsed >= 0) {
            lastWorkSeconds = elapsed;
            workAverage.calculate(elapsed);
            measuringWork = true;
            if (alertsEnabled) updateAlert();
        }
    }

    /**
     * Measure the time since the last call. Call this exactly once per loop, in {@code robotPeriodic()}.
     *
     * <p>This measures the loop <em>period</em>. See {@link #begin()} for measuring the work itself,
     * and {@link #OVERRUN_FACTOR} for why the two are judged differently.
     *
     * <p>The very first call only records the timestamp (there is nothing yet to measure against).
     * A non-positive interval, which happens if the clock is reset, is skipped rather than counted.
     */
    public void record() {
        double now = clock.getAsDouble();
        if (!Double.isNaN(lastTimestamp)) {
            double dt = now - lastTimestamp;
            if (dt > 0) {
                lastLoopSeconds = dt;
                average.calculate(dt);
                if (dt > maxLoopSeconds) maxLoopSeconds = dt;
                totalCount++;
                if (dt > overrunThreshold()) overCount++;
                if (loggingEnabled) publish();
                if (alertsEnabled) updateAlert();
            }
        }
        lastTimestamp = now;
    }

    private void publish() {
        // Relative, like every other CatalystLog caller: the sink supplies the "Catalyst" root table.
        CatalystLog.log("Loop/" + name + "/LastMs", lastLoopSeconds * 1000.0);
        CatalystLog.log("Loop/" + name + "/AverageMs", average.get() * 1000.0);
        CatalystLog.log("Loop/" + name + "/MaxMs", maxLoopSeconds * 1000.0);
        CatalystLog.log("Loop/" + name + "/OverBudget", isOverBudget());
        if (measuringWork) {
            // Only when it is real. A WorkMs of 0 on a dashboard reads as "the loop does nothing",
            // not as "nobody called begin()".
            CatalystLog.log("Loop/" + name + "/WorkMs", lastWorkSeconds * 1000.0);
            CatalystLog.log("Loop/" + name + "/AverageWorkMs", workAverage.get() * 1000.0);
        }
    }

    /**
     * Fraction of the budget the average must fall back to before the warning clears.
     *
     * <p>Without this the alert churns: a robot averaging exactly its budget — which is the common
     * case, because that is what the loop is tuned to — crosses the threshold in both directions
     * every few loops, and each crossing raises or clears the alert and prints to the Driver Station
     * console. One warning that stays up until the loop is genuinely healthy again is the useful
     * signal; a hundred identical lines is noise that buries everything else.
     */
    private static final double ALERT_CLEAR_FRACTION = 0.9;

    // Warn/clear only on the transition, with a fixed message, so the alert list never churns.
    private void updateAlert() {
        double current;
        if (measuringWork) {
            current = workAverage.isFull() ? workAverage.get() : lastWorkSeconds;
        } else {
            current = average.isFull() ? average.get() : lastLoopSeconds;
        }
        double threshold = overrunThreshold();
        if (!alertActive && current > threshold) {
            AlertManager.getInstance().warning("LoopMonitor:" + name, overBudgetMessage);
            alertActive = true;
        } else if (alertActive && current < threshold * ALERT_CLEAR_FRACTION) {
            AlertManager.getInstance().clearWarning("LoopMonitor:" + name, overBudgetMessage);
            alertActive = false;
        }
    }

    /**
     * Whether the loop is currently over budget. Uses the rolling average once the window has filled
     * (so a single spike does not trip it), and the most recent loop until then.
     */
    public boolean isOverBudget() {
        if (measuringWork) {
            // Real work against a real budget, which is the comparison the budget was written for.
            return workAverage.isFull() ? workAverage.get() > budgetSeconds
                                        : lastWorkSeconds > budgetSeconds;
        }
        return average.isFull() ? average.get() > overrunThreshold()
                                : lastLoopSeconds > overrunThreshold();
    }

    /** The value a measurement has to exceed to count, which depends on what is being measured. */
    private double overrunThreshold() {
        return measuringWork ? budgetSeconds : budgetSeconds * OVERRUN_FACTOR;
    }

    /** The most recent measured work time, in milliseconds, or 0 if begin/end are not being used. */
    public double getWorkMs() {
        return lastWorkSeconds * 1000.0;
    }

    /** The rolling-average work time, in milliseconds, or 0 if begin/end are not being used. */
    public double getAverageWorkMs() {
        return workAverage.get() * 1000.0;
    }

    /** The most recent loop time, in milliseconds. */
    public double getLastMs() {
        return lastLoopSeconds * 1000.0;
    }

    /** The rolling-average loop time over the window, in milliseconds. */
    public double getAverageMs() {
        return average.get() * 1000.0;
    }

    /** The longest loop time seen since construction or the last {@link #reset()}, in milliseconds. */
    public double getMaxMs() {
        return maxLoopSeconds * 1000.0;
    }

    /** The fraction of measured loops that ran over budget, from 0.0 to 1.0. */
    public double getOverBudgetFraction() {
        return totalCount == 0 ? 0.0 : (double) overCount / totalCount;
    }

    /** How many loop intervals have been measured (the first {@code record()} does not count). */
    public long getSampleCount() {
        return totalCount;
    }

    /** Turn telemetry publishing on or off (on by default). Returns {@code this} for chaining. */
    public LoopMonitor withLogging(boolean enabled) {
        this.loggingEnabled = enabled;
        return this;
    }

    /** Turn the over-budget alert on or off (on by default). Returns {@code this} for chaining. */
    public LoopMonitor withAlerts(boolean enabled) {
        this.alertsEnabled = enabled;
        return this;
    }

    /** Clear the peak, the rolling average, the counters, and any active alert. */
    public void reset() {
        average.reset();
        lastTimestamp = Double.NaN;
        lastLoopSeconds = 0.0;
        maxLoopSeconds = 0.0;
        overCount = 0;
        totalCount = 0;
        if (alertActive) {
            AlertManager.getInstance().clearWarning("LoopMonitor:" + name, overBudgetMessage);
            alertActive = false;
        }
    }
}
