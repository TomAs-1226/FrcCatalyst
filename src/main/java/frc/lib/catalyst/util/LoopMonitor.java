package frc.lib.catalyst.util;

import java.util.Locale;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;

import frc.lib.catalyst.logging.CatalystLog;

/**
 * Measures how long your robot loop actually takes and tells you when it runs over budget.
 *
 * <p>The RIO runs {@code robotPeriodic()} every 20&nbsp;ms (50&nbsp;Hz). When a loop takes longer
 * than that, the scheduler falls behind, controllers get stale, and driving feels laggy. This is the
 * "Loop time overrun" problem. {@code LoopMonitor} is the measurement tool: call {@link #record()}
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
 *     CommandScheduler.getInstance().run();
 *     loop.record();   // measures the time since the last call
 * }
 * }</pre>
 *
 * <p>Then watch {@code Catalyst/Loop/Robot/AverageMs} in AdvantageScope. If it creeps toward 20, see
 * the loop-cost guidance in the logging docs. The engine takes an injectable clock, so the statistics
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
        this(name, budgetSeconds, 50, Timer::getFPGATimestamp);
    }

    /**
     * Full control, including the clock — the constructor tests use.
     *
     * @param name          label used in telemetry keys and the alert
     * @param budgetSeconds the loop time you want to stay under, in seconds
     * @param averageWindow how many recent loops the rolling average covers (must be positive)
     * @param clock         source of the current time in seconds; the robot uses
     *                      {@code Timer::getFPGATimestamp}, a test passes its own
     */
    public LoopMonitor(String name, double budgetSeconds, int averageWindow, DoubleSupplier clock) {
        if (budgetSeconds <= 0) throw new IllegalArgumentException("budgetSeconds must be > 0");
        if (clock == null) throw new IllegalArgumentException("clock must not be null");
        this.name = name;
        this.budgetSeconds = budgetSeconds;
        this.clock = clock;
        this.average = new MovingAverage(averageWindow);
        this.overBudgetMessage =
                String.format(Locale.ROOT, "Robot loop is averaging over its %.0f ms budget", budgetSeconds * 1000.0);
    }

    /**
     * Measure the time since the last call. Call this exactly once per loop, in {@code robotPeriodic()}.
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
                if (dt > budgetSeconds) overCount++;
                if (loggingEnabled) publish();
                if (alertsEnabled) updateAlert();
            }
        }
        lastTimestamp = now;
    }

    private void publish() {
        CatalystLog.log("Catalyst/Loop/" + name + "/LastMs", lastLoopSeconds * 1000.0);
        CatalystLog.log("Catalyst/Loop/" + name + "/AverageMs", average.get() * 1000.0);
        CatalystLog.log("Catalyst/Loop/" + name + "/MaxMs", maxLoopSeconds * 1000.0);
        CatalystLog.log("Catalyst/Loop/" + name + "/OverBudget", isOverBudget());
    }

    // Warn/clear only on the transition, with a fixed message, so the alert list never churns.
    private void updateAlert() {
        boolean over = isOverBudget();
        if (over && !alertActive) {
            AlertManager.getInstance().warning("LoopMonitor:" + name, overBudgetMessage);
            alertActive = true;
        } else if (!over && alertActive) {
            AlertManager.getInstance().clearWarning("LoopMonitor:" + name, overBudgetMessage);
            alertActive = false;
        }
    }

    /**
     * Whether the loop is currently over budget. Uses the rolling average once the window has filled
     * (so a single spike does not trip it), and the most recent loop until then.
     */
    public boolean isOverBudget() {
        return average.isFull() ? average.get() > budgetSeconds : lastLoopSeconds > budgetSeconds;
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
