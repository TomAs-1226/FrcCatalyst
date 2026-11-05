package frc.lib.catalyst.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.DoubleSupplier;

import org.junit.jupiter.api.Test;

/**
 * Pure-Java tests for {@link LoopMonitor}. Logging and alerts are turned off so the statistics run
 * with no HAL, no NetworkTables, and no robot; the clock is a mutable field the test advances by hand.
 */
class LoopMonitorTest {

    /** A monitor driven by {@code t[0]}, with telemetry and alerts disabled. */
    private static LoopMonitor monitor(double[] t, double budget, int window) {
        DoubleSupplier clock = () -> t[0];
        return new LoopMonitor("Test", budget, window, clock).withLogging(false).withAlerts(false);
    }

    @Test
    void firstCallIsBaselineAndTimingIsMeasured() {
        double[] t = {0.0};
        LoopMonitor m = monitor(t, 0.020, 5);

        m.record();                       // first call: nothing to measure yet
        assertEquals(0, m.getSampleCount());
        assertEquals(0.0, m.getLastMs(), 1e-9);

        for (int i = 0; i < 4; i++) {
            t[0] += 0.015;                 // steady 15 ms loops
            m.record();
        }
        assertEquals(4, m.getSampleCount());
        assertEquals(15.0, m.getLastMs(), 1e-3);
        assertEquals(15.0, m.getAverageMs(), 1e-3);
        assertEquals(15.0, m.getMaxMs(), 1e-3);
        assertFalse(m.isOverBudget());    // 15 ms average is under the 20 ms budget
    }

    @Test
    void maxTracksThePeakEvenAfterItPasses() {
        double[] t = {0.0};
        LoopMonitor m = monitor(t, 0.020, 5);

        m.record();
        t[0] += 0.010; m.record();        // 10 ms
        t[0] += 0.050; m.record();        // one 50 ms spike
        t[0] += 0.010; m.record();        // back to 10 ms

        assertEquals(10.0, m.getLastMs(), 1e-3);
        assertEquals(50.0, m.getMaxMs(), 1e-3);   // peak is remembered
    }

    @Test
    void overBudgetFollowsTheRollingAverageOnceTheWindowFills() {
        double[] t = {0.0};
        LoopMonitor m = monitor(t, 0.020, 5);

        m.record();
        for (int i = 0; i < 5; i++) {     // fill the window with 25 ms loops
            t[0] += 0.025;
            m.record();
        }
        assertTrue(m.isOverBudget());
        assertEquals(25.0, m.getAverageMs(), 1e-3);

        for (int i = 0; i < 5; i++) {     // flush it with 10 ms loops
            t[0] += 0.010;
            m.record();
        }
        assertFalse(m.isOverBudget());
        assertEquals(10.0, m.getAverageMs(), 1e-3);
    }

    @Test
    void overBudgetFractionCountsSlowLoops() {
        double[] t = {0.0};
        LoopMonitor m = monitor(t, 0.020, 10);

        m.record();
        t[0] += 0.025; m.record();        // over
        t[0] += 0.025; m.record();        // over
        t[0] += 0.025; m.record();        // over
        t[0] += 0.010; m.record();        // under

        assertEquals(4, m.getSampleCount());
        assertEquals(0.75, m.getOverBudgetFraction(), 1e-9);
    }

    @Test
    void nonPositiveIntervalsFromAClockResetAreSkipped() {
        double[] t = {0.0};
        LoopMonitor m = monitor(t, 0.020, 5);

        m.record();
        t[0] += 0.015; m.record();
        assertEquals(1, m.getSampleCount());

        t[0] = 0.0; m.record();           // clock jumped backwards: skip, do not count
        assertEquals(1, m.getSampleCount());

        t[0] += 0.015; m.record();        // normal loop resumes from the new baseline
        assertEquals(2, m.getSampleCount());
    }

    @Test
    void resetClearsEverythingAndReBaselines() {
        double[] t = {0.0};
        LoopMonitor m = monitor(t, 0.020, 5);

        m.record();
        t[0] += 0.030; m.record();
        t[0] += 0.030; m.record();
        assertTrue(m.getSampleCount() > 0);
        assertTrue(m.getMaxMs() > 0);

        m.reset();
        assertEquals(0, m.getSampleCount());
        assertEquals(0.0, m.getMaxMs(), 1e-9);
        assertEquals(0.0, m.getAverageMs(), 1e-9);
        assertEquals(0.0, m.getOverBudgetFraction(), 1e-9);

        t[0] += 0.015; m.record();        // first call after reset is a baseline again
        assertEquals(0, m.getSampleCount());
        t[0] += 0.015; m.record();
        assertEquals(1, m.getSampleCount());
    }

    @Test
    void badConstructorArgumentsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> new LoopMonitor("T", 0.0, 5, () -> 0.0));
        assertThrows(IllegalArgumentException.class, () -> new LoopMonitor("T", -1.0, 5, () -> 0.0));
        assertThrows(IllegalArgumentException.class, () -> new LoopMonitor("T", 0.020, 5, null));
    }

    /**
     * A robot hitting its loop period exactly must not be reported as over budget.
     *
     * <p>{@code record()} measures the interval between successive calls — the loop *period*, not
     * the work done inside it. On a TimedRobot the notifier is absolute-scheduled, so that interval
     * is the period, and its long-run value on a completely healthy robot is exactly the period.
     *
     * <p>With the default 20 ms budget that puts the measurement at the threshold. Half the jitter
     * lands above it, the alert latches, and it can only clear below 18 ms — which the loop cannot
     * reach, because it cannot run faster than the notifier that drives it. The result is a
     * permanent "averaging over its 20 ms budget" on a robot with most of its budget spare, which
     * is worse than no warning: a warning that is always on gets filtered out, and takes the real
     * ones with it.
     */
    @Test
    void aHealthyRobotIsNotReportedAsOverBudget() {
        double[] t = { 0.0 };
        LoopMonitor loop = monitor(t, 0.020, 50);

        // 200 loops at exactly the period, with the sub-microsecond jitter any real clock has.
        for (int i = 0; i < 200; i++) {
            t[0] += 0.020 + (i % 2 == 0 ? 1e-6 : -1e-6);
            loop.record();
        }

        assertFalse(loop.isOverBudget(),
                "a loop running exactly on its period has not overrun anything; average was "
                        + loop.getAverageMs() + " ms");
    }

    /** A loop that genuinely takes too long still has to be reported. */
    @Test
    void aLoopThatOverrunsItsPeriodIsStillCaught() {
        double[] t = { 0.0 };
        LoopMonitor loop = monitor(t, 0.020, 50);

        // 30 ms per loop: the notifier is being missed every time, which is a real overrun.
        for (int i = 0; i < 200; i++) {
            t[0] += 0.030;
            loop.record();
        }

        assertTrue(loop.isOverBudget(), "30 ms loops against a 20 ms budget must report over budget");
    }

    /**
     * With begin/end the budget means what it says.
     *
     * <p>A loop that sits on a 20 ms period but only works for 5 ms of it is healthy; one that works
     * for 25 ms is not, even though both are handed the same period. The period alone cannot tell
     * those apart, which is the whole reason for measuring the work.
     */
    @Test
    void workMeasurementJudgesAgainstTheBudgetDirectly() {
        double[] t = { 0.0 };
        LoopMonitor light = monitor(t, 0.020, 50);
        for (int i = 0; i < 100; i++) {
            light.begin();
            t[0] += 0.005;          // 5 ms of work
            light.end();
            t[0] += 0.015;          // idle until the next notifier
            light.record();
        }
        assertFalse(light.isOverBudget(), "5 ms of work against a 20 ms budget is not over");
        assertEquals(5.0, light.getAverageWorkMs(), 0.001);

        double[] u = { 0.0 };
        LoopMonitor heavy = monitor(u, 0.020, 50);
        for (int i = 0; i < 100; i++) {
            heavy.begin();
            u[0] += 0.025;          // 25 ms of work, overrunning the period
            heavy.end();
            heavy.record();
        }
        assertTrue(heavy.isOverBudget(), "25 ms of work against a 20 ms budget is over");
    }
}
