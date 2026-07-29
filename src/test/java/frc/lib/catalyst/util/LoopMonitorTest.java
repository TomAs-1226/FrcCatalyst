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
}
