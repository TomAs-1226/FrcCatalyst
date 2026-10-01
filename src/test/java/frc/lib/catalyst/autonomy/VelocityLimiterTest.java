package frc.lib.catalyst.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class VelocityLimiterTest {

    @Test
    void aSlammedStickRampsAtTheLimit() {
        VelocityLimiter l = new VelocityLimiter();
        double[] v = l.limit(0.02, 4.0, 0.0, 10.0, 10.0);
        assertEquals(0.2, v[0], 1e-12, "10 m/s^2 for 20 ms");
        for (int i = 0; i < 30; i++) {
            v = l.limit(0.02, 4.0, 0.0, 10.0, 10.0);
        }
        assertEquals(4.0, v[0], 1e-12, "and it arrives, without overshoot");
    }

    @Test
    void aDiagonalAcceleratesAtTheLimitNotAtRootTwoTimesIt() {
        VelocityLimiter l = new VelocityLimiter();
        double[] v = l.limit(0.02, 3.0, 3.0, 10.0, 10.0);
        assertEquals(0.2, Math.hypot(v[0], v[1]), 1e-12, "the size of the change, not each axis");
        assertEquals(v[0], v[1], 1e-12, "and straight along the diagonal");
    }

    @Test
    void speedingUpBackwardsIsAccelerationNotBraking() {
        VelocityLimiter l = new VelocityLimiter();
        // A sign-based limiter calls a falling value deceleration and would use the 30 here.
        double[] v = l.limit(0.02, -4.0, 0.0, 5.0, 30.0);
        assertEquals(-0.1, v[0], 1e-12);
    }

    @Test
    void lettingGoBrakesAtTheBrakingLimit() {
        VelocityLimiter l = new VelocityLimiter();
        l.reset(4.0, 0.0);
        double[] v = l.limit(0.02, 0.0, 0.0, 5.0, 12.0);
        assertEquals(4.0 - 0.24, v[0], 1e-12);
    }

    @Test
    void aReversalNeverLeavesTheLineBetweenTheTwoVelocities() {
        VelocityLimiter l = new VelocityLimiter();
        l.reset(3.0, 1.0);
        for (int i = 0; i < 100; i++) {
            double[] v = l.limit(0.02, -3.0, -1.0, 10.0, 10.0);
            assertEquals(0.0, v[0] * 1.0 - v[1] * 3.0, 1e-9, "always on the line through both");
        }
        assertEquals(-3.0, l.velocity()[0], 1e-9);
    }

    @Test
    void noLimitAndBadNumbersAreSafe() {
        VelocityLimiter l = new VelocityLimiter();
        assertEquals(4.0, l.limit(0.02, 4.0, 0.0, Double.POSITIVE_INFINITY, 0.0)[0], 1e-12, "no limit: straight through");
        assertEquals(4.0, l.limit(0.02, 4.0, 0.0, Double.NaN, Double.NaN)[0], 1e-12);
        double[] v = l.limit(0.02, Double.NaN, 1.0, 10.0, 10.0);
        assertEquals(3.8, v[0], 1e-9, "a command that is not a number is a command to stop, at the limit");
        l.reset(Double.NaN, 2.0);
        assertEquals(0.0, l.velocity()[0], 1e-12);
    }
}
