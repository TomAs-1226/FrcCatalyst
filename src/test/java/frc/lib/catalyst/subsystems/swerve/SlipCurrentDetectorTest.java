package frc.lib.catalyst.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** The slip-current measurement against a model of a robot held by a wall, and of one that is not. */
class SlipCurrentDetectorTest {

    private static final double DT = 0.02;

    /**
     * Four drive motors: held, each draws volts / ohms and its wheel does not turn, until one reaches its slip
     * current - then that wheel spins up and its current falls, as a slipping wheel's does. {@code rolling} is a
     * robot not against anything: every wheel turns from the first volt, on a few amps.
     */
    private static SlipCurrentDetector run(double ohms, double[] slipAt, boolean rolling, int loops) {
        SlipCurrentDetector m = new SlipCurrentDetector();
        double volts = 0.0;
        boolean[] slipped = new boolean[4];
        for (int k = 0; k < loops && !m.done(); k++) {
            double[] speed = new double[4];
            double[] amps = new double[4];
            for (int i = 0; i < 4; i++) {
                if (rolling) {
                    amps[i] = 3.0;
                    speed[i] = volts * 0.4;
                } else {
                    amps[i] = volts / ohms;
                    if (amps[i] >= slipAt[i]) {
                        slipped[i] = true;
                    }
                    if (slipped[i]) {
                        amps[i] = 0.6 * slipAt[i];
                        speed[i] = 1.5;
                    }
                }
            }
            volts = m.step(k * DT, speed, amps);
        }
        return m;
    }

    @Test
    void heldByTheWallItReportsTheCurrentTheFirstWheelBrokeLooseAt() {
        SlipCurrentDetector m = run(0.05, new double[] {55, 52, 48.3, 60}, false, 2000);
        assertEquals(SlipCurrentDetector.Phase.SLIPPED, m.phase());
        assertEquals(2, m.slipWheel());
        // The ramp adds 0.2 A a loop here, so the reading is within one loop of the true 48.3 A.
        assertEquals(48.3, m.slipAmps(), 0.25);
        assertEquals(45.0, m.recommendedAmps(), 0.0);
        assertEquals(0.0, m.volts(), 0.0);
        assertEquals(0.0, m.step(99, new double[] {0, 0, 0, 0}, new double[] {0, 0, 0, 0}), 0.0, "stays at 0 V once done");
    }

    @Test
    void aRobotRollingFreeIsStoppedNotMeasured() {
        SlipCurrentDetector m = run(0.05, null, true, 2000);
        assertEquals(SlipCurrentDetector.Phase.STOPPED, m.phase());
        assertTrue(m.reason().contains("rolling"), m.reason());
        assertTrue(Double.isNaN(m.slipAmps()));
        assertTrue(Double.isNaN(m.recommendedAmps()));
    }

    @Test
    void itNeverRampsPastItsCapAndSaysWhyWhenNothingSlips() {
        SlipCurrentDetector m = run(0.05, new double[] {1e9, 1e9, 1e9, 1e9}, false, 5000);
        assertEquals(SlipCurrentDetector.Phase.STOPPED, m.phase());
        assertTrue(m.reason().contains("no slip"), m.reason());
        assertTrue(m.peakAmps() <= SlipCurrentDetector.MAX_VOLTS / 0.05 + 1e-9, "peak " + m.peakAmps());
    }

    @Test
    void aMotorThatReportsNothingStopsIt() {
        SlipCurrentDetector m = new SlipCurrentDetector();
        m.step(0, new double[] {0, 0, 0, 0}, new double[] {1, 1, 1, 1});
        double v = m.step(0.02, new double[] {0, Double.NaN, 0, 0}, new double[] {1, 1, 1, 1});
        assertEquals(0.0, v, 0.0);
        assertEquals(SlipCurrentDetector.Phase.STOPPED, m.phase());
    }

    @Test
    void theRampIsSlowAndStartsFromNothing() {
        SlipCurrentDetector m = new SlipCurrentDetector();
        assertEquals(0.0, m.step(10.0, new double[] {0, 0, 0, 0}, new double[] {0, 0, 0, 0}), 0.0);
        assertEquals(SlipCurrentDetector.RAMP_V_PER_S * 1.0,
                m.step(11.0, new double[] {0, 0, 0, 0}, new double[] {0, 0, 0, 0}), 1e-12);
    }
}
