package frc.lib.catalyst.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * A whole match of cycling, driven by a made-up clock. Every one of these is a thing the old
 * two-line selector in {@code Autopilot} got wrong and could not be tested for.
 */
class CycleCoreTest {

    private static final int ACQUIRE = 0;
    private static final int SCORE = 1;

    @Test
    void aChatteringSensorDoesNotThrashTheCycle() {
        // A game-piece sensor sitting on its threshold. Without dwell this alternated phases at
        // loop rate, so neither action ever got far enough to do anything.
        CycleCore cycle = new CycleCore(2, 0.25, 0);
        double t = 0;

        assertEquals(ACQUIRE, cycle.decide(t, ACQUIRE, true).phase());
        for (int i = 0; i < 10; i++) {
            t += 0.02;
            cycle.decide(t, SCORE, true);     // "holding!"
            t += 0.02;
            cycle.decide(t, ACQUIRE, true);   // "no I'm not"
        }
        assertEquals(ACQUIRE, cycle.phase(), "40 flips in 0.4 s and it never left the phase it was on");
    }

    @Test
    void aRealChangeStillHappensOnceItHasSettled() {
        CycleCore cycle = new CycleCore(2, 0.25, 0);
        double t = 0;
        cycle.decide(t, ACQUIRE, true);

        for (t = 0.02; t < 0.24; t += 0.02) {
            assertEquals(ACQUIRE, cycle.decide(t, SCORE, true).phase(), "still inside the dwell at t=" + t);
        }
        assertEquals(SCORE, cycle.decide(0.30, SCORE, true).phase(), "held for a quarter second, so it switches");
    }

    @Test
    void withNoDwellItSwitchesImmediately() {
        // The behaviour Autopilot has today, preserved for anyone who wants it.
        CycleCore cycle = new CycleCore(2, 0, 0);
        cycle.decide(0.0, ACQUIRE, true);
        assertEquals(SCORE, cycle.decide(0.02, SCORE, true).phase());
    }

    @Test
    void aPhaseThatCannotStartHoldsRatherThanRunning() {
        CycleCore cycle = new CycleCore(2, 0, 0);
        CycleCore.Decision d = cycle.decide(0.0, ACQUIRE, false);

        assertEquals(CycleCore.Act.HOLD, d.act());
        assertEquals(ACQUIRE, d.phase());
        assertEquals("cannot start", d.reason());
    }

    @Test
    void aStallLongEnoughHandsTheRobotBack() {
        // The lockout. An acquire that can never succeed used to hold the drivetrain until the
        // driver noticed and released the button.
        CycleCore cycle = new CycleCore(2, 0, 3.0);

        assertEquals(CycleCore.Act.HOLD, cycle.decide(0.0, ACQUIRE, false).act());
        assertEquals(CycleCore.Act.HOLD, cycle.decide(2.9, ACQUIRE, false).act());

        CycleCore.Decision d = cycle.decide(3.1, ACQUIRE, false);
        assertEquals(CycleCore.Act.HAND_BACK, d.act());
        assertTrue(d.reason().contains("handing back"), d.reason());
        assertEquals(3.1, cycle.stalledSeconds(3.1), 1e-9);
    }

    @Test
    void handbackIsOptOutAndOffByDefaultForExistingRobots() {
        CycleCore never = new CycleCore(2, 0, 0);
        for (double t = 0; t < 60; t += 0.5) {
            assertEquals(CycleCore.Act.HOLD, never.decide(t, ACQUIRE, false).act(),
                    "with handback disabled it holds forever, as it always did");
        }
    }

    @Test
    void recoveringFromAStallClearsIt() {
        CycleCore cycle = new CycleCore(2, 0, 3.0);
        cycle.decide(0.0, ACQUIRE, false);
        cycle.decide(2.0, ACQUIRE, false);
        assertEquals(2.0, cycle.stalledSeconds(2.0), 1e-9);

        assertEquals(CycleCore.Act.RUN, cycle.decide(2.5, ACQUIRE, true).act());
        assertEquals(0.0, cycle.stalledSeconds(2.5), 1e-9, "the stall clock resets, so it does not hand back later");

        cycle.decide(3.0, ACQUIRE, false);
        assertEquals(CycleCore.Act.HOLD, cycle.decide(5.0, ACQUIRE, false).act(),
                "and the new stall is timed from when it actually started");
    }

    @Test
    void aCycleCanHaveMoreThanTwoPhases() {
        // Autopilot is the two-phase case; nothing here is limited to it.
        CycleCore cycle = new CycleCore(4, 0, 0);
        assertEquals(3, cycle.decide(0.0, 3, true).phase());
        assertEquals(0, cycle.decide(0.02, 4, true).phase(), "indices wrap rather than throwing");
        assertEquals(2, cycle.decide(0.04, -2, true).phase(), "including negative ones");
    }

    @Test
    void aCycleNeedsAtLeastOnePhase() {
        try {
            new CycleCore(0, 0, 0);
            throw new AssertionError("expected a refusal at construction");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("at least one phase"));
        }
    }

    @Test
    void resetPutsItBackAsIfNewlyEngaged() {
        CycleCore cycle = new CycleCore(3, 0, 5.0);
        cycle.decide(0.0, 2, false);
        cycle.decide(1.0, 2, false);
        assertEquals(2, cycle.phase());

        cycle.reset();
        assertEquals(0, cycle.phase());
        assertEquals(0.0, cycle.stalledSeconds(1.0), 1e-9);
    }
}
