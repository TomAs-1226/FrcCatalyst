package frc.lib.catalyst.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.catalyst.autonomy.SharedControl.Decision;
import frc.lib.catalyst.autonomy.SharedControl.Driver;
import frc.lib.catalyst.autonomy.SharedControl.HeadingProposal;
import frc.lib.catalyst.autonomy.SharedControl.TranslationProposal;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** The rules of shared control, one test each. */
class SharedControlTest {

    private static final HeadingProposal HOLD = HeadingProposal.hold("HOLD", 5.0, 3.0, "holding the heading");
    private static final Driver FORWARD = new Driver(2.0, 0.0, 0.0, false);

    private static SharedControl control() {
        return new SharedControl(new SharedControl.Config());
    }

    /** Runs loops at 50 Hz from {@code t0}, returning the last decision. */
    private static Decision run(SharedControl c, double t0, int loops, Driver d, double heading, double yawRate,
                                List<HeadingProposal> h) {
        Decision last = null;
        for (int i = 0; i < loops; i++) {
            last = c.decide(t0 + i * 0.02, d, heading, yawRate, h, Optional.empty());
        }
        return last;
    }

    @Test
    void rule1TheDriversStickWinsTheSameLoop() {
        SharedControl c = control();
        run(c, 0.0, 50, FORWARD, 0.0, 0.0, List.of(HOLD));
        Decision d = c.decide(1.0, new Driver(2.0, 0.0, 1.7, true), 0.5, 0.0, List.of(HOLD), Optional.empty());
        assertEquals(SharedControl.DRIVER, d.headingOwner());
        assertEquals(1.7, d.omega(), 1e-12, "exactly what the driver asked, the loop they asked it");
        // Even an assist that does not wait for rest gives way to the stick.
        Decision e = c.decide(1.02, new Driver(2.0, 0.0, -0.4, true), 0.5, 0.0,
                List.of(HeadingProposal.external("AIM", "aiming"), HOLD), Optional.empty());
        assertEquals(SharedControl.DRIVER, e.headingOwner());
        assertEquals(-0.4, e.omega(), 1e-12);
    }

    @Test
    void rule1TheHoldWaitsForTheStickToRestAndTheRobotToStopTurning() {
        SharedControl c = control();
        c.decide(0.0, new Driver(2.0, 0.0, 2.0, true), 0.0, 2.0, List.of(HOLD), Optional.empty());
        // Stick released, the robot still coasting round at 1 rad/s: nothing takes the heading.
        Decision coasting = run(c, 0.02, 40, FORWARD, 0.3, 1.0, List.of(HOLD));
        assertEquals(SharedControl.NONE, coasting.headingOwner(), "0.8 s on, still turning: no hold");
        assertEquals(0.0, coasting.omega(), 1e-12);
        // Settled: the hold takes the heading the robot has NOW (0.9), not the one at stick release.
        Decision settled = c.decide(0.84, FORWARD, 0.9, 0.05, List.of(HOLD), Optional.empty());
        assertEquals("HOLD", settled.headingOwner());
        assertEquals(0.9, settled.headingTargetRad(), 1e-12);
        assertEquals(0.0, settled.omega(), 1e-12, "on its target: it never swings the robot back");

        // A stick released while the robot is already still waits only the rest time.
        SharedControl quick = control();
        quick.decide(0.0, new Driver(2.0, 0.0, 2.0, true), 0.0, 0.0, List.of(HOLD), Optional.empty());
        assertEquals(SharedControl.NONE, run(quick, 0.02, 10, FORWARD, 0.0, 0.0, List.of(HOLD)).headingOwner());
        assertEquals("HOLD", run(quick, 0.22, 10, FORWARD, 0.0, 0.0, List.of(HOLD)).headingOwner());
    }

    @Test
    void rule2AnAssistNeverSpeedsTheRobotUpOrTurnsItBack() {
        SharedControl c = control();
        // Offered 3 m/s along a bent direction when the driver asked 2: the direction, at 2 m/s.
        Decision bent = c.decide(0.0, FORWARD, 0.0, 0.0, List.of(),
                Optional.of(new TranslationProposal("TRENCH", 2.4, -1.8, "centring")));
        assertEquals(2.0, Math.hypot(bent.vx(), bent.vy()), 1e-9);
        assertEquals("TRENCH", bent.translationOwner());
        assertTrue(bent.vy() < 0);
        // Against the driver's direction: dropped.
        Decision back = c.decide(0.02, FORWARD, 0.0, 0.0, List.of(),
                Optional.of(new TranslationProposal("TRENCH", -1.0, 0.5, "wrong")));
        assertEquals(2.0, back.vx(), 1e-12);
        assertEquals(0.0, back.vy(), 1e-12);
        assertEquals(SharedControl.DRIVER, back.translationOwner());
        // A driver at rest is never moved.
        Decision still = c.decide(0.04, new Driver(0, 0, 0, false), 0.0, 0.0, List.of(),
                Optional.of(new TranslationProposal("TRENCH", 1.0, 0.0, "push")));
        assertEquals(0.0, Math.hypot(still.vx(), still.vy()), 1e-12);
        // And a proposal that is not a number is ignored.
        Decision nan = c.decide(0.06, FORWARD, 0.0, 0.0, List.of(),
                Optional.of(new TranslationProposal("TRENCH", Double.NaN, 0.0, "broken")));
        assertEquals(2.0, nan.vx(), 1e-12);
    }

    @Test
    void rule3AnAssistRampsInFromNothing() {
        SharedControl c = control();
        HeadingProposal square = HeadingProposal.to("BUMP", 1.0, 5.0, 3.0, "squaring to the bump");
        run(c, 0.0, 20, FORWARD, 0.0, 0.0, List.of());
        Decision first = c.decide(0.40, FORWARD, 0.0, 0.0, List.of(square), Optional.empty());
        assertEquals(0.0, first.omega(), 0.3, "the loop it takes over it commands almost nothing");
        double previous = Math.abs(first.omega());
        for (int i = 1; i <= 13; i++) {
            Decision d = c.decide(0.40 + i * 0.02, FORWARD, 0.0, 0.0, List.of(square), Optional.empty());
            assertTrue(Math.abs(d.omega()) >= previous - 1e-9, "authority only rises while it holds");
            previous = Math.abs(d.omega());
        }
        assertEquals(3.0, previous, 1e-9, "full authority after the ramp: 5 x 1 rad, capped at 3");
    }

    @Test
    void rule4TheFirstProposalThatMayActOwnsTheHeading() {
        SharedControl c = control();
        HeadingProposal bump = HeadingProposal.to("BUMP", 1.57, 5.0, 3.0, "squaring");
        Decision d = run(c, 0.0, 30, FORWARD, 0.0, 0.0, List.of(bump, HOLD));
        assertEquals("BUMP", d.headingOwner());
        // The bump gone, the hold takes over - at the heading the robot has then.
        Decision h = c.decide(0.60, FORWARD, 1.5, 0.0, List.of(HOLD), Optional.empty());
        assertEquals("HOLD", h.headingOwner());
        assertEquals(1.5, h.headingTargetRad(), 1e-12);
        // An external owner is named, and commands no turn of its own.
        Decision aim = c.decide(0.62, FORWARD, 1.5, 0.0,
                List.of(HeadingProposal.external("AIM", "aiming"), HOLD), Optional.empty());
        assertEquals("AIM", aim.headingOwner());
        assertTrue(aim.external());
        assertEquals(0.0, aim.omega(), 1e-12);
        // Nothing offered: nobody owns it, and nothing turns the robot.
        Decision none = c.decide(0.64, FORWARD, 1.5, 0.0, List.of(), Optional.empty());
        assertEquals(SharedControl.NONE, none.headingOwner());
        assertEquals(0.0, none.omega(), 1e-12);
    }

    @Test
    void rule5AnAssistsTurnIsBounded() {
        SharedControl c = control();
        HeadingProposal far = HeadingProposal.to("BUMP", Math.PI, 50.0, 1.2, "a long way round");
        Decision d = run(c, 0.0, 60, FORWARD, 0.0, 0.0, List.of(far));
        assertEquals(1.2, Math.abs(d.omega()), 1e-9, "however large the error and the gain");
    }

    @Test
    void theHoldPullsBackOntoItsHeadingTheShortWay() {
        SharedControl c = control();
        run(c, 0.0, 40, FORWARD, 3.1, 0.0, List.of(HOLD));
        // Knocked across the +-pi seam to -3.1: 0.083 rad the short way, not 6.2 the long way.
        Decision d = c.decide(0.80, FORWARD, -3.1, 0.0, List.of(HOLD), Optional.empty());
        assertEquals(3.1, d.headingTargetRad(), 1e-12);
        assertEquals(-5.0 * (2 * Math.PI - 6.2), d.omega(), 1e-6);
        assertFalse(d.external());
    }

    @Test
    void resetForgetsTheHeldHeading() {
        SharedControl c = control();
        run(c, 0.0, 40, FORWARD, 0.2, 0.0, List.of(HOLD));
        c.reset();
        // The pose's heading was reset to 2.0: the hold must not turn the robot back to 0.2.
        Decision d = run(c, 1.0, 40, FORWARD, 2.0, 0.0, List.of(HOLD));
        assertEquals(2.0, d.headingTargetRad(), 1e-12);
        assertEquals(0.0, d.omega(), 1e-12);
    }
}
