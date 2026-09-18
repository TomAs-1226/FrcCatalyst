package frc.lib.catalyst.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The aiming speed governor, checked against the swing it is meant to protect. Ported from the Catalyst
 * X1's drive shaper, whose governor this is; the X1's harness, which measured what it buys on a model of
 * that robot, stays with the X1.
 */
class AimSpeedGovernorTest {

    /** The X1: Falcons at 5.24 m/s, wheels 0.391 m from the centre. */
    private static final double TOP = 5.24;
    private static final double RADIUS = 0.391;

    private static AimSpeedGovernor governor() {
        return new AimSpeedGovernor(TOP, RADIUS);
    }

    @Test
    void itCapsOnlyTheSpeedThatSwingsTheAim() {
        AimSpeedGovernor g = governor();
        g.config().capSlewMps2(50);
        // Straight at a target 1 m off at 4 m/s: closing does not swing the aim, so no cap.
        double[] closing = g.govern(0.02, 4.0, 0.0, 0, 0, 1.0, 0.0);
        assertEquals(4.0, closing[0], 1e-9);
        assertFalse(g.limiting());
        // Across it at 4 m/s, 1 m away: a 4 rad/s swing, past what is left after the reserve - capped,
        // direction kept.
        g.reset();
        double[] across = null;
        for (int i = 0; i < 10; i++) {
            // The cap starts at the driver's speed and eases down to its target: no step on entry.
            across = g.govern(0.02, 0.0, 4.0, 0, 0, 1.0, 0.0);
        }
        assertTrue(g.limiting());
        double cap = g.capMps();
        assertEquals(0.0, across[0], 1e-9);
        assertEquals(cap, across[1], 1e-9);
        // The swing at the capped speed is what the rate cap leaves after the reserve.
        assertEquals(0.75 * 3.0, cap / 1.0, 1e-9);
    }

    @Test
    void theModulesHeadroomCapsTooWhenItIsTheSmaller() {
        // With a rate cap too high to bind, the modules' top speed is what is shared: at the cap, the swing
        // takes exactly (1 - reserve) of the rotation the modules have left above the speed.
        AimSpeedGovernor g = governor();
        g.config().maxTurnRadps(20.0);
        double safe = g.safeSpeedMps(0.0, 1.0, 0, 0, 1.0, 0.0);
        double swing = safe / 1.0;
        double left = (TOP - safe) / RADIUS;
        assertEquals(0.75 * left, swing, 1e-9);
        assertTrue(safe < 0.75 * 20.0, "the modules bind before the rate cap does");
    }

    @Test
    void theClosingSpeedHasItsOwnCapWhenAskedFor() {
        AimSpeedGovernor g = governor();
        g.config().capSlewMps2(50).radialCapMps(1.5);
        double[] v = null;
        for (int i = 0; i < 10; i++) {
            v = g.govern(0.02, 3.0, 0.0, 0, 0, 5.0, 0.0);
        }
        assertEquals(1.5, v[0], 1e-9, "closing at 3 m/s is held to the 1.5 m/s closing cap");
        // And the direction is kept when both caps act: the smaller scale wins.
        g.reset();
        for (int i = 0; i < 40; i++) {
            v = g.govern(0.02, 3.0, 3.0, 0, 0, 5.0, 0.0);
        }
        assertEquals(1.0, v[1] / v[0], 1e-9, "same direction");
        assertEquals(1.5, v[0], 1e-9, "the closing part is at its cap");
    }

    @Test
    void theCapEasesDownRatherThanStepping() {
        AimSpeedGovernor g = governor();
        // Driving past the target at 3.5 m/s, far enough off that nothing is capped...
        g.govern(0.02, 0.0, 3.5, 0, 0, 5.0, 0.0);
        // ...then suddenly close: the cap comes down at no more than 4 m/s per second.
        double last = 3.5;
        for (int i = 0; i < 40; i++) {
            double[] v = g.govern(0.02, 0.0, 3.5, 0, 0, 1.0, 0.0);
            double speed = Math.hypot(v[0], v[1]);
            assertTrue(last - speed <= 4.0 * 0.02 + 1e-9, "dropped " + (last - speed) + " m/s in one loop");
            last = speed;
        }
        assertTrue(last < 3.5 - 0.5, "and it does come down: " + last);
    }

    @Test
    void switchedOffItPassesTheDriverThrough() {
        AimSpeedGovernor g = governor();
        g.config().enabled(false);
        for (int i = 0; i < 20; i++) {
            double[] v = g.govern(0.02, 0.0, 4.0, 0, 0, 1.0, 0.0);
            assertEquals(4.0, v[1], 0.0);
        }
        assertFalse(g.limiting());
        // The gate still answers: switching the cap off does not make 4 m/s past a target 1 m away safe.
        assertTrue(g.movingBeyondSafeSpeed(0.0, 4.0, 0, 0, 1.0, 0.0));
    }

    @Test
    void theGateAsksWhetherTheRobotHasSlowedToTheCap() {
        AimSpeedGovernor g = governor();
        double safe = g.safeSpeedMps(0.0, 1.0, 0, 0, 1.0, 0.0);
        assertEquals(2.25, safe, 1e-9);
        // Measured at the cap, or a dither above it, it may shoot.
        assertFalse(g.movingBeyondSafeSpeed(0.0, 2.25, 0, 0, 1.0, 0.0));
        assertFalse(g.movingBeyondSafeSpeed(0.0, 2.3, 0, 0, 1.0, 0.0));
        // Still coming down from 3 m/s, it may not.
        assertTrue(g.movingBeyondSafeSpeed(0.0, 3.0, 0, 0, 1.0, 0.0));
        // Straight at the target nothing swings, so no speed is too fast.
        assertFalse(g.movingBeyondSafeSpeed(4.5, 0.0, 0, 0, 1.0, 0.0));
        // And a speed it cannot judge is not a safe one.
        assertTrue(g.movingBeyondSafeSpeed(Double.NaN, 1.0, 0, 0, 1.0, 0.0));
    }

    @Test
    void settingsAreClampedAndTheDrivetrainMustBeGiven() {
        AimSpeedGovernor g = governor();
        g.config().turnReserve(5.0).capSlewMps2(Double.NaN);
        assertEquals(0.9, g.config().turnReserve(), 0.0);
        assertEquals(4.0, g.config().capSlewMps2(), 0.0);
        assertEquals(TOP, g.config().maxModuleSpeedMps(), 0.0);
        assertEquals(RADIUS, g.config().moduleRadiusM(), 0.0);
        assertThrows(IllegalArgumentException.class, () -> new AimSpeedGovernor(Double.NaN, RADIUS));
        assertThrows(IllegalArgumentException.class, () -> new AimSpeedGovernor(TOP, 0.0));
    }

    @Test
    void nothingThatIsNotANumberReachesTheDrivetrain() {
        AimSpeedGovernor g = governor();
        double[] a = g.govern(0.02, Double.NaN, 1.0, 0, 0, 5.0, 0.0);
        assertTrue(Double.isFinite(a[0]) && Double.isFinite(a[1]));
        double[] b = g.govern(0.02, 1.0, 0.5, Double.NaN, 0, 5.0, 0.0);
        assertEquals(1.0, b[0], 0.0, "a lost pose passes the driver's velocity through");
        assertEquals(0.5, b[1], 0.0);
        // On top of the target there is no line to swing across: no cap, and no division by zero.
        double[] c = g.govern(0.02, 1.0, 0.5, 5.0, 0.0, 5.0, 0.0);
        assertEquals(1.0, c[0], 0.0);
        assertEquals(0.5, c[1], 0.0);
    }
}
