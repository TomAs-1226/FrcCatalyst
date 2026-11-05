package frc.lib.catalyst.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import java.util.function.DoubleUnaryOperator;

import org.junit.jupiter.api.Test;

/**
 * The shoot-on-the-move heading tracker's arithmetic: the aim solve, checked by flying the ball, and the
 * reference it builds from it. Ported from the Catalyst X1, where the same class was run against the
 * robot's earlier turret modes and 5805's V7 on a model fitted to its recordings. That harness stays with
 * the X1, calibrated to its recordings; these are the properties that hold for any robot.
 */
class HeadingTrackerTest {

    private static final double[] HUB = {10.0, 4.0};

    /**
     * 5805's Numbers robot's measured scoring time of flight, distance (m) to seconds. Not monotonic: a
     * shot from 2.42 m flies longer than one from 3.54 m, which a bisection cannot solve and the tracker's
     * iteration must.
     */
    private static final InterpolatingTable NUMBERS_TABLE = new InterpolatingTable()
            .add(1.36, 1.017).add(2.42, 1.233).add(3.54, 1.148).add(5.5, 1.348);
    private static final DoubleUnaryOperator NUMBERS_TOF = NUMBERS_TABLE::get;

    /**
     * Where a ball lands: it leaves the exit - robot frame (ex, ey), robot at (x, y) facing {@code heading},
     * moving at (vx, vy) and turning at {@code omega} - along the heading at whatever speed covers the
     * distance in the time of flight the table gives for it, carrying the exit's own velocity.
     */
    private static double[] land(double x, double y, double heading, double vx, double vy, double omega,
            double ex, double ey, double flight, double distance) {
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        double px = x + ex * c - ey * s;
        double py = y + ex * s + ey * c;
        double evx = vx - omega * (ex * s + ey * c);
        double evy = vy + omega * (ex * c - ey * s);
        double shot = distance / flight;
        return new double[] {px + (evx + shot * c) * flight, py + (evy + shot * s) * flight};
    }

    @Test
    void standingStillItFacesTheTarget() {
        HeadingTracker.Aim a = HeadingTracker.solve(7.0, 4.0, 0.3, 0, 0, 0, HUB[0], HUB[1], null, 0, 0);
        assertEquals(0.0, a.headingRad(), 1e-12);
        assertEquals(3.0, a.distanceM(), 1e-12);
        assertEquals(0.0, a.timeOfFlightS(), 0.0);
        assertTrue(a.converged());
    }

    @Test
    void theSwingIsTheBearingsRateAsTheRobotMoves() {
        // Two metres east of the target, strafing north at 1 m/s: the bearing swings at v / d.
        HeadingTracker.Aim a = HeadingTracker.solve(12.0, 4.0, Math.PI, 0, 1.0, 0, HUB[0], HUB[1], null, 0, 0);
        assertEquals(0.5, a.rateRadps(), 1e-9);
    }

    @Test
    void aMovingShotLandsInTheTarget() {
        // Strafing at 2 m/s, 3.3 m out, with Numbers' 1.1-1.2 s flight: the lead is over two metres.
        double x = 13.3;
        double y = 2.5;
        double vx = 0.3;
        double vy = 2.0;
        HeadingTracker.Aim a = HeadingTracker.solve(x, y, Math.PI, vx, vy, 0, HUB[0], HUB[1], NUMBERS_TOF, 0, 0);
        assertTrue(a.converged(), "converged in " + a.iterations());
        assertTrue(Math.hypot(a.aimX() - HUB[0], a.aimY() - HUB[1]) > 2.0, "the lead is over two metres");
        double[] landed = land(x, y, a.headingRad(), vx, vy, 0, 0, 0, a.timeOfFlightS(), a.distanceM());
        assertEquals(HUB[0], landed[0], 1e-3);
        assertEquals(HUB[1], landed[1], 1e-3);
        // And the flight is the table's for the distance it flies.
        assertEquals(NUMBERS_TOF.applyAsDouble(a.distanceM()), a.timeOfFlightS(), 1e-4);
    }

    @Test
    void anOffsetShooterOnATurningChassisLandsInTheTarget() {
        // Numbers' exit is 0.29 m behind the centre. Turning at 2 rad/s the exit moves sideways at 0.58 m/s,
        // which the ball carries: aimed from the centre, that is a miss of 0.58 m per second of flight.
        double x = 5.0;
        double y = 4.0;
        double omega = 2.0;
        DoubleUnaryOperator oneSecond = d -> 1.0;
        HeadingTracker.Aim offset = HeadingTracker.solve(x, y, 0.0, 0, 0, omega, HUB[0], HUB[1], oneSecond, -0.29, 0);
        HeadingTracker.Aim centred = HeadingTracker.solve(x, y, 0.0, 0, 0, omega, HUB[0], HUB[1], oneSecond, 0, 0);
        assertTrue(offset.converged());
        assertEquals(0.0, centred.headingRad(), 1e-9, "a centred shooter gets no lever arm from the turn");
        // Facing +x, the exit at (-0.29, 0) moves at omega x r = (0, -0.58): lead it by about 0.58 m to the
        // north - a little more, since turning to it swings the exit too.
        assertEquals(Math.toRadians(6.5), offset.headingRad(), Math.toRadians(0.5));
        double[] landed = land(x, y, offset.headingRad(), 0, 0, omega, -0.29, 0, offset.timeOfFlightS(),
                offset.distanceM());
        assertEquals(HUB[0], landed[0], 1e-3);
        assertEquals(HUB[1], landed[1], 1e-3);
        double[] missed = land(x, y, centred.headingRad(), 0, 0, omega, -0.29, 0, 1.0, 5.29);
        assertTrue(Math.abs(missed[1] - HUB[1]) > 0.5, "aimed from the centre the shot misses by "
                + Math.abs(missed[1] - HUB[1]) + " m");
    }

    @Test
    void anOffsetShooterDrivingAndTurningStillLands() {
        double x = 13.0;
        double y = 5.5;
        double vx = -0.8;
        double vy = -1.5;
        double omega = -1.2;
        HeadingTracker.Aim a = HeadingTracker.solve(x, y, Math.PI, vx, vy, omega, HUB[0], HUB[1], NUMBERS_TOF,
                -0.286, 0.05);
        assertTrue(a.converged());
        double[] landed = land(x, y, a.headingRad(), vx, vy, omega, -0.286, 0.05, a.timeOfFlightS(), a.distanceM());
        assertEquals(HUB[0], landed[0], 2e-3);
        assertEquals(HUB[1], landed[1], 2e-3);
    }

    @Test
    void aTimeOfFlightThatCannotSettleSaysSo() {
        // Backing away at 2 m/s from a target 3 m off, a flight of 2.5 s inside 5 m and 0.2 s beyond has no fixed
        // point - 2.5 s puts the goal 8 m off, where the flight is 0.2 s, which puts it 3.4 m off - so the solve
        // must stop and say so.
        DoubleUnaryOperator steep = d -> d < 5.0 ? 2.5 : 0.2;
        HeadingTracker.Aim a = HeadingTracker.solve(7.0, 4.0, 0.0, -2.0, 0.0, 0, HUB[0], HUB[1], steep, 0, 0);
        assertFalse(a.converged());
        assertEquals(HeadingTracker.MAX_ITERATIONS, a.iterations());
        assertTrue(Double.isFinite(a.headingRad()));
    }

    /** Drive a tracker at {@code n} loops of a robot standing at (x, y) facing {@code heading}. */
    private static HeadingTracker standing(double x, double y, double heading, int n) {
        HeadingTracker t = new HeadingTracker();
        for (int i = 0; i < n; i++) {
            t.update(i * 0.02, x, y, heading, 0.0, 0, 0, HUB[0], HUB[1]);
        }
        return t;
    }

    @Test
    void startingOffTheAimTheReferenceSwingsOntoItWithoutOvershoot() {
        HeadingTracker t = new HeadingTracker();
        double worst = 0;
        for (int i = 0; i < 100; i++) {
            // A robot that follows the reference exactly, 0.4 rad off the aim at the start.
            double heading = i == 0 ? 0.4 : t.referenceRad();
            t.update(i * 0.02, 7.0, 4.0, heading, 0.0, 0, 0, HUB[0], HUB[1]);
            worst = Math.min(worst, t.referenceRad());
        }
        assertEquals(0.0, t.referenceRad(), Math.toRadians(0.3));
        assertTrue(worst > -Math.toRadians(0.5), "swung past by " + Math.toDegrees(-worst) + " degrees");
    }

    @Test
    void standingStillPoseNoiseDoesNotMoveTheReference() {
        HeadingTracker t = standing(7.0, 4.0, 0.0, 50);
        double settled = t.referenceRad();
        Random noise = new Random(3);
        double moved = 0;
        for (int i = 50; i < 300; i++) {
            // A centimetre of vision noise three metres out: about 0.2 degrees of aim.
            t.update(i * 0.02, 7.0 + noise.nextGaussian() * 0.01, 4.0 + noise.nextGaussian() * 0.01, settled, 0.0,
                    0, 0, HUB[0], HUB[1]);
            moved = Math.max(moved, Math.abs(t.referenceRateRadps()));
        }
        assertEquals(0.0, moved, 1e-9, "the reference turned at up to " + moved + " rad/s on pose noise");
    }

    @Test
    void itIsOnTargetOnlyWhenTheReferenceWillBeOnTheAim() {
        HeadingTracker t = standing(7.0, 4.0, 0.0, 100);
        assertTrue(t.onTargetIn(0.25, Math.toRadians(2)));
        // Facing ten degrees off, it is not - the heading loop still has that to close.
        t.update(2.0, 7.0, 4.0, Math.toRadians(10), 0.0, 0, 0, HUB[0], HUB[1]);
        assertFalse(t.onTargetIn(0.25, Math.toRadians(2)));
    }

    @Test
    void settingsAreClampedAndIgnoreNonsense() {
        HeadingTracker.Config c = new HeadingTracker.Config();
        c.kP(Double.NaN);
        assertEquals(6.0, c.kP(), 0.0);
        c.kP(1e6);
        assertEquals(15.0, c.kP(), 0.0);
        c.maxRateRadps(-1);
        assertEquals(0.1, c.maxRateRadps(), 0.0);
        c.bandStillRad(1.0);
        assertEquals(Math.toRadians(5), c.bandStillRad(), 1e-12, "a band wider than 5 degrees is not a noise band");
    }

    @Test
    void aLoopWithANumberMissingIsSkippedAndLeavesNothingBehind() {
        HeadingTracker t = new HeadingTracker();
        // Strafing north past the target at 1 m/s, the pose moving as commanded, so the share is being learned.
        for (int i = 0; i < 100; i++) {
            t.update(i * 0.02, 7.0, 3.0 + 0.02 * i, 0.0, 0.0, 0, 1.0, HUB[0], HUB[1]);
        }
        // One loop where the estimator's pose is not a number: before the guard, it made the learned share NaN
        // for good, and every direction after it.
        t.update(2.0, Double.NaN, 5.0, 0.0, 0.0, 0, 1.0, HUB[0], HUB[1]);
        assertNull(t.aim(), "no aim from a loop without a pose");
        for (int i = 101; i < 200; i++) {
            t.update(i * 0.02, 7.0, 3.0 + 0.02 * i, 0.0, 0.0, 0, 1.0, HUB[0], HUB[1]);
            assertTrue(Double.isFinite(t.directionRad()) && Double.isFinite(t.feedforwardRadps()), "loop " + i);
        }
        assertTrue(t.deliveredShare() >= 0.5 && t.deliveredShare() <= 1.2, "share " + t.deliveredShare());
    }

    @Test
    void theDeliveredShareIsLearnedFromThePose() {
        // A drivetrain that makes good 0.8 of what it is told, as the X1's did: commanded 1 m/s north, the
        // pose moves 0.8 m/s.
        HeadingTracker t = new HeadingTracker();
        for (int i = 0; i < 250; i++) {
            t.update(i * 0.02, 7.0, 2.0 + 0.8 * 0.02 * i, 0.0, 0.0, 0, 1.0, HUB[0], HUB[1]);
        }
        assertEquals(0.8, t.deliveredShare(), 0.02);
        // Switched off, the command is taken at its word.
        t.config().learnDelivery(false);
        t.update(250 * 0.02, 7.0, 2.0 + 0.8 * 0.02 * 250, 0.0, 0.0, 0, 1.0, HUB[0], HUB[1]);
        assertEquals(1.0, t.deliveredShare(), 0.0);
    }

    @Test
    void theFeedforwardNeverAsksForMoreThanTheRateLimit() {
        HeadingTracker t = new HeadingTracker();
        t.config().leadS(0.3).maxAccelRadps2(60.0).maxRateRadps(3.0);
        double worst = 0;
        double heading = Math.PI / 2;
        // Passing 0.4 m from the target at 3 m/s: its bearing swings at up to 7.5 rad/s, well past the limit.
        for (int i = 0; i < 60; i++) {
            t.update(i * 0.02, 9.6, 2.5 + 3.0 * i * 0.02, heading, 0.0, 0, 3.0, HUB[0], HUB[1]);
            worst = Math.max(worst, Math.abs(t.feedforwardRadps()));
            heading = t.referenceRad();
        }
        assertTrue(worst <= 3.0 + 1e-9, "asked for " + worst + " rad/s");
        assertTrue(worst > 2.5, "the pass should have pushed the feedforward to the limit; it reached " + worst);
    }
}
