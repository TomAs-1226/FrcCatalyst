package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.lib.catalyst.physics.model.ProjectileModel;
import frc.lib.catalyst.physics.model.RobotModel;

/**
 * Pure-Java tests for {@link ProjectileModel}. The important ones are round-trips: solve for a launch
 * angle, fly the shot with it, and check it arrives where it was aimed. A sign error or a rearranged
 * term survives a spot-check against one hand-computed number; it does not survive a round-trip.
 */
class ProjectileModelTest {

    private static final double G = RobotModel.GRAVITY;

    @Test
    void timeOfFlightIsDistanceOverHorizontalSpeed() {
        ProjectileModel shot = ProjectileModel.noDrag();
        Rotation2d angle = Rotation2d.fromDegrees(45);

        double flight = shot.timeOfFlightSeconds(10.0, 20.0, angle).orElseThrow();
        assertEquals(10.0 / (20.0 * Math.cos(Math.toRadians(45))), flight, 1e-12);
    }

    @Test
    void aFlatterShotArrivesSooner() {
        ProjectileModel shot = ProjectileModel.noDrag();

        double flat = shot.timeOfFlightSeconds(8.0, 18.0, Rotation2d.fromDegrees(25)).orElseThrow();
        double lobbed = shot.timeOfFlightSeconds(8.0, 18.0, Rotation2d.fromDegrees(65)).orElseThrow();
        assertTrue(flat < lobbed);
    }

    @Test
    void heightFollowsTheParabola() {
        ProjectileModel shot = ProjectileModel.noDrag();
        double speed = 15.0;
        Rotation2d angle = Rotation2d.fromDegrees(40);

        double t = 0.6;
        double expected = speed * Math.sin(Math.toRadians(40)) * t - 0.5 * G * t * t;
        assertEquals(expected, shot.heightAtTime(t, speed, angle), 1e-12);
        assertEquals(0.0, shot.heightAtTime(0.0, speed, angle), 1e-12);
    }

    @Test
    void aSolvedLaunchAngleActuallyLandsOnTheTarget() {
        ProjectileModel shot = ProjectileModel.noDrag();
        double distance = 7.5;
        double heightDelta = 2.1;      // a goal 2.1 m above the shooter
        double speed = 16.0;

        var angles = shot.launchAnglesFor(distance, heightDelta, speed).orElseThrow();

        // Both solutions must arrive at exactly the target height at exactly that distance.
        assertEquals(heightDelta, shot.heightAtDistance(distance, speed, angles.flat()).orElseThrow(), 1e-9);
        assertEquals(heightDelta, shot.heightAtDistance(distance, speed, angles.lobbed()).orElseThrow(), 1e-9);
        assertTrue(angles.lobbed().getRadians() > angles.flat().getRadians());
        assertTrue(!angles.atMaxRange());
    }

    @Test
    void theRoundTripHoldsForAFlatTargetToo() {
        ProjectileModel shot = ProjectileModel.noDrag();
        var angles = shot.launchAnglesFor(6.0, 0.0, 12.0).orElseThrow();

        assertEquals(0.0, shot.heightAtDistance(6.0, 12.0, angles.flat()).orElseThrow(), 1e-9);
        assertEquals(0.0, shot.heightAtDistance(6.0, 12.0, angles.lobbed()).orElseThrow(), 1e-9);
    }

    @Test
    void maxRangeOnFlatGroundIsTheTextbookValueAndTheTwoSolutionsMeetThere() {
        ProjectileModel shot = ProjectileModel.noDrag();
        double speed = 14.0;

        double range = shot.maxRangeMeters(0.0, speed).orElseThrow();
        assertEquals(speed * speed / G, range, 1e-9);

        // At maximum range the flat and lobbed solutions collapse onto 45 degrees.
        var angles = shot.launchAnglesFor(range, 0.0, speed).orElseThrow();
        assertTrue(angles.atMaxRange());
        assertEquals(45.0, angles.flat().getDegrees(), 1e-6);
    }

    @Test
    void aTargetBeyondRangeHasNoSolution() {
        ProjectileModel shot = ProjectileModel.noDrag();
        double speed = 10.0;
        double range = shot.maxRangeMeters(0.0, speed).orElseThrow();

        assertTrue(shot.launchAnglesFor(range * 1.05, 0.0, speed).isEmpty());
        assertTrue(shot.launchAnglesFor(range * 0.95, 0.0, speed).isPresent());
    }

    @Test
    void raisingTheTargetShortensTheReachableRange() {
        ProjectileModel shot = ProjectileModel.noDrag();

        double flat = shot.maxRangeMeters(0.0, 15.0).orElseThrow();
        double high = shot.maxRangeMeters(2.5, 15.0).orElseThrow();
        assertTrue(high < flat);
    }

    @Test
    void apexHeightMatchesTheClosedForm() {
        ProjectileModel shot = ProjectileModel.noDrag();
        double speed = 12.0;
        Rotation2d angle = Rotation2d.fromDegrees(55);

        double vy = speed * Math.sin(Math.toRadians(55));
        assertEquals(vy * vy / (2 * G), shot.apexHeightMeters(speed, angle), 1e-12);
        assertEquals(0.0, shot.apexHeightMeters(speed, Rotation2d.fromDegrees(-10)), 1e-12);
    }

    @Test
    void dragMakesTheSameShotSlowerAndShorterReaching() {
        ProjectileModel clean = ProjectileModel.noDrag();
        ProjectileModel draggy = ProjectileModel.withLinearDrag(0.3);
        Rotation2d angle = Rotation2d.fromDegrees(40);

        double cleanFlight = clean.timeOfFlightSeconds(8.0, 18.0, angle).orElseThrow();
        double draggyFlight = draggy.timeOfFlightSeconds(8.0, 18.0, angle).orElseThrow();
        assertTrue(draggyFlight > cleanFlight, "drag must lengthen the flight to the same distance");

        // Horizontal reach asymptotes at vx/k; beyond that the shot never arrives.
        double reach = 18.0 * Math.cos(Math.toRadians(40)) / 0.3;
        assertTrue(draggy.timeOfFlightSeconds(reach * 1.01, 18.0, angle).isEmpty());
        assertTrue(draggy.timeOfFlightSeconds(reach * 0.5, 18.0, angle).isPresent());
    }

    @Test
    void aSmallButRealDragStaysCloseToTheParabola() {
        // The drag solution must reduce toward the parabola as k shrinks, or the algebra is wrong.
        // k = 0.01 over a sub-second flight is about a 1% effect, so the two should agree to
        // centimetres - close, but genuinely different, which is what makes this a real check.
        ProjectileModel clean = ProjectileModel.noDrag();
        ProjectileModel lightDrag = ProjectileModel.withLinearDrag(0.01);
        Rotation2d angle = Rotation2d.fromDegrees(35);

        assertTrue(lightDrag.hasDrag());
        assertEquals(clean.timeOfFlightSeconds(9.0, 17.0, angle).orElseThrow(),
                lightDrag.timeOfFlightSeconds(9.0, 17.0, angle).orElseThrow(), 0.01);
        assertEquals(clean.heightAtTime(0.7, 17.0, angle),
                lightDrag.heightAtTime(0.7, 17.0, angle), 0.05);
        assertEquals(clean.apexHeightMeters(17.0, angle),
                lightDrag.apexHeightMeters(17.0, angle), 0.05);
    }

    @Test
    void aNumericallyDangerousDragCoefficientFallsBackToTheExactParabola() {
        // The drag height solution is a difference of two terms that both blow up like 1/k, so at a
        // tiny k it loses all its precision. Below the negligible-drag threshold the model must take
        // the exact path instead of returning a confidently wrong number.
        ProjectileModel clean = ProjectileModel.noDrag();
        ProjectileModel absurd = ProjectileModel.withLinearDrag(1e-7);
        Rotation2d angle = Rotation2d.fromDegrees(35);

        assertFalse(absurd.hasDrag());
        assertEquals(clean.apexHeightMeters(17.0, angle), absurd.apexHeightMeters(17.0, angle), 1e-12);
        assertEquals(clean.heightAtTime(0.7, 17.0, angle), absurd.heightAtTime(0.7, 17.0, angle), 1e-12);
        assertEquals(clean.timeOfFlightSeconds(9.0, 17.0, angle).orElseThrow(),
                absurd.timeOfFlightSeconds(9.0, 17.0, angle).orElseThrow(), 1e-12);
    }

    @Test
    void degenerateShotsReportNoSolutionRatherThanInfinity() {
        ProjectileModel shot = ProjectileModel.noDrag();

        assertTrue(shot.timeOfFlightSeconds(5.0, 0.0, Rotation2d.fromDegrees(45)).isEmpty());
        assertTrue(shot.timeOfFlightSeconds(5.0, 10.0, Rotation2d.fromDegrees(90)).isEmpty());
        assertEquals(0.0, shot.timeOfFlightSeconds(0.0, 10.0, Rotation2d.fromDegrees(45)).orElseThrow(), 1e-12);
    }

    @Test
    void theAngleSolverRefusesToGuessWhenDragIsModelled() {
        ProjectileModel draggy = ProjectileModel.withLinearDrag(0.2);

        // Silently returning the drag-free angle would be worse than refusing.
        assertThrows(UnsupportedOperationException.class, () -> draggy.launchAnglesFor(6.0, 1.0, 14.0));
        assertThrows(UnsupportedOperationException.class, () -> draggy.maxRangeMeters(0.0, 14.0));
        assertThrows(IllegalArgumentException.class, () -> ProjectileModel.withLinearDrag(-0.1));
    }
}
