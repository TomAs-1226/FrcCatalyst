package frc.lib.catalyst.hardware;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import frc.lib.catalyst.hardware.LinearAccelerationSample.Reading;

class LinearAccelerationSampleTest {
    private static final double G = LinearAccelerationSample.STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED;
    private static Reading value(double v) { return new Reading(v, 1.0, true); }

    @Test void stationaryTiltHasNoLinearAcceleration() {
        var result = LinearAccelerationSample.from(value(.5 * G), value(-.25 * G),
                value(.5), value(-.25), 1.05, .10, .02).orElseThrow();
        assertEquals(0, result.getX(), 1e-12);
        assertEquals(0, result.getY(), 1e-12);
    }

    @Test void translationSurvivesGravityRemovalWithBothAxisSigns() {
        var result = LinearAccelerationSample.from(value(.2 * G + 2.5), value(-.1 * G - 1.2),
                value(.2), value(-.1), 1.05, .10, .02).orElseThrow();
        assertEquals(2.5, result.getX(), 1e-12);
        assertEquals(-1.2, result.getY(), 1e-12);
    }

    @Test void everySignalMustBeValidFiniteAndRecent() {
        Reading[] rejected = { null, new Reading(0, 1, false), value(Double.NaN),
                value(Double.POSITIVE_INFINITY), new Reading(0, Double.NaN, true),
                new Reading(0, Double.POSITIVE_INFINITY, true), new Reading(0, .94, true),
                new Reading(0, 1.06, true) };
        for (var bad : rejected) {
            for (int axis = 0; axis < 4; axis++) {
                Reading[] input = { value(0), value(0), value(0), value(0) };
                input[axis] = bad;
                assertTrue(LinearAccelerationSample.from(input[0], input[1], input[2], input[3],
                        1.05, .10, .02).isEmpty(), "invalid signal " + axis + ": " + bad);
            }
        }
    }

    @Test void timestampSpreadIncludesCrossAxisDisagreement() {
        // Each accel/gravity pair agrees, but X and Y describe different instants.
        var earlier = new Reading(0, .97, true);
        assertTrue(LinearAccelerationSample.from(value(0), earlier, value(0), earlier,
                1.05, .10, .02).isEmpty());
    }

    @Test void exactFreshnessBoundariesAndZeroAccelerationAreValid() {
        // Binary-exact times avoid a floating-point approximation to the boundary.
        var earlier = new Reading(0, .875, true);
        var latest = new Reading(0, .9375, true);
        assertTrue(LinearAccelerationSample.from(earlier, latest, earlier, latest,
                1, .125, .0625).isPresent());
        assertTrue(LinearAccelerationSample.from(earlier, latest, earlier, latest,
                1, .124, .0625).isEmpty());
        assertTrue(LinearAccelerationSample.from(earlier, latest, earlier, latest,
                1, .125, .0624).isEmpty());
    }

    @Test void levelFramePreservesFlatMotionAndRejectsVerticalBumps() {
        for (double polarity : new double[] { -1, 1 }) {
            var result = LinearAccelerationSample.level(value(2), value(-3), value(polarity * G + 4),
                    value(0), value(0), value(polarity), 1.05, .10, .02).orElseThrow();
            assertEquals(2, result.getX(), 1e-12);
            assertEquals(-3, result.getY(), 1e-12);
        }
    }

    @Test void positiveAndNegativePitchRemoveGravityAndLevelTrueTranslation() {
        for (double degrees : new double[] { -35, 25 }) {
            double angle = Math.toRadians(degrees), sin = Math.sin(angle), cos = Math.cos(angle);
            // Inverse of the minimal Y-axis leveling rotation: known horizontal travel (2,3).
            for (double polarity : new double[] { -1, 1 }) {
                double gx = polarity * sin, gz = -polarity * cos;
                var result = LinearAccelerationSample.level(value(2 * cos + gx * G), value(3),
                        value(2 * sin + gz * G), value(gx), value(0), value(gz),
                        1.05, .10, .02).orElseThrow();
                assertEquals(2, result.getX(), 1e-12);
                assertEquals(3, result.getY(), 1e-12);
                var rest = LinearAccelerationSample.level(value(gx * G), value(0), value(gz * G),
                        value(gx), value(0), value(gz), 1.05, .10, .02).orElseThrow();
                assertEquals(0, rest.getNorm(), 1e-12);
            }
        }
    }

    @Test void positiveAndNegativeRollLevelTranslation() {
        for (double degrees : new double[] { -30, 40 }) {
            double angle = Math.toRadians(degrees), sin = Math.sin(angle), cos = Math.cos(angle);
            for (double polarity : new double[] { -1, 1 }) {
                double gy = -polarity * sin, gz = -polarity * cos;
                var result = LinearAccelerationSample.level(value(2), value(3 * cos + gy * G),
                        value(-3 * sin + gz * G), value(0), value(gy), value(gz),
                        1.05, .10, .02).orElseThrow();
                assertEquals(2, result.getX(), 1e-12);
                assertEquals(3, result.getY(), 1e-12);
            }
        }
    }

    @Test void levelFrameRejectsInvalidGravityAndEveryBadOrMisalignedSignal() {
        for (double z : new double[] { 0, -.8, -1.2, .8, 1.2, Double.NaN }) {
            assertTrue(LinearAccelerationSample.level(value(0), value(0), value(0),
                    value(0), value(0), value(z), 1.05, .10, .02).isEmpty());
        }
        for (int axis = 0; axis < 6; axis++) {
            for (Reading bad : new Reading[] { null, new Reading(0, 1, false),
                    value(Double.NaN), new Reading(0, .94, true), new Reading(0, 1.06, true),
                    new Reading(0, .97, true) }) {
                Reading[] input = { value(0), value(0), value(-G), value(0), value(0), value(-1) };
                input[axis] = bad;
                assertTrue(LinearAccelerationSample.level(input[0], input[1], input[2],
                        input[3], input[4], input[5], 1.05, .10, .02).isEmpty());
            }
        }
    }

    @Test void nearHorizontalGravityFailsClosedUnderEitherPolarity() {
        for (double polarity : new double[] { -1, 1 }) {
            assertTrue(LinearAccelerationSample.level(value(0), value(0), value(0),
                    value(Math.sqrt(.99)), value(0), value(polarity * .1),
                    1.05, .10, .02).isEmpty());
            assertTrue(LinearAccelerationSample.level(value(0), value(0), value(0),
                    value(Math.sqrt(1 - .51 * .51)), value(0), value(polarity * .51),
                    1.05, .10, .02).isPresent());
        }
    }

    @Test void invalidLimitsAndOverflowFailClosed() {
        for (double bad : new double[] { -1, Double.NaN, Double.POSITIVE_INFINITY }) {
            assertTrue(LinearAccelerationSample.from(value(0), value(0), value(0), value(0),
                    1.05, bad, .02).isEmpty());
            assertTrue(LinearAccelerationSample.from(value(0), value(0), value(0), value(0),
                    1.05, .10, bad).isEmpty());
        }
        assertTrue(LinearAccelerationSample.from(value(0), value(0), value(0), value(0),
                Double.NaN, .10, .02).isEmpty());
        assertTrue(LinearAccelerationSample.from(value(0), value(0), value(Double.MAX_VALUE), value(0),
                1.05, .10, .02).isEmpty());
    }
}
