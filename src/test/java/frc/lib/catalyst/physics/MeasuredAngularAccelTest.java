package frc.lib.catalyst.physics;

import frc.lib.catalyst.physics.estimation.PhysicalStateEstimator;

import org.junit.jupiter.api.Test;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Angular acceleration: measured when two IMUs can supply it, derived when they cannot.
 *
 * <p>The derived path differences the gyro rate across a loop and smooths the result. On a clean
 * signal that is exact — differencing a perfect ramp gives the right answer, which is worth knowing
 * because it means a noiseless test proves nothing at all here. The difference is noise:
 * differentiating scales it by 1/dt, so at 50 Hz a rate wobbling a tenth of a degree per second
 * becomes several degrees per second squared of acceleration that is not happening.
 *
 * <p>The point of two IMUs is to skip that entirely. These tests check the two paths behave the way
 * that claim requires, because the difference is invisible in normal use: both produce a number, and
 * only one of them is the number.
 */
class MeasuredAngularAccelTest {

    private static PhysicalStateEstimator estimator() {
        return PhysicalStateEstimator.builder().build();
    }

    /** One loop of a robot rotating steadily, at the given time. */
    private static double step(PhysicalStateEstimator est, double t, double yawRate, Double alpha) {
        return est.update(
                t,
                new Pose2d(0, 0, Rotation2d.fromRadians(yawRate * t)),
                new ChassisVelocities(0, 0, yawRate),
                new Translation2d(0, 0),
                yawRate,
                0.0,
                alpha
        ).angularAccelerationRadPerSecSq();
    }

    @Test
    void aMeasuredValueIsUsedExactly() {
        // Unsmoothed and unmodified. The filter on the derived path exists to make a derivative
        // usable; applying it to something that was never differentiated only adds lag.
        PhysicalStateEstimator est = estimator();
        step(est, 0.00, 1.0, 3.5);
        assertEquals(3.5, step(est, 0.02, 1.0, 3.5), 1e-9);
    }

    @Test
    void aMeasuredValueOfZeroIsStillAMeasurement() {
        // Steady rotation has no angular acceleration. A null check that treats 0.0 as "nothing
        // supplied" would silently fall back to the derived path at exactly the moment the two
        // disagree least, hiding the bug.
        PhysicalStateEstimator est = estimator();
        step(est, 0.00, 2.0, 0.0);
        assertEquals(0.0, step(est, 0.02, 2.0, 0.0), 1e-9);
    }

    @Test
    void withoutAMeasurementItStillDerivesOne() {
        // The fallback has to keep working: a robot with one IMU, or one whose second sensor dropped
        // out, must not lose angular acceleration altogether.
        PhysicalStateEstimator est = estimator();
        step(est, 0.00, 0.0, null);
        step(est, 0.02, 1.0, null);
        double derived = step(est, 0.04, 2.0, null);

        assertTrue(derived > 0, "spinning up should show positive angular acceleration, got " + derived);
    }

    @Test
    void theDerivedPathAmplifiesGyroNoiseAndTheMeasuredOneDoesNot() {
        // The reason this is worth wiring, stated as the thing that is actually different.
        //
        // On a perfectly clean signal both paths agree - differencing an exact ramp gives the exact
        // answer, which is worth knowing because it means a noiseless test proves nothing. The
        // difference is noise: dividing by dt scales it by 1/dt, so at 50 Hz a rate wobbling a
        // tenth of a degree per second becomes several degrees per second squared.
        //
        // A robot rotating at a steady rate, with a small amount of gyro noise on it. The truth is
        // zero angular acceleration throughout.
        PhysicalStateEstimator measured = estimator();
        PhysicalStateEstimator derived = estimator();

        double dt = 0.02;
        double trueRate = 2.0;
        double noise = 0.02;                  // rad/s, small
        double measuredWorst = 0;
        double derivedWorst = 0;

        for (int i = 0; i < 40; i++) {
            double t = i * dt;
            // Deterministic alternating noise rather than a random number: this has to fail the same
            // way on every machine, and the point is the amplification, not the distribution.
            double wobble = (i % 2 == 0 ? noise : -noise);
            double rate = trueRate + wobble;

            // The two-accelerometer measurement of a steadily rotating robot is zero, and it is not
            // affected by gyro noise at all - it never touches the gyro rate.
            double m = step(measured, t, rate, 0.0);
            double d = step(derived, t, rate, null);

            if (i > 5) {                      // let both settle
                measuredWorst = Math.max(measuredWorst, Math.abs(m));
                derivedWorst = Math.max(derivedWorst, Math.abs(d));
            }
        }

        assertEquals(0.0, measuredWorst, 1e-9, "the measured path should report the truth: zero");
        assertTrue(derivedWorst > 0.5,
                "differencing a wobbling rate should produce a large false acceleration, saw "
                        + derivedWorst);
    }

    @Test
    void aDropoutFallsBackOnTheNextLoopRatherThanForever() {
        // A sensor that stops reporting mid-match degrades to the derived value and recovers when it
        // comes back. Deciding once at startup would mean a dropout costs the measurement for the
        // rest of the match.
        PhysicalStateEstimator est = estimator();
        step(est, 0.00, 1.0, 4.0);
        assertEquals(4.0, step(est, 0.02, 1.0, 4.0), 1e-9);

        double duringDropout = step(est, 0.04, 1.2, null);
        assertTrue(Double.isFinite(duringDropout), "the derived path should carry it");

        assertEquals(4.0, step(est, 0.06, 1.4, 4.0), 1e-9, "and the measurement resumes");
    }
}
