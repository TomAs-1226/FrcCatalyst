package frc.lib.catalyst.hardware;

import org.junit.jupiter.api.Test;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Two-IMU fusion, checked against cases with known answers.
 *
 * <p>The angular-acceleration solve is the part worth testing hard. It is real rigid-body physics
 * and it fails quietly when wrong: a sign error or a mis-scaled lever arm produces a number that
 * looks like an angular acceleration, feeds Physics Core, and biases everything downstream without
 * anything reporting an error. So each case here has an analytic answer rather than a recorded one.
 */
class DualIMUTest {

    /** A gyro and accelerometer whose readings are set by hand. */
    private static final class FakeIMU implements CatalystIMU {
        double yawDeg;
        double yawRateDegPerSec;

        @Override public Rotation2d getHeading() { return Rotation2d.fromDegrees(yawDeg); }
        @Override public double getYaw() { return yawDeg; }
        @Override public double getPitch() { return 0; }
        @Override public double getRoll() { return 0; }
        @Override public double getYawRate() { return yawRateDegPerSec; }
        @Override public void zeroYaw() { yawDeg = 0; }
        @Override public void setYaw(double degrees) { yawDeg = degrees; }
        @Override public String deviceType() { return "Fake"; }
    }

    /** Primary one metre ahead of the secondary, which is the sort of separation a robot offers. */
    private static DualIMU oneMetreApart(FakeIMU a, FakeIMU b) {
        return new DualIMU(a, b, new Translation2d(0.5, 0), new Translation2d(-0.5, 0));
    }

    // --- the primary stays authoritative -------------------------------------

    @Test
    void headingComesFromThePrimaryAlone() {
        FakeIMU pigeon = new FakeIMU();
        FakeIMU onboard = new FakeIMU();
        pigeon.yawDeg = 90;
        onboard.yawDeg = 85;   // drifted

        DualIMU dual = oneMetreApart(pigeon, onboard);
        assertEquals(90, dual.getYaw(), 1e-9,
                "yaw is an integral; averaging two drifting integrals hides which one moved");
        assertEquals(90, dual.getHeading().getDegrees(), 1e-9);
    }

    @Test
    void yawRateIsAveragedBecauseItIsAMeasurementNotAnIntegral() {
        FakeIMU a = new FakeIMU();
        FakeIMU b = new FakeIMU();
        a.yawRateDegPerSec = 100;
        b.yawRateDegPerSec = 90;

        assertEquals(95, oneMetreApart(a, b).getYawRate(), 1e-9);
    }

    @Test
    void zeroingAndSettingReachBothSensors() {
        FakeIMU a = new FakeIMU();
        FakeIMU b = new FakeIMU();
        a.yawDeg = 40;
        b.yawDeg = 37;

        DualIMU dual = oneMetreApart(a, b);
        dual.setYaw(180);
        assertEquals(180, a.getYaw(), 1e-9);
        assertEquals(180, b.getYaw(), 1e-9,
                "leaving the secondary behind would manufacture a disagreement that is not real");

        dual.zeroYaw();
        assertEquals(0, a.getYaw(), 1e-9);
        assertEquals(0, b.getYaw(), 1e-9);
    }

    // --- disagreement --------------------------------------------------------

    @Test
    void agreeingSensorsShowNoDisagreement() {
        FakeIMU a = new FakeIMU();
        FakeIMU b = new FakeIMU();
        a.yawDeg = b.yawDeg = 45;
        a.yawRateDegPerSec = b.yawRateDegPerSec = 30;

        DualIMU dual = oneMetreApart(a, b);
        assertEquals(0, dual.yawDisagreementDegrees(), 1e-9);
        assertEquals(0, dual.yawRateDisagreementDegPerSec(), 1e-9);
    }

    @Test
    void aFailedSensorShowsUpAsRateDisagreement() {
        FakeIMU good = new FakeIMU();
        FakeIMU stuck = new FakeIMU();
        good.yawRateDegPerSec = 180;
        stuck.yawRateDegPerSec = 0;   // stopped reporting while the robot spins

        assertEquals(180, oneMetreApart(good, stuck).yawRateDisagreementDegPerSec(), 1e-9,
                "a rigid body cannot have two different yaw rates; this is the fault signal");
    }

    @Test
    void yawDisagreementWrapsRatherThanReportingHundredsOfDegrees() {
        FakeIMU a = new FakeIMU();
        FakeIMU b = new FakeIMU();
        a.yawDeg = 179;
        b.yawDeg = -179;

        // The true disagreement is 2 degrees across the wrap, not 358.
        assertEquals(2, Math.abs(oneMetreApart(a, b).yawDisagreementDegrees()), 1e-6);
    }

    // --- angular acceleration, the thing only two sensors can measure ---------

    @Test
    void pureRotationalAccelerationIsRecovered() {
        FakeIMU a = new FakeIMU();
        FakeIMU b = new FakeIMU();
        // Stationary in rotation, so no centripetal term to remove.
        a.yawRateDegPerSec = b.yawRateDegPerSec = 0;

        DualIMU dual = oneMetreApart(a, b);

        // Sensors at x = +0.5 and x = -0.5. With alpha about +Z, a point at radius r sees
        // alpha x r, which for the +x sensor points +y and for the -x sensor points -y.
        // alpha = 2 rad/s^2 gives +1.0 m/s^2 and -1.0 m/s^2 respectively.
        Translation2d accelPrimary = new Translation2d(0, 1.0);
        Translation2d accelSecondary = new Translation2d(0, -1.0);

        assertEquals(2.0,
                dual.angularAccelerationRadPerSecSq(accelPrimary, accelSecondary).orElseThrow(),
                1e-9);
    }

    @Test
    void theSignFollowsTheDirectionOfRotation() {
        FakeIMU a = new FakeIMU();
        FakeIMU b = new FakeIMU();
        DualIMU dual = oneMetreApart(a, b);

        double ccw = dual.angularAccelerationRadPerSecSq(
                new Translation2d(0, 1.0), new Translation2d(0, -1.0)).orElseThrow();
        double cw = dual.angularAccelerationRadPerSecSq(
                new Translation2d(0, -1.0), new Translation2d(0, 1.0)).orElseThrow();

        assertTrue(ccw > 0, "CCW should be positive to match WPILib's convention");
        assertEquals(-ccw, cw, 1e-9, "reversing the sense must reverse the sign");
    }

    @Test
    void steadyRotationAtConstantRateReadsAsZeroAcceleration() {
        // The case a naive implementation gets wrong. Spinning steadily produces a large centripetal
        // difference between the two sensors with no angular acceleration at all; failing to remove
        // it reports a constant fake acceleration whenever the robot turns.
        FakeIMU a = new FakeIMU();
        FakeIMU b = new FakeIMU();
        double omega = 2.0;                                  // rad/s
        a.yawRateDegPerSec = b.yawRateDegPerSec = Math.toDegrees(omega);

        DualIMU dual = oneMetreApart(a, b);

        // Centripetal acceleration points inward at each sensor: -omega^2 * r.
        Translation2d accelPrimary = new Translation2d(-omega * omega * 0.5, 0);
        Translation2d accelSecondary = new Translation2d(omega * omega * 0.5, 0);

        assertEquals(0.0,
                dual.angularAccelerationRadPerSecSq(accelPrimary, accelSecondary).orElseThrow(),
                1e-9);
    }

    @Test
    void commonMotionCancels() {
        // Both sensors on one body share whatever the body does as a whole. Driving forwards hard
        // must not look like rotation.
        FakeIMU a = new FakeIMU();
        FakeIMU b = new FakeIMU();
        DualIMU dual = oneMetreApart(a, b);

        Translation2d sameAccel = new Translation2d(5.0, 0);
        assertEquals(0.0,
                dual.angularAccelerationRadPerSecSq(sameAccel, sameAccel).orElseThrow(), 1e-9);
    }

    @Test
    void sensorsTooCloseTogetherReportNothing() {
        FakeIMU a = new FakeIMU();
        FakeIMU b = new FakeIMU();
        // 1 cm apart: the difference between them is sensor noise divided by a small number.
        DualIMU dual = new DualIMU(a, b, new Translation2d(0.005, 0), new Translation2d(-0.005, 0));

        assertTrue(dual.angularAccelerationRadPerSecSq(
                        new Translation2d(0, 1.0), new Translation2d(0, -1.0)).isEmpty(),
                "a lever arm this short cannot carry an angular acceleration; empty beats a guess");
    }

    @Test
    void aLongerLeverArmGivesTheSameAnswer() {
        // The solve divides by the lever arm, so geometry must cancel: the same physical rotation
        // seen by sensors twice as far apart produces twice the acceleration difference.
        FakeIMU a = new FakeIMU();
        FakeIMU b = new FakeIMU();

        DualIMU near = new DualIMU(a, b, new Translation2d(0.5, 0), new Translation2d(-0.5, 0));
        DualIMU far = new DualIMU(a, b, new Translation2d(1.0, 0), new Translation2d(-1.0, 0));

        double fromNear = near.angularAccelerationRadPerSecSq(
                new Translation2d(0, 1.0), new Translation2d(0, -1.0)).orElseThrow();
        double fromFar = far.angularAccelerationRadPerSecSq(
                new Translation2d(0, 2.0), new Translation2d(0, -2.0)).orElseThrow();

        assertEquals(fromNear, fromFar, 1e-9);
    }

    @Test
    void leverArmIsTheDistanceBetweenTheSensors() {
        DualIMU dual = new DualIMU(new FakeIMU(), new FakeIMU(),
                new Translation2d(0.3, 0.4), new Translation2d(0, 0));
        assertEquals(0.5, dual.leverArmMeters(), 1e-9);
    }
}
