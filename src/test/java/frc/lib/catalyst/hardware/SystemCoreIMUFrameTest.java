package frc.lib.catalyst.hardware;

import org.junit.jupiter.api.Test;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

import java.util.Optional;
import java.util.OptionalDouble;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Which frame the Systemcore IMU's rates and accelerations are in, and what happens when nobody
 * knows.
 *
 * <p>WPILib applies mount orientation to yaw and to the Euler angles and to nothing else. There is
 * one {@code rawgyro} topic and one {@code rawaccel} topic, with no per-orientation form — confirmed
 * in the alpha-6 bytecode and again by dumping the topics a real board publishes. So on a board that
 * is not FLAT, the acceleration is in the sensor's frame and still contains gravity.
 *
 * <p>That is worse than a wrong number in isolation, because {@link DualIMU} differences the two
 * sensors' accelerations expecting gravity to cancel. When one is in the wrong frame it does not,
 * and the residual becomes a large constant angular acceleration that is not happening. Measured on
 * a stationary bench robot at -1.89 rad/s² before this was handled.
 */
class SystemCoreIMUFrameTest {

    /** An IMU that reports a fixed acceleration, standing in for the Pigeon. */
    private record FixedIMU(Translation2d accel) implements CatalystIMU {
        @Override public Rotation2d getHeading() { return Rotation2d.kZero; }

        @Override public double getYaw() { return 0; }

        @Override public double getPitch() { return 0; }

        @Override public double getRoll() { return 0; }

        @Override public double getYawRate() { return 0; }

        @Override public void zeroYaw() {}

        @Override public void setYaw(double degrees) {}

        @Override public Optional<Translation2d> getAcceleration() { return Optional.of(accel); }
    }

    /** An IMU that cannot say, standing in for a Systemcore mounted in an unknown frame. */
    private record SilentIMU() implements CatalystIMU {
        @Override public Rotation2d getHeading() { return Rotation2d.kZero; }

        @Override public double getYaw() { return 0; }

        @Override public double getPitch() { return 0; }

        @Override public double getRoll() { return 0; }

        @Override public double getYawRate() { return 0; }

        @Override public void zeroYaw() {}

        @Override public void setYaw(double degrees) {}

        @Override public Optional<Translation2d> getAcceleration() { return Optional.empty(); }
    }

    // --- the fusion, which is where the cost lands -------------------------

    @Test
    void aSensorThatCannotSayItsFrameMakesAngularAccelerationUnavailable() {
        // The bench case. The Pigeon is reporting happily; the Systemcore cannot put its
        // acceleration in the robot's frame. The answer has to be "no measurement", not a number
        // derived from one accelerometer and a guess.
        DualIMU fused = new DualIMU(
                new FixedIMU(new Translation2d(0.2, 0.0)),
                new SilentIMU(),
                new Translation2d(0.30, 0.0),
                Translation2d.kZero);

        assertTrue(fused.angularAccelerationRadPerSecSq().isEmpty(),
                "one silent sensor is not a measurement, and -1.89 rad/s^2 on a stationary robot is "
                        + "what inventing one looks like");
    }

    @Test
    void twoSensorsInTheSameFrameStillMeasureAngularAcceleration() {
        // The other half: the guard must not have switched the feature off. A real difference
        // across a real lever arm still produces an answer.
        DualIMU fused = new DualIMU(
                new FixedIMU(new Translation2d(0.0, 0.60)),
                new FixedIMU(new Translation2d(0.0, 0.0)),
                new Translation2d(0.30, 0.0),
                Translation2d.kZero);

        OptionalDouble alpha = fused.angularAccelerationRadPerSecSq();
        assertTrue(alpha.isPresent(), "both sensors reported, so there is a measurement");
        // 0.60 m/s^2 of difference across a 0.30 m arm is 2 rad/s^2.
        assertEquals(2.0, alpha.orElseThrow(), 1e-9);
    }

    @Test
    void gravityCancelsWhenBothSensorsAreInTheSameFrame() {
        // Why the frame matters at all. Identical readings - which is what two co-located sensors
        // see when the robot is still - must produce zero, however large the common term is.
        DualIMU fused = new DualIMU(
                new FixedIMU(new Translation2d(0.0, 9.81)),
                new FixedIMU(new Translation2d(0.0, 9.81)),
                new Translation2d(0.30, 0.0),
                Translation2d.kZero);

        assertEquals(0.0, fused.angularAccelerationRadPerSecSq().orElseThrow(), 1e-9,
                "a common acceleration is not a rotation");
    }

    // --- the sensor itself, which needs a HAL ------------------------------

    @Test
    void aFlatBoardReportsAccelerationAndATiltedOneDoesNot() {
        assertTrue(HAL.initialize(500, 0), "no HAL, no IMU");

        SystemCoreIMU flat = new SystemCoreIMU(OnboardIMU.MountOrientation.FLAT);
        assertTrue(flat.getAcceleration().isPresent(),
                "FLAT means the sensor frame and the robot frame agree");
        assertTrue(flat.isYawRateTrustworthy());

        SystemCoreIMU tilted = new SystemCoreIMU(OnboardIMU.MountOrientation.LANDSCAPE);
        assertTrue(tilted.getAcceleration().isEmpty(),
                "LANDSCAPE with nobody saying how it sits: the vector would contain gravity");
        assertFalse(tilted.isYawRateTrustworthy());
        assertEquals(0.0, tilted.getYawRate(), 0.0,
                "getYawRate returns a primitive and cannot say 'unknown', so it says nothing rather "
                        + "than a pitch rate wearing a yaw label");
    }

    @Test
    void supplyingTheRotationBringsATiltedBoardBack() {
        assertTrue(HAL.initialize(500, 0), "no HAL, no IMU");

        // The opt-in half of the design: a caller who knows how the board sits gets its readings.
        SystemCoreIMU told = new SystemCoreIMU(OnboardIMU.MountOrientation.LANDSCAPE,
                Rotation2d.fromDegrees(90));

        assertTrue(told.getAcceleration().isPresent(),
                "the frame is known now, so there is nothing to withhold");
        assertTrue(told.isYawRateTrustworthy());
    }
}
