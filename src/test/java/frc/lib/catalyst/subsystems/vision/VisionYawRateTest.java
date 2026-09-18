package frc.lib.catalyst.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

/**
 * The spin gate and MegaTag2's yaw rate come from the gyro, not the wheels. Both used to read
 * {@code getChassisSpeeds().omega}, the rotation the wheel kinematics report. On the Catalyst X1 (recordings
 * of 2026-09-17) that disagreed with the gyro in 13 of 41 turning seconds, some of them in sign, and led it
 * by 100-200 ms at every turn onset: the gate rejected good frames the chassis had not yet turned for, and
 * passed blurred ones. The first two tests fail against that build.
 */
class VisionYawRateTest {

    private static final Pose2d WHERE = new Pose2d(3.0, 2.0, Rotation2d.kZero);

    /** A drivetrain whose wheels and gyro disagree about the turn, as the X1's did. */
    static final class SlippingDrive implements VisionPoseSink {
        final double wheelOmega;
        final double gyroRate;
        int measurements = 0;

        SlippingDrive(double wheelOmega, double gyroRate) {
            this.wheelOmega = wheelOmega;
            this.gyroRate = gyroRate;
        }

        @Override public Pose2d getPose() { return WHERE; }
        @Override public ChassisVelocities getChassisSpeeds() { return new ChassisVelocities(0, 0, wheelOmega); }
        @Override public double getYawRateRadPerSec() { return gyroRate; }
        @Override public void addVisionMeasurement(Pose2d p, double ts, Matrix<N3, N1> stdDevs) { measurements++; }
    }

    /** A sink written before the gyro rate existed: it has only the wheels. */
    static final class WheelsOnly implements VisionPoseSink {
        final double wheelOmega;
        int measurements = 0;

        WheelsOnly(double wheelOmega) {
            this.wheelOmega = wheelOmega;
        }

        @Override public Pose2d getPose() { return WHERE; }
        @Override public ChassisVelocities getChassisSpeeds() { return new ChassisVelocities(0, 0, wheelOmega); }
        @Override public void addVisionMeasurement(Pose2d p, double ts, Matrix<N3, N1> stdDevs) { measurements++; }
    }

    /** A camera that sees the robot where it is and remembers the yaw rate it was handed for MegaTag2. */
    static final class RecordingCamera implements CameraSource {
        final SimCameraSource inner = SimCameraSource.builder("sim")
                .truePose(() -> WHERE)
                .translationStdDevMeters(0)
                .rotationStdDevDegrees(0)
                .latencySeconds(0.01)
                .tagCount(2)
                .build();
        double yawRateDegPerSec = Double.NaN;

        @Override public String getName() { return inner.getName(); }
        @Override public Optional<PoseEstimate> getEstimatedPose() { return inner.getEstimatedPose(); }

        @Override
        public void setRobotOrientation(double yawDegrees, double yawRate, double pitchDegrees, double rollDegrees) {
            yawRateDegPerSec = yawRate;
        }
    }

    private static VisionSubsystem vision(VisionPoseSink sink, CameraSource camera) {
        return new VisionSubsystem(VisionConfig.builder()
                .addCamera(camera)
                .poseSink(sink)
                .seedFromVision(false)
                .rejectDuringSpin(2.0)
                .build());
    }

    @Test
    void theSpinGateReadsTheGyroNotTheWheels() {
        // The chassis is spinning at 3 rad/s; the wheels, slipping, say it is not.
        SlippingDrive spinning = new SlippingDrive(0.0, 3.0);
        VisionSubsystem blurred = vision(spinning, new RecordingCamera());
        blurred.periodic();
        assertEquals(1, blurred.getTotalRejected(), "a frame taken at 3 rad/s is rejected");
        assertEquals(0, spinning.measurements);

        // The wheels report a turn onset the chassis has not made yet: the frame is sharp, and kept.
        SlippingDrive still = new SlippingDrive(3.0, 0.0);
        VisionSubsystem sharp = vision(still, new RecordingCamera());
        sharp.periodic();
        assertEquals(0, sharp.getTotalRejected());
        assertEquals(1, still.measurements);
    }

    @Test
    void megaTag2IsHandedTheGyroRate() {
        RecordingCamera camera = new RecordingCamera();
        vision(new SlippingDrive(0.5, -1.2), camera).periodic();
        assertEquals(Math.toDegrees(-1.2), camera.yawRateDegPerSec, 1e-9);
    }

    @Test
    void aSinkWithoutAGyroKeepsTheWheelsRate() {
        // Source-compatible: a sink that never heard of the gyro rate behaves exactly as before.
        WheelsOnly wheels = new WheelsOnly(3.0);
        RecordingCamera camera = new RecordingCamera();
        VisionSubsystem v = vision(wheels, camera);
        v.periodic();
        assertEquals(1, v.getTotalRejected(), "the wheels' 3 rad/s is all it has, and the gate uses it");
        assertEquals(Math.toDegrees(3.0), camera.yawRateDegPerSec, 1e-9);
    }

    @Test
    void aGyroWithNoNumberFallsBackToTheWheels() {
        RecordingCamera camera = new RecordingCamera();
        VisionSubsystem v = vision(new SlippingDrive(3.0, Double.NaN), camera);
        v.periodic();
        assertEquals(1, v.getTotalRejected());
        assertEquals(Math.toDegrees(3.0), camera.yawRateDegPerSec, 1e-9);
    }
}
