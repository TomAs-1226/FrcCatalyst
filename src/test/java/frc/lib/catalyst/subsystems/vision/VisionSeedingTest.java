package frc.lib.catalyst.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

/**
 * A drivetrain's pose always "exists", so before this every estimate on a robot that had been
 * carried onto the field failed the too-far gate against the origin it booted at. Seen on the
 * Catalyst X1 with four cameras looking straight at tags. The first test fails against that build.
 */
class VisionSeedingTest {

    /** Odometry that is stuck: measurements do not move it, only a reset does. */
    static final class StuckOdometry implements VisionPoseSink {
        Pose2d pose = Pose2d.ZERO;
        int measurements = 0;
        int resets = 0;

        @Override public Pose2d getPose() { return pose; }
        @Override public ChassisVelocities getChassisSpeeds() { return new ChassisVelocities(); }
        @Override public void addVisionMeasurement(Pose2d p, double ts, Matrix<N3, N1> stdDevs) { measurements++; }
        @Override public void resetPose(Pose2d p, double ts) { resets++; pose = p; }
    }

    private static SimCameraSource camera(Pose2d[] truth, int tags, double tagDistance) {
        return SimCameraSource.builder("sim")
                .truePose(() -> truth[0])
                .translationStdDevMeters(0)
                .rotationStdDevDegrees(0)
                .latencySeconds(0.01)
                .tagCount(tags)
                .averageTagDistance(tagDistance)
                .build();
    }

    @Test
    void aDrivetrainAtTheOriginIsSeededByTheFirstGoodEstimate() {
        Pose2d[] truth = {new Pose2d(12.0, 6.0, Rotation2d.fromDegrees(45))};
        StuckOdometry odometry = new StuckOdometry();
        VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
                .addCamera(camera(truth, 2, 2.0))
                .poseSink(odometry)
                .maxAcceptableDistance(2.0)
                .build());

        assertFalse(vision.isSeeded());
        vision.periodic();

        assertTrue(vision.isSeeded());
        assertEquals(1, odometry.resets, "the seed replaces the pose");
        assertEquals(0, odometry.measurements, "and is not also fused");
        assertEquals(12.0, odometry.pose.getX(), 1e-9);
        assertEquals(45.0, odometry.pose.getRotation().getDegrees(), 1e-9);
        assertEquals(0, vision.getTotalRejected(), "nothing was 'too far' from an origin nobody was at");

        // Now anchored: the next frame is close to the seed and is fused normally.
        truth[0] = new Pose2d(12.3, 6.0, Rotation2d.fromDegrees(45));
        vision.periodic();
        assertEquals(1, odometry.resets);
        assertEquals(1, odometry.measurements);
    }

    @Test
    void aFarSingleTagIsFusedButDoesNotSeed() {
        Pose2d[] truth = {new Pose2d(12.0, 6.0, Rotation2d.ZERO)};
        StuckOdometry odometry = new StuckOdometry();
        VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
                .addCamera(camera(truth, 1, 5.0))
                .poseSink(odometry)
                .maxAcceptableDistance(2.0)
                .build());

        vision.periodic();

        assertFalse(vision.isSeeded(), "one tag five metres away can put the robot on its wrong side");
        assertEquals(0, odometry.resets);
        assertEquals(1, odometry.measurements, "still fused - the gate stands down while unseeded");
    }

    @Test
    void persistentDisagreementReanchorsThePose() throws InterruptedException {
        Pose2d[] truth = {new Pose2d(5.0, 5.0, Rotation2d.ZERO)};
        StuckOdometry odometry = new StuckOdometry();
        VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
                .addCamera(camera(truth, 2, 2.0))
                .poseSink(odometry)
                .maxAcceptableDistance(2.0)
                .reanchorAfterSeconds(0.05)
                .build());

        vision.periodic();
        assertEquals(1, odometry.resets, "seeded");

        // Odometry goes wrong: a slip, a drift, a bad reset. The cameras keep seeing (5, 5).
        odometry.pose = Pose2d.ZERO;
        vision.periodic();
        assertEquals(1, vision.getTotalRejected(), "too far from where odometry thinks it is");
        assertEquals(1, odometry.resets, "one frame of disagreement is not enough");

        Thread.sleep(80);
        vision.periodic();
        assertEquals(2, odometry.resets, "after the window, the cameras win");
        assertEquals(5.0, odometry.pose.getX(), 1e-9);
        assertEquals(1, vision.getReanchorCount());
    }

    @Test
    void seedingOffKeepsTheOldRule() {
        Pose2d[] truth = {new Pose2d(12.0, 6.0, Rotation2d.ZERO)};
        StuckOdometry odometry = new StuckOdometry();
        VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
                .addCamera(camera(truth, 2, 2.0))
                .poseSink(odometry)
                .maxAcceptableDistance(2.0)
                .seedFromVision(false)
                .build());

        vision.periodic();

        assertTrue(vision.isSeeded(), "nothing to wait for");
        assertEquals(0, odometry.resets);
        assertEquals(1, vision.getTotalRejected(), "the gate measures from the origin, as before");
    }
}
