package frc.lib.catalyst.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.system.Timer;

/** A queued solve may pass image geometry gates but be too old to fuse or to seed odometry. */
class VisionFrameAgeTest {
    private static final Pose2d POSE = new Pose2d(3, 2, Rotation2d.ZERO);

    private static final class Sink implements VisionPoseSink {
        int fused;
        int seeded;
        double timestamp;
        @Override public Pose2d getPose() { return POSE; }
        @Override public ChassisVelocities getChassisSpeeds() { return new ChassisVelocities(); }
        @Override public void addVisionMeasurement(Pose2d pose, double ts, Matrix<N3, N1> stdDevs) {
            fused++;
            timestamp = ts;
        }
        @Override public void resetPose(Pose2d pose, double ts) { seeded++; }
    }

    private static CameraSource camera(boolean prefiltered, double timestamp) {
        return new CameraSource() {
            @Override public String getName() { return "queued-frame"; }
            @Override public boolean isPrefiltered() { return prefiltered; }
            @Override public Optional<PoseEstimate> getEstimatedPose() {
                return Optional.of(new PoseEstimate(POSE, timestamp, 2, 2, 0));
            }
        };
    }

    private static VisionSubsystem vision(Sink sink, boolean prefiltered, double timestamp,
            boolean seed, double maxAge) {
        return new VisionSubsystem(VisionConfig.builder().addCamera(camera(prefiltered, timestamp))
                .poseSink(sink).seedFromVision(seed).maxLatency(maxAge).build());
    }

    @Test
    void geometryPrefilteringCannotBypassTheDefaultHalfSecondAgeLimit() {
        for (boolean prefiltered : new boolean[] {false, true}) {
            Sink sink = new Sink();
            var v = vision(sink, prefiltered, Timer.getTimestamp() - .6, false, .5);
            v.periodic();
            assertEquals(1, v.getTotalRejected());
            assertEquals(0, sink.fused);
        }
    }

    @Test
    void anOldPrefilteredFrameCannotSeedTheRobotOrMarkItFresh() {
        Sink sink = new Sink();
        var v = vision(sink, true, Timer.getTimestamp() - 1, true, .5);
        v.periodic();
        assertFalse(v.isSeeded());
        assertEquals(0, sink.seeded);
        assertEquals(0, sink.fused);
        assertEquals(1, v.getTotalRejected());
    }

    @Test
    void futureAndNonfiniteFramesCannotReachThePoseSinkOnEitherApi() {
        for (boolean prefiltered : new boolean[] {false, true}) {
            for (double ts : new double[] {Timer.getTimestamp() + 1, Double.NaN, Double.POSITIVE_INFINITY}) {
                Sink sink = new Sink();
                var v = vision(sink, prefiltered, ts, false, .5);
                v.periodic();
                assertEquals(1, v.getTotalRejected());
                assertEquals(0, sink.fused);
            }
        }
    }

    @Test
    void theConfiguredAgeLimitAppliesToBothApiPathsAndFreshFramesKeepTheirCaptureTime() {
        for (boolean prefiltered : new boolean[] {false, true}) {
            Sink stale = new Sink();
            vision(stale, prefiltered, Timer.getTimestamp() - .2, false, .1).periodic();
            assertEquals(0, stale.fused);
            Sink fresh = new Sink();
            double timestamp = Timer.getTimestamp() - .05;
            var v = vision(fresh, prefiltered, timestamp, false, .5);
            v.periodic();
            assertEquals(1, fresh.fused);
            assertEquals(timestamp, fresh.timestamp, 1e-9);
            assertEquals(0, v.getTotalRejected());
        }
    }
}
