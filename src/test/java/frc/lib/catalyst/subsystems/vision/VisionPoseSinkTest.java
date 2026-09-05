package frc.lib.catalyst.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * VisionSubsystem with no drivetrain: the whole pipeline runs into a StandaloneVisionPose.
 *
 * <p>Before this, {@code periodic()} returned on its first line without a SwerveSubsystem, so a
 * tank-drive team or a bench got no fusion, no per-camera telemetry and no health. The first test
 * here fails against that build.
 */
class VisionPoseSinkTest {

    private static SimCameraSource exact(String name, Pose2d[] truth) {
        return SimCameraSource.builder(name)
                .truePose(() -> truth[0])
                .translationStdDevMeters(0)
                .rotationStdDevDegrees(0)
                .latencySeconds(0.01)
                .build();
    }

    @Test
    void aStandaloneSinkReceivesWhatTheCamerasSee() {
        Pose2d[] truth = {new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(90))};
        StandaloneVisionPose sink = new StandaloneVisionPose();
        VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
                .addCamera(exact("sim", truth))
                .poseSink(sink)
                .build());

        vision.periodic();

        assertTrue(sink.hasPose());
        assertEquals(3.0, sink.getPose().getX(), 1e-9);
        assertEquals(2.0, sink.getPose().getY(), 1e-9);
        assertEquals(90.0, sink.getPose().getRotation().getDegrees(), 1e-9);
        assertEquals(1, vision.getTotalAccepted());
        assertEquals(VisionHealth.Level.OK, vision.health().level());
        assertEquals(VisionHealth.State.OK, vision.health().reports().get(0).state());
    }

    @Test
    void theFirstEstimateIsNotRejectedForBeingFarFromAnOriginNobodyIsAt() {
        // Far corner of the field, well beyond the 1 m "too far from current pose" gate. With no
        // pose yet the gate has nothing to measure from and must stand down; once anchored, it
        // must work again.
        Pose2d[] truth = {new Pose2d(14.0, 7.0, Rotation2d.fromDegrees(0))};
        StandaloneVisionPose sink = new StandaloneVisionPose();
        VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
                .addCamera(exact("sim", truth))
                .poseSink(sink)
                .maxAcceptableDistance(1.0)
                .build());

        vision.periodic();
        assertEquals(14.0, sink.getPose().getX(), 1e-9, "unanchored: accepted");

        truth[0] = new Pose2d(10.0, 7.0, Rotation2d.fromDegrees(0));
        vision.periodic();
        assertEquals(14.0, sink.getPose().getX(), 1e-9, "anchored: a 4 m jump is rejected");
        assertEquals(1, vision.getTotalRejected());
    }

    @Test
    void withoutAnySinkTheCamerasAreStillWatched() {
        Pose2d[] truth = {new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0))};
        VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
                .addCamera(exact("sim", truth))
                .build());

        vision.periodic();

        assertEquals(0, vision.getTotalAccepted(), "nothing to fuse into");
        assertEquals(0, vision.getTotalRejected(), "and nothing was rejected either");
        assertEquals(VisionHealth.State.OK, vision.health().reports().get(0).state(),
                "a camera delivering estimates is healthy whether or not anyone fuses them");
    }

    @Test
    void aCameraThatSeesNothingIsNotAFault() {
        Pose2d[] truth = {null};
        StandaloneVisionPose sink = new StandaloneVisionPose();
        VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
                .addCamera(exact("sim", truth))
                .poseSink(sink)
                .build());

        vision.periodic();

        assertFalse(sink.hasPose());
        assertEquals(VisionHealth.Level.OK, vision.health().level());
        assertEquals(VisionHealth.State.NO_TARGETS, vision.health().reports().get(0).state());
    }
}
