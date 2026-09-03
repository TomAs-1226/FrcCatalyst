package frc.lib.catalyst.subsystems.vision;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * That MegaTag2 knows whether anyone is telling it where the robot points.
 *
 * <p>MT2 resolves tag ambiguity against an externally supplied heading. If nothing supplies it the
 * camera does not fail and does not complain — it uses the last value it was given, which at boot
 * is zero. Every pose it then produces is confidently wrong by exactly the robot's true heading,
 * and it looks entirely plausible, so it goes into the estimator and drags the fused pose with it.
 *
 * <p>Confirmed on a Limelight 4 on the bench rather than reasoned about: an untouched camera reports
 * {@code botorient = {alpha: 0.001, imumode: 0, interpbotyaw: 0.0}}. imumode 0 means "use the
 * externally supplied yaw", and interpbotyaw stays 0.0 until something publishes one.
 *
 * <h2>What is deliberately not asserted here</h2>
 *
 * <p>That {@code getEstimatedPose()} returns empty while the yaw is stale. It returns empty with no
 * camera attached either, so on a desktop that assertion passes whether or not the guard exists —
 * it would read as coverage and be worth nothing. What is testable without hardware is the freshness
 * logic the guard consults, which is what these do.
 */
class MegaTag2YawGuardTest {

    private static final Transform3d MOUNT =
            new Transform3d(new Translation3d(0.2, 0.0, 0.5), new Rotation3d(0, 0, 0));

    @Test
    void aMegaTag2CameraStartsOutNotBeingFed() {
        LimelightSource cam = new LimelightSource("limelight-guardtest-a", MOUNT, true);

        assertFalse(cam.isReceivingRobotOrientation(),
                "a camera nobody has published yaw to must not claim it is being fed");
    }

    @Test
    void publishingYawMakesItFresh() {
        LimelightSource cam = new LimelightSource("limelight-guardtest-b", MOUNT, true);
        cam.setRobotOrientation(37.0, 0.0, 0.0, 0.0);

        assertTrue(cam.isReceivingRobotOrientation(),
                "yaw was just published; the camera should count as fed");
    }

    @Test
    void theSharedPublishCountsToo() {
        // A four-camera robot is expected to use the shared table. A camera must not look unwired
        // just because the team chose the efficient route.
        LimelightSource cam = new LimelightSource("limelight-guardtest-c", MOUNT, true);
        assertFalse(cam.isReceivingRobotOrientation());

        LimelightSource.setSharedRobotOrientation(90.0);

        assertTrue(cam.isReceivingRobotOrientation(),
                "the shared publish should satisfy every camera, not only the one it was called on");
    }

    @Test
    void megaTag1NeedsNoYawAndMustNeverBeBlocked() {
        // MT1 is self-contained. Blocking it for a missing yaw would break the one mode that does
        // not need one - and that is the tempting shape of a wrong fix here.
        LimelightSource cam = new LimelightSource("limelight-guardtest-d", MOUNT, false);

        assertTrue(cam.isReceivingRobotOrientation(),
                "MegaTag1 does not use robot yaw and must not be gated on it");
    }

    @Test
    void yawGoesStaleWhenTheRobotStopsPublishing() throws InterruptedException {
        // The case that matters mid-match: the feed existed and then stopped, because the subsystem
        // that published it died or was never scheduled after a mode change. A one-shot "has it
        // ever been fed" flag would pass every other test here and miss this entirely.
        LimelightSource cam = new LimelightSource("limelight-guardtest-e", MOUNT, true);
        cam.setRobotOrientation(10.0, 0.0, 0.0, 0.0);
        assertTrue(cam.isReceivingRobotOrientation());

        Thread.sleep(700);   // > ORIENTATION_STALE_SECONDS

        assertFalse(cam.isReceivingRobotOrientation(),
                "yaw published 0.7s ago is stale; MegaTag2 is resolving against an old heading");
    }
}
