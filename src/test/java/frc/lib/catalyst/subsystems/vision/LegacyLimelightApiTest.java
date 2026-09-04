package frc.lib.catalyst.subsystems.vision;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Reading a Limelight that speaks the older per-key API.
 *
 * <p>This is not a legacy nicety, it is the only path that works on hardware today. Measured on two
 * Limelight 4s: the newest camera image Limelight publishes is 2026.1, no 2027 image exists for any
 * camera, and a 2026 camera publishes fifty per-key topics and no {@code results_msgpack} — which is
 * the single topic LimelightLib subscribes to. Against a real camera, the modern path reported
 * {@code NO_DATA} on 1976 frames where the camera was plainly tracking a tag.
 *
 * <p>The tests publish the same topics a 2026 camera publishes, to the default NetworkTables
 * instance, and read them back through {@link LimelightSource}. That exercises the real parsing:
 * array layout, the all-zero sentinel, the non-finite guards and the path choice. What it cannot
 * exercise is the camera's own behaviour, which is what the bench harness was for.
 *
 * <p><b>Every rejection test here ends by proving the reader then accepts a good pose.</b> Without
 * that, each one asserts only that {@code getEstimatedPose()} came back empty - which is equally
 * true if the per-key path does not exist at all. Six of these were written that way first and six
 * of them passed with the whole feature deleted. An assertion that survives the removal of the thing
 * it tests is not coverage, it just looks like it.
 */
class LegacyLimelightApiTest {

    private static final Transform3d MOUNT =
            new Transform3d(new Translation3d(0.2, 0.0, 0.5), new Rotation3d(0, 0, 0));

    /** [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgTagDist, avgTagArea] */
    private static double[] botpose(double x, double y, double yawDeg, int tags, double dist) {
        return new double[] {x, y, 0.4, 0, 0, yawDeg, 25.0, tags, 0.8, dist, 1.2};
    }

    private String fresh(String suffix) {
        return "limelight-legacytest-" + suffix;
    }

    private NetworkTable publish(String camera, double[] pose, double tv) {
        NetworkTable t = NetworkTableInstance.getDefault().getTable(camera);
        t.getEntry("botpose_orb_wpiblue").setDoubleArray(pose);
        t.getEntry("tv").setDouble(tv);
        return t;
    }

    @Test
    void aPoseFromTheOldApiIsReadCorrectly() {
        String cam = fresh("a");
        publish(cam, botpose(3.25, 4.5, 90.0, 2, 2.75), 1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(90.0, 0.0, 0.0, 0.0);

        var est = src.getEstimatedPose().orElseThrow(
                () -> new AssertionError("no estimate from a camera that is publishing a pose"));

        assertEquals(3.25, est.pose().getX(), 1e-9);
        assertEquals(4.5, est.pose().getY(), 1e-9);
        assertEquals(90.0, est.pose().getRotation().getDegrees(), 1e-9);
        assertEquals(2, est.tagCount());
        assertEquals(2.75, est.averageTagDistance(), 1e-9);
        assertTrue(src.isUsingLegacyApi(), "should have settled on the per-key API");
    }

    @Test
    void theLatencyIsSubtractedFromTheTimestamp() {
        // 25 ms of latency means the measurement describes the robot 25 ms ago. Handing a pose
        // estimator "now" for a measurement that is a loop old is a quiet, permanent bias.
        String cam = fresh("b");
        publish(cam, botpose(1.0, 1.0, 0.0, 1, 2.0), 1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        double now = org.wpilib.system.Timer.getTimestamp();
        var est = src.getEstimatedPose().orElseThrow();

        assertTrue(est.timestampSeconds() < now,
                "the estimate must be stamped in the past, not at read time");
        assertEquals(0.025, now - est.timestampSeconds(), 0.02);
    }

    @Test
    void noTargetMeansNoEstimate() {
        String cam = fresh("c");
        publish(cam, botpose(3.0, 4.0, 0.0, 1, 2.0), 0);   // tv = 0

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        assertTrue(src.getEstimatedPose().isEmpty(), "tv=0 means nothing is in view");
        assertAcceptsAGoodPose(src, cam, "tv=0 rejection");
    }

    @Test
    void theAllZeroArrayIsNotTheOrigin() {
        // A camera with nothing in view publishes zeros rather than clearing the topic, so
        // "the origin, right now" is what no-detection looks like on this API. Taken at face value
        // it teleports the fused pose to the blue-alliance corner - and tv can lag a frame, so
        // the zero-array check has to stand on its own rather than lean on tv.
        String cam = fresh("d");
        publish(cam, new double[] {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}, 1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        assertTrue(src.getEstimatedPose().isEmpty(), "an all-zero botpose is not a measurement");
        assertAcceptsAGoodPose(src, cam, "all-zero rejection");
    }

    @Test
    void aNonFinitePoseIsRefused() {
        String cam = fresh("e");
        publish(cam, botpose(Double.NaN, 4.0, 0.0, 1, 2.0), 1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        assertTrue(src.getEstimatedPose().isEmpty(),
                "a NaN coordinate must not reach a pose estimator");
        assertAcceptsAGoodPose(src, cam, "NaN rejection");
    }

    @Test
    void zeroFieldedTagsMeansNoAnswer() {
        // The camera saw something it could not place on the field. Whatever it published was
        // computed from nothing.
        String cam = fresh("f");
        publish(cam, botpose(3.0, 4.0, 0.0, 0, 2.0), 1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        assertTrue(src.getEstimatedPose().isEmpty());
        assertAcceptsAGoodPose(src, cam, "zero-fielded-tags rejection");
    }

    @Test
    void theMegaTag2YawGuardStillAppliesOnThisPath() throws InterruptedException {
        // The old path is not an escape hatch from the yaw requirement: botpose_orb_wpiblue IS
        // MegaTag2, and it is just as wrong without a heading here as through LimelightLib.
        //
        // The wait is for the shared-orientation clock, which is static because one shared publish
        // really does feed every camera on a robot. In a test JVM that means a sibling class
        // publishing yaw makes this camera look fed, and the assertion below would fail for a
        // reason unrelated to the code under test. This coupling runs both ways and bit both
        // classes once each before it was understood.
        Thread.sleep(700);
        String cam = fresh("g");
        publish(cam, botpose(3.0, 4.0, 0.0, 2, 2.0), 1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        // deliberately no setRobotOrientation

        assertFalse(src.isReceivingRobotOrientation());
        assertTrue(src.getEstimatedPose().isEmpty(),
                "MegaTag2 with no heading is wrong on either API, and must be refused on both");

        // ...and feeding the heading is what unblocks it, which is what makes the assertion above
        // about the guard rather than about the path being absent.
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);
        assertTrue(src.getEstimatedPose().isPresent(),
                "with a heading published, the same pose should be accepted");
    }

    @Test
    void aCameraPublishingNothingDoesNotGetLatchedToEitherPath() {
        // A camera that boots late must still be picked up. Deciding "legacy" the first time it is
        // silent would strand a modern camera on the wrong reader for the rest of the match.
        String cam = fresh("h");
        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        assertTrue(src.getEstimatedPose().isEmpty());
        assertFalse(src.isUsingLegacyApi(), "the choice must stay open while the camera is silent");

        // The camera turns up late. It must be read, not written off.
        publish(cam, botpose(2.0, 3.0, 45.0, 2, 1.5), 1);
        assertTrue(src.getEstimatedPose().isPresent(),
                "a camera that starts publishing later must still be picked up");
        assertTrue(src.isUsingLegacyApi());
    }

    @Test
    void theLegacyPathDoesNotClaimToBePrefiltered() {
        // isPrefiltered() tells VisionSubsystem "the camera already applied its own gates, do not
        // repeat them". That is true on the modern path, where LimelightLib's accepted queue applies
        // tag-count, ambiguity, distance, area and field-bounds rules using the raw detections.
        //
        // It is false here. The per-key path reads botpose_* straight off NetworkTables with nothing
        // applied. Claiming prefiltered would make VisionSubsystem skip ITS checks too, and the one
        // that matters is field bounds - a pose off the end of the field would go into the estimator
        // with nothing standing in its way.
        //
        // This method returned an unconditional true until the per-key reader was added, at which
        // point it quietly became wrong for every camera on a shipping image.
        String cam = fresh("prefilter");
        publish(cam, botpose(3.0, 4.0, 0.0, 2, 2.0), 1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);
        assertTrue(src.getEstimatedPose().isPresent(), "precondition: the legacy path is in use");
        assertTrue(src.isUsingLegacyApi());

        assertFalse(src.isPrefiltered(),
                "the per-key path applies no camera-side gates, so Catalyst's must run");
    }

    @Test
    void aModernPathCameraStillReportsPrefiltered() {
        // The other half, and the reason this is a condition rather than a flat false: on the modern
        // path the camera really has filtered, and making Catalyst re-filter with a second set of
        // thresholds nobody tuned would be its own bug.
        LimelightSource src = new LimelightSource(fresh("nevertalked"), MOUNT, true);

        assertFalse(src.isUsingLegacyApi(), "nothing published, so no path chosen");
        assertTrue(src.isPrefiltered(),
                "a source that has not settled on the per-key path must not claim to be unfiltered");
    }

    // --- frame identity -------------------------------------------------------

    @Test
    void thesameFrameIsNotHandedOverTwice() {
        // The duplicate-injection case, and it happens in normal operation rather than in faults:
        // the robot loop runs at 50 Hz and an AprilTag pipeline does not, so the same botpose array
        // is read several times per unique frame. Stamped at read time, each re-read looked like an
        // independent measurement and the pose estimator weighted it as one - ending up several
        // times more confident in vision than the evidence supports, on every camera at once.
        String cam = fresh("dup");
        publish(cam, botpose(3.0, 4.0, 0.0, 2, 2.0), 1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        assertTrue(src.getEstimatedPose().isPresent(), "the first read is a real frame");
        assertTrue(src.getEstimatedPose().isEmpty(),
                "nothing was republished, so there is no new evidence to hand over");
        assertTrue(src.getEstimatedPose().isEmpty(), "and still none");
    }

    @Test
    void aNewFrameIsHandedOverAgain() {
        String cam = fresh("newframe");
        publish(cam, botpose(3.0, 4.0, 0.0, 2, 2.0), 1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);
        assertTrue(src.getEstimatedPose().isPresent());
        assertTrue(src.getEstimatedPose().isEmpty());

        publish(cam, botpose(3.5, 4.25, 10.0, 2, 2.1), 1);

        var est = src.getEstimatedPose().orElseThrow(
                () -> new AssertionError("a genuinely new frame must be delivered"));
        assertEquals(3.5, est.pose().getX(), 1e-9);
    }

    @Test
    void aRejectedFrameIsStillConsumed() {
        // A frame refused for tv=0 or a bad pose is not new evidence next loop either. Leaving it
        // unconsumed would mean re-examining the same bad frame forever.
        String cam = fresh("consumed");
        publish(cam, botpose(3.0, 4.0, 0.0, 0, 2.0), 1);   // tagCount 0 -> rejected

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);
        assertTrue(src.getEstimatedPose().isEmpty(), "zero fielded tags is refused");

        publish(cam, botpose(6.0, 1.0, 0.0, 2, 2.0), 1);
        assertTrue(src.getEstimatedPose().isPresent(),
                "and the next real frame still gets through");
    }

    @Test
    void staleness_isMeasurableAndWasNotBefore() {
        // The gate VisionSubsystem thought it had. With the timestamp built as (now - latency), its
        // age calculation reduced algebraically to the camera's own self-reported latency - a small
        // constant - so StaleData could not fire on this path however old the data was.
        String cam = fresh("stale");
        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        assertTrue(src.secondsSinceLastFrame().isEmpty(),
                "a camera that has never produced a frame has no age, rather than an age of zero");

        publish(cam, botpose(3.0, 4.0, 0.0, 2, 2.0), 1);
        assertTrue(src.getEstimatedPose().isPresent());

        double age = src.secondsSinceLastFrame().orElseThrow();
        assertTrue(age >= 0 && age < 1.0, "just-published frame should read as fresh, got " + age);
    }

    @Test
    void theEstimateIsStampedFromThePublishTimeNotTheReadTime() {
        // What makes staleness real. The stamp must trail now by the frame's NetworkTables age plus
        // the camera's reported pipeline latency - not be recomputed as "now" on every read.
        String cam = fresh("stamp");
        publish(cam, botpose(3.0, 4.0, 0.0, 2, 2.0), 1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        double before = org.wpilib.system.Timer.getTimestamp();
        var est = src.getEstimatedPose().orElseThrow();

        assertTrue(est.timestampSeconds() <= before,
                "a measurement cannot have been captured after it was read");
        assertTrue(before - est.timestampSeconds() >= 0.025 - 1e-6,
                "at least the 25 ms of reported pipeline latency should be subtracted");
    }

    /**
     * The control that makes a rejection test mean something.
     *
     * <p>Publishes a plainly good pose on the same source and requires it through. Without this, a
     * test that only asserts "empty" passes just as happily when the per-key path has been deleted
     * outright, so it would be measuring nothing.
     */
    private void assertAcceptsAGoodPose(LimelightSource src, String camera, String what) {
        publish(camera, botpose(5.0, 2.0, 30.0, 2, 1.9), 1);
        src.setRobotOrientation(30.0, 0.0, 0.0, 0.0);
        var est = src.getEstimatedPose();
        assertTrue(est.isPresent(),
                what + " must be specific: a good pose on the same camera should still be read");
        assertEquals(5.0, est.orElseThrow().pose().getX(), 1e-9);
    }
}
