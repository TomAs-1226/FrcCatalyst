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

    /** As {@link #botpose}, with the pipeline latency as a parameter rather than fixed at 25 ms. */
    private static double[] botposeWithLatency(double x, double y, double yawDeg, int tags,
                                               double dist, double latencyMs) {
        return new double[] {x, y, 0.4, 0, 0, yawDeg, latencyMs, tags, 0.8, dist, 1.2};
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
        //
        // Asserted as a DIFFERENCE between two reads rather than as a wall-clock number. captureTime
        // also subtracts how long ago NetworkTables received the value, and that age is real elapsed
        // time this test cannot control - JIT, GC and NT round-trip all land in it. Pinning an
        // absolute figure made this a stopwatch on the JVM: it measured 0.048 s against a 0.045 s
        // ceiling and reported a vision bug that did not exist. Differencing cancels the elapsed
        // time and leaves exactly the quantity under test.
        String slow = fresh("lat-slow");
        LimelightSource slowSrc = new LimelightSource(slow, MOUNT, true);
        slowSrc.setRobotOrientation(0.0, 0.0, 0.0, 0.0);
        publish(slow, botposeWithLatency(1.0, 1.0, 0.0, 1, 2.0, 125.0), 1);
        double slowNow = org.wpilib.system.Timer.getTimestamp();
        var slowEst = slowSrc.getEstimatedPose().orElseThrow();
        double slowAge = slowNow - slowEst.timestampSeconds();

        String fast = fresh("lat-fast");
        LimelightSource fastSrc = new LimelightSource(fast, MOUNT, true);
        fastSrc.setRobotOrientation(0.0, 0.0, 0.0, 0.0);
        publish(fast, botposeWithLatency(1.0, 1.0, 0.0, 1, 2.0, 25.0), 1);
        double fastNow = org.wpilib.system.Timer.getTimestamp();
        var fastEst = fastSrc.getEstimatedPose().orElseThrow();
        double fastAge = fastNow - fastEst.timestampSeconds();

        assertTrue(fastEst.timestampSeconds() < fastNow,
                "the estimate must be stamped in the past, not at read time");
        // 100 ms more reported latency must move the timestamp 100 ms further back, and nothing else
        // about the two reads differs. 15 ms of slack for the elapsed-time noise that does not cancel.
        assertEquals(0.100, slowAge - fastAge, 0.015,
                "extra pipeline latency must push the capture time further into the past");
        assertTrue(fastAge >= 0.025,
                "an estimate cannot be newer than the latency the camera reported");
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
    void aTagTheCameraCannotPlaceIsNotTheCentreOfTheField() {
        // Seen on four Limelight 4s at once: a tag in view, tv = 1, one tag counted, and the pose
        // (8.2705, 4.0345, 0) to the millimetre - the centre of the field. The camera solved
        // nothing (its centre-origin botpose is all zeros) and the blue-origin copy is that plus
        // half a field. A robot on a bench was "measured" at the field's centre a hundred times a
        // second and everything real was rejected for being too far from it.
        String cam = fresh("k");
        NetworkTable t = publish(cam, botpose(8.2705, 4.0345, 0.0, 1, 0.6), 1);
        t.getEntry("botpose_orb").setDoubleArray(new double[] {0, 0, 0, 0, 0, 0, 23.5, 1, 0, 0.6, 1.2});

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        assertTrue(src.getEstimatedPose().isEmpty(), "a zero centre-origin solve is not a measurement");

        // The same camera with a real solve behind the blue-origin pose is read normally.
        t.getEntry("botpose_orb").setDoubleArray(new double[] {-5.02, 0.47, 0, 0, 0, 90.0, 23.5, 1, 0, 0.6, 1.2});
        publish(cam, botpose(3.25, 4.5, 90.0, 1, 0.6), 1);
        assertTrue(src.getEstimatedPose().isPresent(), "a placed tag is a measurement");
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

    /**
     * A camera whose MegaTag2 has no fix must still deliver its MegaTag1 one.
     *
     * <p>Measured on a Limelight 4 with a mapped tag in view, 1,540 frames across four
     * configurations - no heading fed, heading fed at 50 Hz, internal IMU, internal IMU seeded.
     * MegaTag1 solved in every frame of all four; MegaTag2 solved in none. The camera looked
     * perfectly healthy throughout: {@code tv} 1, a tag counted, every per-tag 3D pose correct.
     *
     * <p>The trap is that an unsolved MegaTag2 does not leave its topic empty. It publishes six
     * zeros, and the blue-origin copy is those zeros plus half a field - the exact centre of the
     * field, full length, finite, and indistinguishable from a real pose by any check except the
     * centre-origin array. So "prefer MegaTag2 when the array is present" preferred it forever, and
     * the fallback that existed for a missing topic never once fired.
     */
    @Test
    void anUnsolvedMegaTag2FallsBackToMegaTag1() {
        String cam = fresh("mt2unsolved");
        NetworkTable t = NetworkTableInstance.getDefault().getTable(cam);

        // MegaTag2: no fix. Centre-origin all zeros, blue-origin therefore the field's centre.
        t.getEntry("botpose_orb").setDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
        t.getEntry("botpose_orb_wpiblue")
                .setDoubleArray(botpose(8.2705, 4.0345, 0.0, 1, 0.36));
        // MegaTag1: a real fix on the same frame.
        t.getEntry("botpose").setDoubleArray(new double[] {4.06, 3.26, 0.9, 0, 0, 170.9});
        t.getEntry("botpose_wpiblue").setDoubleArray(botpose(12.33, 7.36, 170.9, 1, 0.36));
        t.getEntry("tv").setDouble(1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        var est = src.getEstimatedPose().orElseThrow(() -> new AssertionError(
                "MegaTag2 had no fix but MegaTag1 did; the reader must fall back rather than "
                        + "report nothing"));

        assertEquals(12.33, est.pose().getX(), 1e-6, "should be the MegaTag1 pose");
        assertEquals(7.36, est.pose().getY(), 1e-6, "should be the MegaTag1 pose");
        assertEquals(170.9, est.pose().getRotation().getDegrees(), 1e-6);
    }

    /**
     * ...and the centre-of-field pose is still rejected when it is all the camera has.
     *
     * <p>The pair matters. A fallback that fires whenever MegaTag2 is unplaceable is only correct if
     * the thing it falls back to is checked just as hard - otherwise this fix would have traded a
     * camera that reports nothing for one that reports the middle of the field.
     */
    @Test
    void withNoMegaTag1EitherTheCentreOfTheFieldIsStillRejected() {
        String cam = fresh("mt2unsolved2");
        NetworkTable t = NetworkTableInstance.getDefault().getTable(cam);

        t.getEntry("botpose_orb").setDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
        t.getEntry("botpose_orb_wpiblue").setDoubleArray(botpose(8.2705, 4.0345, 0.0, 1, 0.36));
        t.getEntry("botpose").setDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
        t.getEntry("botpose_wpiblue").setDoubleArray(botpose(8.2705, 4.0345, 0.0, 1, 0.36));
        t.getEntry("tv").setDouble(1);

        LimelightSource src = new LimelightSource(cam, MOUNT, true);
        src.setRobotOrientation(0.0, 0.0, 0.0, 0.0);

        assertTrue(src.getEstimatedPose().isEmpty(),
                "neither solve had a fix; the centre of the field is not a measurement");

        // And the reader is not wedged: a real pose on the next frame is accepted.
        t.getEntry("botpose").setDoubleArray(new double[] {4.06, 3.26, 0.9, 0, 0, 170.9});
        t.getEntry("botpose_wpiblue").setDoubleArray(botpose(12.33, 7.36, 170.9, 1, 0.36));
        var est = src.getEstimatedPose().orElseThrow(
                () -> new AssertionError("a good MegaTag1 pose after a rejection must be accepted"));
        assertEquals(12.33, est.pose().getX(), 1e-6);
    }
}
