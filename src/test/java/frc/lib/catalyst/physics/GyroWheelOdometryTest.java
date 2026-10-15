package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import frc.lib.catalyst.physics.estimation.GyroWheelOdometry;
import frc.lib.catalyst.physics.estimation.GyroWheelOdometry.Estimate;
import frc.lib.catalyst.physics.observation.PoseObservation;

class GyroWheelOdometryTest {
    private static final Translation2d[] LOCATIONS = {
        new Translation2d(.3, .3), new Translation2d(.3, -.3),
        new Translation2d(-.3, .3), new Translation2d(-.3, -.3)
    };

    /** Synthesizes integrated module distances for a body twist, not a pose/velocity shortcut. */
    private static final class Rig {
        final GyroWheelOdometry odometry;
        final double[] distances = new double[4];
        final SwerveModulePosition[] modules = new SwerveModulePosition[4];
        double timestamp;
        double rawHeading;
        Pose2d nativePose = Pose2d.ZERO;
        Rig() { this(new GyroWheelOdometry(LOCATIONS)); }
        Rig(GyroWheelOdometry odometry) {
            this.odometry = odometry;
            for (int i = 0; i < 4; i++) modules[i] = new SwerveModulePosition();
        }
        Estimate baseline() {
            return odometry.update(timestamp, nativePose, new Rotation2d(rawHeading), modules);
        }
        Estimate step(double dt, double dx, double dy, double dtheta, double... errors) {
            for (int i = 0; i < 4; i++) {
                double x = dx - dtheta * LOCATIONS[i].getY();
                double y = dy + dtheta * LOCATIONS[i].getX();
                double distance = Math.hypot(x, y);
                distances[i] += distance + (i < errors.length ? errors[i] : 0);
                modules[i] = new SwerveModulePosition(distances[i],
                        distance == 0 ? Rotation2d.ZERO : new Rotation2d(x, y));
            }
            timestamp += dt;
            rawHeading += dtheta;
            nativePose = nativePose.plus(new Twist2d(dx, dy, dtheta).exp());
            return baseline();
        }
    }

    private static void poseEquals(Pose2d expected, Pose2d actual) {
        assertEquals(expected.getX(), actual.getX(), 1e-9);
        assertEquals(expected.getY(), actual.getY(), 1e-9);
        assertEquals(0, expected.getRotation().minus(actual.getRotation()).getRadians(), 1e-9);
    }

    @Test void baselineAnchorsNativePoseWithoutClaimingStationaryVelocity() {
        var rig = new Rig();
        rig.nativePose = new Pose2d(2, 3, new Rotation2d(.7));
        rig.rawHeading = -1;
        Estimate baseline = rig.baseline();
        assertTrue(baseline.valid());
        assertFalse(baseline.initializedVelocity());
        assertEquals(0, baseline.inliers());
        assertTrue(baseline.reason().contains("velocity unavailable"));
        poseEquals(rig.nativePose, baseline.pose());
    }

    @Test void coherentTranslationAndRotationUseSe2RatherThanFieldEulerIntegration() {
        var rig = new Rig();
        rig.baseline();
        Pose2d expected = Pose2d.ZERO;
        for (int i = 0; i < 30; i++) {
            Estimate result = rig.step(.02, .04, .01, .025);
            expected = expected.plus(new Twist2d(.04, .01, .025).exp());
            assertTrue(result.valid());
            assertTrue(result.initializedVelocity());
            assertEquals(4, result.inliers());
            assertEquals(-1, result.rejectedModule());
            assertEquals(2, result.robotRelativeVelocity().vx, 1e-9);
            assertEquals(.5, result.robotRelativeVelocity().vy, 1e-9);
            assertEquals(1.25, result.robotRelativeVelocity().omega, 1e-9);
            poseEquals(expected, result.pose());
        }
    }

    @Test void oneWheelDriftIsExcludedAndRecoveryDoesNotRestoreItsAccumulatedError() {
        var rig = new Rig();
        rig.baseline();
        rig.step(.02, .02, 0, 0);
        for (int i = 0; i < 5; i++) {
            // Native odometry can drift arbitrarily; this helper must retain its own pose offset.
            rig.nativePose = new Pose2d(50, 0, Rotation2d.ZERO);
            Estimate result = rig.step(.02, .02, 0, 0, .10);
            assertTrue(result.valid());
            assertEquals(3, result.inliers());
            assertEquals(0, result.rejectedModule());
            assertEquals(.04 + i * .02, result.pose().getX(), 1e-9);
        }
        Estimate recovered = rig.step(.02, .02, 0, 0);
        assertTrue(recovered.valid());
        assertEquals(4, recovered.inliers());
        assertEquals(.14, recovered.pose().getX(), 1e-9);
        assertEquals(1, recovered.robotRelativeVelocity().vx, 1e-9);
        assertTrue(rig.nativePose.getX() > 49);
    }

    @Test void twoDisagreeingPairsCannotProduceAThreeModuleConsensus() {
        var rig = new Rig();
        rig.baseline();
        Estimate rejected = rig.step(.02, .02, 0, 0, .05, .05);
        assertFalse(rejected.valid());
        assertFalse(rejected.initializedVelocity());
        assertEquals(0, rejected.pose().getX(), 1e-9);
        assertTrue(rejected.reason().contains("fewer than three"));
        Estimate next = rig.step(.02, .02, 0, 0);
        assertTrue(next.valid());
        assertEquals(.02, next.pose().getX(), 1e-9,
                "a later valid interval must not replay travel from the rejected interval");
    }

    @Test void tiedThreeModuleCliquesAreRejectedInsteadOfSelectingArbitrarily() {
        var rig = new Rig();
        rig.baseline();
        Estimate result = rig.step(.02, .02, 0, 0, -.003, 0, 0, .003);
        assertFalse(result.valid());
        assertTrue(result.reason().contains("ambiguous"));
        assertEquals(0, result.pose().getX(), 1e-9);
    }

    @Test void pairwiseRuleRejectsANonTransitiveNeighborChain() {
        var rig = new Rig();
        rig.baseline();
        Estimate result = rig.step(.02, .02, 0, 0, -.0045, -.0015, .0015, .0045);
        assertFalse(result.valid());
        assertTrue(result.reason().contains("fewer than three"));
    }

    @Test void commonWheelspinIsExplicitlyNotCertifiedAsGroundTruth() {
        var rig = new Rig();
        rig.baseline();
        Estimate result = rig.step(.02, .06, 0, 0);
        assertTrue(result.valid(), "encoder/gyro consensus cannot distinguish common wheelspin");
        assertEquals(.06, result.pose().getX(), 1e-9);
        assertTrue(result.reason().contains("ground motion unverified"));
        // The caller's IMU motion observer must make the traction decision independently.
    }

    @Test void wrappedRawGyroUsesTheShortMatchedHeadingInterval() {
        var rig = new Rig();
        rig.rawHeading = Math.PI - .01;
        rig.baseline();
        rig.step(.02, .02, 0, .02);
        // Feed the identical captured angle using the other +/-pi representation.
        rig.rawHeading -= 2 * Math.PI;
        Estimate next = rig.step(.02, .02, 0, .02);
        poseEquals(Pose2d.ZERO.plus(new Twist2d(.04, 0, .04).exp()), next.pose());
        assertEquals(1, next.robotRelativeVelocity().omega, 1e-9);
    }

    @Test void delayedPositionFixCorrectsCaptureInnovationWithoutRewindingMotionOrHeading() {
        var rig = new Rig();
        rig.baseline();
        rig.step(.02, .02, 0, 0);
        rig.step(.02, .02, 0, 0);
        Estimate before = rig.step(.02, .02, 0, 0);
        // At .03 s the predicted x is .03; observed .23 gives +.20 innovation, gain .5.
        assertTrue(rig.odometry.observe(new PoseObservation(
                new Pose2d(.23, .10, new Rotation2d(2)), .03, .10, "camera")));
        Estimate corrected = rig.odometry.estimate();
        assertEquals(.16, corrected.pose().getX(), 1e-9);
        assertEquals(.05, corrected.pose().getY(), 1e-9);
        assertEquals(0, corrected.pose().getRotation().getRadians(), 1e-9);
        assertEquals(before.robotRelativeVelocity(), corrected.robotRelativeVelocity());
        assertTrue(corrected.initializedVelocity());
        // Historical correction is retained so the same innovation is not applied again.
        assertTrue(rig.odometry.observe(new PoseObservation(
                new Pose2d(.15, .05, Rotation2d.ZERO), .05, .10, "camera")));
        assertEquals(.16, rig.odometry.estimate().pose().getX(), 1e-9);
        assertEquals(.18, rig.step(.02, .02, 0, 0).pose().getX(), 1e-9);
    }

    @Test void lowerObservationUncertaintyHasMoreInfluenceThanHigherUncertainty() {
        var low = new Rig();
        var high = new Rig();
        low.baseline(); high.baseline();
        low.step(.02, .02, 0, 0); high.step(.02, .02, 0, 0);
        assertTrue(low.odometry.observe(new PoseObservation(
                new Pose2d(.22, 0, Rotation2d.ZERO), .02, .02, "camera")));
        assertTrue(high.odometry.observe(new PoseObservation(
                new Pose2d(.22, 0, Rotation2d.ZERO), .02, .40, "camera")));
        assertEquals(.02 + .20 * .10 / .12, low.odometry.estimate().pose().getX(), 1e-9);
        assertEquals(.02 + .20 * .10 / .50, high.odometry.estimate().pose().getX(), 1e-9);
        assertTrue(low.odometry.estimate().pose().getX() > high.odometry.estimate().pose().getX());
    }

    @Test void fixesRejectFutureDuplicateOlderOutlierAndUnusableUncertainty() {
        var rig = new Rig();
        rig.baseline();
        rig.step(.02, .02, 0, 0);
        rig.step(.02, .02, 0, 0);
        Pose2d fix = new Pose2d(.03, 0, Rotation2d.ZERO);
        assertFalse(rig.odometry.observe(new PoseObservation(fix, .05, .1, "camera")));
        assertFalse(rig.odometry.observe(new PoseObservation(
                new Pose2d(2, 0, Rotation2d.ZERO), .03, .1, "camera")));
        assertFalse(rig.odometry.observe(new PoseObservation(fix, .03, .51, "camera")));
        assertFalse(rig.odometry.observe(new PoseObservation(fix, .03, Double.POSITIVE_INFINITY, "camera")));
        assertFalse(rig.odometry.observe(new PoseObservation(fix, Double.NaN, .1, "camera")));
        assertTrue(rig.odometry.observe(new PoseObservation(fix, .03, .1, "camera")));
        assertFalse(rig.odometry.observe(new PoseObservation(fix, .03, .1, "other-camera")));
        assertFalse(rig.odometry.observe(new PoseObservation(fix, .02, .1, "camera")));
        for (int i = 0; i < 30; i++) rig.step(.02, .02, 0, 0);
        assertFalse(rig.odometry.observe(new PoseObservation(fix, .03, .1, "camera")));
    }

    @Test void resetRejectsCachedPreResetFramesAndRequiresANewIntervalForVelocity() {
        var rig = new Rig();
        rig.baseline();
        rig.step(.02, .02, 0, 0);
        Pose2d reset = new Pose2d(3, 4, new Rotation2d(.5));
        rig.odometry.resetTo(reset, .05);
        assertFalse(rig.odometry.estimate().valid());
        Estimate cached = rig.step(.02, .02, 0, 0);
        poseEquals(reset, cached.pose());
        assertFalse(cached.valid());
        assertFalse(rig.odometry.observe(new PoseObservation(reset, .04, .1, "camera")));
        Estimate baseline = rig.step(.02, .02, 0, 0);
        assertTrue(baseline.valid());
        assertFalse(baseline.initializedVelocity());
        poseEquals(reset, baseline.pose());
        Estimate moved = rig.step(.02, .02, 0, .01);
        assertTrue(moved.initializedVelocity());
        poseEquals(reset.plus(new Twist2d(.02, 0, .01).exp()), moved.pose());
    }

    @Test void gapHoldsPoseAndDiscardsUnknownTravelWithoutInterpolatingAcrossIt() {
        var rig = new Rig();
        rig.baseline();
        rig.step(.02, .02, 0, 0);
        Estimate gap = rig.step(.20, .50, 0, .3);
        assertFalse(gap.valid());
        assertFalse(gap.initializedVelocity());
        poseEquals(new Pose2d(.02, 0, Rotation2d.ZERO), gap.pose());
        assertFalse(rig.odometry.observe(new PoseObservation(
                new Pose2d(.1, 0, Rotation2d.ZERO), .1, .1, "camera")));
        Estimate next = rig.step(.02, .02, 0, .01);
        assertTrue(next.valid());
        poseEquals(new Pose2d(.02, 0, Rotation2d.ZERO).plus(new Twist2d(.02, 0, .01).exp()), next.pose());
    }

    @Test void missingAndNonfiniteFramesRequireReacquisitionWithoutTeleporting() {
        var rig = new Rig();
        rig.baseline();
        rig.step(.02, .02, 0, 0);
        Estimate invalid = rig.odometry.update(.04, rig.nativePose, Rotation2d.ZERO, null);
        assertFalse(invalid.valid());
        assertEquals(.02, invalid.pose().getX(), 1e-9);
        Estimate reacquired = rig.step(.04, .10, 0, 0);
        assertFalse(reacquired.valid());
        assertFalse(reacquired.initializedVelocity());
        assertEquals(.02, reacquired.pose().getX(), 1e-9);
        assertEquals(.04, rig.step(.02, .02, 0, 0).pose().getX(), 1e-9);
        rig.modules[0].distance = Double.NaN;
        assertFalse(rig.odometry.update(.10, rig.nativePose, Rotation2d.ZERO, rig.modules).valid());
        assertEquals(.04, rig.odometry.estimate().pose().getX(), 1e-9);
    }

    @Test void duplicateAndOlderFramesCannotIntegrateTwiceOrReplaceTheNewerBaseline() {
        var rig = new Rig();
        rig.baseline();
        Estimate good = rig.step(.02, .02, 0, 0);
        assertSame(good, rig.baseline());
        Estimate old = rig.odometry.update(.01, rig.nativePose, Rotation2d.ZERO, rig.modules);
        assertFalse(old.valid());
        assertEquals(.02, old.timestamp(), 1e-9);
        assertEquals(.02, old.pose().getX(), 1e-9);
        assertEquals(.04, rig.step(.02, .02, 0, 0).pose().getX(), 1e-9);
    }

    @Test void estimateAndGeometryDoNotLeakMutableCallerState() {
        Translation2d[] locations = LOCATIONS.clone();
        var odometry = new GyroWheelOdometry(locations);
        locations[0] = new Translation2d(100, 100);
        var rig = new Rig(odometry);
        rig.baseline();
        Estimate result = rig.step(.02, .02, 0, .02);
        assertTrue(result.valid());
        assertEquals(4, result.inliers());
        result.robotRelativeVelocity().vx = 999;
        assertEquals(1, result.robotRelativeVelocity().vx, 1e-9);
        rig.modules[0].distance = 999;
        assertEquals(1, result.robotRelativeVelocity().vx, 1e-9);
    }

    @Test void validatesGeometryNoiseAndResetValues() {
        assertThrows(IllegalArgumentException.class, () -> new GyroWheelOdometry(null));
        assertThrows(IllegalArgumentException.class, () -> new GyroWheelOdometry(new Translation2d[2]));
        Translation2d[] invalid = LOCATIONS.clone();
        invalid[0] = new Translation2d(Double.NaN, 0);
        assertThrows(IllegalArgumentException.class, () -> new GyroWheelOdometry(invalid));
        assertThrows(IllegalArgumentException.class, () -> new GyroWheelOdometry(LOCATIONS, 0));
        assertThrows(IllegalArgumentException.class, () -> new GyroWheelOdometry(LOCATIONS, Double.NaN));
        var odometry = new GyroWheelOdometry(LOCATIONS);
        assertThrows(IllegalArgumentException.class, () -> odometry.resetTo(Pose2d.ZERO, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> odometry.resetTo(null, 0));
        assertThrows(IllegalArgumentException.class,
                () -> odometry.resetTo(new Pose2d((Translation2d) null, Rotation2d.ZERO), 0));
        var rig = new Rig(odometry);
        assertFalse(odometry.update(0, new Pose2d(Translation2d.ZERO, null),
                Rotation2d.ZERO, rig.modules).valid());
    }

    @Test void explicitClearReanchorsNativePoseAndDiscardsOldEvidenceAndVelocity() {
        var rig = new Rig();
        rig.baseline();
        rig.step(.02, .02, 0, 0);
        assertTrue(rig.odometry.observe(new PoseObservation(
                new Pose2d(.12, 0, Rotation2d.ZERO), .02, .10, "camera")));
        rig.odometry.reset();
        Estimate cleared = rig.odometry.estimate();
        assertFalse(cleared.valid());
        assertFalse(cleared.initializedVelocity());
        assertTrue(Double.isNaN(cleared.timestamp()));
        assertFalse(rig.odometry.observe(new PoseObservation(Pose2d.ZERO, .02, .1, "camera")));
        rig.nativePose = new Pose2d(7, 3, new Rotation2d(.4));
        rig.timestamp = 1;
        Estimate baseline = rig.baseline();
        poseEquals(rig.nativePose, baseline.pose());
        assertTrue(baseline.valid());
        assertFalse(baseline.initializedVelocity());
        assertFalse(rig.odometry.observe(new PoseObservation(Pose2d.ZERO, .02, .1, "camera")));
        Estimate next = rig.step(.02, .02, 0, .01);
        poseEquals(new Pose2d(7, 3, new Rotation2d(.4)).plus(new Twist2d(.02, 0, .01).exp()), next.pose());
        assertTrue(next.initializedVelocity());
    }

    @Test void acceptedFixMetadataPreservesPreCorrectionCaptureInnovationAndDelayedZero() {
        var rig = new Rig();
        assertTrue(rig.odometry.lastAcceptedPoseFix().isEmpty());
        rig.baseline();
        rig.step(.02, .02, 0, 0);
        rig.step(.02, .02, 0, 0);
        assertTrue(rig.odometry.observe(new PoseObservation(
                new Pose2d(.23, 0, Rotation2d.ZERO), .03, .10, "camera")));
        var first = rig.odometry.lastAcceptedPoseFix().orElseThrow();
        assertEquals(.03, first.timestampSeconds(), 1e-12);
        assertEquals(.20, first.innovationMeters(), 1e-9,
                "record capture-time innovation, not the .10 remaining after gain=.5");
        assertEquals(.14, rig.odometry.estimate().pose().getX(), 1e-9);
        rig.step(.02, .02, 0, 0);
        assertTrue(rig.odometry.observe(new PoseObservation(
                new Pose2d(.14, 0, Rotation2d.ZERO), .04, .10, "camera")));
        var delayed = rig.odometry.lastAcceptedPoseFix().orElseThrow();
        assertEquals(.04, delayed.timestampSeconds(), 1e-12);
        assertEquals(0, delayed.innovationMeters(), 1e-9,
                "capture pose agrees even though current pose has moved another .02m");
        assertEquals(.16, rig.odometry.estimate().pose().getX(), 1e-9);
    }

    @Test void rejectedFixKeepsAcceptedMetadataAndBothResetPathsClearIt() {
        var rig = new Rig();
        rig.baseline();
        rig.step(.02, .02, 0, 0);
        assertTrue(rig.odometry.observe(new PoseObservation(
                new Pose2d(.02, 0, Rotation2d.ZERO), .01, .10, "camera")));
        var accepted = rig.odometry.lastAcceptedPoseFix().orElseThrow();
        assertFalse(rig.odometry.observe(new PoseObservation(Pose2d.ZERO, .03, .1, "camera")));
        assertSame(accepted, rig.odometry.lastAcceptedPoseFix().orElseThrow());
        assertFalse(rig.odometry.observe(new PoseObservation(Pose2d.ZERO, .02, .6, "camera")));
        assertSame(accepted, rig.odometry.lastAcceptedPoseFix().orElseThrow());
        assertFalse(rig.odometry.observe(new PoseObservation(Pose2d.ZERO, .01, .1, "camera")));
        assertSame(accepted, rig.odometry.lastAcceptedPoseFix().orElseThrow());
        rig.odometry.resetTo(Pose2d.ZERO, .03);
        assertTrue(rig.odometry.lastAcceptedPoseFix().isEmpty());
        rig.timestamp = .03;
        rig.baseline();
        assertTrue(rig.odometry.observe(new PoseObservation(Pose2d.ZERO, .03, .1, "camera")));
        assertTrue(rig.odometry.lastAcceptedPoseFix().isPresent());
        rig.odometry.reset();
        assertTrue(rig.odometry.lastAcceptedPoseFix().isEmpty());
    }
}
