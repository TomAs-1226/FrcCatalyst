package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.*;
import java.util.function.DoubleUnaryOperator;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import frc.lib.catalyst.physics.estimation.VisionPoseRecovery;
import frc.lib.catalyst.physics.observation.PoseObservation;

/** Independent trajectories and rejection cases; no wheel data or perfect-native adapter. */
class VisionPoseRecoveryTest {
    private static final double INPUT_STD = .02;
    private static final DoubleUnaryOperator CONSTANT_X = t -> 1 + .4 * t;
    private static final DoubleUnaryOperator CONSTANT_Y = t -> 3 - .2 * t;

    private static PoseObservation capture(double timestamp, double relativeTime, String source,
            DoubleUnaryOperator x, DoubleUnaryOperator y) {
        return new PoseObservation(new Pose2d(x.applyAsDouble(relativeTime), y.applyAsDouble(relativeTime),
                new Rotation2d(.3)), timestamp, INPUT_STD, source);
    }

    private static double fill(VisionPoseRecovery recovery, double epoch, double step, int count,
            String source, DoubleUnaryOperator x, DoubleUnaryOperator y) {
        double last = Double.NaN;
        for (int i = 1; i <= count; i++) {
            double t = i * step;
            last = epoch + t;
            assertTrue(recovery.observe(capture(last, t, source, x, y), last, epoch),
                    "qualified capture " + i + " was rejected: " + recovery.reason());
        }
        return last;
    }

    private static VisionPoseRecovery constantMotion() {
        var recovery = new VisionPoseRecovery(5);
        fill(recovery, 0, .05, 40, "rear", CONSTANT_X, CONSTANT_Y);
        return recovery;
    }

    @Test void quadraticDecelerationUsesEndpointVelocityRatherThanMeanTravelSpeed() {
        var recovery = new VisionPoseRecovery(5);
        // Actual constant accelerations (-1,-.25), starting velocity (2,.5), stopped at t=2.
        // A straight-line/interval-mean estimator instead reports about (1,.25) at that endpoint.
        DoubleUnaryOperator x = t -> 2 * t - .5 * t * t;
        DoubleUnaryOperator y = t -> .5 * t - .125 * t * t;
        fill(recovery, 0, .05, 40, "rear", x, y);
        var stopped = recovery.evidenceAt(2, 0).orElseThrow(() -> new AssertionError(recovery.reason()));
        assertEquals(0, stopped.fieldVelocity().getX(), 1e-9);
        assertEquals(0, stopped.fieldVelocity().getY(), 1e-9);
        assertEquals(2, stopped.pose().getX(), 1e-9);
        assertEquals(.5, stopped.pose().getY(), 1e-9);
        assertTrue((x.applyAsDouble(1.95) - x.applyAsDouble(.05)) / 1.90 > .9,
                "counterexample must actually separate endpoint speed from mean travel speed");
        // Evaluate at the current native epoch, not just the last camera epoch.
        var projected = recovery.evidenceAt(2.01, 0).orElseThrow(() -> new AssertionError(recovery.reason()));
        assertEquals(-.01, projected.fieldVelocity().getX(), 1e-8);
        assertEquals(-.0025, projected.fieldVelocity().getY(), 1e-8);
        assertEquals(x.applyAsDouble(2.01), projected.pose().getX(), 1e-8);
        assertEquals(y.applyAsDouble(2.01), projected.pose().getY(), 1e-8);
        assertEquals(2, projected.captureTimestampSeconds(), 1e-12);
        assertEquals(2.01, projected.timestampSeconds(), 1e-12);
        assertTrue(projected.velocityStdDevMps() > stopped.velocityStdDevMps());
        assertTrue(projected.positionStdDevMeters() > stopped.positionStdDevMeters());
    }

    @Test void sixTrainingCapturesWithoutFreshHoldoutCannotCreateEvidence() {
        var recovery = new VisionPoseRecovery(5);
        fill(recovery, 0, .10, 6, "rear", CONSTANT_X, CONSTANT_Y);
        assertEquals(6, recovery.sampleCount());
        assertTrue(recovery.evidenceAt(.6, 0).isEmpty());
        assertTrue(recovery.observe(capture(.7, .7, "rear", CONSTANT_X, CONSTANT_Y), .7, 0));
        // A holdout is necessary but cannot waive the separate endpoint-uncertainty ceiling.
        assertTrue(recovery.evidenceAt(.7, 0).isEmpty());
        assertTrue(recovery.reason().contains("uncertainty"));
    }

    @Test void heldoutOutlierIsNotUsedToRefitItselfIntoAgreement() {
        var recovery = new VisionPoseRecovery(5);
        fill(recovery, 0, .05, 39, "rear", CONSTANT_X, CONSTANT_Y);
        var outlier = new PoseObservation(new Pose2d(CONSTANT_X.applyAsDouble(2) + .15,
                CONSTANT_Y.applyAsDouble(2), Rotation2d.ZERO), 2, INPUT_STD, "rear");
        assertTrue(recovery.observe(outlier, 2, 0), "capture passes input checks; its holdout fit must fail");
        assertTrue(recovery.evidenceAt(2, 0).isEmpty());
        assertTrue(recovery.reason().contains("new camera capture disagrees"));
    }

    @Test void inconsistentTrainingTrajectoryCannotBeCertifiedByAnAgreeingLastCapture() {
        var recovery = new VisionPoseRecovery(5);
        for (int i = 1; i <= 40; i++) {
            double t = i * .05;
            double x = CONSTANT_X.applyAsDouble(t) + (i == 20 ? .25 : 0);
            assertTrue(recovery.observe(new PoseObservation(
                    new Pose2d(x, CONSTANT_Y.applyAsDouble(t), Rotation2d.ZERO), t, INPUT_STD, "rear"), t, 0));
        }
        assertTrue(recovery.evidenceAt(2, 0).isEmpty());
        assertTrue(recovery.reason().contains("trajectory inconsistent"));
    }

    @Test void duplicateOlderStaleAndFutureCapturesCannotRefreshEvidence() {
        var recovery = constantMotion();
        var original = recovery.evidenceAt(2, 0).orElseThrow();
        int samples = recovery.sampleCount();
        assertFalse(recovery.observe(capture(2, 2, "rear", CONSTANT_X, CONSTANT_Y), 2, 0));
        assertFalse(recovery.observe(capture(1.99, 1.99, "rear", CONSTANT_X, CONSTANT_Y), 2, 0));
        assertFalse(recovery.observe(capture(2.03, 2.03, "rear", CONSTANT_X, CONSTANT_Y), 2.02, 0));
        assertFalse(recovery.observe(capture(2.10, 2.10, "rear", CONSTANT_X, CONSTANT_Y), 2.30, 0));
        assertEquals(samples, recovery.sampleCount());
        var unchanged = recovery.evidenceAt(2, 0).orElseThrow();
        assertEquals(original.captureTimestampSeconds(), unchanged.captureTimestampSeconds(), 0);
        assertEquals(original.firstCaptureTimestampSeconds(), unchanged.firstCaptureTimestampSeconds(), 0);
        assertTrue(recovery.evidenceAt(1.99, 0).isEmpty(), "holdout cannot arrive from the future");
        assertTrue(recovery.evidenceAt(2.13, 0).isEmpty(), "old evidence expires without a new holdout");
        assertTrue(recovery.evidenceAt(2, .1).isEmpty(), "another fault epoch cannot reuse this evidence");
        assertTrue(recovery.evidenceAt(Double.NaN, 0).isEmpty());
    }

    @Test void sourceChangesResetCutoffsAndFaultEpochsRequireAnEntireNewWindow() {
        var recovery = constantMotion();
        assertTrue(recovery.evidenceAt(2, 0).isPresent());
        assertTrue(recovery.observe(capture(2.05, 2.05, "front", CONSTANT_X, CONSTANT_Y), 2.05, 0));
        assertEquals(1, recovery.sampleCount(), "distinct source captures cannot silently pool evidence");
        assertTrue(recovery.evidenceAt(2.05, 0).isEmpty());
        recovery.invalidateBefore(2.06);
        assertEquals(0, recovery.sampleCount());
        assertFalse(recovery.observe(capture(2.06, 2.06, "front", CONSTANT_X, CONSTANT_Y), 2.06, 0));
        assertTrue(recovery.observe(capture(2.07, 2.07, "front", CONSTANT_X, CONSTANT_Y), 2.07, 0));
        // A later motion fault cannot reuse even a previously accepted fresh camera capture.
        assertFalse(recovery.observe(capture(2.07, 2.07, "front", CONSTANT_X, CONSTANT_Y), 2.09, 2.08));
        assertEquals(0, recovery.sampleCount());
        double last = fill(recovery, 2.08, .05, 40, "front", CONSTANT_X, CONSTANT_Y);
        var renewed = recovery.evidenceAt(last, 2.08).orElseThrow(() -> new AssertionError(recovery.reason()));
        assertEquals(2.08, renewed.faultEpoch(), 1e-12);
        assertTrue(renewed.firstCaptureTimestampSeconds() > 2.08);
        assertTrue(renewed.validAfterTimestampSeconds() >= 2.08);
        assertTrue(recovery.evidenceAt(last, 0).isEmpty());
        recovery.reset();
        assertEquals(0, recovery.sampleCount());
        assertTrue(recovery.evidenceAt(last, 2.08).isEmpty());
    }

    @Test void acquisitionGapOrUncertainInputClearsPreviouslyReadyEvidence() {
        var gap = constantMotion();
        assertTrue(gap.observe(capture(2.2, 2.2, "rear", CONSTANT_X, CONSTANT_Y), 2.2, 0));
        assertEquals(1, gap.sampleCount());
        assertTrue(gap.evidenceAt(2.2, 0).isEmpty());
        var uncertain = constantMotion();
        assertFalse(uncertain.observe(new PoseObservation(new Pose2d(1.8, 2.6, Rotation2d.ZERO),
                2.05, .09, "rear"), 2.05, 0));
        assertEquals(0, uncertain.sampleCount());
        assertTrue(uncertain.evidenceAt(2.05, 0).isEmpty());
    }

    @Test void largeAbsoluteClockRetainsFiniteAccurateEndpointEstimate() {
        var recovery = new VisionPoseRecovery(5);
        double epoch = 1_000_000_000.0;
        double last = fill(recovery, epoch, .05, 40, "rear", CONSTANT_X, CONSTANT_Y);
        var evidence = recovery.evidenceAt(last, epoch).orElseThrow(() -> new AssertionError(recovery.reason()));
        assertEquals(.4, evidence.fieldVelocity().getX(), 2e-5);
        assertEquals(-.2, evidence.fieldVelocity().getY(), 2e-5);
        assertEquals(CONSTANT_X.applyAsDouble(2), evidence.pose().getX(), 1e-9);
        assertEquals(CONSTANT_Y.applyAsDouble(2), evidence.pose().getY(), 1e-9);
        assertTrue(Double.isFinite(evidence.velocityStdDevMps()));
        assertTrue(evidence.velocityStdDevMps() > 0);
        assertTrue(evidence.positionStdDevMeters() >= INPUT_STD);
        assertEquals(epoch, evidence.faultEpoch(), 0);
    }

    @Test void increasedSampleDensityCannotInventIndependentVelocityInformation() {
        var sparse = new VisionPoseRecovery(5);
        double sparseEnd = fill(sparse, 0, .10, 20, "rear", CONSTANT_X, CONSTANT_Y);
        var dense = new VisionPoseRecovery(5);
        double denseEnd = fill(dense, 0, .04, 48, "rear", CONSTANT_X, CONSTANT_Y);
        var a = sparse.evidenceAt(sparseEnd, 0).orElseThrow(() -> new AssertionError(sparse.reason()));
        var b = dense.evidenceAt(denseEnd, 0).orElseThrow(() -> new AssertionError(dense.reason()));
        // Same nearly two-second trajectory: 48 frames must not gain sqrt(48/20) confidence.
        assertTrue(b.velocityStdDevMps() >= .80 * a.velocityStdDevMps(),
                "correlated high-FPS captures must retain a bounded effective information count");
        assertEquals(a.fieldVelocity().getX(), b.fieldVelocity().getX(), 1e-9);
    }

    @Test void oneHundredHertzCapturesAreThinnedAndWindowStorageRemainsBounded() {
        var recovery = new VisionPoseRecovery(5);
        int accepted = 0;
        double lastAccepted = Double.NaN;
        for (int i = 1; i <= 200; i++) {
            double t = i * .01;
            if (recovery.observe(capture(t, t, "rear", CONSTANT_X, CONSTANT_Y), t, 0)) {
                accepted++;
                lastAccepted = t;
            }
        }
        assertTrue(accepted <= 51, "100Hz frames cannot each count as independent captures");
        assertTrue(recovery.sampleCount() <= 48, "fixed-size evidence storage must remain bounded");
        var evidence = recovery.evidenceAt(lastAccepted, 0).orElseThrow(() -> new AssertionError(recovery.reason()));
        assertTrue(evidence.sampleCount() <= 48);
        assertEquals(.4, evidence.fieldVelocity().getX(), 1e-8);
    }
}
