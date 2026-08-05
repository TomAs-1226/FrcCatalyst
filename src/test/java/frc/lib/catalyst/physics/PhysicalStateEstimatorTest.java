package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.lib.catalyst.physics.estimation.PhysicalStateEstimator;

/**
 * Pure-Java tests for {@link PhysicalStateEstimator}. Every measurement is an argument and the clock
 * is the test's, so this runs with no HAL, no NetworkTables, and no robot.
 */
class PhysicalStateEstimatorTest {

    private static final Pose2d ORIGIN = Pose2d.kZero;

    private static ChassisSpeeds forward(double vx) {
        return new ChassisSpeeds(vx, 0.0, 0.0);
    }

    @Test
    void theFirstSampleTakesTheWheelsAtFaceValue() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();

        PhysicalRobotState state = estimator.update(0.0, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);

        assertEquals(2.0, state.fieldVelocity().vxMetersPerSecond, 1e-9);
        assertEquals(0.0, state.accelerationMetersPerSecSq(), 1e-9);   // nothing to differentiate yet
        assertTrue(estimator.isInitialized());
    }

    @Test
    void wheelTrustFallsAsSlipRises() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();

        assertEquals(0.98, estimator.kinematicWeight(0.0), 1e-9);
        // The floor is deliberately tiny. A complementary filter settles onto whatever it is weighted
        // toward, so leaving the wheels a meaningful vote means the estimate re-joins them part way
        // through a slip - which the simulation validation caught as a 0% improvement over the raw
        // encoders. What bounds the resulting dead reckoning is the slip budget, not this floor.
        assertEquals(0.005, estimator.kinematicWeight(1.0), 1e-9);
        assertEquals(0.4925, estimator.kinematicWeight(0.5), 1e-9);
        assertEquals(0.005, estimator.kinematicWeight(5.0), 1e-9);     // clamped
    }

    @Test
    void aLongSlipEventuallyReAnchorsToTheWheels() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().slipBudget(1.0).build();
        estimator.update(0.0, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);

        // Under 60% of the budget, the floor stays down and the estimate rides the IMU.
        for (int i = 1; i <= 25; i++) {
            estimator.update(i * 0.02, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 1.0);
        }
        assertEquals(0.0, estimator.slipBudgetExhaustion(), 0.6);
        double duringNormalSlip = estimator.kinematicWeight(1.0);

        // Keep "slipping" well past the budget and the wheels have to be trusted again, because
        // dead reckoning that never ends is not dead reckoning, it is drift.
        for (int i = 26; i <= 80; i++) {
            estimator.update(i * 0.02, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 1.0);
        }
        assertEquals(1.0, estimator.slipBudgetExhaustion(), 1e-9);
        assertEquals(0.98, estimator.kinematicWeight(1.0), 1e-9);
        assertTrue(estimator.kinematicWeight(1.0) > duringNormalSlip);
    }

    @Test
    void theSlipBudgetRefillsWhileTheWheelsGrip() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().slipBudget(1.0).build();
        estimator.update(0.0, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);

        for (int i = 1; i <= 60; i++) {
            estimator.update(i * 0.02, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 1.0);
        }
        assertTrue(estimator.slipSeconds() > 0.5);

        for (int i = 61; i <= 200; i++) {
            estimator.update(i * 0.02, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);
        }
        assertEquals(0.0, estimator.slipSeconds(), 1e-9);
    }

    @Test
    void aSlippingWheelIsLargelyIgnoredInFavourOfTheImu() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        estimator.update(0.00, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);

        // The wheels suddenly claim 4 m/s; the IMU says the robot is not accelerating at all.
        PhysicalRobotState slipping =
                estimator.update(0.02, ORIGIN, forward(4.0), Translation2d.kZero, 0.0, 1.0);

        // 0.005 * 4.0 + 0.995 * 2.0 - the wheels barely get a vote, which is the whole point.
        assertEquals(2.01, slipping.fieldVelocity().vxMetersPerSecond, 1e-9);
        assertTrue(slipping.fieldVelocity().vxMetersPerSecond < 2.5);
    }

    @Test
    void withNoSlipTheWheelsWinAlmostOutright() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        estimator.update(0.00, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);

        PhysicalRobotState rolling =
                estimator.update(0.02, ORIGIN, forward(4.0), Translation2d.kZero, 0.0, 0.0);

        assertEquals(3.96, rolling.fieldVelocity().vxMetersPerSecond, 1e-9);   // 0.98 * 4 + 0.02 * 2
    }

    @Test
    void withoutAnAccelerometerSlipCannotDownWeightTheWheels() {
        PhysicalStateEstimator estimator =
                PhysicalStateEstimator.builder().withoutAccelerometer().build();

        assertEquals(0.98, estimator.kinematicWeight(0.0), 1e-9);
        assertEquals(0.98, estimator.kinematicWeight(1.0), 1e-9);   // nothing to fall back to

        estimator.update(0.00, ORIGIN, forward(2.0), null, 0.0, 0.0);
        PhysicalRobotState state = estimator.update(0.02, ORIGIN, forward(4.0), null, 0.0, 1.0);
        assertEquals(3.96, state.fieldVelocity().vxMetersPerSecond, 1e-9);
    }

    @Test
    void robotRelativeInputsAreRotatedIntoFieldCoordinates() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        Pose2d facingLeft = new Pose2d(0, 0, Rotation2d.fromDegrees(90));

        // Driving straight ahead while facing +Y means moving along the field's +Y axis.
        PhysicalRobotState state =
                estimator.update(0.0, facingLeft, forward(3.0), Translation2d.kZero, 0.0, 0.0);

        assertEquals(0.0, state.fieldVelocity().vxMetersPerSecond, 1e-9);
        assertEquals(3.0, state.fieldVelocity().vyMetersPerSecond, 1e-9);
    }

    @Test
    void confidenceDecaysWithoutAnAbsoluteFixAndRecoversWithOne() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();

        estimator.recordAbsoluteFix(0.0);
        PhysicalRobotState fresh = estimator.update(0.0, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);
        assertEquals(1.0, fresh.quality().confidence(), 1e-9);
        assertEquals(LocalizationQuality.Level.HIGH, fresh.quality().level());

        // Four seconds with no vision saturates the staleness penalty at 0.40.
        PhysicalRobotState stale = estimator.update(4.0, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);
        assertEquals(0.60, stale.quality().confidence(), 1e-9);
        assertEquals(LocalizationQuality.Level.MODERATE, stale.quality().level());
        assertTrue(stale.quality().reason().startsWith("vision stale"));

        estimator.recordAbsoluteFix(4.0);
        PhysicalRobotState corrected =
                estimator.update(4.02, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);
        assertTrue(corrected.quality().confidence() > 0.99);
    }

    @Test
    void aRobotThatHasNeverSeenATagSaysSo() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();

        PhysicalRobotState state = estimator.update(0.0, ORIGIN, forward(1.0), Translation2d.kZero, 0.0, 0.0);

        assertEquals(0.60, state.quality().confidence(), 1e-9);
        assertEquals("no absolute fix yet", state.quality().reason());
        assertTrue(Double.isInfinite(estimator.secondsSinceAbsoluteFix()));
    }

    @Test
    void slipCostsConfidenceAndIsNamedAsTheReason() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        estimator.recordAbsoluteFix(0.0);
        estimator.update(0.00, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);

        PhysicalRobotState state =
                estimator.update(0.02, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 1.0);

        assertTrue(state.quality().confidence() < 0.7);
        assertTrue(state.quality().reason().startsWith("wheel slip"));
    }

    @Test
    void uncertaintyGrowsAsConfidenceFalls() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();

        estimator.recordAbsoluteFix(0.0);
        PhysicalRobotState confident =
                estimator.update(0.0, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);
        PhysicalRobotState unsure =
                estimator.update(10.0, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);

        assertTrue(unsure.quality().confidence() < confident.quality().confidence());
        assertTrue(unsure.quality().translationStdDevMeters() > confident.quality().translationStdDevMeters());
        assertTrue(unsure.quality().velocityStdDevMetersPerSecond()
                > confident.quality().velocityStdDevMetersPerSecond());
        assertTrue(unsure.quality().rotationStdDevRadians() > confident.quality().rotationStdDevRadians());
    }

    @Test
    void aLongGapRestartsFromTheWheelsRatherThanIntegratingAcrossIt() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        estimator.update(0.00, ORIGIN, forward(2.0), new Translation2d(5.0, 0.0), 0.0, 0.0);
        estimator.update(0.02, ORIGIN, forward(2.1), new Translation2d(5.0, 0.0), 0.0, 0.0);

        // A one-second gap - disabled, or a seek during replay.
        PhysicalRobotState afterGap =
                estimator.update(1.02, ORIGIN, forward(0.5), new Translation2d(5.0, 0.0), 0.0, 0.0);

        assertEquals(0.5, afterGap.fieldVelocity().vxMetersPerSecond, 1e-9);   // wheels, not integration
        assertEquals(0.0, afterGap.accelerationMetersPerSecSq(), 1e-9);        // no bogus spike
    }

    @Test
    void sensorDisagreementIsReportedForCalibrationHunting() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        estimator.update(0.00, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);
        estimator.update(0.02, ORIGIN, forward(3.0), Translation2d.kZero, 0.0, 0.0);

        // Wheels say 3.0, IMU integration says 2.0.
        assertEquals(1.0, estimator.sensorDisagreementMps(), 1e-9);
    }

    @Test
    void poseIsPassedThroughUntouched() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        Pose2d pose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(45));

        PhysicalRobotState state = estimator.update(0.0, pose, forward(2.0), Translation2d.kZero, 0.0, 0.0);

        assertEquals(pose, state.pose());
    }

    @Test
    void resetReturnsItToTheUnknownState() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        estimator.update(0.0, ORIGIN, forward(2.0), Translation2d.kZero, 0.0, 0.0);
        assertTrue(estimator.isInitialized());

        estimator.reset();

        assertFalse(estimator.isInitialized());
        assertEquals(0.0, estimator.state().quality().confidence(), 1e-9);
        assertEquals(0.0, estimator.sensorDisagreementMps(), 1e-9);
    }

    @Test
    void badConfigurationIsRejected() {
        assertThrows(IllegalStateException.class,
                () -> PhysicalStateEstimator.builder().kinematicTrust(0.0).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicalStateEstimator.builder().kinematicTrust(1.5).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicalStateEstimator.builder().minimumKinematicTrust(0.0).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicalStateEstimator.builder().kinematicTrust(0.3).minimumKinematicTrust(0.5).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicalStateEstimator.builder().slipBudget(0.0).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicalStateEstimator.builder().absoluteFixHalfLife(0.0).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicalStateEstimator.builder().residualScale(-1.0).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicalStateEstimator.builder().maxSampleGap(0.0).build());
    }
}
