package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.lib.catalyst.physics.prediction.LaunchState;
import frc.lib.catalyst.physics.prediction.LaunchStatePredictor;
import frc.lib.catalyst.physics.prediction.StatePredictor;

/** Pure-Java tests for {@link StatePredictor} and {@link LaunchStatePredictor}. */
class StatePredictorTest {

    /** A robot at the origin doing {@code vx} m/s and accelerating at {@code ax} m/s^2. */
    private static PhysicalRobotState moving(double vx, double ax) {
        return moving(vx, ax, 0.0);
    }

    private static PhysicalRobotState moving(double vx, double ax, double omega) {
        return new PhysicalRobotState(
                0.0,
                Pose2d.kZero,
                new ChassisSpeeds(vx, 0.0, omega),
                new Translation2d(ax, 0.0),
                0.0,
                new LocalizationQuality(1.0, 0.02, 0.01, 0.05, 0.0, "nominal"));
    }

    @Test
    void constantAccelerationMatchesTheTextbookIntegrals() {
        StatePredictor predictor = StatePredictor.builder().constantAcceleration().build();

        PhysicalRobotState at1s = predictor.predict(moving(2.0, 1.0), 1.0);

        assertEquals(3.0, at1s.fieldVelocity().vxMetersPerSecond, 1e-9);   // v + at
        assertEquals(2.5, at1s.pose().getX(), 1e-9);                        // vt + at^2/2
        assertEquals(1.0, at1s.fieldAcceleration().getX(), 1e-9);           // held constant
        assertEquals(1.0, at1s.timestampSeconds(), 1e-9);
    }

    @Test
    void decayingAccelerationPredictsLessMotionThanHoldingItConstant() {
        StatePredictor decaying = StatePredictor.withDefaults();
        StatePredictor constant = StatePredictor.builder().constantAcceleration().build();

        PhysicalRobotState start = moving(2.0, 4.0);
        PhysicalRobotState decayed = decaying.predict(start, 1.0);
        PhysicalRobotState held = constant.predict(start, 1.0);

        assertTrue(decayed.pose().getX() < held.pose().getX());
        assertTrue(decayed.fieldVelocity().vxMetersPerSecond < held.fieldVelocity().vxMetersPerSecond);
        assertTrue(decayed.fieldAcceleration().getX() < held.fieldAcceleration().getX());
    }

    @Test
    void decayedAccelerationFollowsTheClosedFormSolution() {
        double tau = 0.30;
        StatePredictor predictor = StatePredictor.builder().accelerationTimeConstant(tau).build();
        double h = 0.30;
        double v0 = 2.0;
        double a0 = 4.0;

        PhysicalRobotState at = predictor.predict(moving(v0, a0), h);

        double velocityGain = tau * (1.0 - Math.exp(-h / tau));
        double positionGain = tau * (h - tau * (1.0 - Math.exp(-h / tau)));
        assertEquals(v0 + a0 * velocityGain, at.fieldVelocity().vxMetersPerSecond, 1e-9);
        assertEquals(v0 * h + a0 * positionGain, at.pose().getX(), 1e-9);
        assertEquals(a0 * Math.exp(-h / tau), at.fieldAcceleration().getX(), 1e-9);
    }

    @Test
    void overShortHorizonsBothModelsAgreeCloselyEnoughForAShot() {
        StatePredictor decaying = StatePredictor.withDefaults();
        StatePredictor constant = StatePredictor.builder().constantAcceleration().build();

        PhysicalRobotState start = moving(3.0, 3.0);
        double decayedX = decaying.predict(start, 0.12).pose().getX();
        double heldX = constant.predict(start, 0.12).pose().getX();

        assertEquals(heldX, decayedX, 0.01);   // under a centimetre apart at 120 ms
    }

    @Test
    void zeroAndNegativeHorizonsReturnTheStartingState() {
        StatePredictor predictor = StatePredictor.withDefaults();
        PhysicalRobotState start = moving(2.0, 1.0);

        assertSame(start, predictor.predict(start, 0.0));
        assertSame(start, predictor.predict(start, -1.0));
    }

    @Test
    void horizonsBeyondTheMaximumAreClampedRatherThanRefused() {
        StatePredictor predictor = StatePredictor.builder().constantAcceleration().maxHorizon(0.5).build();

        PhysicalRobotState clamped = predictor.predict(moving(2.0, 0.0), 10.0);

        assertEquals(0.5, clamped.timestampSeconds(), 1e-9);
        assertEquals(1.0, clamped.pose().getX(), 1e-9);   // 2 m/s for the 0.5 s it would allow
    }

    @Test
    void headingIntegratesWithTheTurnRate() {
        StatePredictor predictor = StatePredictor.builder().constantAcceleration().build();

        PhysicalRobotState turned = predictor.predict(moving(0.0, 0.0, Math.PI / 2.0), 1.0);

        assertEquals(90.0, turned.pose().getRotation().getDegrees(), 1e-6);
        assertEquals(Math.PI / 2.0, turned.fieldVelocity().omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void predictionCostsConfidenceAndAddsUncertainty() {
        StatePredictor predictor = StatePredictor.withDefaults();
        PhysicalRobotState start = moving(3.0, 2.0);

        PhysicalRobotState near = predictor.predict(start, 0.05);
        PhysicalRobotState far = predictor.predict(start, 0.80);

        assertTrue(near.quality().confidence() < start.quality().confidence());
        assertTrue(far.quality().confidence() < near.quality().confidence());
        assertTrue(far.quality().translationStdDevMeters() > near.quality().translationStdDevMeters());
        assertTrue(far.quality().velocityStdDevMetersPerSecond()
                > near.quality().velocityStdDevMetersPerSecond());
        assertTrue(far.quality().reason().endsWith("(predicted)"));
    }

    @Test
    void launchStateAdvancesTheRobotByTheReleaseDelay() {
        LaunchStatePredictor launcher = new LaunchStatePredictor(0.12);

        LaunchState launch = launcher.predict(moving(3.0, 0.0));

        assertEquals(0.12, launch.releaseDelaySeconds(), 1e-9);
        assertEquals(0.36, launch.pose().getX(), 1e-9);          // 3 m/s for 120 ms
        assertEquals(3.0, launch.speedMetersPerSecond(), 1e-9);
        assertEquals(0.12, launch.timestampSeconds(), 1e-9);
    }

    @Test
    void aVariableDelayAddsToTheConfiguredOne() {
        LaunchStatePredictor launcher = new LaunchStatePredictor(0.10);

        LaunchState waited = launcher.predict(moving(2.0, 0.0), 0.15);   // flywheel had to recover

        assertEquals(0.25, waited.releaseDelaySeconds(), 1e-9);
        assertEquals(0.50, waited.pose().getX(), 1e-9);
    }

    @Test
    void missRadiusGrowsWithFlightTimeAndGatesTheShot() {
        LaunchState launch = new LaunchState(
                Pose2d.kZero,
                new ChassisSpeeds(3.0, 0.0, 0.0),
                0.12,
                0.12,
                new LocalizationQuality(0.9, 0.05, 0.01, 0.20, 0.1, "nominal"));

        assertEquals(0.05, launch.missRadiusMeters(0.0), 1e-9);          // position error alone
        assertEquals(0.25, launch.missRadiusMeters(1.0), 1e-9);          // plus 0.20 m/s for 1 s
        assertEquals(0.05, launch.missRadiusMeters(-1.0), 1e-9);         // negative flight is clamped

        assertTrue(launch.fitsTarget(1.0, 0.30));
        assertTrue(!launch.fitsTarget(1.0, 0.20));
    }

    @Test
    void anUncertainRobotProducesAWiderMissRadius() {
        LaunchState confident = new LaunchState(Pose2d.kZero, new ChassisSpeeds(), 0.1, 0.1,
                new LocalizationQuality(0.95, 0.02, 0.01, 0.05, 0.0, "nominal"));
        LaunchState lost = new LaunchState(Pose2d.kZero, new ChassisSpeeds(), 0.1, 0.1,
                new LocalizationQuality(0.10, 0.90, 0.30, 1.40, 8.0, "vision stale 8.0 s"));

        assertTrue(lost.missRadiusMeters(1.0) > confident.missRadiusMeters(1.0));
        assertEquals(LocalizationQuality.Level.LOST, lost.quality().level());
    }

    @Test
    void badConfigurationIsRejected() {
        assertThrows(IllegalStateException.class,
                () -> StatePredictor.builder().accelerationTimeConstant(0.0).build());
        assertThrows(IllegalStateException.class, () -> StatePredictor.builder().maxHorizon(0.0).build());
        assertThrows(IllegalStateException.class,
                () -> StatePredictor.builder().confidenceHalfLife(-1.0).build());
        assertThrows(IllegalArgumentException.class, () -> new LaunchStatePredictor(-0.1));
        assertThrows(IllegalArgumentException.class, () -> new LaunchStatePredictor(0.1, null));
    }

    @Test
    void rotationHelpersOnTheStateBehaveAtAStandstill() {
        PhysicalRobotState stopped = moving(0.0, 0.0);
        assertEquals(Rotation2d.kZero, stopped.headingOfTravel());

        PhysicalRobotState rolling = moving(2.0, 0.0);
        assertEquals(0.0, rolling.headingOfTravel().getDegrees(), 1e-9);
        assertEquals(2.0, rolling.speedMetersPerSecond(), 1e-9);
    }
}
