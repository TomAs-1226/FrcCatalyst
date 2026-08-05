package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.catalyst.physics.constraints.PhysicsConstraints;
import frc.lib.catalyst.physics.estimation.PhysicalStateEstimator;
import frc.lib.catalyst.physics.model.ArticulatedRobotModel;
import frc.lib.catalyst.physics.model.MechanismModel;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.model.StabilityModel;
import frc.lib.catalyst.physics.observation.BearingObservation;
import frc.lib.catalyst.physics.observation.ContactObservation;
import frc.lib.catalyst.physics.observation.RangeObservation;
import frc.lib.catalyst.physics.observation.VelocityObservation;
import frc.lib.catalyst.physics.replay.PhysicsReplay;
import frc.lib.catalyst.physics.sim.DisturbanceInjector;

/**
 * End-to-end tests for the pieces that tie Physics Core together: velocity fusion, the opt-in
 * constraint layer, replay, and the observation contracts. Everything runs with an injected clock and
 * logging off, so there is no HAL and no robot.
 */
class PhysicsIntegrationTest {

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(0.4, 0.4), new Translation2d(0.4, -0.4),
            new Translation2d(-0.4, 0.4), new Translation2d(-0.4, -0.4));

    private static RobotModel chassis() {
        return RobotModel.builder()
                .massKg(60.0).footprintMeters(0.8, 0.8)
                .centerOfMassHeightMeters(0.25).coefficientOfFriction(1.0).build();
    }

    private static PhysicsCore core(double[] clock) {
        return PhysicsCore.builder()
                .robotModel(chassis()).kinematics(KINEMATICS)
                .clock(() -> clock[0]).withLogging(false).withAlerts(false).build();
    }

    private static PhysicsSample driving(double t, double vx, Translation2d accel) {
        ChassisSpeeds speeds = new ChassisSpeeds(vx, 0.0, 0.0);
        return new PhysicsSample(t, Pose2d.kZero, speeds,
                KINEMATICS.toSwerveModuleStates(speeds), accel, 0.0);
    }

    // ===========================================
    //             Velocity fusion
    // ===========================================

    @Test
    void aConfidentVelocityObservationPullsTheEstimateTowardItAndTightensIt() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        estimator.update(0.0, Pose2d.kZero, new ChassisSpeeds(2.0, 0, 0), Translation2d.kZero, 0, 0);
        double before = estimator.state().quality().velocityStdDevMetersPerSecond();

        // The wheels say 2.0; an optical-flow sensor with a very tight standard deviation says 3.0.
        assertTrue(estimator.applyVelocityObservation(new Translation2d(3.0, 0.0), 0.001));

        assertEquals(3.0, estimator.state().fieldVelocity().vxMetersPerSecond, 0.01);
        assertTrue(estimator.state().quality().velocityStdDevMetersPerSecond() < before,
                "fusing a measurement must reduce the uncertainty");
    }

    @Test
    void aVagueObservationBarelyMovesTheEstimate() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        estimator.update(0.0, Pose2d.kZero, new ChassisSpeeds(2.0, 0, 0), Translation2d.kZero, 0, 0);

        // A 5 m/s standard deviation against an estimate good to ~0.6 m/s: almost no information.
        estimator.applyVelocityObservation(new Translation2d(10.0, 0.0), 5.0);

        assertTrue(estimator.state().fieldVelocity().vxMetersPerSecond < 2.2,
                "a vague observation must not drag the estimate to 10 m/s");
    }

    @Test
    void fusionFollowsTheInverseVarianceFormulaExactly() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        estimator.recordAbsoluteFix(0.0);
        estimator.update(0.0, Pose2d.kZero, new ChassisSpeeds(2.0, 0, 0), Translation2d.kZero, 0, 0);

        double estimateStdDev = estimator.state().quality().velocityStdDevMetersPerSecond();
        double observationStdDev = 0.2;
        double expectedWeight = (estimateStdDev * estimateStdDev)
                / (estimateStdDev * estimateStdDev + observationStdDev * observationStdDev);

        estimator.applyVelocityObservation(new Translation2d(4.0, 0.0), observationStdDev);

        assertEquals(2.0 + expectedWeight * (4.0 - 2.0),
                estimator.state().fieldVelocity().vxMetersPerSecond, 1e-9);
        double expectedVariance = estimateStdDev * estimateStdDev * observationStdDev * observationStdDev
                / (estimateStdDev * estimateStdDev + observationStdDev * observationStdDev);
        assertEquals(Math.sqrt(expectedVariance),
                estimator.state().quality().velocityStdDevMetersPerSecond(), 1e-9);
    }

    @Test
    void theBenefitOfAnObservationDecaysAsTheRobotKeepsMoving() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        estimator.recordAbsoluteFix(0.0);
        estimator.update(0.0, Pose2d.kZero, new ChassisSpeeds(2.0, 0, 0), Translation2d.kZero, 0, 0);
        estimator.applyVelocityObservation(new Translation2d(2.0, 0.0), 0.01);

        double tightened = estimator.state().quality().velocityStdDevMetersPerSecond();
        assertTrue(tightened < 0.02);

        for (int i = 1; i <= 100; i++) {
            estimator.update(i * 0.02, Pose2d.kZero, new ChassisSpeeds(2.0, 0, 0), Translation2d.kZero, 0, 0);
        }

        assertTrue(estimator.state().quality().velocityStdDevMetersPerSecond() > tightened * 5,
                "a two-second-old velocity fix should no longer be claimed as fresh");
    }

    @Test
    void observationsBeforeTheFirstUpdateAreRejected() {
        PhysicalStateEstimator estimator = PhysicalStateEstimator.builder().build();
        assertFalse(estimator.applyVelocityObservation(new Translation2d(1, 0), 0.1));
    }

    @Test
    void physicsCoreRoutesAVelocityObservationIntoTheEstimator() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        physics.update(driving(0.0, 2.0, Translation2d.kZero));

        assertTrue(physics.observe(new VelocityObservation(
                new Translation2d(3.0, 0.0), 0.0, 0.005, "flow")));
        assertEquals(3.0, physics.state().fieldVelocity().vxMetersPerSecond, 0.05);
    }

    // ===========================================
    //          Observation residuals
    // ===========================================

    @Test
    void aRangeObservationReportsHowFarTheEstimateIsOut() {
        var range = new RangeObservation(new Translation2d(5.0, 0.0), 3.0, 0.0, 0.02, "tof");

        assertEquals(0.0, range.residualFrom(new Translation2d(2.0, 0.0)), 1e-9);
        // The estimate says the robot is 4 m away; the sensor says 3.
        assertEquals(-1.0, range.residualFrom(new Translation2d(1.0, 0.0)), 1e-9);
    }

    @Test
    void aBearingObservationWrapsItsResidualAcrossTheDiscontinuity() {
        var bearing = new BearingObservation(new Translation2d(5.0, 0.0),
                Rotation2d.fromDegrees(179), 0.0, 0.02, "cam");

        // Estimate implies -179 degrees; the wrapped difference is 2, not 358.
        double residual = Math.toDegrees(bearing.residualFrom(new Translation2d(10.0, -0.0873)));
        assertTrue(Math.abs(residual) < 10, "expected a small wrapped residual, got " + residual);
    }

    @Test
    void aBearingResidualIsZeroWhenTheEstimateAgrees() {
        var bearing = new BearingObservation(new Translation2d(4.0, 3.0),
                new Rotation2d(Math.atan2(3.0, 4.0)), 0.0, 0.02, "cam");
        assertEquals(0.0, bearing.residualFrom(Translation2d.kZero), 1e-12);
        // Degenerate: sitting on the target has no bearing at all.
        assertEquals(0.0, bearing.residualFrom(new Translation2d(4.0, 3.0)), 1e-12);
    }

    @Test
    void aContactObservationConstrainsOneAxisOnly() {
        // Squared against a wall at x = 0 facing +X, with a 0.45 m bumper half-length.
        var contact = new ContactObservation(0.0, Rotation2d.kZero, 0.45, 0.0, 0.02, "wall");

        assertEquals(0.45, contact.impliedCoordinate(), 1e-12);
        assertEquals(0.0, contact.residualFrom(0.45, 0.0), 1e-12);
        assertEquals(0.10, contact.residualFrom(0.55, 0.0), 1e-12);
        // Moving along the wall changes nothing - the constraint says nothing about that axis.
        assertEquals(0.0, contact.residualFrom(0.45, 3.7), 1e-12);
    }

    @Test
    void everyObservationRejectsAnImpossibleUncertainty() {
        assertThrows(IllegalArgumentException.class,
                () -> new RangeObservation(Translation2d.kZero, 1.0, 0.0, 0.0, "x"));
        assertThrows(IllegalArgumentException.class,
                () -> new RangeObservation(Translation2d.kZero, -1.0, 0.0, 0.1, "x"));
        assertThrows(IllegalArgumentException.class,
                () -> new BearingObservation(Translation2d.kZero, Rotation2d.kZero, 0.0, 0.0, "x"));
        assertThrows(IllegalArgumentException.class,
                () -> new ContactObservation(0.0, Rotation2d.kZero, 0.4, 0.0, -0.1, "x"));
    }

    // ===========================================
    //            PhysicsConstraints
    // ===========================================

    private static PhysicsConstraints constraints(PhysicsCore physics, StabilityModel stability) {
        return PhysicsConstraints.builder()
                .physics(physics).stability(stability).maxSpeedMps(4.0).build();
    }

    @Test
    void aConfidentPlantedRobotIsNotLimitedAtAll() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        physics.observe(frc.lib.catalyst.physics.observation.PoseObservation.of(
                Pose2d.kZero, 0.0, "cam"));
        physics.update(driving(0.0, 2.0, Translation2d.kZero));

        var limits = constraints(physics, StabilityModel.ofChassisOnly(chassis()));

        assertEquals(1.0, limits.speedScale(), 1e-9);
        assertEquals(4.0, limits.maxSpeedMps(), 1e-9);
        assertTrue(limits.explain().contains("no limit binding"));
        assertFalse(limits.isCautionAdvised());
    }

    @Test
    void aRobotThatHasNotSeenATagInAgesIsSlowedDown() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        physics.update(driving(0.0, 2.0, Translation2d.kZero));   // never any absolute fix

        var limits = constraints(physics, StabilityModel.ofChassisOnly(chassis()));

        assertTrue(limits.speedScale() < 1.0);
        assertTrue(limits.explain().contains("confidence"));
        assertTrue(limits.isCautionAdvised());
    }

    @Test
    void slippingWheelsReduceTheSpeedScale() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        physics.observe(frc.lib.catalyst.physics.observation.PoseObservation.of(Pose2d.kZero, 0.0, "cam"));
        physics.update(driving(0.0, 3.0, Translation2d.kZero));

        var limits = constraints(physics, StabilityModel.ofChassisOnly(chassis()));
        double clean = limits.speedScale();

        // Now corrupt the wheels so the slip estimator sees it.
        ChassisSpeeds speeds = new ChassisSpeeds(3.0, 0.0, 0.0);
        SwerveModuleState[] slipping = KINEMATICS.toSwerveModuleStates(speeds);
        for (int i = 0; i < slipping.length; i++) {
            slipping[i] = new SwerveModuleState(slipping[i].speedMetersPerSecond + 2.0, slipping[i].angle);
        }
        for (int i = 1; i <= 6; i++) {
            physics.observe(frc.lib.catalyst.physics.observation.PoseObservation.of(
                    Pose2d.kZero, i * 0.02, "cam"));
            physics.update(new PhysicsSample(i * 0.02, Pose2d.kZero, speeds, slipping,
                    Translation2d.kZero, 0.0));
        }

        assertTrue(limits.speedScale() < clean,
                "slip should reduce the speed scale, got " + limits.speedScale() + " vs " + clean);
        assertTrue(limits.explain().contains("wheel slip"));
    }

    @Test
    void raisingTheElevatorTightensTheAccelerationLimit() {
        double[] clock = {0.0};
        double[] height = {0.0};
        PhysicsCore physics = core(clock);
        physics.update(driving(0.0, 1.0, Translation2d.kZero));

        ArticulatedRobotModel articulated = ArticulatedRobotModel.builder()
                .chassis(chassis())
                .add(MechanismModel.linear("Elevator", 15.0)
                        .mountedAt(new Translation3d(0, 0, 0.2)).position(() -> height[0]).build())
                .build();
        var limits = constraints(physics, new StabilityModel(articulated));

        double stowed = limits.maxAccelerationMpsSq();
        height[0] = 1.5;
        assertTrue(limits.maxAccelerationMpsSq() < stowed);
    }

    @Test
    void theSpeedScaleNeverFallsBelowItsFloor() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        physics.update(driving(0.0, 0.0, Translation2d.kZero));

        var limits = PhysicsConstraints.builder()
                .physics(physics).minimumSpeedScale(0.3).build();

        assertTrue(limits.speedScale() >= 0.3);
    }

    // ===========================================
    //          Replay and disturbance injection
    // ===========================================

    /** Twelve loops of steady driving. */
    private static List<PhysicsSample> steadyRun() {
        List<PhysicsSample> samples = new ArrayList<>();
        for (int i = 0; i < 12; i++) samples.add(driving(i * 0.02, 3.0, Translation2d.kZero));
        return samples;
    }

    @Test
    void replayingTheSameSamplesTwiceGivesIdenticalResults() {
        double[] clock = {0.0};
        PhysicsReplay replay = PhysicsReplay.of(steadyRun());

        var first = replay.run(() -> core(clock));
        var second = replay.run(() -> core(clock));

        assertEquals(first.meanConfidence(), second.meanConfidence(), 1e-15);
        assertEquals(first.peakSlipFactor(), second.peakSlipFactor(), 1e-15);
        assertEquals(first.finalState().fieldVelocity().vxMetersPerSecond,
                second.finalState().fieldVelocity().vxMetersPerSecond, 1e-15);
    }

    @Test
    void replayReportsWhatHappenedAcrossTheRun() {
        double[] clock = {0.0};
        var result = PhysicsReplay.of(steadyRun()).run(() -> core(clock));

        assertEquals(12, result.states().size());
        assertEquals(3.0, result.peakSpeedMps(), 0.01);
        assertEquals(0, result.collisions());
        assertTrue(result.describe().contains("12 samples"));
    }

    @Test
    void aCounterfactualShowsWhatADifferentConfigurationWouldHaveDone() {
        double[] clock = {0.0};
        List<PhysicsSample> samples = steadyRun();
        PhysicsReplay replay = PhysicsReplay.of(samples);

        var withVision = replay.run(() -> {
            PhysicsCore physics = core(clock);
            physics.observe(frc.lib.catalyst.physics.observation.PoseObservation.of(
                    Pose2d.kZero, 0.0, "cam"));
            return physics;
        });
        var blind = replay.run(() -> core(clock));

        assertTrue(withVision.meanConfidence() > blind.meanConfidence(),
                "a run with an absolute fix should hold confidence better");
        assertTrue(withVision.compareTo(blind).contains("mean confidence"));
    }

    @Test
    void aTransformCanRemoveASensorToSeeWhatItWasWorth() {
        double[] clock = {0.0};
        List<PhysicsSample> samples = new ArrayList<>();
        for (int i = 0; i < 12; i++) samples.add(driving(i * 0.02, 3.0, new Translation2d(0.5, 0)));

        var withImu = PhysicsReplay.of(samples).run(() -> core(clock));
        var withoutImu = PhysicsReplay.of(samples).run(() -> core(clock),
                s -> new PhysicsSample(s.timestampSeconds(), s.pose(), s.robotRelativeSpeeds(),
                        s.moduleStates(), null, s.yawRateRadPerSec()));

        // With no accelerometer, diagnostics go quiet - which is the honest degradation, not a crash.
        assertEquals(0.0, withoutImu.analyses().get(11).disturbanceMpsSq(), 1e-12);
        assertTrue(withImu.analyses().get(11).disturbanceMpsSq() >= 0.0);
    }

    @Test
    void anInjectedDifferentialSlipIsDetectedAndClearsWhenItStops() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        DisturbanceInjector injector = DisturbanceInjector.builder().slipModule(2, 2.5).build();

        physics.update(driving(0.0, 3.0, Translation2d.kZero));
        for (int i = 1; i <= 8; i++) {
            physics.update(injector.apply(driving(i * 0.02, 3.0, Translation2d.kZero)));
        }
        assertTrue(physics.analyze().isSlipping(), "one module reading 2.5 m/s fast must be detected");
        assertEquals(2, physics.analyze().worstModule());

        injector.setEnabled(false);
        for (int i = 9; i <= 30; i++) {
            physics.update(injector.apply(driving(i * 0.02, 3.0, Translation2d.kZero)));
        }
        assertFalse(physics.analyze().isSlipping(), "the detector must clear once the slip stops");
    }

    @Test
    void uniformSlipIsInvisibleToModuleScoringButShowsUpAgainstTheImu() {
        // All four wheels breaking loose together is the one case per-module residuals cannot see:
        // every module reads fast, forward kinematics reads exactly as fast, and every residual is
        // zero. The wheels-versus-IMU comparison is what catches it, which is why both checks exist.
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        DisturbanceInjector injector = DisturbanceInjector.builder().build();

        physics.update(driving(0.0, 3.0, Translation2d.kZero));
        for (int i = 1; i <= 10; i++) {
            // The wheels wind up 0.25 m/s per loop - 12.5 m/s^2 of implied acceleration - while the
            // IMU reports the robot is not accelerating at all.
            injector.setWheelSpeedBias(i * 0.25);
            physics.update(injector.apply(driving(i * 0.02, 3.0, Translation2d.kZero)));
        }

        assertEquals(0.0, physics.analyze().slipFactor(), 1e-9,
                "uniform slip leaves no per-module residual - this is the documented blind spot");
        assertTrue(physics.analyze().disturbanceMpsSq() > 5.0,
                "but the wheels and the IMU must disagree loudly, got "
                        + physics.analyze().disturbanceMpsSq());
        assertTrue(physics.analyze().isDisturbed());
    }

    @Test
    void anInjectedImpactBecomesACollision() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        DisturbanceInjector injector = DisturbanceInjector.builder()
                .impact(new Translation2d(0.0, -16.0)).build();

        physics.update(driving(0.0, 2.0, Translation2d.kZero));
        for (int i = 1; i <= 6; i++) {
            physics.update(injector.apply(driving(i * 0.02, 2.0, Translation2d.kZero)));
        }

        assertTrue(physics.analyze().lastCollision().isPresent());
    }

    @Test
    void anInjectorWithNothingConfiguredChangesNothing() {
        DisturbanceInjector injector = DisturbanceInjector.builder().build();
        PhysicsSample clean = driving(0.0, 3.0, new Translation2d(1, 0));

        PhysicsSample same = injector.apply(clean);
        assertEquals(clean.robotRelativeSpeeds().vxMetersPerSecond,
                same.robotRelativeSpeeds().vxMetersPerSecond, 1e-12);
        assertEquals(clean.robotRelativeAcceleration(), same.robotRelativeAcceleration());
    }

    @Test
    void badConfigurationIsRejected() {
        assertThrows(IllegalStateException.class, () -> PhysicsConstraints.builder().build());
        assertThrows(IllegalArgumentException.class, () -> PhysicsReplay.of(List.of()));
        assertThrows(IllegalArgumentException.class, () -> PhysicsReplay.of(null));
        assertThrows(IllegalArgumentException.class,
                () -> DisturbanceInjector.builder().slipModule(-1, 1.0));
    }
}
