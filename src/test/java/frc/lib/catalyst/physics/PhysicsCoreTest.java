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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.observation.PoseObservation;
import frc.lib.catalyst.physics.observation.VelocityObservation;
import frc.lib.catalyst.physics.prediction.LaunchState;

/**
 * End-to-end tests for {@link PhysicsCore}, driven through {@link PhysicsSample}. Logging, alerts,
 * and the FPGA clock are all replaced, so the whole pipeline runs with no HAL and no robot.
 */
class PhysicsCoreTest {

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(0.35, 0.35),
            new Translation2d(0.35, -0.35),
            new Translation2d(-0.35, 0.35),
            new Translation2d(-0.35, -0.35));

    private static RobotModel model() {
        return RobotModel.builder()
                .massKg(60.0)
                .footprintMeters(0.7, 0.7)
                .centerOfMassHeightMeters(0.20)
                .coefficientOfFriction(1.0)
                .build();
    }

    /** A core wired for tests: no telemetry, no alerts, and a clock the test controls. */
    private static PhysicsCore core(double[] clock) {
        return PhysicsCore.builder()
                .robotModel(model())
                .kinematics(KINEMATICS)
                .releaseDelaySeconds(0.12)
                .clock(() -> clock[0])
                .withLogging(false)
                .withAlerts(false)
                .build();
    }

    private static SwerveModuleState[] rolling(ChassisSpeeds speeds) {
        return KINEMATICS.toSwerveModuleStates(speeds);
    }

    /** A loop's worth of measurements for a robot driving straight and rolling cleanly. */
    private static PhysicsSample driving(double t, double vx, Translation2d acceleration) {
        ChassisSpeeds speeds = new ChassisSpeeds(vx, 0.0, 0.0);
        return new PhysicsSample(t, Pose2d.kZero, speeds, rolling(speeds), acceleration, 0.0);
    }

    @Test
    void aFreshCoreReportsThatItIsNotRunningYet() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);

        assertFalse(physics.health().running());
        assertFalse(physics.health().isHealthy());
        assertTrue(physics.health().describe().contains("never been called"));
        assertEquals(0.0, physics.state().quality().confidence(), 1e-9);
        assertEquals("nominal", physics.analyze().describe());
    }

    @Test
    void oneUpdateProducesAFusedStateAndAHealthyCore() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);

        PhysicalRobotState state = physics.update(driving(0.0, 3.0, Translation2d.kZero));

        assertEquals(3.0, state.fieldVelocity().vxMetersPerSecond, 1e-9);
        assertTrue(physics.health().running());
        assertTrue(physics.health().isHealthy());
        assertEquals(0.0, physics.analyze().slipFactor(), 1e-9);
    }

    @Test
    void healthGoesStaleWhenUpdatesStop() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        physics.update(driving(0.0, 3.0, Translation2d.kZero));

        clock[0] = 2.0;   // two seconds with no update

        assertTrue(physics.health().running());
        assertFalse(physics.health().isHealthy());
        assertTrue(physics.health().describe().startsWith("stale"));
    }

    @Test
    void aSlippingModuleReachesTheAnalysisAndCostsConfidence() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        physics.update(driving(0.00, 3.0, Translation2d.kZero));

        ChassisSpeeds speeds = new ChassisSpeeds(3.0, 0.0, 0.0);
        SwerveModuleState[] measured = rolling(speeds);
        measured[1] = new SwerveModuleState(6.0, measured[1].angle);   // module 1 is spinning

        for (int i = 1; i <= 5; i++) {   // let the smoothing settle
            physics.update(new PhysicsSample(i * 0.02, Pose2d.kZero, speeds, measured,
                    Translation2d.kZero, 0.0));
        }

        PhysicsAnalysis analysis = physics.analyze();
        assertEquals(1, analysis.worstModule());
        assertTrue(analysis.peakSlip() > 0.8);
        assertTrue(analysis.isSlipping());
        assertTrue(analysis.describe().contains("module 1"));
        assertTrue(physics.state().quality().confidence() < 0.6);
    }

    @Test
    void anImpactBecomesACollisionEventInTheAnalysis() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);

        // Coasting at a steady 2 m/s, then the IMU sees a hard sideways acceleration the wheels
        // never produced.
        physics.update(driving(0.00, 2.0, Translation2d.kZero));
        for (int i = 1; i <= 8; i++) {
            physics.update(driving(i * 0.02, 2.0, new Translation2d(0.0, -15.0)));
        }

        assertTrue(physics.analyze().lastCollision().isPresent());
        assertTrue(physics.analyze().disturbanceMpsSq() > 5.0);
        assertTrue(physics.analyze().lastCollision().get().peakForceNewtons() > 0);
    }

    @Test
    void withoutAnAccelerometerDiagnosticsStayQuietButEstimationKeepsWorking() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);

        for (int i = 0; i <= 5; i++) {
            ChassisSpeeds speeds = new ChassisSpeeds(2.0, 0.0, 0.0);
            physics.update(new PhysicsSample(i * 0.02, Pose2d.kZero, speeds, rolling(speeds), null, 0.0));
        }

        assertEquals(2.0, physics.state().fieldVelocity().vxMetersPerSecond, 1e-9);
        assertEquals(0.0, physics.analyze().disturbanceMpsSq(), 1e-9);
        assertTrue(physics.analyze().lastCollision().isEmpty());
    }

    @Test
    void aGapInAccelerometerDataDoesNotFakeAHugeWheelAcceleration() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);

        // Accelerometer present, then absent for a second while the robot speeds up, then back.
        physics.update(driving(0.00, 1.0, Translation2d.kZero));
        physics.update(driving(0.02, 1.0, Translation2d.kZero));
        for (int i = 2; i <= 50; i++) {
            ChassisSpeeds speeds = new ChassisSpeeds(1.0 + i * 0.05, 0.0, 0.0);
            physics.update(new PhysicsSample(i * 0.02, Pose2d.kZero, speeds, rolling(speeds), null, 0.0));
        }
        physics.update(driving(1.02, 3.5, Translation2d.kZero));

        // The wheel-acceleration derivative restarts across the gap rather than differencing a
        // fresh velocity against a one-second-old one, so nothing looks like a collision.
        assertTrue(physics.analyze().disturbanceMpsSq() < 1.0);
        assertTrue(physics.analyze().lastCollision().isEmpty());
    }

    @Test
    void theMinimalProfileSkipsSlipAndDiagnostics() {
        double[] clock = {0.0};
        PhysicsCore physics = PhysicsCore.builder()
                .robotModel(model())
                .kinematics(KINEMATICS)
                .profile(PhysicsProfile.MINIMAL)
                .clock(() -> clock[0])
                .withLogging(false)
                .withAlerts(false)
                .build();

        ChassisSpeeds speeds = new ChassisSpeeds(3.0, 0.0, 0.0);
        SwerveModuleState[] measured = rolling(speeds);
        measured[0] = new SwerveModuleState(9.0, measured[0].angle);   // wildly slipping

        for (int i = 0; i <= 5; i++) {
            physics.update(new PhysicsSample(i * 0.02, Pose2d.kZero, speeds, measured,
                    new Translation2d(0.0, -20.0), 0.0));
        }

        assertEquals(0.0, physics.analyze().slipFactor(), 1e-9);
        assertEquals(-1, physics.analyze().worstModule());
        assertTrue(physics.analyze().lastCollision().isEmpty());
        assertEquals(3.0, physics.state().fieldVelocity().vxMetersPerSecond, 1e-9);
    }

    @Test
    void aTeleportingVisionFrameIsRejectedRatherThanBelieved() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        physics.update(driving(0.0, 0.0, Translation2d.kZero));   // pose is the origin

        assertFalse(physics.observe(new PoseObservation(
                new Pose2d(5.0, 5.0, Rotation2d.kZero), 0.0, 0.05, "limelight")));
        assertEquals(1, physics.rejectedObservationCount());

        assertTrue(physics.observe(PoseObservation.of(
                new Pose2d(0.3, 0.0, Rotation2d.kZero), 0.0, "limelight")));
        assertEquals(1, physics.rejectedObservationCount());
    }

    @Test
    void anAcceptedVisionFrameRestoresConfidenceWithoutMovingThePose() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        Pose2d truth = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(30));

        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.0, 0.0);
        physics.update(new PhysicsSample(0.0, truth, speeds, rolling(speeds), Translation2d.kZero, 0.0));
        double before = physics.state().quality().confidence();

        physics.observe(PoseObservation.of(new Pose2d(2.1, 1.0, Rotation2d.fromDegrees(30)), 0.0, "cam"));
        physics.update(new PhysicsSample(0.02, truth, speeds, rolling(speeds), Translation2d.kZero, 0.0));

        assertTrue(physics.state().quality().confidence() > before);
        assertEquals(truth, physics.state().pose());   // the observation never moved the pose
    }

    @Test
    void velocityObservationsAreAcceptedAndNullsAreNot() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);

        assertTrue(physics.observe(new VelocityObservation(
                new Translation2d(2.0, 0.0), 0.0, 0.1, "flow")));
        assertFalse(physics.observe(null));
    }

    @Test
    void theLaunchStateLeadsTheRobotByTheConfiguredReleaseDelay() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        physics.update(driving(0.0, 4.0, Translation2d.kZero));

        LaunchState launch = physics.predictLaunchState();

        assertEquals(0.12, launch.releaseDelaySeconds(), 1e-9);
        assertEquals(0.48, launch.pose().getX(), 1e-9);   // 4 m/s for 120 ms
        assertTrue(launch.missRadiusMeters(1.0) > 0.0);

        LaunchState waited = physics.predictLaunchState(0.08);
        assertEquals(0.20, waited.releaseDelaySeconds(), 1e-9);
        assertEquals(0.80, waited.pose().getX(), 1e-9);
    }

    @Test
    void tractionAndTippingUsageComeOutOfTheModel() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);

        // Accelerate from rest to 0.2 m/s in one 20 ms loop: 10 m/s^2, just over one g.
        physics.update(driving(0.00, 0.0, Translation2d.kZero));
        physics.update(driving(0.02, 0.2, new Translation2d(10.0, 0.0)));

        assertTrue(physics.analyze().tractionUsage() > 0.0);
        // This robot is low, so tipping is the looser of the two limits.
        assertTrue(physics.analyze().tippingUsage() < physics.analyze().tractionUsage());
        assertFalse(physics.drivetrainModel().isTippingLimited());
    }

    @Test
    void physicsCoreSatisfiesTheRobotStateSourceContract() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));

        ChassisSpeeds speeds = new ChassisSpeeds(2.0, 0.0, 0.0);
        physics.update(new PhysicsSample(1.5, pose, speeds, rolling(speeds), Translation2d.kZero, 0.0));

        UncertainRobotStateSource source = physics;
        assertEquals(pose, source.pose());
        assertEquals(1.5, source.timestampSeconds(), 1e-9);
        assertTrue(source.fieldVelocity().vxMetersPerSecond > 0);
        assertTrue(source.quality().confidence() > 0);

        // The default covariance is diagonal, built from the quality's standard deviations.
        double expectedXY = Math.pow(source.quality().translationStdDevMeters(), 2);
        assertEquals(expectedXY, source.poseCovariance().get(0, 0), 1e-12);
        assertEquals(expectedXY, source.poseCovariance().get(1, 1), 1e-12);
        assertEquals(0.0, source.poseCovariance().get(0, 1), 1e-12);
    }

    @Test
    void resetClearsEverything() {
        double[] clock = {0.0};
        PhysicsCore physics = core(clock);
        physics.update(driving(0.0, 3.0, Translation2d.kZero));
        assertTrue(physics.health().running());

        physics.reset();

        assertFalse(physics.health().running());
        assertEquals(0.0, physics.analyze().slipFactor(), 1e-9);
        assertEquals(0.0, physics.state().quality().confidence(), 1e-9);
    }

    @Test
    void updateWithoutSourcesTellsYouWhichOnesAreMissing() {
        PhysicsCore physics = PhysicsCore.builder()
                .robotModel(model())
                .clock(() -> 0.0)
                .withLogging(false)
                .withAlerts(false)
                .build();

        IllegalStateException thrown = assertThrows(IllegalStateException.class, physics::update);
        assertTrue(thrown.getMessage().contains("poseSource"));
    }

    @Test
    void badConfigurationIsRejected() {
        assertThrows(IllegalStateException.class, () -> PhysicsCore.builder().build());
        assertThrows(IllegalStateException.class,
                () -> PhysicsCore.builder().robotModel(model()).releaseDelaySeconds(-1).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicsCore.builder().robotModel(model()).poseOutlierGate(0).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicsCore.builder().robotModel(model()).clock(null).build());
        assertThrows(IllegalArgumentException.class,
                () -> new PhysicsSample(0.0, null, new ChassisSpeeds(), null, null, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> new PoseObservation(Pose2d.kZero, 0.0, 0.0, "cam"));
    }
}
