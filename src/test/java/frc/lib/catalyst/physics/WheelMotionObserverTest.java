package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.lib.catalyst.physics.estimation.WheelMotionObserver;
import frc.lib.catalyst.physics.estimation.WheelMotionObserver.Mode;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.observation.PoseObservation;

class WheelMotionObserverTest {
    private static PhysicsSample sample(double t, double x, double speed, double accel) {
        return new PhysicsSample(t, new Pose2d(x, 0, Rotation2d.ZERO),
                new ChassisVelocities(speed, 0, 0), null, new Translation2d(accel, 0), 0);
    }
    private static WheelMotionObserver stalled() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 0, 0), 0);
        double x = 0;
        for (int i = 1; i <= 200; i++) {
            double v = Math.min(3, i * .2);
            x += v * .02;
            observer.update(sample(i * .02, x, v, 0), 0);
        }
        return observer;
    }
    @Test void coherentWheelspinNeverBecomesTrustedJustBecauseItsAccelerationEnds() {
        var observer = stalled();
        assertEquals(Mode.UNCERTAIN, observer.state().mode());
        assertTrue(observer.state().requiresAbsoluteFix());
        assertFalse(observer.state().usable());
        assertTrue(observer.state().pose().getX() < .10, "guarded pose must not travel meters while blocked");
        assertEquals(0, observer.state().fieldVelocity().getNorm(), 1e-9);
        assertTrue(observer.state().rejectedWheelTravelMeters() > 10);
    }
    @Test void realAccelerationAndConstantSpeedCoastingRemainUsableWithoutCamera() {
        var observer = new WheelMotionObserver();
        double x = 0, speed = 0;
        observer.update(sample(0, 0, 0, 0), 0);
        for (int i = 1; i <= 300; i++) {
            double a = i <= 50 ? 2 : 0;
            speed += a * .02;
            x += speed * .02;
            var state = observer.update(sample(i * .02, x, speed, a), 0);
            assertTrue(state.usable(), "clean frame " + i + ": " + state.reason());
            assertEquals(x, state.pose().getX(), 1e-9);
            assertEquals(speed, state.fieldVelocity().getX(), 1e-9);
        }
    }
    @Test void constantSpeedAtStartupIsNotFalselyCalledStuck() {
        var observer = new WheelMotionObserver();
        for (int i = 0; i < 200; i++) {
            var state = observer.update(sample(i * .02, 2 * i * .02, 2, 0), 0);
            assertTrue(state.usable());
            assertEquals(2, state.fieldVelocity().getX(), 1e-9);
        }
    }
    @Test void briefSlipUsesPriorRealVelocityInsteadOfFreezingAStillMovingRobot() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 1, 0), 0);
        for (int i = 1; i <= 10; i++) observer.update(sample(i * .02, .06 * i, 3, 0), 0);
        assertEquals(Mode.INERTIAL, observer.state().mode());
        assertEquals(.20, observer.state().pose().getX(), 1e-9);
        assertEquals(1, observer.state().fieldVelocity().getX(), 1e-9);
        assertTrue(observer.state().requiresAbsoluteFix());
        assertFalse(observer.state().usable());
    }
    @Test void stoppingTheWheelsDoesNotRepairLostAbsolutePosition() {
        var observer = stalled();
        for (int i = 1; i <= 30; i++) observer.update(sample(4 + i * .02, 11.6, 0, 0), 0);
        assertTrue(observer.state().requiresAbsoluteFix());
        assertFalse(observer.state().usable());
        assertFalse(observer.acceptAbsoluteFix(new Pose2d(11.6, 0, Rotation2d.ZERO), 4.6));
        observer.reset(); // caller's deliberately validated native reset, not automatic timeout
        assertTrue(observer.update(sample(4.62, .05, 0, 0), 0).usable());
    }
    @Test void staleAndPreFaultFixesCannotUnlockAndSmallAgreeingRecentFixCan() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 0, 0), 0);
        observer.invalidate(.1, "native stale");
        observer.update(sample(.12, 0, 0, 0), 0);
        assertFalse(observer.acceptAbsoluteFix(Pose2d.ZERO, .05));
        assertFalse(observer.acceptAbsoluteFix(Pose2d.ZERO, .14));
        assertFalse(observer.acceptAbsoluteFix(Pose2d.ZERO, .12));
        assertTrue(observer.acceptVelocity(Translation2d.ZERO, .12, .01));
        assertTrue(observer.acceptAbsoluteFix(Pose2d.ZERO, .12));
        assertTrue(observer.state().usable());
    }
    @Test void expiredBridgeZeroIsNotProofOfStandstillAfterPositionRepair() {
        var observer = stalled();
        observer.update(sample(4.02, 11.6, 0, 0), 0);
        observer.resetTo(Pose2d.ZERO, 4.03);
        observer.update(sample(4.04, 0, 0, 0), 0);
        assertEquals(Mode.UNCERTAIN, observer.state().mode());
        assertFalse(observer.acceptAbsoluteFix(Pose2d.ZERO, 4.04));
        assertFalse(observer.state().usable());
        assertTrue(observer.acceptVelocity(Translation2d.ZERO, 4.04, .01));
        assertTrue(observer.acceptAbsoluteFix(Pose2d.ZERO, 4.04));
        assertTrue(observer.state().usable());
    }
    @Test void missingAccelerationDuringSlipDoesNotFabricateAZeroOrRestoreWheels() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 0, 0), 0);
        observer.update(sample(.02, .06, 3, 0), 0);
        var s = new PhysicsSample(.04, new Pose2d(.12, 0, Rotation2d.ZERO),
                new ChassisVelocities(3, 0, 0), null, null, 0);
        observer.update(s, 0);
        assertEquals(Mode.UNCERTAIN, observer.state().mode());
        assertTrue(observer.state().requiresAbsoluteFix());
    }
    @Test void staleGapInvalidatesButRepeatedTimestampDoesNotIntegrateTwice() {
        var observer = new WheelMotionObserver();
        var first = observer.update(sample(1, 2, 2, 0), 0);
        assertEquals(first, observer.update(sample(1, 3, 2, 0), 0));
        assertEquals(Mode.UNCERTAIN, observer.update(sample(1.3, 3, 2, 0), 0).mode());
        assertEquals(2, observer.state().pose().getX(), 1e-9);
    }
    @Test void nonfiniteSensorValueCannotPoisonGuardedPose() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 0, 0), 0);
        observer.update(sample(.02, .1, 2, Double.NaN), 0);
        assertFalse(observer.state().usable());
        assertTrue(Double.isFinite(observer.state().pose().getX()));
    }
    @Test void positionReanchorCannotCertifyContinuingUniformWheelspin() {
        var observer = stalled();
        observer.resetTo(Pose2d.ZERO, 4);
        for (int i = 1; i <= 50; i++) observer.update(sample(4 + i * .02, i * .06, 3, 0), 0);
        assertFalse(observer.state().usable());
        assertEquals(Mode.UNCERTAIN, observer.state().mode());
        assertEquals(0, observer.state().pose().getX(), 1e-9);
        assertFalse(observer.acceptAbsoluteFix(Pose2d.ZERO, 5));
    }
    @Test void gradualWheelspinIsDetectedWithoutBeingResetEachQuietReferenceWindow() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 0, 0), 0);
        double x = 0;
        for (int i = 1; i <= 150; i++) {
            double v = Math.min(3, .6 * i * .02);
            x += v * .02;
            observer.update(sample(i * .02, x, v, 0), 0);
        }
        assertFalse(observer.state().usable());
        assertTrue(observer.state().requiresAbsoluteFix());
        assertTrue(observer.state().pose().getX() < .2);
    }
    @Test void smallAccelerometerBiasDoesNotTurnRealCoastingIntoAFalseStop() {
        var observer = new WheelMotionObserver();
        for (int i = 0; i <= 500; i++) assertTrue(
                observer.update(sample(i * .02, i * .04, 2, .10), 0).usable());
    }
    @Test void independentVelocityRenewsMotionButNeverDeclaresDisagreeingWheelsSafe() {
        var observer = stalled();
        assertFalse(observer.acceptVelocity(Translation2d.ZERO, .1, .01));
        assertFalse(observer.acceptVelocity(Translation2d.ZERO, 4.1, .01));
        assertFalse(observer.acceptVelocity(Translation2d.ZERO, 4, 1));
        assertTrue(observer.acceptVelocity(Translation2d.ZERO, 4, .01));
        assertFalse(observer.acceptVelocity(new Translation2d(3, 0), 4, .01));
        assertFalse(observer.state().usable());
        observer.resetTo(Pose2d.ZERO, 4);
        assertFalse(observer.acceptAbsoluteFix(Pose2d.ZERO, 4));
        for (int i = 1; i <= 10; i++) observer.update(sample(4 + i * .02, .06 * i, 3, 0), 0);
        assertFalse(observer.state().usable());
        assertEquals(0, observer.state().pose().getX(), 1e-9);
    }
    @Test void aGapUsesNewWheelMeasurementForIndependentRecoveryWithoutTrustingItAlone() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 0, 0), 0);
        observer.update(sample(.30, .12, 1, 0), 0);
        assertFalse(observer.state().usable());
        observer.update(sample(.32, .14, 1, 0), 0);
        assertTrue(observer.acceptVelocity(new Translation2d(1, 0), .32, .01));
        assertTrue(observer.acceptAbsoluteFix(new Pose2d(.14, 0, Rotation2d.ZERO), .32));
        assertTrue(observer.state().usable());
    }
    @Test void independentEvidenceAtEndOfSamplingGapCanRepairTheUnobservedInterval() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 0, 0), 0);
        observer.update(sample(.30, .12, 1, 0), 0);
        assertFalse(observer.state().usable());
        assertEquals(0, observer.state().faultTimestampSeconds(), 1e-9);
        assertFalse(observer.acceptAbsoluteFix(new Pose2d(.12, 0, Rotation2d.ZERO), .30));
        assertFalse(observer.acceptVelocity(new Translation2d(1, 0), 0, .01));
        assertFalse(observer.acceptVelocity(new Translation2d(1, 0), .31, .01));
        assertTrue(observer.acceptVelocity(new Translation2d(1, 0), .30, .01));
        assertTrue(observer.acceptAbsoluteFix(new Pose2d(.12, 0, Rotation2d.ZERO), .30));
        assertTrue(observer.state().usable());
    }
    @Test void disagreeingIndependentVelocityAtGapEndCannotCertifyTheWheels() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 0, 0), 0);
        observer.update(sample(.30, .12, 1, 0), 0);
        assertTrue(observer.acceptVelocity(Translation2d.ZERO, .30, .01));
        assertFalse(observer.acceptAbsoluteFix(new Pose2d(.12, 0, Rotation2d.ZERO), .30));
        assertFalse(observer.state().usable());
    }
    @Test void aCachedPreResetFrameCannotMoveTheKnownStationaryStartBack() {
        var observer = new WheelMotionObserver();
        observer.seedAtRest(new Pose2d(4, 1, Rotation2d.ZERO), 1);
        observer.update(sample(.99, 0, 0, 0), 0);
        assertTrue(observer.state().usable());
        assertEquals(4, observer.state().pose().getX(), 1e-9);
        observer.update(new PhysicsSample(1.02, new Pose2d(4, 1, Rotation2d.ZERO),
                new ChassisVelocities(), null, Translation2d.ZERO, 0), 0);
        assertTrue(observer.state().usable());
    }
    @Test void coreReanchorUsesResetEpochRatherThanLastProcessedNativeEpoch() {
        double[] clock = {1};
        var core = PhysicsCore.builder().robotModel(RobotModel.builder().massKg(60)
                .footprintMeters(.7, .7).centerOfMassHeightMeters(.2).build())
                .withMotionObserver(true).withLogging(false).withAlerts(false)
                .clock(() -> clock[0]).build();
        core.update(sample(1, 0, 0, 0));
        clock[0] = 1.01;
        core.resetMotion(new Pose2d(4, 0, Rotation2d.ZERO));
        core.update(sample(1.005, 0, 0, 0));
        assertEquals(4, core.motion().pose().getX(), 1e-9);
        assertEquals(1.01, core.motion().timestampSeconds(), 1e-9);
        core.update(sample(1.03, 4, 0, 0));
        assertTrue(core.motion().usable());
        assertEquals(4, core.motion().pose().getX(), 1e-9);
    }
    @Test void deliberateHeadingResetRotatesTheReferenceInsteadOfInventingSlip() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 1, 0), 0);
        observer.resetTo(new Pose2d(0, 0, Rotation2d.k180deg), 0);
        var state = observer.update(new PhysicsSample(.02,
                new Pose2d(-.02, 0, Rotation2d.k180deg), new ChassisVelocities(1, 0, 0),
                null, Translation2d.ZERO, 0), 0);
        assertTrue(state.usable());
        assertEquals(-1, state.fieldVelocity().getX(), 1e-9);
    }
    @Test void gyroRotationGivesFieldRelativeInertialDirection() {
        var observer = new WheelMotionObserver();
        var heading = Rotation2d.fromDegrees(90);
        observer.update(new PhysicsSample(0, new Pose2d(0, 0, heading),
                new ChassisVelocities(1, 0, 0), null, Translation2d.ZERO, 0), 0);
        var state = observer.update(new PhysicsSample(.02, new Pose2d(0, .06, heading),
                new ChassisVelocities(3, 0, 0), null, Translation2d.ZERO, 0), 0);
        assertEquals(.02, state.pose().getY(), 1e-9);
        assertEquals(0, state.pose().getX(), 1e-9);
    }
    @Test void differentiallySlippingModuleAlsoUsesBoundedMotion() {
        var observer = new WheelMotionObserver();
        observer.update(sample(0, 0, 0, 0), 0);
        for (int i = 1; i <= 5; i++) observer.update(sample(i * .02, 0, 0, 0), .7);
        assertTrue(observer.state().requiresAbsoluteFix());
    }
    @Test void invalidCoreSampleDoesNotPoisonNativeStateOrBecomeAValidMeasurement() {
        var core = PhysicsCore.builder().robotModel(RobotModel.builder().massKg(60)
                .footprintMeters(.7, .7).centerOfMassHeightMeters(.2).build())
                .withMotionObserver(true).withLogging(false).withAlerts(false).clock(() -> .3).build();
        core.update(sample(0, 1, 0, 0));
        core.update(sample(.02, Double.NaN, 0, 0));
        assertEquals(1, core.state().pose().getX(), 1e-9);
        assertFalse(core.motion().usable());
        assertEquals(0, core.state().timestampSeconds(), 1e-9);
        assertTrue(core.quality().level() == LocalizationQuality.Level.LOST);
    }
    @Test void coreKeepsLegacyPoseUntouchedAndExposesSeparateGuardedPose() {
        var core = PhysicsCore.builder().robotModel(RobotModel.builder().massKg(60)
                .footprintMeters(.7, .7).centerOfMassHeightMeters(.2).build())
                .withMotionObserver(true).withLogging(false).withAlerts(false).clock(() -> 2).build();
        core.update(sample(0, 0, 0, 0));
        for (int i = 1; i <= 100; i++) core.update(sample(i * .02, i * .06, 3, 0));
        assertEquals(6, core.state().pose().getX(), 1e-9);
        assertTrue(core.motion().pose().getX() < .10);
        assertFalse(core.motion().usable());
        assertFalse(core.observe(PoseObservation.of(new Pose2d(6, 0, Rotation2d.ZERO), .02, "old"))
                && core.motion().usable());
    }
}
