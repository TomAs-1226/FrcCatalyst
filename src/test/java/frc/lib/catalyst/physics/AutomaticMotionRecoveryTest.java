package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.*;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.observation.PoseObservation;

/** Exercises the real recovery seam without synthetic velocity fixes or a standstill seed. */
class AutomaticMotionRecoveryTest {
    private final double[] clock={0};
    private PhysicsCore core(boolean enabled) {
        return PhysicsCore.builder().robotModel(RobotModel.builder().massKg(63)
                .footprintMeters(.7,.7).centerOfMassHeightMeters(.2).coefficientOfFriction(1).build())
                .profile(PhysicsProfile.MINIMAL).clock(()->clock[0]).withMotionObserver(true)
                .withVisionRecovery(enabled).withLogging(false).withAlerts(false).build();
    }
    private void frame(PhysicsCore core,double t,double x,double speed) {
        clock[0]=t;
        core.update(new PhysicsSample(t,new Pose2d(x,0,Rotation2d.ZERO),
                new ChassisVelocities(speed,0,0),null,Translation2d.ZERO,0));
    }
    private PoseObservation camera(double t,double x) {
        return new PoseObservation(new Pose2d(x,0,Rotation2d.ZERO),t,.02,"real-qualified-camera");
    }
    private void lose(PhysicsCore core) {
        frame(core,0,0,0);
        for(int i=1;i<=80;i++)frame(core,i*.02,i*.06,3);
        assertFalse(core.motion().usable());
    }
    private void captures(PhysicsCore core,double start,double nativeOffset,double nativeSpeed,
            double actualSpeed,boolean recoveryInput) {
        for(int i=0;i<160;i++) {
            double t=start+i*.02, x=actualSpeed*(t-start);
            frame(core,t,x+nativeOffset,nativeSpeed);
            if(i%2==0) {
                var observation=camera(t,x);
                if(recoveryInput)core.observeRecoveryPose(observation);
                else core.observe(observation);
            }
        }
    }
    @Test void nativeCameraAgreementAutomaticallyRecoversAfterTheWheelsActuallyStop() {
        var core=core(true);lose(core);
        captures(core,1.62,0,0,0,true);
        assertTrue(core.motion().usable(),core.visionRecoveryReason());
        assertEquals(1,core.visionRecoveryCount());
        assertEquals(0,core.motion().fieldVelocity().getNorm(),1e-8);
    }
    @Test void movingRobotRecoversToMeasuredVelocityInsteadOfAnInventedStandstill() {
        var core=core(true);lose(core);
        captures(core,1.62,0,1,1,true);
        assertTrue(core.motion().usable(),core.visionRecoveryReason());
        assertEquals(1,core.motion().fieldVelocity().getX(),1e-8);
        assertEquals(1,core.visionRecoveryCount());
    }
    @Test void stationaryCameraCannotCertifyUniformWheelspin() {
        var core=core(true);lose(core);
        captures(core,1.62,0,3,0,true);
        assertFalse(core.motion().usable());
        assertEquals(0,core.visionRecoveryCount());
    }
    @Test void disagreeingNativePositionIsNotAutomaticallyReset() {
        var core=core(true);lose(core);
        captures(core,1.62,2,0,0,true);
        assertFalse(core.motion().usable());
        assertEquals(0,core.visionRecoveryCount());
        assertEquals(2,core.state().pose().getX(),1e-8);
    }
    @Test void confidencePoseObservationsAloneCannotRepairExhaustedMotion() {
        var core=core(true);lose(core);
        captures(core,1.62,0,0,0,false);
        assertFalse(core.motion().usable());
        assertEquals(0,core.visionRecoverySamples());
    }
    @Test void recoveryRemainsOptInForOtherRobots() {
        var core=core(false);lose(core);
        captures(core,1.62,0,0,0,true);
        assertFalse(core.motion().usable());
        assertFalse(core.visionRecoveryEnabled());
    }
    @Test void resetAndSecondDisturbanceDiscardAnAlmostReadyWindow() {
        var core=core(true);lose(core);
        for(int i=0;i<45;i++) {
            double t=1.62+i*.02;frame(core,t,0,0);
            if(i%2==0)core.observeRecoveryPose(camera(t,0));
        }
        assertFalse(core.motion().usable());
        double event=clock[0]+.001;core.invalidateMotion(event,"another impact");
        assertFalse(core.observeRecoveryPose(camera(event-.05,0)));
        assertEquals(0,core.visionRecoverySamples());
        clock[0]=event;core.resetMotion(Pose2d.ZERO);
        assertFalse(core.observeRecoveryPose(camera(event-.01,0)));
        assertEquals(0,core.visionRecoverySamples());
        assertFalse(core.motion().usable());
    }
    @Test void invalidNativeFrameCannotUseACachedReadyCameraWindow() {
        var core=core(true);lose(core);
        for(int i=0;i<101;i++) {
            double t=1.62+i*.02;frame(core,t,2,0);
            if(i%2==0)core.observeRecoveryPose(camera(t,0));
        }
        assertFalse(core.motion().usable());
        clock[0]+=.02;
        core.update(new PhysicsSample(clock[0],Pose2d.ZERO,new ChassisVelocities(Double.NaN,0,0),
                null,Translation2d.ZERO,0));
        assertFalse(core.motion().usable());
        assertEquals(0,core.visionRecoveryCount());
        frame(core,clock[0]+.02,0,0);
        assertFalse(core.motion().usable(),"pre-invalid evidence must not recover on next good frame");
        assertEquals(0,core.visionRecoverySamples());
    }
    @Test void mutuallyMatchingButStaleNativeAndCameraCannotRecover() {
        var core=core(true);lose(core);
        for(int i=0;i<101;i++) {
            double t=1.62+i*.02;frame(core,t,2,0);
            if(i%2==0)core.observeRecoveryPose(camera(t,0));
        }
        assertTrue(core.visionRecoveryReason().contains("native")
                || core.visionRecoveryReason().contains("fit"),core.visionRecoveryReason());
        double capture=clock[0];clock[0]+=1;
        core.update(new PhysicsSample(capture,Pose2d.ZERO,new ChassisVelocities(0,0,0),
                null,Translation2d.ZERO,0));
        assertFalse(core.motion().usable());
        assertEquals(0,core.visionRecoveryCount());
    }

}
