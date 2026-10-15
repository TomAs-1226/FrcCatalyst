package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.*;
import org.wpilib.math.kinematics.*;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.observation.*;

/** End-to-end fault tests: encoder agreement never substitutes for independent ground motion. */
class GyroWheelPhysicsIntegrationTest {
    private static final Translation2d[] LOCATIONS = {
        new Translation2d(.3,.3), new Translation2d(.3,-.3),
        new Translation2d(-.3,.3), new Translation2d(-.3,-.3)
    };
    private static final class Rig {
        double now, distance;
        final double[] wheels = new double[4];
        final PhysicsCore core = PhysicsCore.builder()
            .robotModel(RobotModel.builder().massKg(60).footprintMeters(.7,.7)
                .centerOfMassHeightMeters(.2).build())
            .kinematics(new SwerveDriveKinematics(LOCATIONS))
            .clock(() -> now).withLogging(false).withAlerts(false)
            .withMotionObserver(true).withGyroWheelOdometryControl(true).withGyroWheelOdometry(LOCATIONS).build();
        Rig() { frame(0,0,0); }
        PhysicsSample sample(double x,double v,double a) {
            SwerveModuleVelocity[] modules=new SwerveModuleVelocity[4];
            for(int i=0;i<4;i++) modules[i]=new SwerveModuleVelocity(v,Rotation2d.ZERO);
            return new PhysicsSample(now,new Pose2d(x,0,Rotation2d.ZERO),
                new ChassisVelocities(v,0,0),modules,new Translation2d(a,0),0);
        }
        SwerveModulePosition[] positions() {
            SwerveModulePosition[] positions=new SwerveModulePosition[4];
            for(int i=0;i<4;i++) positions[i]=new SwerveModulePosition(wheels[i],Rotation2d.ZERO);
            return positions;
        }
        void frame(double x,double v,double a) {
            core.update(sample(x,v,a),Rotation2d.ZERO,positions());
        }
        void step(double dx,double v,double a,double badWheelExtra) {
            now+=.02; distance+=dx;
            for(int i=0;i<4;i++) wheels[i]+=dx+(i==0?badWheelExtra:0);
            frame(distance+(wheels[0]-distance)/4,v,a);
        }
    }
    @Test void diagnosticTrialCannotStopOrCertifyTheActualNativeMotionPose() {
        var c=PhysicsCore.builder().robotModel(RobotModel.builder().massKg(60)
            .footprintMeters(.7,.7).centerOfMassHeightMeters(.2).build())
            .withLogging(false).withAlerts(false).clock(()->.02).withMotionObserver(true)
            .withGyroWheelOdometry(LOCATIONS).build();
        assertFalse(c.gyroWheelOdometryControlsMotion());
        // Deliberately absent trial positions are invalid, while actual native motion is coherent.
        for(int i=0;i<2;i++) c.update(new PhysicsSample(i*.02,Pose2d.ZERO,
            new ChassisVelocities(),null,Translation2d.ZERO,0),Rotation2d.ZERO,null);
        assertFalse(c.gyroWheelEstimate().orElseThrow().valid());
        assertTrue(c.motion().usable(),"diagnostic failure cannot become a control fault");
        assertTrue(c.observe(PoseObservation.of(Pose2d.ZERO,.02,"camera")),
            "camera agreement belongs to actual native pose while diagnostic history is unavailable");
    }
    @Test void diagnosticTimestampCannotHideAnInvalidNativeEpoch() {
        var c=PhysicsCore.builder().robotModel(RobotModel.builder().massKg(60)
            .footprintMeters(.7,.7).centerOfMassHeightMeters(.2).build())
            .withLogging(false).withAlerts(false).clock(()->.02).withMotionObserver(true)
            .withGyroWheelOdometry(LOCATIONS).build();
        var positions=new SwerveModulePosition[4];
        for(int i=0;i<4;i++) positions[i]=new SwerveModulePosition();
        for(double t:new double[]{0,.02}) c.update(new PhysicsSample(t,Pose2d.ZERO,
            new ChassisVelocities(),null,Translation2d.ZERO,0),Rotation2d.ZERO,positions);
        assertTrue(c.motion().usable());
        c.update(new PhysicsSample(.01,Pose2d.ZERO,new ChassisVelocities(),null,
            Translation2d.ZERO,0),Rotation2d.ZERO,positions);
        assertFalse(c.motion().usable(),"diagnostic timestamp handling cannot hide a native epoch fault");
    }
    @Test void isolatedWheelErrorIsRejectedWithoutGivingUpOdometryOrChangingNativePose() {
        var r=new Rig();
        // Start with a coherent moving prior; acceleration zero is then correct for coasting.
        r.core.resetMotion(); r.frame(0,1,0);
        for(int i=0;i<10;i++) r.step(.02,1,0,.10);
        assertTrue(r.core.motion().usable());
        assertEquals(.20,r.core.motion().pose().getX(),1e-9);
        assertEquals(.45,r.core.state().pose().getX(),1e-9);
        assertEquals(0,r.core.gyroWheelEstimate().orElseThrow().rejectedModule());
        r.step(.02,1,0,0);
        assertTrue(r.core.motion().usable());
        assertEquals(.22,r.core.motion().pose().getX(),1e-9);
        assertEquals(.47,r.core.state().pose().getX(),1e-9);
    }
    @Test void hardLaunchUsesEndpointVelocityRatherThanMistakingIntervalMeanForSlip() {
        var r=new Rig();
        r.now=.08; r.distance=.032; // x=.5*a*t^2; endpoint v=a*t, interval mean=.4.
        for(int i=0;i<4;i++) r.wheels[i]=.032;
        r.frame(.032,.8,10);
        assertEquals(.4,r.core.gyroWheelEstimate().orElseThrow().robotRelativeVelocity().vx,1e-9);
        assertTrue(r.core.motion().usable(),"a legitimate 80ms acceleration interval is not slip");
        assertEquals(.8,r.core.motion().fieldVelocity().getX(),1e-9);
        assertEquals(.032,r.core.motion().pose().getX(),1e-9);
    }
    @Test void fourSpinningWheelsCannotManufactureTravelOrRepairTheirOwnFault() {
        var r=new Rig();
        for(int i=0;i<100;i++) r.step(.06,3,0,0);
        assertTrue(r.core.gyroWheelEstimate().orElseThrow().valid());
        assertFalse(r.core.motion().usable());
        assertTrue(r.core.motion().requiresAbsoluteFix());
        assertTrue(r.core.motion().pose().getX()<.1);
        assertEquals(6,r.core.state().pose().getX(),1e-8);
        r.core.observe(PoseObservation.of(new Pose2d(6,0,Rotation2d.ZERO),r.now,"camera"));
        assertFalse(r.core.motion().usable(),"position alone cannot certify stopped ground motion");
    }
    @Test void omissionOfCoherentPositionsHoldsAndLaterWheelsCannotClearFault() {
        var r=new Rig();
        r.now=.02; r.core.update(r.sample(1,0,0));
        assertFalse(r.core.motion().usable());
        assertEquals(0,r.core.motion().pose().getX(),1e-9);
        r.now=.04; r.frame(1,0,0);
        assertFalse(r.core.motion().usable());
    }
    @Test void invalidRawVelocityCannotBeHiddenByValidEncoderConsensus() {
        var r=new Rig();
        r.now=.02; r.frame(0,Double.NaN,0);
        assertFalse(r.core.motion().usable());
        assertTrue(Double.isFinite(r.core.state().fieldVelocity().vx));
        assertEquals(0,r.core.state().timestampSeconds(),1e-9);
    }
    @Test void invalidNativePoseCannotPoisonLegacyState() {
        var r=new Rig();
        r.now=.02; r.frame(Double.NaN,0,0);
        assertFalse(r.core.motion().usable());
        assertEquals(0,r.core.state().pose().getX(),1e-9);
        assertEquals(0,r.core.state().timestampSeconds(),1e-9);
    }
    @Test void delayedWeightedPoseFixPersistsIntoNextGuardedSample() {
        var r=new Rig();
        r.core.resetMotion(); r.frame(0,1,0);
        r.step(.02,1,0,0); r.step(.02,1,0,0);
        assertTrue(r.core.observe(new PoseObservation(new Pose2d(.22,0,Rotation2d.ZERO),
            .02,.10,"camera")));
        r.step(.02,1,0,0);
        assertTrue(r.core.motion().usable());
        assertEquals(.16,r.core.motion().pose().getX(),1e-9);
        assertEquals(.06,r.core.state().pose().getX(),1e-9);
    }
    @Test void explicitRestSeedUsesResetEpochAndPreservesValidatedPosition() {
        var r=new Rig();
        r.now=.03; r.core.seedMotionAtRest(new Pose2d(4,1,Rotation2d.ZERO));
        r.now=.02; r.frame(0,0,0);
        assertEquals(4,r.core.motion().pose().getX(),1e-9);
        r.now=.04; r.frame(0,0,0);
        r.now=.06; r.frame(0,0,0);
        assertTrue(r.core.motion().usable());
        assertEquals(4,r.core.motion().pose().getX(),1e-9);
    }
    @Test void freshIndependentEvidenceCanRepairGapButEncoderConsensusCannot() {
        var r=new Rig();
        r.now=.20; r.frame(0,0,0);
        assertFalse(r.core.motion().usable());
        assertTrue(r.core.observe(new VelocityObservation(Translation2d.ZERO,.20,.01,"optical")));
        assertTrue(r.core.observe(PoseObservation.of(Pose2d.ZERO,.20,"camera")));
        assertTrue(r.core.motion().usable());
        r.now=.22; r.frame(0,0,0);
        assertTrue(r.core.motion().usable());
    }
}
