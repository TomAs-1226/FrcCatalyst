package frc.lib.catalyst.hardware;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
class ImuTrustRegressionTest {
    private record Fixed(double rate) implements CatalystIMU {
        public Rotation2d getHeading(){return Rotation2d.ZERO;}
        public double getYaw(){return 0;}public double getPitch(){return 0;}public double getRoll(){return 0;}
        public double getYawRate(){return rate;}public void zeroYaw(){}public void setYaw(double d){}
    }
    private static DualIMU pair(){return new DualIMU(new Fixed(100),new Fixed(80),
            new Translation2d(.3,0),Translation2d.ZERO);}
    @Test void hugeAndTinyFiniteNoiseStillProduceFiniteEqualWeight() {
        for(double sigma:new double[]{1e200,1e-200}) {
            var imu=pair().withYawRateNoise(sigma,sigma);
            assertEquals(.5,imu.primaryYawRateWeight(),1e-12);assertEquals(90,imu.getYawRate(),1e-12);
        }
    }
    @Test void nonfiniteNoiseIsRejectedWithoutDestroyingPreviousWeight() {
        var imu=pair().withYawRateNoise(.1,.2);double previous=imu.primaryYawRateWeight();
        assertThrows(IllegalArgumentException.class,()->imu.withYawRateNoise(Double.POSITIVE_INFINITY,1));
        assertEquals(previous,imu.primaryYawRateWeight());
    }
    @Test void verticalRawAxesCannotBeCertifiedByAnInPlaneRotation() {
        assertTrue(HAL.initialize());
        for(var mount:new OnboardIMU.MountOrientation[]{OnboardIMU.MountOrientation.LANDSCAPE,
                OnboardIMU.MountOrientation.PORTRAIT}) {
            var imu=new SystemCoreIMU(mount,Rotation2d.fromDegrees(90));
            assertFalse(imu.isYawRateTrustworthy());assertEquals(0,imu.getYawRate());
            assertTrue(imu.getAcceleration().isEmpty());
        }
    }
    @Test void unavailableSecondaryDoesNotHalveThePrimaryYawRate() {
        assertTrue(HAL.initialize());
        var secondary=new SystemCoreIMU(OnboardIMU.MountOrientation.LANDSCAPE);
        assertFalse(secondary.isYawRateTrustworthy());
        var fused=new DualIMU(new Fixed(100),secondary,new Translation2d(.3,0),Translation2d.ZERO)
                .withYawRateNoise(.2,.2);
        assertEquals(100,fused.getYawRate(),1e-12);
    }
}
