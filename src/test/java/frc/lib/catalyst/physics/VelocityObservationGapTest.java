package frc.lib.catalyst.physics;
import static org.junit.jupiter.api.Assertions.*;
import frc.lib.catalyst.physics.estimation.PhysicalStateEstimator;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
class VelocityObservationGapTest {
    @Test void replacingVelocityAfterGapOrReplaySeekDiscardsOldObservationPrecision() {
        for(double next:new double[]{100,9}) {
            var estimator=PhysicalStateEstimator.builder().build();
            estimator.update(10,Pose2d.ZERO,new ChassisVelocities(),null,0,0);
            assertTrue(estimator.applyVelocityObservation(Translation2d.ZERO,.001));
            assertTrue(estimator.state().quality().velocityStdDevMetersPerSecond()<.002);
            var reset=estimator.update(next,Pose2d.ZERO,new ChassisVelocities(3,0,0),null,0,0);
            assertEquals(3,reset.fieldVelocity().vx,1e-12);
            var unobserved=PhysicalStateEstimator.builder().build()
                    .update(next,Pose2d.ZERO,new ChassisVelocities(3,0,0),null,0,0);
            assertEquals(unobserved.quality().velocityStdDevMetersPerSecond(),
                    reset.quality().velocityStdDevMetersPerSecond(),1e-12,
                    "an old independent observation cannot certify a newly rebased wheel velocity");
        }
    }
}
