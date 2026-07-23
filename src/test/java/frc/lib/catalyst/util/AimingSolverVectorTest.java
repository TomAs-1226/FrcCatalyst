package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pure-math checks for the vector-adding shoot-on-the-fly solver. No HAL, no hardware.
 */
class AimingSolverVectorTest {

    private static final Translation2d TARGET = new Translation2d(8.0, 4.0);

    private static AimingSolverVector solver() {
        InterpolatingDoubleTreeMap rps = new InterpolatingDoubleTreeMap();
        rps.put(0.0, 40.0);
        rps.put(10.0, 60.0);
        return new AimingSolverVector.Builder()
                .rpsMap(rps)
                .staticHood(Rotation2d.fromDegrees(40.0))
                .targetPosition(TARGET)
                .wheelDiameterInches(4.0)
                .efficiency(0.5)
                .build();
    }

    @Test
    void stationaryRobotAimsStraightAtTheTarget() {
        // From (4,4), the target at (8,4) is due +X, so the field-relative yaw is 0 degrees.
        AimingSolverVector.TargetState s = solver().calculate(
                new Pose2d(4.0, 4.0, Rotation2d.fromDegrees(123.0)),
                new ChassisSpeeds(0, 0, 0));
        assertEquals(0.0, s.yaw().getDegrees(), 1e-6,
                "a stationary robot must aim straight at the target");
        assertEquals(40.0, s.hoodPitch().getDegrees(), 1e-6, "fixed hood stays at its angle");
        assertTrue(s.rps() > 0, "flywheel speed must be positive");
    }

    @Test
    void drivingLeftOfTheTargetLeadsTheShotAndNeedsMoreSpeed() {
        Pose2d pose = new Pose2d(4.0, 4.0, Rotation2d.fromDegrees(0.0));
        double stationaryRps = solver().calculate(pose, new ChassisSpeeds(0, 0, 0)).rps();

        // Strafing in +Y (perpendicular to the shot) must swing the aim off the straight bearing
        // and require a faster wheel to keep the same range.
        AimingSolverVector.TargetState moving = solver().calculate(
                pose, new ChassisSpeeds(0.0, 2.0, 0.0));
        assertTrue(Math.abs(moving.yaw().getDegrees()) > 1.0,
                "crossing velocity must lead the aim off the straight bearing, got "
                        + moving.yaw().getDegrees());
        assertTrue(moving.rps() > stationaryRps,
                "compensating for crossing motion needs more wheel speed");
    }

    @Test
    void buildRejectsMissingConfiguration() {
        try {
            new AimingSolverVector.Builder().targetPosition(TARGET).build();
            org.junit.jupiter.api.Assertions.fail("expected an exception for missing maps");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().toLowerCase().contains("map"));
        }
    }
}
