package frc.lib.catalyst.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.linalg.VecBuilder;

class StandaloneVisionPoseTest {

    private static final Pose2d A = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30));
    private static final Pose2d B = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(-45));

    @Test
    void startsWithNoPose() {
        StandaloneVisionPose p = new StandaloneVisionPose();
        assertFalse(p.hasPose());
        assertEquals(Pose2d.kZero, p.getPose());
        assertEquals(0, p.measurementCount());
        assertTrue(p.lastMeasurementTimestamp().isEmpty());
        assertEquals(0.0, p.getChassisSpeeds().vx);
        assertEquals(0.0, p.getChassisSpeeds().omega);
    }

    @Test
    void theLatestMeasurementIsThePose() {
        StandaloneVisionPose p = new StandaloneVisionPose();
        p.addVisionMeasurement(A, 10.0, VecBuilder.fill(0.1, 0.1, 0.1));
        assertTrue(p.hasPose());
        assertEquals(A, p.getPose());
        p.addVisionMeasurement(B, 11.0, VecBuilder.fill(0.1, 0.1, 0.1));
        assertEquals(B, p.getPose());
        assertEquals(2, p.measurementCount());
        assertEquals(11.0, p.lastMeasurementTimestamp().getAsDouble());
    }

    @Test
    void anOlderFrameCannotMoveThePoseBackwards() {
        StandaloneVisionPose p = new StandaloneVisionPose();
        p.addVisionMeasurement(A, 10.0, VecBuilder.fill(0.1, 0.1, 0.1));
        p.addVisionMeasurement(B, 9.5, VecBuilder.fill(0.1, 0.1, 0.1));
        assertEquals(A, p.getPose());
        assertEquals(1, p.measurementCount());
    }

    @Test
    void resetForgetsEverything() {
        StandaloneVisionPose p = new StandaloneVisionPose();
        p.addVisionMeasurement(A, 10.0, VecBuilder.fill(0.1, 0.1, 0.1));
        p.reset();
        assertFalse(p.hasPose());
        assertEquals(0, p.measurementCount());
        // And an older-than-before frame is accepted again, because there is no "before".
        p.addVisionMeasurement(B, 1.0, VecBuilder.fill(0.1, 0.1, 0.1));
        assertEquals(B, p.getPose());
    }
}
