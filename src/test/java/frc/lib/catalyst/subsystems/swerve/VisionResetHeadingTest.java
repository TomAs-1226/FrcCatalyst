package frc.lib.catalyst.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class VisionResetHeadingTest {

    private static final Pose2d VISION = new Pose2d(3.2, 4.1, Rotation2d.fromDegrees(37));
    private static final Rotation2d GYRO = Rotation2d.fromDegrees(-12);

    @Test
    void enabledAVisionResetMovesThePositionAndKeepsTheGyroHeading() {
        Pose2d after = SwerveSubsystem.visionReset(VISION, GYRO, true);
        assertEquals(VISION.getTranslation(), after.getTranslation());
        assertEquals(GYRO.getDegrees(), after.getRotation().getDegrees(), 1e-9,
                "field-centric forward must not swing under a driving robot");
    }

    @Test
    void disabledTheSeedTakesTheVisionPoseWhole() {
        assertEquals(VISION, SwerveSubsystem.visionReset(VISION, GYRO, false));
    }
}
