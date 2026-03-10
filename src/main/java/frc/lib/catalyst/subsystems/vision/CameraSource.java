package frc.lib.catalyst.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

/**
 * Interface for a camera that provides robot pose estimates.
 * Implemented by LimelightSource and PhotonSource.
 */
public interface CameraSource {

    /** Get the camera's name. */
    String getName();

    /**
     * Get the latest estimated robot pose from this camera.
     * Returns empty if no valid estimate is available.
     */
    Optional<PoseEstimate> getEstimatedPose();

    /**
     * Set the robot's current orientation for cameras that use it
     * (e.g., Limelight MegaTag2).
     */
    default void setRobotOrientation(double yawDegrees, double yawRate,
                                     double pitchDegrees, double rollDegrees) {}

    /** A pose estimate from a vision camera. */
    record PoseEstimate(
            Pose2d pose,
            double timestampSeconds,
            int tagCount,
            double averageTagDistance,
            double ambiguity
    ) {}
}
