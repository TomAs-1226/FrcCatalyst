package frc.lib.catalyst.subsystems.vision;

import org.wpilib.math.geometry.Pose2d;
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

    /**
     * Whether this source already rejects bad estimates before returning them.
     *
     * <p>This exists to divide the work honestly rather than do it twice. The two sides filter on
     * different information and neither can do the other's job:
     *
     * <ul>
     *   <li><b>The camera</b> knows what it can see — tag count, ambiguity, tag distance and area,
     *       field bounds, how old the frame is. LimelightLib 2 does all of this, and better than
     *       Catalyst could, because it has the raw detections.</li>
     *   <li><b>Catalyst</b> knows what the robot is doing — how far the estimate sits from the
     *       current fused pose, how fast the robot is spinning or driving, whether the estimate's
     *       heading disagrees with the gyro. The camera cannot compute any of those; it has no idea
     *       where the robot thinks it is.</li>
     * </ul>
     *
     * <p>When this is true, {@code VisionSubsystem} skips the checks the camera has already made and
     * applies only the robot-aware ones. Running both would mean two sets of thresholds for the same
     * rejection — a team tunes one, the other silently keeps rejecting, and the symptom is vision
     * that "doesn't work" for no visible reason.
     *
     * <p>Defaults to false, which is right for any source that hands back raw estimates.
     */
    default boolean isPrefiltered() {
        return false;
    }

    /**
     * Whether data is arriving from this camera at all.
     *
     * <p>Not "does it see a target" - that is {@link #getEstimatedPose()} being empty, which is
     * normal for most of a match. This is the difference between a camera with nothing in view and
     * a camera that is unplugged, and it is the one question about vision that is asked most often
     * and was previously answerable only by walking round the robot.
     *
     * <p>A source that has no way to tell answers {@code true}: reporting a working camera as
     * absent would be the worse error. {@code LimelightSource} answers from the camera's heartbeat.
     */
    default boolean isConnected() {
        return true;
    }

    /** A pose estimate from a vision camera. */
    record PoseEstimate(
            Pose2d pose,
            double timestampSeconds,
            int tagCount,
            double averageTagDistance,
            double ambiguity
    ) {}
}
