package frc.lib.catalyst.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.catalyst.subsystems.swerve.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * Configuration for VisionSubsystem with Kalman filter tuning parameters.
 *
 * <p>The standard deviation parameters control how the Kalman filter
 * weighs vision measurements against odometry. Lower values = more trust
 * in vision. These should be tuned for your specific robot and camera setup.
 *
 * <p>Example:
 * <pre>{@code
 * VisionConfig config = VisionConfig.builder()
 *     .addLimelight("limelight-front",
 *         new Transform3d(0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-15), 0)))
 *     .addPhotonCamera("cam-back",
 *         new Transform3d(-0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-20), Math.PI)),
 *         fieldLayout)
 *     .driveSubsystem(drive)
 *     .baseXYStdDev(0.3)           // trust XY pretty well
 *     .baseRotStdDev(0.7)          // trust rotation somewhat
 *     .xyDistanceScaling(0.5)      // moderate distance penalty
 *     .rotDistanceScaling(1.0)     // strong rotation distance penalty
 *     .maxAmbiguity(0.25)          // strict ambiguity filter
 *     .rejectDuringSpin(2.0)       // ignore vision when spinning fast
 *     .build();
 * }</pre>
 */
public class VisionConfig {

    final List<CameraSource> cameras;
    final SwerveSubsystem driveSubsystem;

    // Filtering
    final double maxAmbiguity;
    final double maxAcceptableDistance;
    final double maxLatencySeconds;
    final double rejectDuringSpinThreshold;

    // Kalman filter std dev tuning
    final double baseXYStdDev;
    final double baseRotStdDev;
    final double xyDistanceScaling;
    final double rotDistanceScaling;
    final double singleTagRotDistanceThreshold;

    private VisionConfig(Builder b) {
        this.cameras = List.copyOf(b.cameras);
        this.driveSubsystem = b.driveSubsystem;
        this.maxAmbiguity = b.maxAmbiguity;
        this.maxAcceptableDistance = b.maxAcceptableDistance;
        this.maxLatencySeconds = b.maxLatencySeconds;
        this.rejectDuringSpinThreshold = b.rejectDuringSpinThreshold;
        this.baseXYStdDev = b.baseXYStdDev;
        this.baseRotStdDev = b.baseRotStdDev;
        this.xyDistanceScaling = b.xyDistanceScaling;
        this.rotDistanceScaling = b.rotDistanceScaling;
        this.singleTagRotDistanceThreshold = b.singleTagRotDistanceThreshold;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private final List<CameraSource> cameras = new ArrayList<>();
        private SwerveSubsystem driveSubsystem;

        // Filtering defaults
        private double maxAmbiguity = 0.3;
        private double maxAcceptableDistance = 5.0;
        private double maxLatencySeconds = 0.5;
        private double rejectDuringSpinThreshold = 0; // 0 = disabled

        // Kalman filter std dev defaults
        private double baseXYStdDev = 0.5;
        private double baseRotStdDev = 0.9;
        private double xyDistanceScaling = 1.0;    // multiplied by distance^2
        private double rotDistanceScaling = 1.5;   // rotation degrades faster with distance
        private double singleTagRotDistanceThreshold = 4.0; // meters

        /**
         * Add a Limelight camera.
         * @param name Limelight NetworkTables name (e.g., "limelight-front")
         * @param robotToCamera transform from robot center to camera
         */
        public Builder addLimelight(String name, Transform3d robotToCamera) {
            cameras.add(new LimelightSource(name, robotToCamera));
            return this;
        }

        /**
         * Add a PhotonVision camera.
         * @param name PhotonVision camera name
         * @param robotToCamera transform from robot center to camera
         * @param fieldLayout AprilTag field layout
         */
        public Builder addPhotonCamera(String name, Transform3d robotToCamera,
                                       AprilTagFieldLayout fieldLayout) {
            cameras.add(new PhotonSource(name, robotToCamera, fieldLayout));
            return this;
        }

        /** Add a custom camera source. */
        public Builder addCamera(CameraSource camera) {
            cameras.add(camera);
            return this;
        }

        /** Set the swerve drive subsystem to feed vision poses to. */
        public Builder driveSubsystem(SwerveSubsystem drive) {
            this.driveSubsystem = drive;
            return this;
        }

        // --- Filtering ---

        /** Max pose ambiguity for single-tag estimates (default 0.3). */
        public Builder maxAmbiguity(double maxAmbiguity) {
            this.maxAmbiguity = maxAmbiguity;
            return this;
        }

        /** Max acceptable distance from current Kalman filter estimate in meters (default 5.0). */
        public Builder maxAcceptableDistance(double meters) {
            this.maxAcceptableDistance = meters;
            return this;
        }

        /** Max latency before rejecting a measurement as stale (default 0.5s). */
        public Builder maxLatency(double seconds) {
            this.maxLatencySeconds = seconds;
            return this;
        }

        /**
         * Reject vision measurements when the robot is spinning faster than this threshold.
         * Motion blur during fast rotation makes vision unreliable.
         * @param radiansPerSecond angular velocity threshold (0 = disabled, try 2.0-3.0)
         */
        public Builder rejectDuringSpin(double radiansPerSecond) {
            this.rejectDuringSpinThreshold = radiansPerSecond;
            return this;
        }

        // --- Kalman Filter Std Dev Tuning ---

        /**
         * Base XY standard deviation before distance scaling (default 0.5m).
         * Lower = more trust in vision XY position.
         * This value represents the expected accuracy at 1 meter with 1 tag.
         */
        public Builder baseXYStdDev(double stdDev) {
            this.baseXYStdDev = stdDev;
            return this;
        }

        /**
         * Base rotation standard deviation before distance scaling (default 0.9rad).
         * Lower = more trust in vision heading.
         */
        public Builder baseRotStdDev(double stdDev) {
            this.baseRotStdDev = stdDev;
            return this;
        }

        /**
         * How much distance affects XY standard deviations (default 1.0).
         * Final xyStdDev = baseXYStdDev * (1 + distance^2 * scaling).
         * Higher = less trust at distance.
         */
        public Builder xyDistanceScaling(double scaling) {
            this.xyDistanceScaling = scaling;
            return this;
        }

        /**
         * How much distance affects rotation standard deviations (default 1.5).
         * Rotation is typically less reliable than XY at distance.
         */
        public Builder rotDistanceScaling(double scaling) {
            this.rotDistanceScaling = scaling;
            return this;
        }

        /**
         * Distance threshold for single-tag rotation rejection (default 4.0m).
         * Beyond this distance, single-tag rotation is treated as infinitely uncertain
         * so the Kalman filter ignores it.
         */
        public Builder singleTagRotDistanceThreshold(double meters) {
            this.singleTagRotDistanceThreshold = meters;
            return this;
        }

        public VisionConfig build() {
            if (cameras.isEmpty()) {
                throw new IllegalStateException("At least one camera must be added");
            }
            if (driveSubsystem == null) {
                throw new IllegalStateException("Drive subsystem must be set for vision to feed poses");
            }
            return new VisionConfig(this);
        }
    }
}
