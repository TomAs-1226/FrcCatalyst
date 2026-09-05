package frc.lib.catalyst.subsystems.vision;

import org.wpilib.math.geometry.Transform3d;
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
    /** Where accepted poses go. The drive subsystem when one was given, else whatever poseSink() set. */
    final VisionPoseSink poseSink;
    // Health thresholds
    final double cameraHotCelsius;
    final double cameraMinFps;
    final double staleFrameSeconds;

    // Filtering
    final double maxAmbiguity;
    final double maxAcceptableDistance;
    final boolean seedFromVision;
    final double reanchorAfterSeconds;
    final double maxLatencySeconds;
    final double rejectDuringSpinThreshold;

    // Kalman filter std dev tuning
    final double baseXYStdDev;
    final double baseRotStdDev;
    final double xyDistanceScaling;
    final double rotDistanceScaling;
    final double singleTagRotDistanceThreshold;

    // Advanced filtering
    final double rejectDuringHighSpeedThreshold;
    final double maxHeadingDivergenceDegrees;

    // Field dimensions (configurable for different games)
    final double fieldLengthMeters;
    final double fieldWidthMeters;
    final double fieldBoundsMargin;

    private VisionConfig(Builder b) {
        this.cameras = List.copyOf(b.cameras);
        this.driveSubsystem = b.driveSubsystem;
        this.poseSink = b.poseSink != null ? b.poseSink : b.driveSubsystem;
        this.cameraHotCelsius = b.cameraHotCelsius;
        this.cameraMinFps = b.cameraMinFps;
        this.staleFrameSeconds = b.staleFrameSeconds;
        this.maxAmbiguity = b.maxAmbiguity;
        this.maxAcceptableDistance = b.maxAcceptableDistance;
        this.seedFromVision = b.seedFromVision;
        this.reanchorAfterSeconds = b.reanchorAfterSeconds;
        this.maxLatencySeconds = b.maxLatencySeconds;
        this.rejectDuringSpinThreshold = b.rejectDuringSpinThreshold;
        this.baseXYStdDev = b.baseXYStdDev;
        this.baseRotStdDev = b.baseRotStdDev;
        this.xyDistanceScaling = b.xyDistanceScaling;
        this.rotDistanceScaling = b.rotDistanceScaling;
        this.singleTagRotDistanceThreshold = b.singleTagRotDistanceThreshold;
        this.rejectDuringHighSpeedThreshold = b.rejectDuringHighSpeedThreshold;
        this.maxHeadingDivergenceDegrees = b.maxHeadingDivergenceDegrees;
        this.fieldLengthMeters = b.fieldLengthMeters;
        this.fieldWidthMeters = b.fieldWidthMeters;
        this.fieldBoundsMargin = b.fieldBoundsMargin;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private VisionPoseSink poseSink;
        private double cameraHotCelsius = 80.0;
        private double cameraMinFps = 10.0;
        private double staleFrameSeconds = 1.0;

        private final List<CameraSource> cameras = new ArrayList<>();
        private SwerveSubsystem driveSubsystem;

        // Filtering defaults
        private double maxAmbiguity = 0.3;
        private double maxAcceptableDistance = 5.0;
        private boolean seedFromVision = true;
        private double reanchorAfterSeconds = 1.0;
        private double maxLatencySeconds = 0.5;
        private double rejectDuringSpinThreshold = 0; // 0 = disabled

        // Kalman filter std dev defaults
        private double baseXYStdDev = 0.5;
        private double baseRotStdDev = 0.9;
        private double xyDistanceScaling = 1.0;    // multiplied by distance^2
        private double rotDistanceScaling = 1.5;   // rotation degrades faster with distance
        private double singleTagRotDistanceThreshold = 4.0; // meters

        // Advanced filtering
        private double rejectDuringHighSpeedThreshold = 0; // 0 = disabled, m/s
        private double maxHeadingDivergenceDegrees = 0; // 0 = disabled

        // Field dimensions
        private double fieldLengthMeters = 16.54;
        private double fieldWidthMeters = 8.21;
        private double fieldBoundsMargin = 0.5;

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
         * Add a Limelight camera, choosing MegaTag2 (needs the robot yaw every loop, which
         * {@code VisionSubsystem} supplies; steadier at range) or MegaTag1 (self-contained).
         * Either way the camera is read with MegaTag1 until the pose has been seeded.
         */
        public Builder addLimelight(String name, Transform3d robotToCamera, boolean useMegaTag2) {
            cameras.add(new LimelightSource(name, robotToCamera, useMegaTag2));
            return this;
        }

        // addPhotonCamera(...) was removed in 2.0.0.
        //
        // PhotonVision has no 2027 vendordep, so PhotonSource cannot be built on this branch, and
        // the AprilTagFieldLayout it took no longer exists either - WPILib 2027 replaced it with
        // org.wpilib.fields.Field. Catalyst is Limelight-first on Systemcore because the pipeline
        // is built into the hardware; use addLimelight(...) or addCamera(...) with your own
        // CameraSource. The PhotonSource file is kept out of the compile rather than deleted, so
        // this can come back if PhotonVision ships for 2027.

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

        /**
         * Where accepted poses go, for a robot without a Catalyst drivetrain.
         *
         * <p>{@link #driveSubsystem(SwerveSubsystem)} is this with the swerve as the sink, and is
         * still the normal call. This one is for everything else: a tank drive, a team's own swerve
         * code, or a bench with four cameras and no drivetrain - give it a
         * {@link StandaloneVisionPose} and the whole pipeline runs.
         *
         * <p>If both are set, this one wins.
         */
        public Builder poseSink(VisionPoseSink sink) {
            this.poseSink = sink;
            return this;
        }

        /**
         * Camera temperature at which {@link VisionHealth} calls a camera hot (default 80 C).
         *
         * <p>A Limelight 4 throttles its pipeline when hot, which shows up as a slow camera and then
         * as an absent one. The number is the camera's self-reported CPU temperature.
         */
        public Builder cameraHotCelsius(double celsius) {
            this.cameraHotCelsius = celsius;
            return this;
        }

        /** Reported frame rate below which a camera is called slow (default 10). */
        public Builder cameraMinFps(double fps) {
            this.cameraMinFps = fps;
            return this;
        }

        /** How long a connected camera's frames may stop before it is called stale (default 1 s). */
        public Builder staleFrameSeconds(double seconds) {
            this.staleFrameSeconds = seconds;
            return this;
        }

        // --- Filtering ---

        /** Max pose ambiguity for single-tag estimates (default 0.3). */
        public Builder maxAmbiguity(double maxAmbiguity) {
            this.maxAmbiguity = maxAmbiguity;
            return this;
        }

        /**
         * Max acceptable distance from current Kalman filter estimate in meters (default 5.0).
         *
         * <p>Measured against the pose only once it has been seeded (see {@link #seedFromVision})
         * or the sink reports {@link VisionPoseSink#hasPose()}. Before that there is nothing real
         * to measure from.
         */
        public Builder maxAcceptableDistance(double meters) {
            this.maxAcceptableDistance = meters;
            return this;
        }

        /**
         * Whether the first good estimate replaces the pose outright instead of being fused into it
         * (default true). A drivetrain boots believing it is at the origin; a robot carried onto the
         * field is not, and every estimate would fail the distance gate against that origin
         * forever. Seeding resets the pose to the first estimate with two or more tags, or a single
         * tag within three metres, and gates against it from then on. Limelights are read with
         * MegaTag1 until seeded, because MegaTag2 needs the very heading the seed provides.
         */
        public Builder seedFromVision(boolean enabled) {
            this.seedFromVision = enabled;
            return this;
        }

        /**
         * How long every camera may disagree with the pose - all of them rejected as too far, none
         * accepted, and their estimates agreeing with each other - before the pose is reset to what
         * they see (default 1.0 s; 0 disables). This is the recovery from odometry that has gone
         * wrong: a wheel that slipped over a defence, a gyro that drifted, a pose reset to the
         * wrong auto start. It is counted and alerted, because it means something upstream was wrong.
         */
        public Builder reanchorAfterSeconds(double seconds) {
            this.reanchorAfterSeconds = seconds;
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

        // --- Advanced Filtering ---

        /**
         * Reject vision measurements when robot is moving faster than threshold.
         * Motion blur during fast translation makes vision unreliable.
         * @param metersPerSecond speed threshold (0 = disabled, try 3.0-4.0)
         */
        public Builder rejectDuringHighSpeed(double metersPerSecond) {
            this.rejectDuringHighSpeedThreshold = metersPerSecond;
            return this;
        }

        /**
         * Reject single-tag poses where vision heading diverges from gyro heading.
         * Useful for catching flipped AprilTag ambiguity.
         * @param degrees max allowable divergence (0 = disabled, try 20-40)
         */
        public Builder maxHeadingDivergence(double degrees) {
            this.maxHeadingDivergenceDegrees = degrees;
            return this;
        }

        /**
         * Set field dimensions for off-field rejection.
         * Defaults to 2025+ FRC field (16.54m x 8.21m).
         * @param lengthMeters field length (X dimension)
         * @param widthMeters field width (Y dimension)
         */
        public Builder fieldDimensions(double lengthMeters, double widthMeters) {
            this.fieldLengthMeters = lengthMeters;
            this.fieldWidthMeters = widthMeters;
            return this;
        }

        /**
         * Set the margin for off-field rejection (default 0.5m).
         * Poses within this margin outside the field are still accepted.
         */
        public Builder fieldBoundsMargin(double meters) {
            this.fieldBoundsMargin = meters;
            return this;
        }

        public VisionConfig build() {
            if (cameras.isEmpty()) {
                throw new IllegalStateException("At least one camera must be added");
            }
            // No sink at all is allowed: the cameras are still watched, their health still published,
            // and VisionSubsystem says at construction that nothing is being fused. A team on the
            // bench with cameras and no drivetrain is exactly who needs that, and a thrown exception
            // here was why the no-drivetrain path could never be reached.
            return new VisionConfig(this);
        }
    }
}
