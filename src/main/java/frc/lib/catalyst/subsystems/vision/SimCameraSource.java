package frc.lib.catalyst.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;
import java.util.Random;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A {@link CameraSource} that fabricates pose estimates from a known simulated
 * robot pose, so the whole {@link VisionSubsystem} fusion pipeline (accept /
 * reject, std-dev weighting, feeding the swerve estimator) can run in the
 * WPILib simulator without any real camera or AprilTag.
 *
 * <p>Pair it with the maple-sim seam or the drive simulation: feed it the true
 * simulated pose and it emits noisy, latency-delayed estimates just like a real
 * PhotonVision / Limelight source, so you can tune {@code maxAmbiguity}, the
 * std-dev model, and rejection thresholds before the robot exists.
 *
 * <pre>{@code
 * VisionConfig cfg = VisionConfig.builder()
 *     .driveSubsystem(drive)
 *     .addCamera(SimCameraSource.builder("SimFront")
 *         .truePose(() -> swerveSim.getSimulatedDriveTrainPose())
 *         .translationStdDevMeters(0.03)
 *         .rotationStdDevDegrees(1.5)
 *         .latencySeconds(0.03)
 *         .tagCount(2)
 *         .build())
 *     .build();
 * }</pre>
 */
public final class SimCameraSource implements CameraSource {

    private final String name;
    private final Supplier<Pose2d> truePose;
    private final BooleanSupplier hasTarget;
    private final double translationStdDev;   // meters (per axis)
    private final double rotationStdDev;       // radians
    private final double latencySeconds;
    private final int tagCount;
    private final double averageTagDistance;
    private final double ambiguity;
    private final Random random = new Random();

    private SimCameraSource(Builder b) {
        this.name = b.name;
        this.truePose = b.truePose;
        this.hasTarget = b.hasTarget;
        this.translationStdDev = b.translationStdDev;
        this.rotationStdDev = Math.toRadians(b.rotationStdDevDeg);
        this.latencySeconds = b.latencySeconds;
        this.tagCount = b.tagCount;
        this.averageTagDistance = b.averageTagDistance;
        this.ambiguity = b.ambiguity;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Optional<PoseEstimate> getEstimatedPose() {
        if (truePose == null || !hasTarget.getAsBoolean()) {
            return Optional.empty();
        }
        Pose2d truth = truePose.get();
        if (truth == null) {
            return Optional.empty();
        }
        Pose2d noisy = new Pose2d(
                truth.getX() + random.nextGaussian() * translationStdDev,
                truth.getY() + random.nextGaussian() * translationStdDev,
                truth.getRotation().plus(Rotation2d.fromRadians(random.nextGaussian() * rotationStdDev)));
        double timestamp = Timer.getFPGATimestamp() - latencySeconds;
        return Optional.of(new PoseEstimate(noisy, timestamp, tagCount, averageTagDistance, ambiguity));
    }

    public static Builder builder(String name) {
        return new Builder(name);
    }

    /** Fluent configuration for a {@link SimCameraSource}. */
    public static final class Builder {
        private final String name;
        private Supplier<Pose2d> truePose;
        private BooleanSupplier hasTarget = () -> true;
        private double translationStdDev = 0.03;
        private double rotationStdDevDeg = 1.5;
        private double latencySeconds = 0.03;
        private int tagCount = 2;
        private double averageTagDistance = 2.0;
        private double ambiguity = 0.1;

        private Builder(String name) {
            this.name = name;
        }

        /** Supplier of the true simulated robot pose (e.g. from maple-sim). Required. */
        public Builder truePose(Supplier<Pose2d> truePose) { this.truePose = truePose; return this; }

        /** Gate estimates on/off to simulate the target coming in and out of view. */
        public Builder hasTarget(BooleanSupplier hasTarget) { this.hasTarget = hasTarget; return this; }

        /** Per-axis translation noise standard deviation, meters (default 0.03). */
        public Builder translationStdDevMeters(double m) { this.translationStdDev = m; return this; }

        /** Heading noise standard deviation, degrees (default 1.5). */
        public Builder rotationStdDevDegrees(double deg) { this.rotationStdDevDeg = deg; return this; }

        /** Reported pipeline latency, seconds (default 0.03). */
        public Builder latencySeconds(double s) { this.latencySeconds = s; return this; }

        /** Number of tags the fake estimate "sees" (default 2 = multi-tag, low ambiguity). */
        public Builder tagCount(int n) { this.tagCount = n; return this; }

        /** Average tag distance in meters, used by the std-dev model (default 2.0). */
        public Builder averageTagDistance(double m) { this.averageTagDistance = m; return this; }

        /** Reported single-tag ambiguity (default 0.1). */
        public Builder ambiguity(double a) { this.ambiguity = a; return this; }

        public SimCameraSource build() {
            if (truePose == null) {
                throw new IllegalStateException("SimCameraSource requires a truePose supplier");
            }
            return new SimCameraSource(this);
        }
    }
}
