package frc.lib.catalyst.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.catalyst.subsystems.swerve.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Multi-camera vision subsystem with Kalman filter pose estimation.
 *
 * <p>This subsystem processes poses from all cameras each cycle and feeds them
 * into the drivetrain's built-in Kalman filter pose estimator (used by both
 * CTRE's SwerveDrivetrain and WPILib's SwerveDrivePoseEstimator). The Kalman
 * filter optimally fuses odometry and vision measurements, weighting each
 * by their uncertainty (standard deviation matrices).
 *
 * <p><b>How the Kalman filter fusion works:</b>
 * <ol>
 *   <li>Odometry provides continuous pose updates with low noise but drift over time</li>
 *   <li>Vision provides discrete pose updates with higher noise but no drift</li>
 *   <li>The Kalman filter weighs each source by its standard deviations:
 *       lower std dev = more trust in that measurement</li>
 *   <li>Multi-tag estimates get lower std devs (more accurate) than single-tag</li>
 *   <li>Closer tags get lower std devs than distant tags</li>
 * </ol>
 *
 * <p><b>Standard Deviation Tuning Guide:</b>
 * <ul>
 *   <li>Lower baseXYStdDev = trust vision XY more (default 0.5m)</li>
 *   <li>Lower baseRotStdDev = trust vision rotation more (default 0.9rad)</li>
 *   <li>The distance scaling factor quadratically increases std devs with distance</li>
 *   <li>Multi-tag divides std devs by tag count (2 tags = half the uncertainty)</li>
 *   <li>Set xyDistanceScaling/rotDistanceScaling for fine-grained control</li>
 * </ul>
 *
 * <p>Example usage:
 * <pre>{@code
 * VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
 *     .addLimelight("limelight-front",
 *         new Transform3d(0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-15), 0)))
 *     .addPhotonCamera("cam-back",
 *         new Transform3d(-0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-20), Math.PI)),
 *         fieldLayout)
 *     .driveSubsystem(drive)
 *     .baseXYStdDev(0.3)
 *     .baseRotStdDev(0.7)
 *     .maxAmbiguity(0.25)
 *     .build());
 * }</pre>
 */
public class VisionSubsystem extends SubsystemBase {

    private final VisionConfig config;
    private final List<CameraSource> cameras;
    private final SwerveSubsystem driveSubsystem;

    // Telemetry
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> visionPosePub;
    private int totalAccepted = 0;
    private int totalRejected = 0;
    private int cycleAccepted = 0;
    private int cycleRejected = 0;

    public VisionSubsystem(VisionConfig config) {
        this.config = config;
        this.cameras = config.cameras;
        this.driveSubsystem = config.driveSubsystem;

        telemetryTable = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("Vision");
        visionPosePub = telemetryTable.getStructTopic("LatestAcceptedPose", Pose2d.struct).publish();
    }

    @Override
    public void periodic() {
        if (driveSubsystem == null) return;

        cycleAccepted = 0;
        cycleRejected = 0;

        // Update robot orientation for cameras that need it (Limelight MegaTag2)
        Pose2d currentPose = driveSubsystem.getPose();
        double yaw = currentPose.getRotation().getDegrees();

        // Optionally also provide yaw rate for better MegaTag2 accuracy
        double yawRate = driveSubsystem.getChassisSpeeds().omegaRadiansPerSecond;

        for (CameraSource camera : cameras) {
            camera.setRobotOrientation(yaw, Math.toDegrees(yawRate), 0, 0);

            Optional<CameraSource.PoseEstimate> estimate = camera.getEstimatedPose();
            if (estimate.isEmpty()) continue;

            CameraSource.PoseEstimate pe = estimate.get();

            // --- Filtering ---
            String rejectReason = filterEstimate(pe, currentPose);
            if (rejectReason != null) {
                totalRejected++;
                cycleRejected++;
                logCamera(camera.getName(), "Rejected: " + rejectReason, pe);
                continue;
            }

            // --- Standard Deviation Scaling for Kalman Filter ---
            // These std devs tell the Kalman filter how much to trust this measurement.
            // Lower values = more trust. The filter optimally fuses vision with odometry.
            Matrix<N3, N1> stdDevs = calculateStdDevs(pe);

            // --- Feed to Kalman filter pose estimator ---
            // CTRE's SwerveDrivetrain internally uses a Kalman filter that fuses
            // odometry (predicted state) with vision measurements (corrections).
            // The standard deviation matrix determines the Kalman gain:
            //   K = P * H^T * (H * P * H^T + R)^-1
            // where R is constructed from our stdDevs.
            driveSubsystem.addVisionMeasurement(pe.pose(), pe.timestampSeconds(), stdDevs);
            totalAccepted++;
            cycleAccepted++;
            visionPosePub.set(pe.pose());
            logCamera(camera.getName(), "Accepted", pe);
        }

        // Overall telemetry
        telemetryTable.getEntry("TotalAccepted").setDouble(totalAccepted);
        telemetryTable.getEntry("TotalRejected").setDouble(totalRejected);
        telemetryTable.getEntry("CycleAccepted").setDouble(cycleAccepted);
        telemetryTable.getEntry("CycleRejected").setDouble(cycleRejected);
        telemetryTable.getEntry("CameraCount").setDouble(cameras.size());
    }

    /**
     * Filter a pose estimate. Returns null if accepted, or a rejection reason string.
     */
    private String filterEstimate(CameraSource.PoseEstimate pe, Pose2d currentPose) {
        // Reject if no tags seen
        if (pe.tagCount() == 0) return "NoTags";

        // Reject if ambiguity is too high (single tag only — multi-tag PnP has low ambiguity)
        if (pe.tagCount() == 1 && pe.ambiguity() > config.maxAmbiguity) {
            return "HighAmbiguity(" + String.format("%.2f", pe.ambiguity()) + ")";
        }

        // Reject if too far from current Kalman filter estimate (likely outlier)
        double distFromCurrent = currentPose.getTranslation()
                .getDistance(pe.pose().getTranslation());
        if (distFromCurrent > config.maxAcceptableDistance) {
            return "TooFar(" + String.format("%.1fm", distFromCurrent) + ")";
        }

        // Reject if pose is off the field (sanity check)
        double x = pe.pose().getX();
        double y = pe.pose().getY();
        if (x < -0.5 || x > 17.0 || y < -0.5 || y > 9.0) {
            return "OffField";
        }

        // Reject if timestamp is too old (stale data degrades Kalman filter accuracy)
        double latency = Timer.getFPGATimestamp() - pe.timestampSeconds();
        if (latency > config.maxLatencySeconds) {
            return "StaleData(" + String.format("%.0fms", latency * 1000) + ")";
        }

        // Reject during high angular velocity (motion blur)
        if (config.rejectDuringSpinThreshold > 0) {
            double spinRate = Math.abs(driveSubsystem.getChassisSpeeds().omegaRadiansPerSecond);
            if (spinRate > config.rejectDuringSpinThreshold) {
                return "Spinning(" + String.format("%.1frad/s", spinRate) + ")";
            }
        }

        return null; // accepted
    }

    /**
     * Calculate standard deviations for the Kalman filter based on measurement quality.
     *
     * <p>The Kalman filter uses these as the measurement noise covariance matrix R.
     * Lower standard deviations = higher Kalman gain = more correction from this measurement.
     *
     * <p>Scaling strategy:
     * <ul>
     *   <li>Base std devs represent accuracy at 1 meter with 1 tag</li>
     *   <li>Distance scaling: quadratic (distance^2) — farther = exponentially less accurate</li>
     *   <li>Tag count scaling: divide by tagCount — more tags = linear improvement</li>
     *   <li>Single distant tag: very high rotation std dev (rotation is unreliable)</li>
     * </ul>
     */
    private Matrix<N3, N1> calculateStdDevs(CameraSource.PoseEstimate pe) {
        double distance = pe.averageTagDistance();
        int tagCount = pe.tagCount();

        // Base standard deviations (tunable via config)
        double xyStdDev = config.baseXYStdDev;
        double rotStdDev = config.baseRotStdDev;

        // Scale by distance — quadratic because angular error grows with distance
        double distFactor = 1.0 + (distance * distance * config.xyDistanceScaling);
        double rotDistFactor = 1.0 + (distance * distance * config.rotDistanceScaling);
        xyStdDev *= distFactor;
        rotStdDev *= rotDistFactor;

        // Scale by tag count — more tags = PnP solution is more constrained
        if (tagCount >= 2) {
            xyStdDev /= tagCount;
            rotStdDev /= tagCount;
        }

        // Single tag at distance: rotation from a single tag is extremely noisy
        if (tagCount == 1 && distance > config.singleTagRotDistanceThreshold) {
            rotStdDev = 999.0; // effectively infinite uncertainty → Kalman filter ignores rotation
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
    }

    private void logCamera(String cameraName, String status, CameraSource.PoseEstimate pe) {
        NetworkTable camTable = telemetryTable.getSubTable(cameraName);
        camTable.getEntry("Status").setString(status);
        camTable.getEntry("TagCount").setDouble(pe.tagCount());
        camTable.getEntry("AvgDistance").setDouble(pe.averageTagDistance());
        camTable.getEntry("Ambiguity").setDouble(pe.ambiguity());
        camTable.getEntry("X").setDouble(pe.pose().getX());
        camTable.getEntry("Y").setDouble(pe.pose().getY());
        camTable.getEntry("RotDeg").setDouble(pe.pose().getRotation().getDegrees());
    }

    /** Get the list of camera sources. */
    public List<CameraSource> getCameras() {
        return cameras;
    }

    /** Get total accepted vision measurements this session. */
    public int getTotalAccepted() {
        return totalAccepted;
    }

    /** Get total rejected vision measurements this session. */
    public int getTotalRejected() {
        return totalRejected;
    }
}
