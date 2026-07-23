package frc.lib.catalyst.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.subsystems.swerve.SwerveSubsystem;
import frc.lib.catalyst.util.AlertManager;

import java.util.ArrayList;
import java.util.Comparator;
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
    private int totalAccepted = 0;
    private int totalRejected = 0;
    private int cycleAccepted = 0;
    private int cycleRejected = 0;

    public VisionSubsystem(VisionConfig config) {
        this.config = config;
        this.cameras = config.cameras;
        this.driveSubsystem = config.driveSubsystem;

        // Surface a loud warning if vision is constructed without a drive subsystem.
        // Previously this silently no-op'd in periodic(), which made the issue
        // invisible to teams expecting vision fusion to "just work."
        if (driveSubsystem == null) {
            AlertManager.getInstance().warning("Vision",
                    "VisionSubsystem constructed without driveSubsystem — pose fusion is disabled. "
                            + "Call VisionConfig.builder().driveSubsystem(...) to enable.");
        }
    }

    /**
     * One accepted measurement, held until the whole camera set has been
     * gathered so we can fuse them in a deterministic order rather than
     * in camera-list order. {@code cameraIndex} is the final stable
     * tiebreak so the result is reproducible with any number of cameras.
     */
    private record Accepted(
            int cameraIndex,
            String cameraName,
            CameraSource.PoseEstimate pe,
            Matrix<N3, N1> stdDevs,
            double quality) {}

    @Override
    public void periodic() {
        if (driveSubsystem == null) return;

        cycleAccepted = 0;
        cycleRejected = 0;

        Pose2d currentPose = driveSubsystem.getPose();
        double yaw = currentPose.getRotation().getDegrees();
        double yawRate = driveSubsystem.getChassisSpeeds().omegaRadiansPerSecond;

        // ---- Phase 1: snapshot every camera once, filter independently ----
        // Snapshotting up front means an async NT update mid-loop can't make
        // two reads of the same camera disagree, and it decouples gathering
        // from fusing so 4+ cameras fuse deterministically.
        List<Accepted> accepted = new ArrayList<>(cameras.size());

        for (int i = 0; i < cameras.size(); i++) {
            CameraSource camera = cameras.get(i);
            camera.setRobotOrientation(yaw, Math.toDegrees(yawRate), 0, 0);

            Optional<CameraSource.PoseEstimate> estimate = camera.getEstimatedPose();
            if (estimate.isEmpty()) continue;

            CameraSource.PoseEstimate pe = estimate.get();

            // Guard against NaN/garbage poses before anything else touches them.
            if (!isFinitePose(pe)) {
                totalRejected++;
                cycleRejected++;
                logCamera(camera.getName(), "Rejected: NonFinite", pe);
                continue;
            }

            String rejectReason = filterEstimate(pe, currentPose);
            if (rejectReason != null) {
                totalRejected++;
                cycleRejected++;
                logCamera(camera.getName(), "Rejected: " + rejectReason, pe);
                continue;
            }

            Matrix<N3, N1> stdDevs = calculateStdDevs(pe);
            accepted.add(new Accepted(i, camera.getName(), pe, stdDevs, qualityScore(pe)));
        }

        // ---- Phase 2: fuse in a deterministic order ----
        // Add by ascending timestamp so the Kalman filter integrates
        // measurements chronologically (out-of-order adds cause jitter as the
        // estimator rewinds and replays). Ties break on quality (best first),
        // then camera index — fully reproducible regardless of camera count.
        accepted.sort(Comparator
                .comparingDouble((Accepted a) -> a.pe().timestampSeconds())
                .thenComparing(Comparator.comparingDouble((Accepted a) -> a.quality()).reversed())
                .thenComparingInt(Accepted::cameraIndex));

        Accepted best = null;
        for (Accepted a : accepted) {
            driveSubsystem.addVisionMeasurement(a.pe().pose(), a.pe().timestampSeconds(), a.stdDevs());
            totalAccepted++;
            cycleAccepted++;
            logCamera(a.cameraName(), "Accepted", a.pe());

            double[] innovation = calculateInnovation(a.pe().pose(), currentPose);
            double innovationNorm = Math.hypot(innovation[0], innovation[1]);
            CatalystLog.log("Vision/" + a.cameraName() + "/InnovationXY", innovationNorm);
            CatalystLog.log("Vision/" + a.cameraName() + "/InnovationRot", Math.toDegrees(Math.abs(innovation[2])));

            if (best == null || a.quality() > best.quality()) best = a;
        }

        // Publish the single highest-quality accepted pose (deterministic),
        // instead of "whichever camera happened to be processed last".
        if (best != null) CatalystLog.log("Vision/LatestAcceptedPose", Pose2d.struct, best.pe().pose());

        CatalystLog.log("Vision/TotalAccepted", (double) totalAccepted);
        CatalystLog.log("Vision/TotalRejected", (double) totalRejected);
        CatalystLog.log("Vision/CycleAccepted", (double) cycleAccepted);
        CatalystLog.log("Vision/CycleRejected", (double) cycleRejected);
        CatalystLog.log("Vision/CameraCount", (double) cameras.size());
    }

    /** Higher = better. More tags and closer tags raise the score. */
    private static double qualityScore(CameraSource.PoseEstimate pe) {
        return pe.tagCount() / (1.0 + pe.averageTagDistance())
                / (1.0 + 5.0 * Math.max(0, pe.ambiguity()));
    }

    /** Reject poses containing NaN/Inf before they reach the estimator. */
    private static boolean isFinitePose(CameraSource.PoseEstimate pe) {
        Pose2d p = pe.pose();
        return Double.isFinite(p.getX())
                && Double.isFinite(p.getY())
                && Double.isFinite(p.getRotation().getRadians())
                && Double.isFinite(pe.timestampSeconds());
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

        // Reject if pose is off the field (configurable bounds with margin)
        double x = pe.pose().getX();
        double y = pe.pose().getY();
        double margin = config.fieldBoundsMargin;
        if (x < -margin || x > config.fieldLengthMeters + margin
                || y < -margin || y > config.fieldWidthMeters + margin) {
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

        // Reject during high translational speed (configurable)
        if (config.rejectDuringHighSpeedThreshold > 0) {
            ChassisSpeeds speeds = driveSubsystem.getChassisSpeeds();
            double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            if (speed > config.rejectDuringHighSpeedThreshold) {
                return "HighSpeed(" + String.format("%.1fm/s", speed) + ")";
            }
        }

        // Reject based on heading consistency (vision heading vs gyro heading)
        if (config.maxHeadingDivergenceDegrees > 0 && pe.tagCount() == 1) {
            double headingDiff = Math.abs(
                    currentPose.getRotation().getDegrees() - pe.pose().getRotation().getDegrees());
            if (headingDiff > 180) headingDiff = 360 - headingDiff;
            if (headingDiff > config.maxHeadingDivergenceDegrees) {
                return "HeadingDivergence(" + String.format("%.0fdeg", headingDiff) + ")";
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

        // Scale by ambiguity — higher ambiguity = less trust
        if (pe.ambiguity() > 0.05) {
            double ambiguityScale = 1.0 + (pe.ambiguity() * 5.0);
            xyStdDev *= ambiguityScale;
            rotStdDev *= ambiguityScale;
        }

        // Single tag at distance: rotation from a single tag is extremely noisy
        if (tagCount == 1 && distance > config.singleTagRotDistanceThreshold) {
            rotStdDev = 999.0; // effectively infinite uncertainty → Kalman filter ignores rotation
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
    }

    /**
     * Calculate the Kalman innovation (difference between predicted and measured pose).
     * Large innovations indicate the vision measurement significantly disagrees with
     * the current estimate. Useful for diagnostics and adaptive filtering.
     *
     * @param visionPose the vision-estimated pose
     * @param currentPose the current Kalman filter estimate
     * @return innovation vector [dx, dy, dtheta]
     */
    private double[] calculateInnovation(Pose2d visionPose, Pose2d currentPose) {
        double dx = visionPose.getX() - currentPose.getX();
        double dy = visionPose.getY() - currentPose.getY();
        double dtheta = visionPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
        // Normalize theta
        while (dtheta > Math.PI) dtheta -= 2 * Math.PI;
        while (dtheta < -Math.PI) dtheta += 2 * Math.PI;
        return new double[]{dx, dy, dtheta};
    }

    private void logCamera(String cameraName, String status, CameraSource.PoseEstimate pe) {
        CatalystLog.log("Vision/" + cameraName + "/Status", status);
        CatalystLog.log("Vision/" + cameraName + "/TagCount", (double) pe.tagCount());
        CatalystLog.log("Vision/" + cameraName + "/AvgDistance", pe.averageTagDistance());
        CatalystLog.log("Vision/" + cameraName + "/Ambiguity", pe.ambiguity());
        CatalystLog.log("Vision/" + cameraName + "/X", pe.pose().getX());
        CatalystLog.log("Vision/" + cameraName + "/Y", pe.pose().getY());
        CatalystLog.log("Vision/" + cameraName + "/RotDeg", pe.pose().getRotation().getDegrees());
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
