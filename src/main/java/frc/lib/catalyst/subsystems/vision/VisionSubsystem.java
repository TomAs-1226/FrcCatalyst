package frc.lib.catalyst.subsystems.vision;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;
import org.wpilib.command3.Mechanism;
import frc.lib.catalyst.identity.RobotIdentity;
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
public class VisionSubsystem extends frc.lib.catalyst.command.CatalystSubsystem {

    private final VisionConfig config;
    private final List<CameraSource> cameras;

    /** Cross-camera transform check. Inert with fewer than two cameras. */
    private final CameraAgreement agreement = new CameraAgreement();
    private final VisionPoseSink poseSink;
    private final VisionHealth health;

    // Telemetry
    private int totalAccepted = 0;
    private int totalRejected = 0;
    private int cycleAccepted = 0;
    private int cycleRejected = 0;

    // Seeding and re-anchoring - see VisionPoseSink's class note.
    /** A single tag this close is trusted to seed the pose; farther, wait for a better view. */
    static final double SINGLE_TAG_SEED_RANGE_M = 3.0;
    /** Rejected estimates must agree with each other this well to count as one disagreement. */
    private static final double REANCHOR_AGREEMENT_M = 1.0;
    private boolean seeded;
    private double disagreementSince = Double.NaN;
    private Pose2d disagreementPose = null;
    private int reanchors = 0;

    public VisionSubsystem(VisionConfig config) {
        this.config = config;
        this.cameras = config.cameras;
        this.poseSink = config.poseSink;
        this.health = new VisionHealth(VisionHealth.probesFor(cameras), new VisionHealth.Thresholds(
                config.cameraHotCelsius, config.cameraMinFps, config.staleFrameSeconds, 0.2));
        // Without seeding, the pose is anchored whenever the sink says it has one - the old rule.
        this.seeded = !config.seedFromVision;
        for (CameraSource c : cameras) {
            String kind = c instanceof LimelightSource ? "Limelight" : c.getClass().getSimpleName();
            frc.lib.catalyst.identity.DeviceRoster.registerCamera(c.getName(), kind, c::isConnected);
        }
        // Commands v3's Mechanism has no constructor to hook, so periodic() is registered
        // explicitly. Without this the method compiles and is simply never called - see
        // CatalystSubsystem#registerPeriodic.
        registerPeriodic();

        // Surface a loud warning if vision is constructed without a drive subsystem.
        // Previously this silently no-op'd in periodic(), which made the issue
        // invisible to teams expecting vision fusion to "just work."
        if (poseSink == null) {
            AlertManager.getInstance().warning("Vision",
                    "VisionSubsystem constructed without a pose sink — pose fusion is disabled. "
                            + "Call VisionConfig.builder().driveSubsystem(...) or .poseSink(...) to enable.");
        }
        // MegaTag2 needs a yaw from somewhere other than itself. Fed from a standalone pose it
        // gets its own last answer back, which is not a constraint, it is a loop.
        if (poseSink instanceof StandaloneVisionPose) {
            for (CameraSource c : cameras) {
                if (c instanceof LimelightSource ll && ll.isUsingMegaTag2()) {
                    AlertManager.getInstance().warning("Vision",
                            ll.getName() + " uses MegaTag2 with a standalone pose sink - there is no "
                                    + "gyro to constrain it. Construct it with useMegaTag2=false.");
                }
            }
        }

        // Names only, not this subsystem: RobotIdentity must never have to load a vision class, so a
        // robot project that excludes the PhotonVision artifacts still publishes its spec sheet.
        RobotIdentity.observeCameras(cameras.stream().map(CameraSource::getName).toList());
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
        // No sink means nothing to fuse into, but the cameras are still worth watching: their
        // health and per-camera telemetry are the whole point of a bench with no drivetrain.
        boolean fusing = poseSink != null;
        double now = Timer.getTimestamp();

        cycleAccepted = 0;
        cycleRejected = 0;

        Pose2d currentPose = fusing ? poseSink.getPose() : Pose2d.ZERO;
        // A sink with no pose yet cannot anchor the gates that compare against it - and neither can
        // a drivetrain that has a pose but has never been told where it really is.
        boolean anchored = fusing && poseSink.hasPose() && seeded;
        boolean seeding = fusing && !seeded;
        for (CameraSource c : cameras) {
            if (c instanceof LimelightSource ll) {
                ll.setSeeding(seeding);
            }
        }
        // The best too-far rejection this cycle: what the cameras would have the pose be.
        Accepted disagreement = null;
        double yaw = currentPose.getRotation().getDegrees();
        double yawRate = fusing ? poseSink.getChassisSpeeds().omega : 0.0;

        // ---- Phase 1: snapshot every camera once, filter independently ----
        // Snapshotting up front means an async NT update mid-loop can't make
        // two reads of the same camera disagree, and it decouples gathering
        // from fusing so 4+ cameras fuse deterministically.
        List<Accepted> accepted = new ArrayList<>(cameras.size());

        // MegaTag2 needs robot yaw. Limelight OS 2027.0 added a shared orientation table that every
        // camera on that OS reads by default, so one publish serves all of them. Every camera is
        // ALSO told individually: a Limelight still on the 2026 per-key API reads only its own
        // robot_orientation_set, and a MegaTag2 camera that never receives a heading does not fail
        // - it returns a zero solve that lands on the field's centre. Four cameras did exactly that
        // on the X1 while the shared table was being written every loop. Four extra sets a loop is
        // nothing next to a wrong pose.
        LimelightSource.setSharedRobotOrientation(yaw);

        for (int i = 0; i < cameras.size(); i++) {
            CameraSource camera = cameras.get(i);
            camera.setRobotOrientation(yaw, Math.toDegrees(yawRate), 0, 0);

            Optional<CameraSource.PoseEstimate> estimate = camera.getEstimatedPose();
            if (estimate.isEmpty()) {
                health.observe(i, VisionHealth.Outcome.NONE, null, now);
                continue;
            }

            CameraSource.PoseEstimate pe = estimate.get();

            // Guard against NaN/garbage poses before anything else touches them.
            if (!isFinitePose(pe)) {
                totalRejected++;
                cycleRejected++;
                logCamera(camera.getName(), "Rejected: NonFinite", pe);
                health.observe(i, VisionHealth.Outcome.REJECTED, "NonFinite", now);
                continue;
            }

            if (!fusing) {
                logCamera(camera.getName(), "Seen (no pose sink)", pe);
                health.observe(i, VisionHealth.Outcome.SEEN, null, now);
                continue;
            }

            String rejectReason = filterEstimate(pe, currentPose, camera.isPrefiltered(), anchored);
            if (rejectReason != null) {
                totalRejected++;
                cycleRejected++;
                logCamera(camera.getName(), "Rejected: " + rejectReason, pe);
                health.observe(i, VisionHealth.Outcome.REJECTED, rejectReason, now);
                if (rejectReason.startsWith("TooFar") && seedWorthy(pe)) {
                    double q = qualityScore(pe);
                    if (disagreement == null || q > disagreement.quality()) {
                        disagreement = new Accepted(i, camera.getName(), pe, null, q);
                    }
                }
                continue;
            }

            Matrix<N3, N1> stdDevs = calculateStdDevs(pe);
            accepted.add(new Accepted(i, camera.getName(), pe, stdDevs, qualityScore(pe)));
            health.observe(i, VisionHealth.Outcome.ACCEPTED, null, now);
        }

        // ---- Phase 1a: seed, or re-anchor ----
        // The first estimate good enough to trust replaces the pose outright. Fusing it would only
        // nudge the origin a little toward the truth, and the distance gate would then reject the
        // next one against a pose that is still mostly wrong.
        if (seeding) {
            Accepted seed = null;
            for (Accepted a : accepted) {
                if (seedWorthy(a.pe()) && (seed == null || a.quality() > seed.quality())) {
                    seed = a;
                }
            }
            if (seed != null) {
                poseSink.resetPose(seed.pe().pose(), seed.pe().timestampSeconds());
                seeded = true;
                accepted.remove(seed);
                totalAccepted++;
                cycleAccepted++;
                currentPose = seed.pe().pose();
                logCamera(seed.cameraName(), "Seeded", seed.pe());
                CatalystLog.log("Vision/Seed/Camera", seed.cameraName());
                CatalystLog.log("Vision/Seed/Pose", Pose2d.struct, seed.pe().pose());
                CatalystLog.log("Vision/Seed/Tags", (double) seed.pe().tagCount());
                AlertManager.getInstance().info("Vision", "Odometry seeded from vision");
            }
        } else if (anchored && config.reanchorAfterSeconds > 0) {
            // Every camera that saw tags was rejected for distance, none accepted, and what they
            // saw has held still relative to itself: the pose is what is wrong.
            if (!accepted.isEmpty() || disagreement == null) {
                disagreementSince = Double.NaN;
                disagreementPose = null;
            } else if (disagreementPose == null || disagreementPose.getTranslation()
                    .getDistance(disagreement.pe().pose().getTranslation()) > REANCHOR_AGREEMENT_M) {
                disagreementSince = now;
                disagreementPose = disagreement.pe().pose();
            } else if (now - disagreementSince >= config.reanchorAfterSeconds) {
                double off = currentPose.getTranslation().getDistance(disagreement.pe().pose().getTranslation());
                poseSink.resetPose(disagreement.pe().pose(), disagreement.pe().timestampSeconds());
                reanchors++;
                disagreementSince = Double.NaN;
                disagreementPose = null;
                currentPose = disagreement.pe().pose();
                logCamera(disagreement.cameraName(), "Re-anchored", disagreement.pe());
                CatalystLog.log("Vision/Reanchor/Count", (double) reanchors);
                CatalystLog.log("Vision/Reanchor/Meters", off);
                CatalystLog.log("Vision/Reanchor/Camera", disagreement.cameraName());
                CatalystLog.log("Vision/Reanchor/Pose", Pose2d.struct, disagreement.pe().pose());
                AlertManager.getInstance().warning("Vision",
                        "Odometry re-anchored to vision - it had drifted; see Vision/Reanchor");
            }
        }
        CatalystLog.log("Vision/Seeded", seeded);
        CatalystLog.log("Vision/Anchored", anchored);

        // ---- Phase 1b: check the cameras against each other ----
        // Only possible with more than one camera, which is the point. A robotToCamera transform has
        // nothing to be checked against on a single-camera robot - LimelightSource says as much and
        // publishes the numbers for a human instead. Two cameras seeing tags in the same loop are
        // describing the same robot, so a disagreement that persists is a wrong transform rather
        // than noise. Advisory: nothing below changes what reaches the estimator.
        if (accepted.size() >= 2) {
            agreement.observe(
                    accepted.stream()
                            .map(a -> new CameraAgreement.Sighting(a.cameraName(), a.pe().pose()))
                            .toList(),
                    currentPose.getRotation());
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
            poseSink.addVisionMeasurement(a.pe().pose(), a.pe().timestampSeconds(), a.stdDevs());
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

        health.update(now);
    }

    /** The current health picture, or null before the first loop. */
    public VisionHealth.Summary health() {
        return health.summary();
    }

    /**
     * Good enough to move the pose to, rather than nudge it: multi-tag PnP, or one tag close
     * enough that its solution is not ambiguous. A far single tag can put the robot on the wrong
     * side of it, and a seed in the wrong place is the exact failure seeding exists to prevent.
     */
    private boolean seedWorthy(CameraSource.PoseEstimate pe) {
        if (pe.tagCount() >= 2) {
            return true;
        }
        return pe.tagCount() == 1
                && pe.averageTagDistance() <= SINGLE_TAG_SEED_RANGE_M
                && pe.ambiguity() <= config.maxAmbiguity;
    }

    /** Whether the pose has been seeded from vision (or seeding is off). Gates measure only after. */
    public boolean isSeeded() {
        return seeded;
    }

    /** How many times the pose has been reset because every camera disagreed with it. */
    public int getReanchorCount() {
        return reanchors;
    }

    /**
     * Forget the seed: the next good estimate replaces the pose outright again. For a "re-localise"
     * button, or after the pose has been reset to something known to be wrong.
     */
    public void reseed() {
        if (config.seedFromVision) {
            seeded = false;
        }
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
     *
     * <p>Split by what each side can know. A source that reports
     * {@link CameraSource#isPrefiltered()} has already applied everything decidable from the image —
     * tag count, ambiguity, tag distance and area, field bounds, frame age — using the raw
     * detections, which it can do better than this can from a finished pose. Those checks are
     * skipped for such a source rather than repeated with a second set of thresholds nobody tuned.
     *
     * <p>What is never skipped is the half the camera cannot do, because it does not know where the
     * robot thinks it is or what it is doing: distance from the current fused pose, angular and
     * translational velocity, heading divergence. Those run for every source.
     */
    private String filterEstimate(CameraSource.PoseEstimate pe, Pose2d currentPose,
                                  boolean prefiltered, boolean anchored) {
        if (!prefiltered) {
            // Reject if no tags seen
            if (pe.tagCount() == 0) return "NoTags";

            // Reject if ambiguity is too high (single tag only — multi-tag PnP has low ambiguity)
            if (pe.tagCount() == 1 && pe.ambiguity() > config.maxAmbiguity) {
                return "HighAmbiguity(" + String.format("%.2f", pe.ambiguity()) + ")";
            }
        }

        // Reject if too far from current Kalman filter estimate (likely outlier). Only once the
        // sink has a pose: before that the "current" pose is a default nobody is at, and this gate
        // would reject every real estimate forever.
        if (anchored) {
            double distFromCurrent = currentPose.getTranslation()
                    .getDistance(pe.pose().getTranslation());
            if (distFromCurrent > config.maxAcceptableDistance) {
                return "TooFar(" + String.format("%.1fm", distFromCurrent) + ")";
            }
        }

        if (!prefiltered) {
            // Reject if pose is off the field (configurable bounds with margin)
            double x = pe.pose().getX();
            double y = pe.pose().getY();
            double margin = config.fieldBoundsMargin;
            if (x < -margin || x > config.fieldLengthMeters + margin
                    || y < -margin || y > config.fieldWidthMeters + margin) {
                return "OffField";
            }

            // Reject if timestamp is too old (stale data degrades Kalman filter accuracy)
            double latency = Timer.getTimestamp() - pe.timestampSeconds();
            if (latency > config.maxLatencySeconds) {
                return "StaleData(" + String.format("%.0fms", latency * 1000) + ")";
            }
        }

        // Reject during high angular velocity (motion blur)
        if (config.rejectDuringSpinThreshold > 0) {
            double spinRate = Math.abs(poseSink.getChassisSpeeds().omega);
            if (spinRate > config.rejectDuringSpinThreshold) {
                return "Spinning(" + String.format("%.1frad/s", spinRate) + ")";
            }
        }

        // Reject during high translational speed (configurable)
        if (config.rejectDuringHighSpeedThreshold > 0) {
            ChassisVelocities speeds = poseSink.getChassisSpeeds();
            double speed = Math.hypot(speeds.vx, speeds.vy);
            if (speed > config.rejectDuringHighSpeedThreshold) {
                return "HighSpeed(" + String.format("%.1fm/s", speed) + ")";
            }
        }

        // Reject based on heading consistency (vision heading vs gyro heading)
        if (anchored && config.maxHeadingDivergenceDegrees > 0 && pe.tagCount() == 1) {
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
