package frc.lib.catalyst.physics.estimation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import frc.lib.catalyst.physics.observation.PoseObservation;

/**
 * Opt-in shadow odometry using matched cumulative wheel-distance and raw-gyro intervals.
 * Never writes the native drivetrain pose. A unique, mutually consistent set of at least three
 * modules determines the center twist; this tolerates one disagreeing module on a four-module
 * chassis. Wheel agreement is NOT independent ground-motion evidence: common wheelspin remains
 * indistinguishable from travel here and requires the separate motion-integrity observer.
 *
 * <p>Module locations and angles must share the configured robot frame, with gyro CCW positive.
 * Distances, steering angles and raw heading must belong to the same captured sensor frame.
 * Like standard incremental swerve odometry, each distance increment uses its ending steering
 * angle; steering motion within an interval is not reconstructed. No mounting or friction model
 * is inferred. Calls must be serialized by the owner.
 */
public final class GyroWheelOdometry {
    /** Capture-time disagreement before weighting; acceptance alone does not certify shot accuracy. */
    public record AcceptedPoseFix(double timestampSeconds, double innovationMeters) {}

    /**
     * Immutable result. Valid means a usable wheel/gyro interval, not independent traction truth.
     * The initial/reset baseline is valid but has initializedVelocity=false; its zero velocity is
     * unavailable, not a stationary measurement. rejectedModule is -1 unless exactly one is excluded.
     * residualMeters is the largest inlier distance from the fitted center displacement.
     */
    public record Estimate(Pose2d pose, ChassisVelocities robotRelativeVelocity,
            double timestamp, int inliers, int rejectedModule, double residualMeters,
            boolean valid, String reason, boolean initializedVelocity) {
        public Estimate {
            robotRelativeVelocity = copy(robotRelativeVelocity);
        }
        @Override public ChassisVelocities robotRelativeVelocity() {
            return copy(robotRelativeVelocity);
        }
    }

    private static final double MAX_GAP_SECONDS = .10;
    private static final double MIN_AGREEMENT_METERS = .003;
    private static final double AGREEMENT_METERS_PER_SECOND = .20;
    private static final double HISTORY_SECONDS = .50;
    private static final int MAX_HISTORY_ENTRIES = 128;
    private static final double MAX_FIX_INNOVATION_METERS = 1.0;
    private static final double MAX_FIX_STD_DEV_METERS = .50;
    private final Translation2d[] locations;
    private final double assumedOdometryStdDevMeters;
    private final List<Frame> history = new ArrayList<>();
    private Pose2d pose = Pose2d.ZERO;
    private double[] previousDistances;
    private Rotation2d previousHeading;
    private double previousTimestamp;
    private double resetTimestamp = Double.NEGATIVE_INFINITY;
    private double lastObservationTimestamp = Double.NEGATIVE_INFINITY;
    private AcceptedPoseFix lastAcceptedPoseFix;
    private boolean anchored;
    private boolean initialBaseline = true;
    private Estimate estimate = new Estimate(Pose2d.ZERO, new ChassisVelocities(),
            Double.NaN, 0, -1, 0, false, "no wheel/gyro sample", false);

    /** @param moduleLocations known module locations in the same order as the sensor array */
    public GyroWheelOdometry(Translation2d[] moduleLocations) {
        this(moduleLocations, .10);
    }

    /**
     * @param moduleLocations known locations in sensor-array order
     * @param assumedOdometryStdDevMeters fixed correction-weight assumption; .10 m by default.
     *     This engineering starting value is not a calibrated or propagated covariance and does
     *     not increase motion-integrity confidence.
     */
    public GyroWheelOdometry(Translation2d[] moduleLocations, double assumedOdometryStdDevMeters) {
        if (moduleLocations == null || moduleLocations.length < 3) {
            throw new IllegalArgumentException("at least three module locations required");
        }
        if (!Double.isFinite(assumedOdometryStdDevMeters) || assumedOdometryStdDevMeters <= 0) {
            throw new IllegalArgumentException("odometry correction standard deviation must be positive and finite");
        }
        this.assumedOdometryStdDevMeters = assumedOdometryStdDevMeters;
        locations = moduleLocations.clone();
        for (Translation2d location : locations) {
            if (location == null || !finite(location.getX(), location.getY())) {
                throw new IllegalArgumentException("module locations must be finite");
            }
        }
    }

    public Estimate estimate() { return estimate; }

    /**
     * Latest accepted fix's pre-correction capture-time innovation, absent after construction/reset.
     * Rejected observations do not update it. A caller certifying a particular observation must
     * require observe() success and match the capture timestamp before using this metadata.
     */
    public Optional<AcceptedPoseFix> lastAcceptedPoseFix() {
        return Optional.ofNullable(lastAcceptedPoseFix);
    }

    /**
     * Integrates one coherent frame. A gap or invalid interval holds translation and rotation,
     * discards the lost displacement, and rebases measurements for the next interval; it does not
     * repair the owner's persistent motion fault. The first baseline alone supplies no velocity.
     * Older/duplicate frames never integrate twice; pre-reset frames are ignored.
     */
    public Estimate update(double timestamp, Pose2d nativePose, Rotation2d rawGyroHeading,
            SwerveModulePosition[] modulePositions) {
        if (Double.isFinite(timestamp) && timestamp < resetTimestamp) return estimate;
        if (!Double.isFinite(timestamp) || !validPose(nativePose) || rawGyroHeading == null
                || !Double.isFinite(rawGyroHeading.getRadians()) || !validModules(modulePositions)) {
            previousDistances = null;
            initialBaseline = false;
            history.clear();
            return held(timestamp, "invalid wheel/gyro frame");
        }
        if (previousDistances != null && timestamp <= previousTimestamp) {
            if (timestamp == previousTimestamp) return estimate;
            return held(estimate.timestamp(), "out-of-order wheel/gyro frame");
        }
        if (!anchored) {
            pose = nativePose;
            anchored = true;
        }
        if (previousDistances == null) {
            capture(timestamp, rawGyroHeading, modulePositions);
            history.clear();
            append(timestamp);
            boolean first = initialBaseline;
            initialBaseline = false;
            estimate = new Estimate(pose, new ChassisVelocities(), timestamp, 0, -1, 0,
                    first, first ? "wheel/gyro baseline; velocity unavailable"
                            : "wheel/gyro baseline reacquired; velocity unavailable", false);
            return estimate;
        }
        double dt = timestamp - previousTimestamp;
        if (dt > MAX_GAP_SECONDS + 1e-12) {
            capture(timestamp, rawGyroHeading, modulePositions);
            history.clear();
            append(timestamp);
            return held(timestamp, "wheel/gyro sampling gap");
        }
        double headingDelta = Math.IEEEremainder(
                rawGyroHeading.getRadians() - previousHeading.getRadians(), 2 * Math.PI);
        Translation2d[] centers = new Translation2d[locations.length];
        for (int i = 0; i < locations.length; i++) {
            double distance = modulePositions[i].distance - previousDistances[i];
            double x = distance * modulePositions[i].angle.getCos()
                    + headingDelta * locations[i].getY();
            double y = distance * modulePositions[i].angle.getSin()
                    - headingDelta * locations[i].getX();
            if (!finite(x, y)) {
                capture(timestamp, rawGyroHeading, modulePositions);
                history.clear();
                append(timestamp);
                return held(timestamp, "non-finite wheel/gyro interval");
            }
            centers[i] = new Translation2d(x, y);
        }
        capture(timestamp, rawGyroHeading, modulePositions);
        double tolerance = Math.max(MIN_AGREEMENT_METERS, AGREEMENT_METERS_PER_SECOND * dt);
        Consensus consensus = new Consensus(centers, tolerance);
        if (consensus.bestSize < 3 || consensus.bestCount != 1) {
            history.clear();
            append(timestamp);
            return held(timestamp, consensus.bestSize < 3
                    ? "fewer than three agreeing modules" : "ambiguous module consensus");
        }
        double dx = 0, dy = 0;
        for (int i : consensus.best) {
            dx += centers[i].getX() / consensus.bestSize;
            dy += centers[i].getY() / consensus.bestSize;
        }
        Translation2d center = new Translation2d(dx, dy);
        double residual = 0;
        for (int i : consensus.best) residual = Math.max(residual, centers[i].getDistance(center));
        Pose2d integrated = pose.plus(new Twist2d(dx, dy, headingDelta).exp());
        ChassisVelocities velocity = new ChassisVelocities(dx / dt, dy / dt, headingDelta / dt);
        if (!validPose(integrated) || !finite(velocity.vx, velocity.vy, velocity.omega)) {
            history.clear();
            append(timestamp);
            return held(timestamp, "non-finite integrated wheel/gyro interval");
        }
        pose = integrated;
        int rejected = -1;
        if (consensus.bestSize == locations.length - 1) {
            outer: for (int i = 0; i < locations.length; i++) {
                for (int inlier : consensus.best) if (i == inlier) continue outer;
                rejected = i;
                break;
            }
        }
        estimate = new Estimate(pose, velocity, timestamp, consensus.bestSize, rejected,
                residual, true, "wheel/gyro consensus only; ground motion unverified", true);
        append(timestamp);
        return estimate;
    }

    /**
     * Applies an externally accepted absolute translation fix at its capture time, never its
     * arrival time. Heading and interval velocity are unchanged. Translation gain is
     * sigmaOdometry/(sigmaOdometry+sigmaObservation), using the configured fixed assumption. This is
     * NOT a full covariance filter: the owner must first accept the source using its observation
     * policy. Positive finite observation standard deviation must be at most .5 m and is not proof
     * of accuracy. Fixes must be within the retained 0.5-second history, globally increasing
     * in capture time, not future, and within one meter of the historical predicted translation.
     * No interpolation crosses a missing/invalid interval. Acceptance never certifies wheel motion.
     */
    public boolean observe(PoseObservation observation) {
        if (observation == null || !validPose(observation.pose())
                || !finite(observation.timestampSeconds(), observation.standardDeviation())
                || observation.standardDeviation() <= 0
                || observation.standardDeviation() > MAX_FIX_STD_DEV_METERS || history.isEmpty()) return false;
        double t = observation.timestampSeconds();
        double current = estimate.timestamp();
        if (!Double.isFinite(current) || t < resetTimestamp || t <= lastObservationTimestamp
                || t > current || current - t > HISTORY_SECONDS
                || t < history.get(0).timestamp) return false;
        Pose2d atCapture = poseAt(t);
        if (atCapture == null) return false;
        Translation2d innovation = observation.pose().getTranslation().minus(atCapture.getTranslation());
        if (!Double.isFinite(innovation.getNorm())
                || innovation.getNorm() > MAX_FIX_INNOVATION_METERS) return false;
        double gain = assumedOdometryStdDevMeters
                / (assumedOdometryStdDevMeters + observation.standardDeviation());
        Translation2d correction = innovation.times(gain);
        int insertion = 0;
        while (insertion < history.size() && history.get(insertion).timestamp < t) insertion++;
        if (insertion == history.size() || history.get(insertion).timestamp != t) {
            history.add(insertion, new Frame(t, atCapture));
        }
        for (int i = insertion; i < history.size(); i++) {
            Frame frame = history.get(i);
            history.set(i, new Frame(frame.timestamp, translated(frame.pose, correction)));
        }
        pose = translated(pose, correction);
        lastObservationTimestamp = t;
        lastAcceptedPoseFix = new AcceptedPoseFix(t, innovation.getNorm());
        estimate = new Estimate(pose, estimate.robotRelativeVelocity(), estimate.timestamp(),
                estimate.inliers(), estimate.rejectedModule(), estimate.residualMeters(),
                estimate.valid(), estimate.reason(), estimate.initializedVelocity());
        trimHistory(current);
        return true;
    }

    /**
     * Sets an externally validated pose/epoch, without certifying sensors or stationary velocity.
     * The first frame at/after this epoch only captures a new wheel/raw-heading baseline. Cached
     * frames preceding this reset cannot move the reset pose or establish that baseline.
     */
    public void resetTo(Pose2d validatedPose, double now) {
        if (!validPose(validatedPose) || !Double.isFinite(now)) {
            throw new IllegalArgumentException("reset pose and timestamp must be finite");
        }
        pose = validatedPose;
        anchored = true;
        resetTimestamp = now;
        previousDistances = null;
        initialBaseline = true;
        history.clear();
        lastObservationTimestamp = Double.NEGATIVE_INFINITY;
        lastAcceptedPoseFix = null;
        estimate = new Estimate(pose, new ChassisVelocities(), now, 0, -1, 0,
                false, "validated pose reset; wheel/gyro baseline unavailable", false);
    }

    /**
     * Clears the anchor, sensor baseline and observation history. This is an explicit caller reset,
     * not automatic fault recovery; the next coherent frame anchors its supplied native pose and
     * has no interval velocity. No previous sensor or observation certification survives.
     */
    public void reset() {
        pose = Pose2d.ZERO;
        previousDistances = null;
        previousHeading = null;
        previousTimestamp = Double.NaN;
        resetTimestamp = Double.NEGATIVE_INFINITY;
        lastObservationTimestamp = Double.NEGATIVE_INFINITY;
        lastAcceptedPoseFix = null;
        anchored = false;
        initialBaseline = true;
        history.clear();
        estimate = new Estimate(Pose2d.ZERO, new ChassisVelocities(), Double.NaN,
                0, -1, 0, false, "no wheel/gyro sample", false);
    }

    private Estimate held(double timestamp, String reason) {
        estimate = new Estimate(pose, new ChassisVelocities(), timestamp, 0, -1, 0,
                false, reason, false);
        return estimate;
    }

    private void capture(double timestamp, Rotation2d heading, SwerveModulePosition[] modules) {
        previousTimestamp = timestamp;
        previousHeading = heading;
        previousDistances = new double[modules.length];
        for (int i = 0; i < modules.length; i++) previousDistances[i] = modules[i].distance;
    }

    private boolean validModules(SwerveModulePosition[] modules) {
        if (modules == null || modules.length != locations.length) return false;
        for (SwerveModulePosition module : modules) {
            if (module == null || module.angle == null
                    || !finite(module.distance, module.angle.getRadians())) return false;
        }
        return true;
    }

    private void append(double timestamp) {
        history.add(new Frame(timestamp, pose));
        trimHistory(timestamp);
    }

    private void trimHistory(double current) {
        while (history.size() > MAX_HISTORY_ENTRIES
                || (!history.isEmpty() && current - history.get(0).timestamp > HISTORY_SECONDS)) {
            history.remove(0);
        }
    }

    private Pose2d poseAt(double timestamp) {
        Frame previous = null;
        for (Frame frame : history) {
            if (frame.timestamp == timestamp) return frame.pose;
            if (frame.timestamp > timestamp) {
                if (previous == null) return null;
                return previous.pose.interpolate(frame.pose,
                        (timestamp - previous.timestamp) / (frame.timestamp - previous.timestamp));
            }
            previous = frame;
        }
        return null;
    }

    private static Pose2d translated(Pose2d source, Translation2d delta) {
        return new Pose2d(source.getTranslation().plus(delta), source.getRotation());
    }

    private static boolean validPose(Pose2d value) {
        return value != null && value.getTranslation() != null && value.getRotation() != null
                && finite(value.getX(), value.getY(), value.getRotation().getRadians());
    }

    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }

    private static ChassisVelocities copy(ChassisVelocities value) {
        return new ChassisVelocities(value.vx, value.vy, value.omega);
    }

    private record Frame(double timestamp, Pose2d pose) {}

    /** Enumerates cliques, rather than accepting a non-transitive neighbor/majority chain. */
    private static final class Consensus {
        private final boolean[][] agrees;
        private int[] best = new int[0];
        private int bestSize;
        private int bestCount;

        Consensus(Translation2d[] centers, double tolerance) {
            agrees = new boolean[centers.length][centers.length];
            for (int i = 0; i < centers.length; i++) {
                for (int j = i + 1; j < centers.length; j++) {
                    agrees[i][j] = agrees[j][i] = centers[i].getDistance(centers[j]) <= tolerance;
                }
            }
            search(0, new int[centers.length], 0);
        }

        private void search(int start, int[] selected, int size) {
            if (size + agrees.length - start < bestSize) return;
            if (size > bestSize) {
                bestSize = size;
                best = java.util.Arrays.copyOf(selected, size);
                bestCount = 1;
            } else if (size == bestSize) {
                bestCount++;
            }
            for (int i = start; i < agrees.length; i++) {
                boolean include = true;
                for (int j = 0; j < size; j++) {
                    if (!agrees[i][selected[j]]) { include = false; break; }
                }
                if (include) {
                    selected[size] = i;
                    search(i + 1, selected, size + 1);
                }
            }
        }
    }
}
