package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;

/**
 * Temporal pose tracking with interpolation for latency compensation.
 *
 * <p>Stores timestamped poses and provides interpolated lookups at arbitrary
 * past timestamps. Essential for vision latency compensation — when a camera
 * reports a pose at time T, you need to know where the robot was at time T
 * to correctly fuse the measurement.
 *
 * <p>Also provides velocity estimation from the pose history, useful for
 * detecting sudden impacts, acceleration events, or simply getting a
 * velocity without a separate velocity sensor.
 *
 * <p>Example:
 * <pre>{@code
 * PoseHistory history = new PoseHistory(1.5); // 1.5 seconds of history
 *
 * // In periodic:
 * history.addSample(drive.getPose());
 *
 * // Get pose at a past timestamp (for vision compensation):
 * Optional<Pose2d> pastPose = history.getPoseAtTime(visionTimestamp);
 *
 * // Get current velocity from pose differences:
 * double speed = history.getTranslationalVelocity();
 * double angularVel = history.getAngularVelocity();
 * }</pre>
 */
public class PoseHistory {

    private final TimeInterpolatableBuffer<Pose2d> buffer;
    private Pose2d latestPose = new Pose2d();
    private double latestTimestamp = 0;
    private Pose2d previousPose = new Pose2d();
    private double previousTimestamp = 0;
    private Pose2d oldPose = new Pose2d();
    private double oldTimestamp = 0;
    private boolean hasPrevious = false;
    private int sampleCount = 0;

    /**
     * @param historySeconds how far back to keep pose data (typically 1.0-2.0 seconds)
     */
    public PoseHistory(double historySeconds) {
        this.buffer = TimeInterpolatableBuffer.createBuffer(historySeconds);
    }

    /** Create with default 1.5 second history. */
    public PoseHistory() {
        this(1.5);
    }

    /**
     * Add a pose sample at the current FPGA timestamp.
     * Call this every robot periodic cycle.
     */
    public void addSample(Pose2d pose) {
        addSample(pose, Timer.getFPGATimestamp());
    }

    /**
     * Add a pose sample at a specific timestamp.
     */
    public void addSample(Pose2d pose, double timestampSeconds) {
        oldPose = previousPose;
        oldTimestamp = previousTimestamp;
        previousPose = latestPose;
        previousTimestamp = latestTimestamp;
        hasPrevious = true;
        sampleCount++;

        latestPose = pose;
        latestTimestamp = timestampSeconds;
        buffer.addSample(timestampSeconds, pose);
    }

    /**
     * Get an interpolated pose at a past timestamp.
     * Returns null if the timestamp is outside the buffer range.
     */
    public Pose2d getPoseAtTime(double timestampSeconds) {
        var result = buffer.getSample(timestampSeconds);
        return result.orElse(null);
    }

    /** Get the most recent pose. */
    public Pose2d getLatestPose() {
        return latestPose;
    }

    /** Get the timestamp of the most recent pose. */
    public double getLatestTimestamp() {
        return latestTimestamp;
    }

    /**
     * Get the translational velocity in m/s estimated from the last two samples.
     */
    public double getTranslationalVelocity() {
        if (!hasPrevious) return 0;
        double dt = latestTimestamp - previousTimestamp;
        if (dt <= 0) return 0;
        double dist = latestPose.getTranslation().getDistance(previousPose.getTranslation());
        return dist / dt;
    }

    /**
     * Get the translational velocity as a Translation2d (vx, vy) in m/s.
     */
    public Translation2d getVelocityVector() {
        if (!hasPrevious) return new Translation2d();
        double dt = latestTimestamp - previousTimestamp;
        if (dt <= 0) return new Translation2d();
        double dx = latestPose.getX() - previousPose.getX();
        double dy = latestPose.getY() - previousPose.getY();
        return new Translation2d(dx / dt, dy / dt);
    }

    /**
     * Get the angular velocity in rad/s estimated from the last two samples.
     */
    public double getAngularVelocity() {
        if (!hasPrevious) return 0;
        double dt = latestTimestamp - previousTimestamp;
        if (dt <= 0) return 0;
        double dTheta = latestPose.getRotation().getRadians() - previousPose.getRotation().getRadians();
        // Normalize to [-pi, pi]
        while (dTheta > Math.PI) dTheta -= 2 * Math.PI;
        while (dTheta < -Math.PI) dTheta += 2 * Math.PI;
        return dTheta / dt;
    }

    /**
     * Get the acceleration magnitude in m/s^2.
     * Uses finite difference of two consecutive velocity estimates (3 pose samples).
     * Returns 0 if fewer than 3 samples have been recorded.
     */
    public double getAcceleration() {
        if (sampleCount < 3) return 0;

        double dt1 = previousTimestamp - oldTimestamp;
        double dt2 = latestTimestamp - previousTimestamp;
        if (dt1 <= 0 || dt2 <= 0) return 0;

        double v1 = previousPose.getTranslation().getDistance(oldPose.getTranslation()) / dt1;
        double v2 = latestPose.getTranslation().getDistance(previousPose.getTranslation()) / dt2;

        double dtMid = (latestTimestamp - oldTimestamp) / 2.0;
        if (dtMid <= 0) return 0;
        return (v2 - v1) / dtMid;
    }

    /**
     * Get the distance traveled since a given timestamp.
     */
    public double getDistanceSince(double timestampSeconds) {
        Pose2d pastPose = getPoseAtTime(timestampSeconds);
        if (pastPose == null) return 0;
        return latestPose.getTranslation().getDistance(pastPose.getTranslation());
    }

    /**
     * Get the heading change in radians since a given timestamp.
     */
    public double getHeadingChangeSince(double timestampSeconds) {
        Pose2d pastPose = getPoseAtTime(timestampSeconds);
        if (pastPose == null) return 0;
        double dTheta = latestPose.getRotation().getRadians() - pastPose.getRotation().getRadians();
        while (dTheta > Math.PI) dTheta -= 2 * Math.PI;
        while (dTheta < -Math.PI) dTheta += 2 * Math.PI;
        return dTheta;
    }

    /** Clear all stored poses. */
    public void clear() {
        buffer.clear();
        hasPrevious = false;
        sampleCount = 0;
    }
}
