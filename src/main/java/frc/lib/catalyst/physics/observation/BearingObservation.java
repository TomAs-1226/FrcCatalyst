package frc.lib.catalyst.physics.observation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A measured direction from the robot to a known point on the field — the yaw a camera reports to a
 * target it can see but cannot range, or a retroreflective bearing without a usable distance.
 *
 * <p>Like a range, a bearing alone does not fix a pose; it constrains one to a line. And like a range
 * it stays available when a full pose solve does not: a tag too far away or too oblique for a
 * trustworthy 3D solve often still gives a perfectly good angle. Throwing that away because it is not
 * a whole pose is throwing away the most reliable part of the measurement.
 *
 * <p>Physics Core reads these as a residual against the bearing its own estimate implies.
 *
 * @param targetPoint       the field-relative point being sighted, in metres
 * @param fieldBearing      measured field-relative bearing from the robot to that point
 * @param timestampSeconds  capture time on the FPGA clock
 * @param standardDeviation 1-sigma angular uncertainty, in radians
 * @param source            short label for logging, e.g. {@code "limelight-front"}
 * @since 1.6.0
 */
public record BearingObservation(
        Translation2d targetPoint,
        Rotation2d fieldBearing,
        double timestampSeconds,
        double standardDeviation,
        String source) implements Observation {

    /** Compact constructor: rejects nulls and a non-positive uncertainty. */
    public BearingObservation {
        if (targetPoint == null) throw new IllegalArgumentException("targetPoint must not be null");
        if (fieldBearing == null) throw new IllegalArgumentException("fieldBearing must not be null");
        if (!(standardDeviation > 0)) {
            throw new IllegalArgumentException("standardDeviation must be > 0 (got " + standardDeviation + ")");
        }
        source = source == null ? "unknown" : source;
    }

    /**
     * How far this measurement disagrees with an estimated position, in radians, wrapped to
     * {@code [-pi, pi]}. Meaningless when the estimate sits on top of the target, so that case
     * returns zero rather than an angle made of noise.
     */
    public double residualFrom(Translation2d estimatedPosition) {
        Translation2d toTarget = targetPoint.minus(estimatedPosition);
        if (toTarget.getNorm() < 1e-6) return 0.0;
        double expected = Math.atan2(toTarget.getY(), toTarget.getX());
        return Rotation2d.fromRadians(fieldBearing.getRadians() - expected).getRadians();
    }
}
