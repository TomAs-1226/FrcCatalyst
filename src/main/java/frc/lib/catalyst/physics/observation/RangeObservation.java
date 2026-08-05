package frc.lib.catalyst.physics.observation;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A measured distance from the robot to a known point on the field — a time-of-flight sensor against
 * a wall, a laser at a scoring station, a practice-field UWB anchor.
 *
 * <p>A range on its own does not fix a pose; it constrains one to a circle. That is still worth
 * having: it is cheap, it works when a camera cannot see a tag, and it is exactly the constraint that
 * stops dead-reckoning drift from growing without bound along one axis. Two ranges from different
 * anchors, or one range plus a heading, pin the robot down.
 *
 * <p>Physics Core uses these as a consistency check in shadow mode: the distance the estimate implies
 * versus the distance actually measured is a residual, and a residual that sits to one side is drift
 * you can see rather than drift you discover when the auto misses.
 *
 * @param anchorPoint       the field-relative point being measured to, in metres
 * @param distanceMeters    the measured distance
 * @param timestampSeconds  capture time on the FPGA clock
 * @param standardDeviation 1-sigma uncertainty of the distance, in metres
 * @param source            short label for logging, e.g. {@code "front-tof"}
 * @since 1.6.0
 */
public record RangeObservation(
        Translation2d anchorPoint,
        double distanceMeters,
        double timestampSeconds,
        double standardDeviation,
        String source) implements Observation {

    /** Compact constructor: rejects a null anchor, a negative range, and a non-positive uncertainty. */
    public RangeObservation {
        if (anchorPoint == null) throw new IllegalArgumentException("anchorPoint must not be null");
        if (distanceMeters < 0) {
            throw new IllegalArgumentException("distanceMeters must be >= 0 (got " + distanceMeters + ")");
        }
        if (!(standardDeviation > 0)) {
            throw new IllegalArgumentException("standardDeviation must be > 0 (got " + standardDeviation + ")");
        }
        source = source == null ? "unknown" : source;
    }

    /**
     * How far this measurement disagrees with an estimated position, in metres. Positive means the
     * sensor reports the robot further from the anchor than the estimate does.
     */
    public double residualFrom(Translation2d estimatedPosition) {
        return distanceMeters - estimatedPosition.getDistance(anchorPoint);
    }
}
