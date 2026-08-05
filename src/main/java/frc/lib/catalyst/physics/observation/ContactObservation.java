package frc.lib.catalyst.physics.observation;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The robot is touching something whose position is known — squared up against a wall, seated in a
 * scoring station, pressed into a feeder.
 *
 * <p>This is the most accurate absolute measurement on the field and almost nobody uses it. A robot
 * pinned flat against a wall knows its distance to that wall to within the bumper tolerance and its
 * heading to within how square it is sitting — better than a camera at that range, available in a
 * scrum where no camera can see anything, and free, because the robot already has the current spike
 * and the velocity collapse that say it has arrived.
 *
 * <p>The constraint is one-dimensional: contact with a wall fixes the distance along the wall's
 * normal and says nothing about position along it. Physics Core treats it as exactly that — a
 * confidence-restoring constraint on one axis, never a full pose reset.
 *
 * <pre>{@code
 * // squared up against the driver-station wall at x = 0
 * physics.observe(new ContactObservation(
 *     0.0, Rotation2d.kZero, BUMPER_HALF_LENGTH, timestamp, 0.02, "wall-square"));
 * }</pre>
 *
 * @param constraintCoordinate the field coordinate of the contact surface along its normal, in metres
 *                             — {@code x} for a wall whose normal is {@code +X}
 * @param surfaceNormal        field-relative direction the surface faces
 * @param standoffMeters       distance from the robot's centre to the contact surface when touching:
 *                             half the bumper dimension along the normal
 * @param timestampSeconds     capture time on the FPGA clock
 * @param standardDeviation    1-sigma uncertainty along the normal, in metres. Bumper compression and
 *                             frame tolerance put this around 1-3 cm
 * @param source               short label for logging, e.g. {@code "wall-square"}
 * @since 1.6.0
 */
public record ContactObservation(
        double constraintCoordinate,
        Rotation2d surfaceNormal,
        double standoffMeters,
        double timestampSeconds,
        double standardDeviation,
        String source) implements Observation {

    /** Compact constructor: rejects a null normal, a negative standoff, and a non-positive uncertainty. */
    public ContactObservation {
        if (surfaceNormal == null) throw new IllegalArgumentException("surfaceNormal must not be null");
        if (standoffMeters < 0) {
            throw new IllegalArgumentException("standoffMeters must be >= 0 (got " + standoffMeters + ")");
        }
        if (!(standardDeviation > 0)) {
            throw new IllegalArgumentException("standardDeviation must be > 0 (got " + standardDeviation + ")");
        }
        source = source == null ? "unknown" : source;
    }

    /**
     * Where the robot's centre must be along the surface normal for it to be touching, in metres.
     * The contact surface plus the standoff, measured back along the normal.
     */
    public double impliedCoordinate() {
        return constraintCoordinate + standoffMeters;
    }

    /**
     * How far an estimated position sits from where contact says it must be, in metres, along the
     * surface normal only. Positive means the estimate has the robot further from the surface than
     * touching it allows.
     *
     * @param estimatedX field-relative x of the estimate
     * @param estimatedY field-relative y of the estimate
     */
    public double residualFrom(double estimatedX, double estimatedY) {
        double projected = estimatedX * surfaceNormal.getCos() + estimatedY * surfaceNormal.getSin();
        return projected - impliedCoordinate();
    }
}
