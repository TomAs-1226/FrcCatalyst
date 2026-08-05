package frc.lib.catalyst.physics.observation;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A field-relative velocity measured by something other than the drive wheels.
 *
 * <p>This is the observation that survives a wheel slip. An optical-flow sensor, a passive odometry
 * pod, or the frame-to-frame delta of a vision pose all measure how fast the <em>robot</em> is
 * moving, not how fast its wheels are turning — so when the two disagree, the wheels are wrong.
 *
 * <p>Physics Core treats this as corroborating evidence for the fused velocity and as an independent
 * check on the slip estimate. Like every observation it carries its own capture time, so a source
 * that reports late is still usable.
 *
 * @param fieldVelocity      measured field-relative velocity, in metres per second
 * @param timestampSeconds   capture time on the FPGA clock
 * @param standardDeviation  1-sigma uncertainty, in metres per second
 * @param source             short label for logging, e.g. {@code "flow-sensor"}
 * @since 1.5.0
 */
public record VelocityObservation(
        Translation2d fieldVelocity,
        double timestampSeconds,
        double standardDeviation,
        String source) implements Observation {

    /** Compact constructor: rejects a null velocity and a non-positive standard deviation. */
    public VelocityObservation {
        if (fieldVelocity == null) throw new IllegalArgumentException("fieldVelocity must not be null");
        if (!(standardDeviation > 0)) {
            throw new IllegalArgumentException("standardDeviation must be > 0 (got " + standardDeviation + ")");
        }
        source = source == null ? "unknown" : source;
    }

    /** Speed in metres per second, ignoring direction. */
    public double speedMetersPerSecond() {
        return fieldVelocity.getNorm();
    }
}
