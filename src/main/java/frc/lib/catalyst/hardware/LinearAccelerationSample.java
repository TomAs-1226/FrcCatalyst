package frc.lib.catalyst.hardware;

import java.util.Optional;
import org.wpilib.math.geometry.Translation2d;

/** Pure validation and gravity removal; timestamps must share one clock and axes one mount frame. */
public final class LinearAccelerationSample {
    /** Standard gravity, m/s²; the Phoenix gravity vector is dimensionless in units of g. */
    public static final double STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665;

    private LinearAccelerationSample() {}

    /** One copied signal value, timestamp in seconds, and combined status/timestamp validity. */
    public record Reading(double value, double timestampSeconds, boolean valid) {}

    /**
     * Acceleration inputs are m/s²; gravity inputs are dimensionless fractions of standard gravity.
     * Invalid or unavailable inputs produce empty, never an invented zero-acceleration reading.
     */
    public static Optional<Translation2d> from(
            Reading accelerationX, Reading accelerationY, Reading gravityX, Reading gravityY,
            double nowSeconds, double maxAgeSeconds, double maxSkewSeconds) {
        if (!Double.isFinite(nowSeconds) || !Double.isFinite(maxAgeSeconds)
                || !Double.isFinite(maxSkewSeconds) || maxAgeSeconds < 0 || maxSkewSeconds < 0
                || !usable(accelerationX, nowSeconds, maxAgeSeconds)
                || !usable(accelerationY, nowSeconds, maxAgeSeconds)
                || !usable(gravityX, nowSeconds, maxAgeSeconds)
                || !usable(gravityY, nowSeconds, maxAgeSeconds)) {
            return Optional.empty();
        }
        double oldest = Math.min(Math.min(accelerationX.timestampSeconds(), accelerationY.timestampSeconds()),
                Math.min(gravityX.timestampSeconds(), gravityY.timestampSeconds()));
        double newest = Math.max(Math.max(accelerationX.timestampSeconds(), accelerationY.timestampSeconds()),
                Math.max(gravityX.timestampSeconds(), gravityY.timestampSeconds()));
        if (newest - oldest > maxSkewSeconds) return Optional.empty();
        double x = accelerationX.value() - gravityX.value() * STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED;
        double y = accelerationY.value() - gravityY.value() * STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED;
        return Double.isFinite(x) && Double.isFinite(y)
                ? Optional.of(new Translation2d(x, y)) : Optional.empty();
    }

    /**
     * Remove gravity in three dimensions, then minimally level gravity onto the same-polarity Z axis.
     * Either gravity polarity is accepted; the mounting frame must be configured externally.
     */
    public static Optional<Translation2d> level(
            Reading accelerationX, Reading accelerationY, Reading accelerationZ,
            Reading gravityX, Reading gravityY, Reading gravityZ,
            double nowSeconds, double maxAgeSeconds, double maxSkewSeconds) {
        if (!Double.isFinite(nowSeconds) || !Double.isFinite(maxAgeSeconds)
                || !Double.isFinite(maxSkewSeconds) || maxAgeSeconds < 0 || maxSkewSeconds < 0
                || !usable(accelerationX, nowSeconds, maxAgeSeconds)
                || !usable(accelerationY, nowSeconds, maxAgeSeconds)
                || !usable(accelerationZ, nowSeconds, maxAgeSeconds)
                || !usable(gravityX, nowSeconds, maxAgeSeconds)
                || !usable(gravityY, nowSeconds, maxAgeSeconds)
                || !usable(gravityZ, nowSeconds, maxAgeSeconds)) return Optional.empty();
        double oldest = Math.min(Math.min(accelerationX.timestampSeconds(), accelerationY.timestampSeconds()),
                Math.min(Math.min(accelerationZ.timestampSeconds(), gravityX.timestampSeconds()),
                        Math.min(gravityY.timestampSeconds(), gravityZ.timestampSeconds())));
        double newest = Math.max(Math.max(accelerationX.timestampSeconds(), accelerationY.timestampSeconds()),
                Math.max(Math.max(accelerationZ.timestampSeconds(), gravityX.timestampSeconds()),
                        Math.max(gravityY.timestampSeconds(), gravityZ.timestampSeconds())));
        if (newest - oldest > maxSkewSeconds) return Optional.empty();
        double gx = gravityX.value(), gy = gravityY.value(), gz = gravityZ.value();
        double norm = Math.hypot(Math.hypot(gx, gy), gz);
        if (!Double.isFinite(norm) || norm < .9 || norm > 1.1 || Math.abs(gz) / norm < .5) {
            return Optional.empty();
        }
        double x = accelerationX.value() - gx * STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED;
        double y = accelerationY.value() - gy * STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED;
        double z = accelerationZ.value() - gz * STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED;
        // Rodrigues' minimal rotation: Rv = v + k×v + k×(k×v)/(1+c).
        // Target Z has the reported gravity's sign, so a flat frame is unchanged under either
        // sensor polarity. Mount configuration is external; polarity does not diagnose inversion.
        double targetSign = Math.signum(gz);
        double kx = targetSign * gy / norm, ky = -targetSign * gx / norm;
        double crossZ = kx * y - ky * x;
        double divisor = 1 + Math.abs(gz) / norm;
        double levelX = x + ky * z + ky * crossZ / divisor;
        double levelY = y - kx * z - kx * crossZ / divisor;
        return Double.isFinite(levelX) && Double.isFinite(levelY) && Double.isFinite(z)
                ? Optional.of(new Translation2d(levelX, levelY)) : Optional.empty();
    }

    private static boolean usable(Reading reading, double now, double maxAge) {
        if (reading == null || !reading.valid() || !Double.isFinite(reading.value())
                || !Double.isFinite(reading.timestampSeconds())) return false;
        double age = now - reading.timestampSeconds();
        return Double.isFinite(age) && age >= 0 && age <= maxAge;
    }
}
