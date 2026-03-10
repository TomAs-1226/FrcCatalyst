package frc.lib.catalyst.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * A rate limiter that smooths sudden changes in a value.
 * Useful for preventing jerky robot motion by limiting how fast
 * a value (like joystick input or motor voltage) can change.
 *
 * <p>Unlike WPILib's SlewRateLimiter, this supports different rates
 * for increasing vs decreasing, which is useful for acceleration
 * vs braking profiles.
 *
 * <p>Example usage:
 * <pre>{@code
 * // Limit acceleration to 3 units/sec, braking to 5 units/sec
 * SlewRateLimiter limiter = new SlewRateLimiter(3.0, 5.0);
 *
 * // In teleop periodic:
 * double smoothedSpeed = limiter.calculate(rawJoystickInput);
 * }</pre>
 */
public class SlewRateLimiter {

    private final double positiveRateLimit;
    private final double negativeRateLimit;
    private double previousValue;
    private double previousTimestamp;

    /**
     * Create a symmetric rate limiter (same rate for increase and decrease).
     * @param rateLimit maximum rate of change in units per second
     */
    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, rateLimit);
    }

    /**
     * Create an asymmetric rate limiter.
     * @param positiveRateLimit max rate when value is increasing (units/sec)
     * @param negativeRateLimit max rate when value is decreasing (units/sec)
     */
    public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        this.previousValue = 0;
        this.previousTimestamp = Timer.getFPGATimestamp();
    }

    /**
     * Filter the input value, limiting its rate of change.
     * @param input the desired value
     * @return the rate-limited value
     */
    public double calculate(double input) {
        double currentTimestamp = Timer.getFPGATimestamp();
        double dt = currentTimestamp - previousTimestamp;
        previousTimestamp = currentTimestamp;

        double delta = input - previousValue;

        if (delta > 0) {
            // Increasing
            double maxDelta = positiveRateLimit * dt;
            previousValue += Math.min(delta, maxDelta);
        } else {
            // Decreasing
            double maxDelta = negativeRateLimit * dt;
            previousValue += Math.max(delta, -maxDelta);
        }

        return previousValue;
    }

    /** Reset the limiter to a specific value. */
    public void reset(double value) {
        previousValue = value;
        previousTimestamp = Timer.getFPGATimestamp();
    }

    /** Reset the limiter to zero. */
    public void reset() {
        reset(0);
    }

    /** Get the current output value. */
    public double get() {
        return previousValue;
    }
}
