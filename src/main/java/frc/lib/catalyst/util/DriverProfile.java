package frc.lib.catalyst.util;

import org.wpilib.math.util.MathUtil;

import java.util.function.DoubleSupplier;

/**
 * Per-driver controller-feel settings — deadband, response curve, speed
 * multipliers, slow-mode behaviour. Apply it to a raw
 * {@link DoubleSupplier} (typically a joystick axis) and get a shaped
 * {@code DoubleSupplier} the swerve drive consumes directly.
 *
 * <p>Drivers usually have strong opinions about deadband size and curve
 * sharpness. Storing those in a profile means swapping driver is one
 * config-file change instead of hunting through subsystem code.
 *
 * <p>Example:
 * <pre>{@code
 * DriverProfile alice = DriverProfile.builder()
 *     .deadband(0.06)
 *     .curve(DriverProfile.Curve.CUBIC)
 *     .maxSpeed(0.85)
 *     .slowMode(0.25)
 *     .build();
 *
 * drive.setDefaultCommand(drive.advancedDrive(
 *     alice.shape(() -> -driver.getLeftY()),
 *     alice.shape(() -> -driver.getLeftX()),
 *     alice.shape(() -> -driver.getRightX()),
 *     0.0));   // deadband already handled by the profile
 *
 * driver.leftBumper().whileTrue(Commands.runOnce(alice::engageSlowMode));
 * driver.leftBumper().onFalse(Commands.runOnce(alice::disengageSlowMode));
 * }</pre>
 */
public final class DriverProfile {

    /** Polynomial response curves for joystick → output. */
    public enum Curve {
        /** {@code y = x}. Closest to raw joystick. */
        LINEAR,
        /** {@code y = x * |x|}. Softer at center, gentler ramp. */
        QUADRATIC,
        /** {@code y = x³}. Even softer near center — common with skilled drivers. */
        CUBIC,
        /** Exponential curve. {@code y = sign(x) * (e^(a|x|) - 1) / (e^a - 1)} with {@code a = 3}. */
        EXPO
    }

    private final double deadband;
    private final Curve curve;
    private final double maxSpeed;
    private final double slowMultiplier;
    private boolean slowEngaged = false;

    private DriverProfile(Builder b) {
        this.deadband = b.deadband;
        this.curve = b.curve;
        this.maxSpeed = b.maxSpeed;
        this.slowMultiplier = b.slowMultiplier;
    }

    public static Builder builder() { return new Builder(); }

    /** Wrap a raw axis supplier with this profile's deadband + curve + speed cap. */
    public DoubleSupplier shape(DoubleSupplier raw) {
        return () -> apply(raw.getAsDouble());
    }

    /** Apply this profile's shaping to a single value. */
    public double apply(double value) {
        double deadbanded = MathUtil.applyDeadband(value, deadband);
        double curved = applyCurve(deadbanded);
        double cap = slowEngaged ? maxSpeed * slowMultiplier : maxSpeed;
        return curved * cap;
    }

    private double applyCurve(double x) {
        switch (curve) {
            case LINEAR:    return x;
            case QUADRATIC: return x * Math.abs(x);
            case CUBIC:     return x * x * x;
            case EXPO:      {
                double a = 3.0;
                double scale = 1.0 / (Math.exp(a) - 1.0);
                return Math.signum(x) * (Math.exp(a * Math.abs(x)) - 1.0) * scale;
            }
            default: return x;
        }
    }

    /** Switch into slow mode — the cap drops to {@code maxSpeed * slowMultiplier}. */
    public void engageSlowMode() { slowEngaged = true; }
    /** Leave slow mode. */
    public void disengageSlowMode() { slowEngaged = false; }
    /** True when slow mode is currently active. */
    public boolean isSlowMode() { return slowEngaged; }

    public double deadband()       { return deadband; }
    public Curve curve()           { return curve; }
    public double maxSpeed()       { return maxSpeed; }
    public double slowMultiplier() { return slowMultiplier; }

    // ===========================================
    //                  BUILDER
    // ===========================================

    public static final class Builder {
        private double deadband = 0.05;
        private Curve curve = Curve.LINEAR;
        private double maxSpeed = 1.0;
        private double slowMultiplier = 0.3;

        /** Radial deadband applied to the raw axis. Default {@code 0.05}. */
        public Builder deadband(double d) { this.deadband = d; return this; }

        /** Joystick response curve. Default {@link Curve#LINEAR}. */
        public Builder curve(Curve c) { this.curve = c; return this; }

        /** Multiplier on shaped output. Default {@code 1.0} (no cap). */
        public Builder maxSpeed(double frac) { this.maxSpeed = frac; return this; }

        /** Speed multiplier when slow mode is engaged. Default {@code 0.3}. */
        public Builder slowMode(double frac) { this.slowMultiplier = frac; return this; }

        public DriverProfile build() { return new DriverProfile(this); }
    }
}
