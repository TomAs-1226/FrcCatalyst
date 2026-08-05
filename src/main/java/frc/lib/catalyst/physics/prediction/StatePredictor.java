package frc.lib.catalyst.physics.prediction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.lib.catalyst.physics.LocalizationQuality;
import frc.lib.catalyst.physics.PhysicalRobotState;

/**
 * Where the robot will be a short time from now, and how much less you should believe it.
 *
 * <p>Anything that takes time to happen needs this. A shot leaves the robot 120 ms after the trigger;
 * a vision frame describes 40 ms ago; an arm reaches its setpoint half a second from now. Aiming at
 * where the robot <em>is</em> means aiming from the wrong place.
 *
 * <h2>The motion model</h2>
 * Constant velocity plus <b>decaying</b> acceleration. A robot cannot keep accelerating indefinitely
 * — it runs into its traction limit, its free speed, or the driver's next input — so holding the
 * current acceleration constant over a long horizon overshoots badly. Acceleration is decayed with
 * a time constant instead:
 *
 * <pre>
 * a(t) = a0 * exp(-t / tau)
 * v(h) = v0 + a0 * tau * (1 - exp(-h / tau))
 * p(h) = p0 + v0 * h + a0 * tau * (h - tau * (1 - exp(-h / tau)))
 * </pre>
 *
 * <p>These are exact integrals of that model, not a discretisation, so the result does not depend on
 * a step size. Setting {@link Builder#constantAcceleration()} takes {@code tau} to infinity and
 * recovers the familiar {@code p + vh + ah^2/2}.
 *
 * <p>Confidence decays exponentially over the horizon and the standard deviations grow with it, so a
 * prediction 500 ms out is visibly less trustworthy than one 50 ms out. Predicting further than
 * {@link Builder#maxHorizon(double)} is clamped rather than refused — a clamped answer with honest
 * uncertainty beats an exception in a control loop.
 *
 * <p>Pure math, no state, no hardware.
 *
 * @since 1.5.0
 */
public final class StatePredictor {

    private final double accelerationTimeConstant;
    private final double maxHorizonSeconds;
    private final double confidenceHalfLife;
    private final double accelerationUncertaintyFraction;
    private final double headingDriftFraction;

    private StatePredictor(Builder builder) {
        this.accelerationTimeConstant = builder.accelerationTimeConstant;
        this.maxHorizonSeconds = builder.maxHorizonSeconds;
        this.confidenceHalfLife = builder.confidenceHalfLife;
        this.accelerationUncertaintyFraction = builder.accelerationUncertaintyFraction;
        this.headingDriftFraction = builder.headingDriftFraction;
    }

    /** A predictor with the default 0.30 s acceleration time constant and 1.0 s horizon cap. */
    public static StatePredictor withDefaults() {
        return builder().build();
    }

    /**
     * Propagate {@code from} forward by {@code horizonSeconds}.
     *
     * @param from            the state to start from
     * @param horizonSeconds  how far ahead, in seconds; negative is treated as zero and anything past
     *                        {@link Builder#maxHorizon(double)} is clamped to it
     * @return the predicted state, timestamped at the predicted instant, with degraded quality
     */
    public PhysicalRobotState predict(PhysicalRobotState from, double horizonSeconds) {
        double h = Math.max(0.0, Math.min(maxHorizonSeconds, horizonSeconds));
        if (h == 0.0) return from;

        Translation2d v0 = new Translation2d(
                from.fieldVelocity().vxMetersPerSecond, from.fieldVelocity().vyMetersPerSecond);
        Translation2d a0 = from.fieldAcceleration();
        double omega0 = from.fieldVelocity().omegaRadiansPerSecond;
        double alpha0 = from.angularAccelerationRadPerSecSq();

        double velocityGain = velocityIntegral(h);
        double positionGain = positionIntegral(h);

        Translation2d velocity = v0.plus(a0.times(velocityGain));
        Translation2d displacement = v0.times(h).plus(a0.times(positionGain));
        double omega = omega0 + alpha0 * velocityGain;
        double headingDelta = omega0 * h + alpha0 * positionGain;

        Pose2d pose = new Pose2d(
                from.pose().getTranslation().plus(displacement),
                from.pose().getRotation().plus(new Rotation2d(headingDelta)));

        Translation2d acceleration = a0.times(accelerationDecay(h));

        return new PhysicalRobotState(
                from.timestampSeconds() + h,
                pose,
                new ChassisSpeeds(velocity.getX(), velocity.getY(), omega),
                acceleration,
                alpha0 * accelerationDecay(h),
                degrade(from.quality(), h, v0, a0, omega0));
    }

    /** The horizon beyond which {@link #predict} clamps, in seconds. */
    public double maxHorizonSeconds() {
        return maxHorizonSeconds;
    }

    /** {@code exp(-h / tau)}, or 1 when acceleration is held constant. */
    private double accelerationDecay(double h) {
        if (Double.isInfinite(accelerationTimeConstant)) return 1.0;
        return Math.exp(-h / accelerationTimeConstant);
    }

    /** Coefficient of {@code a0} in the velocity integral: {@code tau (1 - e^-h/tau)}, or {@code h}. */
    private double velocityIntegral(double h) {
        if (Double.isInfinite(accelerationTimeConstant)) return h;
        double tau = accelerationTimeConstant;
        return tau * (1.0 - Math.exp(-h / tau));
    }

    /** Coefficient of {@code a0} in the position integral: {@code tau(h - tau(1 - e^-h/tau))}, or {@code h^2/2}. */
    private double positionIntegral(double h) {
        if (Double.isInfinite(accelerationTimeConstant)) return 0.5 * h * h;
        double tau = accelerationTimeConstant;
        return tau * (h - tau * (1.0 - Math.exp(-h / tau)));
    }

    /**
     * Degrade quality across the horizon: confidence decays exponentially, position uncertainty grows
     * by the velocity uncertainty carried over the horizon, velocity uncertainty grows with a fraction
     * of the acceleration being extrapolated, and heading uncertainty grows with the turn rate.
     */
    private LocalizationQuality degrade(LocalizationQuality quality, double h,
                                        Translation2d velocity, Translation2d acceleration, double omega) {
        double confidence = quality.confidence() * Math.exp(-h / confidenceHalfLife);

        double velocityStdDev = quality.velocityStdDevMetersPerSecond()
                + accelerationUncertaintyFraction * acceleration.getNorm() * h;
        double translationStdDev = quality.translationStdDevMeters()
                + quality.velocityStdDevMetersPerSecond() * h;
        double rotationStdDev = quality.rotationStdDevRadians()
                + headingDriftFraction * Math.abs(omega) * h;

        return new LocalizationQuality(confidence, translationStdDev, rotationStdDev, velocityStdDev,
                quality.secondsSinceAbsoluteFix() + h,
                quality.reason().isEmpty() ? "predicted" : quality.reason() + " (predicted)");
    }

    /** Start building a predictor. Every parameter has a default that suits a typical swerve. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link StatePredictor}. */
    public static final class Builder {
        private double accelerationTimeConstant = 0.30;
        private double maxHorizonSeconds = 1.0;
        private double confidenceHalfLife = 0.75;
        private double accelerationUncertaintyFraction = 0.25;
        private double headingDriftFraction = 0.05;

        /**
         * How quickly the current acceleration is assumed to fade, in seconds. Defaults to 0.30 —
         * roughly how long an FRC drivetrain sustains a hard acceleration before hitting a limit.
         * Smaller is more conservative over long horizons.
         */
        public Builder accelerationTimeConstant(double accelerationTimeConstant) {
            this.accelerationTimeConstant = accelerationTimeConstant;
            return this;
        }

        /**
         * Hold acceleration constant instead of decaying it — the textbook {@code p + vh + ah^2/2}.
         * Accurate over the very short horizons a shot release uses; increasingly optimistic beyond
         * a few hundred milliseconds.
         */
        public Builder constantAcceleration() {
            this.accelerationTimeConstant = Double.POSITIVE_INFINITY;
            return this;
        }

        /** Longest horizon that will be predicted, in seconds. Longer requests clamp. Defaults to 1.0. */
        public Builder maxHorizon(double maxHorizonSeconds) {
            this.maxHorizonSeconds = maxHorizonSeconds;
            return this;
        }

        /** Horizon over which confidence falls by {@code 1/e}, in seconds. Defaults to 0.75. */
        public Builder confidenceHalfLife(double confidenceHalfLife) {
            this.confidenceHalfLife = confidenceHalfLife;
            return this;
        }

        /**
         * Fraction of the extrapolated acceleration treated as uncertain. Defaults to 0.25 — a quarter
         * of the acceleration you are projecting forward may not happen.
         */
        public Builder accelerationUncertaintyFraction(double accelerationUncertaintyFraction) {
            this.accelerationUncertaintyFraction = accelerationUncertaintyFraction;
            return this;
        }

        /** Fraction of the turn rate added to heading uncertainty per second. Defaults to 0.05. */
        public Builder headingDriftFraction(double headingDriftFraction) {
            this.headingDriftFraction = headingDriftFraction;
            return this;
        }

        /** Validate and build. */
        public StatePredictor build() {
            if (!(accelerationTimeConstant > 0)) {
                throw new IllegalStateException("accelerationTimeConstant must be > 0 (got "
                        + accelerationTimeConstant + ") - use constantAcceleration() for no decay");
            }
            if (!(maxHorizonSeconds > 0)) {
                throw new IllegalStateException("maxHorizon must be > 0 (got " + maxHorizonSeconds + ")");
            }
            if (!(confidenceHalfLife > 0)) {
                throw new IllegalStateException("confidenceHalfLife must be > 0 (got " + confidenceHalfLife + ")");
            }
            return new StatePredictor(this);
        }
    }
}
