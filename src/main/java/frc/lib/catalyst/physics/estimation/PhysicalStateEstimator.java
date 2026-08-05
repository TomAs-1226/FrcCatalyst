package frc.lib.catalyst.physics.estimation;

import java.util.Locale;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.lib.catalyst.physics.LocalizationQuality;
import frc.lib.catalyst.physics.PhysicalRobotState;
import frc.lib.catalyst.util.SignalProcessor;

/**
 * Fuses wheel odometry with the IMU into one velocity estimate, and says how much to believe it.
 *
 * <h2>What it does</h2>
 * The two available velocity sources fail in opposite ways. Wheel odometry is exact at steady state
 * and wrong the instant a tyre breaks loose. Integrated IMU acceleration does not care about traction
 * at all but drifts within a second or two. A complementary observer takes the part of each that
 * works:
 *
 * <pre>
 * v_imu   = v_previous + a_measured * dt        (short-term, traction-blind)
 * v_fused = w * v_wheels + (1 - w) * v_imu      (w falls as slip rises)
 * </pre>
 *
 * <p>{@code w} is the trust placed in the wheels. With everything rolling it sits at
 * {@link Builder#kinematicTrust(double)} (0.98 by default — the wheels really are that good), and it
 * falls toward {@link Builder#minimumKinematicTrust(double)} as {@link SlipEstimator} reports slip.
 * It never reaches zero: an estimate running purely on integrated acceleration diverges, so the
 * wheels always retain a small vote to anchor it.
 *
 * <p>Reported acceleration is the derivative of the fused velocity, which makes the output
 * self-consistent — it equals the IMU reading when the wheels are distrusted, and the finite
 * difference of odometry when they are trusted.
 *
 * <h2>What it deliberately does not do</h2>
 * It never writes a pose. The pose passed in is passed straight back out, and
 * {@link #recordAbsoluteFix(double)} only affects <em>confidence</em>. Phase 1 of Physics Core runs in
 * shadow mode: your drivetrain estimator stays the single writer of {@code Pose2d}, so enabling this
 * cannot move the robot's idea of where it is. Pose fusion is a later, opt-in phase.
 *
 * <h2>Where confidence comes from</h2>
 * Confidence starts at 1.0 and three things subtract from it, each capped so no single factor can
 * zero it alone:
 * <ul>
 *   <li><b>Staleness</b> (up to 0.40) — how long since an absolute fix was accepted, against
 *       {@link Builder#absoluteFixHalfLife(double)}.</li>
 *   <li><b>Slip</b> (up to 0.35) — the current slip factor.</li>
 *   <li><b>Disagreement</b> (up to 0.25) — how far the wheels and the IMU are from each other, against
 *       {@link Builder#residualScale(double)}. Large when something is wrong that slip alone does not
 *       explain.</li>
 * </ul>
 * Standard deviations are then interpolated from confidence between the builder's best-case and
 * worst-case values. The weights are honest engineering judgement, not statistically validated
 * probabilities — they are documented here so they can be argued with, and tuned per robot.
 *
 * <p>No HAL, no NetworkTables, no hardware: every input is an argument and the clock is the caller's.
 *
 * @since 1.5.0
 */
public final class PhysicalStateEstimator {

    private final double kinematicTrust;
    private final double minimumKinematicTrust;
    private final double absoluteFixHalfLife;
    private final double residualScale;
    private final double maxSampleGapSeconds;
    private final double bestTranslationStdDev;
    private final double worstTranslationStdDev;
    private final double bestRotationStdDev;
    private final double worstRotationStdDev;
    private final double bestVelocityStdDev;
    private final double worstVelocityStdDev;
    private final boolean hasAccelerometer;

    private final SignalProcessor.ExponentialMovingAverage accelX;
    private final SignalProcessor.ExponentialMovingAverage accelY;
    private final SignalProcessor.ExponentialMovingAverage angularAccel;

    private PhysicalRobotState state = PhysicalRobotState.unknown();
    private double lastTimestamp = Double.NaN;
    private double lastAbsoluteFixTimestamp = Double.NaN;
    private double lastYawRate = 0.0;
    private double lastDisagreement = 0.0;
    private boolean initialized = false;

    /**
     * Velocity variance after an independent velocity observation was folded in, or {@code NaN} when
     * there is none in effect. Grows with process noise every update until it is no worse than the
     * confidence-derived figure, at which point it is dropped — the observation's benefit has expired.
     */
    private double observedVelocityVariance = Double.NaN;
    private final double velocityProcessNoise;

    private PhysicalStateEstimator(Builder builder) {
        this.kinematicTrust = builder.kinematicTrust;
        this.minimumKinematicTrust = builder.minimumKinematicTrust;
        this.absoluteFixHalfLife = builder.absoluteFixHalfLife;
        this.residualScale = builder.residualScale;
        this.maxSampleGapSeconds = builder.maxSampleGapSeconds;
        this.bestTranslationStdDev = builder.bestTranslationStdDev;
        this.worstTranslationStdDev = builder.worstTranslationStdDev;
        this.bestRotationStdDev = builder.bestRotationStdDev;
        this.worstRotationStdDev = builder.worstRotationStdDev;
        this.bestVelocityStdDev = builder.bestVelocityStdDev;
        this.worstVelocityStdDev = builder.worstVelocityStdDev;
        this.hasAccelerometer = builder.hasAccelerometer;
        this.velocityProcessNoise = builder.velocityProcessNoise;
        this.accelX = new SignalProcessor.ExponentialMovingAverage(builder.accelerationSmoothing);
        this.accelY = new SignalProcessor.ExponentialMovingAverage(builder.accelerationSmoothing);
        this.angularAccel = new SignalProcessor.ExponentialMovingAverage(builder.accelerationSmoothing);
    }

    /**
     * Fold one loop's measurements into the estimate.
     *
     * @param timestampSeconds            FPGA-clock time these measurements describe
     * @param pose                        pose from the drivetrain estimator; passed through untouched
     * @param kinematicRobotRelativeSpeeds forward-kinematic chassis velocity, robot-relative
     * @param robotRelativeAcceleration   IMU translational acceleration, robot-relative, m/s^2, with
     *                                    gravity already removed; pass {@code null} if you have none
     * @param yawRateRadPerSec            gyro yaw rate, rad/s
     * @param slipFactor                  0 (rolling cleanly) to 1 (wheels tell you nothing), typically
     *                                    from {@link SlipEstimator#slipFactor()}
     * @return the updated state, which is also available later from {@link #state()}
     */
    public PhysicalRobotState update(
            double timestampSeconds,
            Pose2d pose,
            ChassisSpeeds kinematicRobotRelativeSpeeds,
            Translation2d robotRelativeAcceleration,
            double yawRateRadPerSec,
            double slipFactor) {

        Rotation2d heading = pose.getRotation();
        ChassisSpeeds kinematicField =
                ChassisSpeeds.fromRobotRelativeSpeeds(kinematicRobotRelativeSpeeds, heading);
        Translation2d wheelVelocity =
                new Translation2d(kinematicField.vxMetersPerSecond, kinematicField.vyMetersPerSecond);
        Translation2d measuredAccelField = robotRelativeAcceleration == null
                ? Translation2d.kZero
                : robotRelativeAcceleration.rotateBy(heading);

        double dt = initialized ? timestampSeconds - lastTimestamp : Double.NaN;
        boolean continuous = initialized && dt > 0 && dt <= maxSampleGapSeconds;

        Translation2d fusedVelocity;
        Translation2d fusedAccel;
        double fusedAngularAccel;

        if (!continuous) {
            // First sample, or a gap long enough that integrating across it would be fiction
            // (disabled period, loop overrun, replay seek). Take the wheels at face value and
            // restart the derivative filters rather than emitting a huge false acceleration.
            fusedVelocity = wheelVelocity;
            fusedAccel = Translation2d.kZero;
            fusedAngularAccel = 0.0;
            lastDisagreement = 0.0;
            accelX.reset();
            accelY.reset();
            angularAccel.reset();
        } else {
            Translation2d previousVelocity = new Translation2d(
                    state.fieldVelocity().vxMetersPerSecond, state.fieldVelocity().vyMetersPerSecond);
            Translation2d imuVelocity = previousVelocity.plus(measuredAccelField.times(dt));

            double weight = kinematicWeight(slipFactor);
            fusedVelocity = wheelVelocity.times(weight).plus(imuVelocity.times(1.0 - weight));

            lastDisagreement = wheelVelocity.minus(imuVelocity).getNorm();

            Translation2d rawAccel = fusedVelocity.minus(previousVelocity).div(dt);
            fusedAccel = new Translation2d(
                    accelX.calculate(rawAccel.getX()), accelY.calculate(rawAccel.getY()));
            fusedAngularAccel = angularAccel.calculate((yawRateRadPerSec - lastYawRate) / dt);
            growObservedVelocityVariance(dt);
        }

        LocalizationQuality quality = buildQuality(timestampSeconds, slipFactor);
        state = new PhysicalRobotState(
                timestampSeconds,
                pose,
                new ChassisSpeeds(fusedVelocity.getX(), fusedVelocity.getY(), yawRateRadPerSec),
                fusedAccel,
                fusedAngularAccel,
                quality);

        lastTimestamp = timestampSeconds;
        lastYawRate = yawRateRadPerSec;
        initialized = true;
        return state;
    }

    /**
     * How much of the fused velocity comes from the wheels, given the current slip factor. Falls
     * linearly from {@link Builder#kinematicTrust(double)} to
     * {@link Builder#minimumKinematicTrust(double)}, and stays pinned at full trust when no
     * accelerometer was configured — with nothing to fall back to, backing off the wheels would just
     * freeze the estimate.
     */
    public double kinematicWeight(double slipFactor) {
        if (!hasAccelerometer) return kinematicTrust;
        double slip = Math.max(0.0, Math.min(1.0, slipFactor));
        return kinematicTrust - slip * (kinematicTrust - minimumKinematicTrust);
    }

    /**
     * Note that an absolute observation was accepted at {@code timestampSeconds}, resetting the
     * staleness term in confidence.
     *
     * <p>This is the <em>only</em> effect an absolute observation has on the estimator. It moves no
     * pose and changes no velocity; it records that something outside the drivetrain agreed with where
     * the robot thinks it is.
     */
    public void recordAbsoluteFix(double timestampSeconds) {
        lastAbsoluteFixTimestamp = timestampSeconds;
    }

    /**
     * Fold in a velocity measured by something other than the drive wheels — an optical-flow sensor, a
     * passive odometry pod, the frame-to-frame delta of a vision pose.
     *
     * <p>Unlike a pose observation, this genuinely <em>changes the estimate</em>, and it is the one
     * measurement that can. Wheels and IMU both fail during a slip: the wheels report motion the robot
     * is not making, and the IMU has nothing absolute to anchor its integration to. A source that
     * measures how fast the robot is actually moving is the only thing that can settle the
     * disagreement, so ignoring it would be leaving the most useful sensor on the bench.
     *
     * <p>Fusion is inverse-variance weighting, which is the optimal linear combination of two unbiased
     * estimates:
     *
     * <pre>
     * w      = σ_est² / (σ_est² + σ_obs²)
     * v      = v_est + w·(v_obs - v_est)
     * σ_new² = σ_est²·σ_obs² / (σ_est² + σ_obs²)
     * </pre>
     *
     * <p>A confident observation moves the estimate a long way and tightens it; a vague one nudges it
     * and barely helps. The tightened variance then grows back at the configured process noise on
     * every update, so the benefit fades over a second or so rather than being claimed forever.
     *
     * @param fieldVelocity     measured field-relative velocity, m/s
     * @param standardDeviation 1-sigma uncertainty of that measurement, m/s
     * @return true if it was folded in; false before the first {@link #update} call, when there is no
     *         estimate to fuse with
     */
    public boolean applyVelocityObservation(Translation2d fieldVelocity, double standardDeviation) {
        if (!initialized || fieldVelocity == null || !(standardDeviation > 0)) return false;

        double estimateVariance = effectiveVelocityVariance();
        double observationVariance = standardDeviation * standardDeviation;
        double weight = estimateVariance / (estimateVariance + observationVariance);

        Translation2d current = new Translation2d(
                state.fieldVelocity().vxMetersPerSecond, state.fieldVelocity().vyMetersPerSecond);
        Translation2d fused = current.plus(fieldVelocity.minus(current).times(weight));

        observedVelocityVariance =
                estimateVariance * observationVariance / (estimateVariance + observationVariance);

        state = new PhysicalRobotState(
                state.timestampSeconds(),
                state.pose(),
                new ChassisSpeeds(fused.getX(), fused.getY(),
                        state.fieldVelocity().omegaRadiansPerSecond),
                state.fieldAcceleration(),
                state.angularAccelerationRadPerSecSq(),
                rebuildQualityWithVelocity(state.quality()));
        return true;
    }

    /** The variance currently in force on the velocity estimate, m²/s². */
    private double effectiveVelocityVariance() {
        double derived = state.quality().velocityStdDevMetersPerSecond();
        double derivedVariance = derived * derived;
        if (Double.isNaN(observedVelocityVariance)) return derivedVariance;
        return Math.min(derivedVariance, observedVelocityVariance);
    }

    /** Process noise: the observation's benefit decays as the robot keeps moving. */
    private void growObservedVelocityVariance(double dt) {
        if (Double.isNaN(observedVelocityVariance)) return;
        observedVelocityVariance += velocityProcessNoise * velocityProcessNoise * dt;
        double derived = state.quality().velocityStdDevMetersPerSecond();
        if (observedVelocityVariance >= derived * derived) observedVelocityVariance = Double.NaN;
    }

    /** Replaces the velocity standard deviation in a quality with the fused one. */
    private LocalizationQuality rebuildQualityWithVelocity(LocalizationQuality quality) {
        double fusedStdDev = Math.sqrt(effectiveVelocityVariance());
        if (fusedStdDev >= quality.velocityStdDevMetersPerSecond()) return quality;
        return new LocalizationQuality(quality.confidence(), quality.translationStdDevMeters(),
                quality.rotationStdDevRadians(), fusedStdDev, quality.secondsSinceAbsoluteFix(),
                quality.reason());
    }

    /** The latest fused state. {@link PhysicalRobotState#unknown()} before the first update. */
    public PhysicalRobotState state() {
        return state;
    }

    /**
     * How far apart the wheels and the IMU were on the last continuous update, in m/s. A persistently
     * large value with no slip reported points at a calibration problem — wrong wheel radius, wrong
     * gear ratio, or an IMU mounted at an angle nobody told the code about.
     */
    public double sensorDisagreementMps() {
        return lastDisagreement;
    }

    /** Seconds since the last accepted absolute fix; infinite if there has never been one. */
    public double secondsSinceAbsoluteFix() {
        if (Double.isNaN(lastAbsoluteFixTimestamp) || !initialized) return Double.POSITIVE_INFINITY;
        return Math.max(0.0, lastTimestamp - lastAbsoluteFixTimestamp);
    }

    /** True once at least one update has been folded in. */
    public boolean isInitialized() {
        return initialized;
    }

    /** Clear everything back to the pre-first-update condition. */
    public void reset() {
        state = PhysicalRobotState.unknown();
        lastTimestamp = Double.NaN;
        lastAbsoluteFixTimestamp = Double.NaN;
        lastYawRate = 0.0;
        lastDisagreement = 0.0;
        initialized = false;
        observedVelocityVariance = Double.NaN;
        accelX.reset();
        accelY.reset();
        angularAccel.reset();
    }

    private LocalizationQuality buildQuality(double timestampSeconds, double slipFactor) {
        double staleness = Double.isNaN(lastAbsoluteFixTimestamp)
                ? Double.POSITIVE_INFINITY
                : Math.max(0.0, timestampSeconds - lastAbsoluteFixTimestamp);

        double stalenessTerm = 0.40 * saturate(staleness / absoluteFixHalfLife);
        double slipTerm = 0.35 * saturate(slipFactor);
        double disagreementTerm = 0.25 * saturate(lastDisagreement / residualScale);

        double confidence = saturate(1.0 - stalenessTerm - slipTerm - disagreementTerm);
        double degradation = 1.0 - confidence;

        double velocityStdDev = lerp(bestVelocityStdDev, worstVelocityStdDev, degradation);
        // A recent independent velocity observation beats the confidence-derived figure until its
        // benefit has decayed away, so take whichever is tighter.
        if (!Double.isNaN(observedVelocityVariance)) {
            velocityStdDev = Math.min(velocityStdDev, Math.sqrt(observedVelocityVariance));
        }

        return new LocalizationQuality(
                confidence,
                lerp(bestTranslationStdDev, worstTranslationStdDev, degradation),
                lerp(bestRotationStdDev, worstRotationStdDev, degradation),
                velocityStdDev,
                staleness,
                dominantReason(stalenessTerm, slipTerm, disagreementTerm, staleness, slipFactor));
    }

    /** Names whichever term cost the most confidence, so the reason string is actionable. */
    private String dominantReason(double stalenessTerm, double slipTerm, double disagreementTerm,
                                  double staleness, double slipFactor) {
        double worst = Math.max(stalenessTerm, Math.max(slipTerm, disagreementTerm));
        if (worst < 0.05) return "nominal";
        if (worst == stalenessTerm) {
            return Double.isInfinite(staleness)
                    ? "no absolute fix yet"
                    : String.format(Locale.ROOT, "vision stale %.1f s", staleness);
        }
        if (worst == slipTerm) {
            return String.format(Locale.ROOT, "wheel slip %.0f%%", slipFactor * 100.0);
        }
        return String.format(Locale.ROOT, "wheels vs IMU disagree %.2f m/s", lastDisagreement);
    }

    private static double saturate(double value) {
        if (Double.isNaN(value)) return 1.0;
        return Math.max(0.0, Math.min(1.0, value));
    }

    private static double lerp(double best, double worst, double t) {
        return best + (worst - best) * t;
    }

    /** Start building an estimator. Every parameter has a default that suits a typical swerve. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link PhysicalStateEstimator}. */
    public static final class Builder {
        private double kinematicTrust = 0.98;
        private double minimumKinematicTrust = 0.15;
        private double absoluteFixHalfLife = 3.0;
        private double residualScale = 1.0;
        private double maxSampleGapSeconds = 0.25;
        private double accelerationSmoothing = 0.25;
        private double bestTranslationStdDev = 0.02;
        private double worstTranslationStdDev = 1.00;
        private double bestRotationStdDev = Math.toRadians(1.0);
        private double worstRotationStdDev = Math.toRadians(20.0);
        private double bestVelocityStdDev = 0.05;
        private double worstVelocityStdDev = 1.50;
        private boolean hasAccelerometer = true;
        private double velocityProcessNoise = 0.5;

        /**
         * Weight given to wheel odometry when nothing is slipping, {@code (0, 1]}. Defaults to 0.98 —
         * on a healthy swerve the wheels genuinely are the better velocity source, and the remaining
         * 2% keeps the IMU path alive so it is warm when slip starts.
         */
        public Builder kinematicTrust(double kinematicTrust) {
            this.kinematicTrust = kinematicTrust;
            return this;
        }

        /**
         * Floor on wheel trust at full slip, {@code (0, 1)}. Defaults to 0.15. Do not set this to zero:
         * an estimate running purely on integrated acceleration walks away within a second or two, and
         * a small anchor costs far less than that drift.
         */
        public Builder minimumKinematicTrust(double minimumKinematicTrust) {
            this.minimumKinematicTrust = minimumKinematicTrust;
            return this;
        }

        /**
         * Seconds without an absolute fix at which the staleness penalty saturates. Defaults to 3.0 —
         * about how long a robot can dead-reckon across a field before the answer stops being useful.
         */
        public Builder absoluteFixHalfLife(double absoluteFixHalfLife) {
            this.absoluteFixHalfLife = absoluteFixHalfLife;
            return this;
        }

        /**
         * Wheel-versus-IMU disagreement, in m/s, at which the disagreement penalty saturates. Defaults
         * to 1.0.
         */
        public Builder residualScale(double residualScale) {
            this.residualScale = residualScale;
            return this;
        }

        /**
         * Longest gap between samples that is still integrated across, in seconds. Defaults to 0.25.
         * A larger gap restarts the estimate from the wheels instead of integrating acceleration over
         * a period nobody measured.
         */
        public Builder maxSampleGap(double maxSampleGapSeconds) {
            this.maxSampleGapSeconds = maxSampleGapSeconds;
            return this;
        }

        /** Smoothing on the reported acceleration, {@code (0, 1]}. Defaults to 0.25. */
        public Builder accelerationSmoothing(double accelerationSmoothing) {
            this.accelerationSmoothing = accelerationSmoothing;
            return this;
        }

        /**
         * Tell the estimator there is no usable accelerometer. Wheel trust then stays at
         * {@link #kinematicTrust(double)} no matter what slip is reported, because there is no second
         * source to fall back on — the estimate degrades to plain odometry, which is what a robot
         * without an IMU has anyway.
         */
        public Builder withoutAccelerometer() {
            this.hasAccelerometer = false;
            return this;
        }

        /** Position standard deviation at full confidence and at zero, in metres. Defaults 0.02 / 1.00. */
        public Builder translationStdDevRange(double best, double worst) {
            this.bestTranslationStdDev = best;
            this.worstTranslationStdDev = worst;
            return this;
        }

        /** Heading standard deviation at full confidence and at zero, in radians. Defaults 1 / 20 degrees. */
        public Builder rotationStdDevRange(double best, double worst) {
            this.bestRotationStdDev = best;
            this.worstRotationStdDev = worst;
            return this;
        }

        /**
         * How fast the benefit of a velocity observation decays, in m/s per √s. Defaults to 0.5 — a
         * tightened velocity estimate is back to its usual uncertainty within roughly a second, which
         * is about how long a wheel-and-IMU estimate stays good on its own.
         */
        public Builder velocityProcessNoise(double velocityProcessNoise) {
            this.velocityProcessNoise = velocityProcessNoise;
            return this;
        }

        /** Velocity standard deviation at full confidence and at zero, in m/s. Defaults 0.05 / 1.50. */
        public Builder velocityStdDevRange(double best, double worst) {
            this.bestVelocityStdDev = best;
            this.worstVelocityStdDev = worst;
            return this;
        }

        /** Validate and build. */
        public PhysicalStateEstimator build() {
            if (!(kinematicTrust > 0) || kinematicTrust > 1) {
                throw new IllegalStateException("kinematicTrust must be in (0, 1] (got " + kinematicTrust + ")");
            }
            if (!(minimumKinematicTrust > 0) || minimumKinematicTrust > kinematicTrust) {
                throw new IllegalStateException("minimumKinematicTrust must be in (0, kinematicTrust] (got "
                        + minimumKinematicTrust + ") - a zero floor lets the estimate drift on the IMU alone");
            }
            if (!(absoluteFixHalfLife > 0)) {
                throw new IllegalStateException("absoluteFixHalfLife must be > 0 (got " + absoluteFixHalfLife + ")");
            }
            if (!(residualScale > 0)) {
                throw new IllegalStateException("residualScale must be > 0 (got " + residualScale + ")");
            }
            if (!(maxSampleGapSeconds > 0)) {
                throw new IllegalStateException("maxSampleGap must be > 0 (got " + maxSampleGapSeconds + ")");
            }
            return new PhysicalStateEstimator(this);
        }
    }
}
