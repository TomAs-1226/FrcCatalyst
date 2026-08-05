package frc.lib.catalyst.physics.estimation;

import java.util.Arrays;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.catalyst.util.SignalProcessor;

/**
 * Scores, per module, how much each wheel is lying about how far the robot moved.
 *
 * <p>Forward kinematics assumes wheels roll. When one breaks traction it keeps reporting speed the
 * robot is not getting, and that error goes straight into odometry — which is why a robot that spun
 * its wheels leaving the wall is 30 cm off by the time it reaches the first game piece.
 *
 * <p>The check is a residual. Take the chassis velocity you currently believe, run it back through
 * inverse kinematics to get the speed each wheel <em>should</em> be turning, and compare. A module
 * matching its prediction scores 0; one turning a full {@code slipThreshold} faster or slower than
 * predicted scores 1.
 *
 * <p>Only the component of the measured wheel velocity along its predicted direction is compared, so
 * a module still rotating toward its setpoint is not mistaken for one that is slipping.
 *
 * <h2>Reading the two aggregates</h2>
 * <ul>
 *   <li>{@link #slipFactor()} is the <b>mean</b> across modules. Forward kinematics averages the
 *       modules, so the mean is what actually describes how corrupted the odometry estimate is —
 *       this is the number the state estimator uses to decide how far to back off the wheels.</li>
 *   <li>{@link #peakSlip()} is the <b>max</b>. One wheel in the air is a real fault even though it
 *       barely moves the mean, so this is the number to alert and diagnose on.</li>
 * </ul>
 *
 * <p>Below {@link Builder#minSpeedForDetection(double)} the scores decay toward zero: at a standstill
 * every residual is measurement noise, and reporting slip there would cry wolf every time the robot
 * sits still. Scores are smoothed so a single noisy frame cannot trip anything.
 *
 * <p>No hardware, no HAL — feed it module states and it is fully unit testable.
 *
 * @since 1.5.0
 */
public final class SlipEstimator {

    private final SwerveDriveKinematics kinematics;
    private final double slipThresholdMps;
    private final double minSpeedForDetection;
    private final SignalProcessor.ExponentialMovingAverage[] filters;
    private final double[] scores;

    private SlipEstimator(Builder builder) {
        this.kinematics = builder.kinematics;
        this.slipThresholdMps = builder.slipThresholdMps;
        this.minSpeedForDetection = builder.minSpeedForDetection;
        this.scores = new double[builder.moduleCount];
        this.filters = new SignalProcessor.ExponentialMovingAverage[builder.moduleCount];
        Arrays.setAll(filters, i -> new SignalProcessor.ExponentialMovingAverage(builder.smoothing));
    }

    /**
     * Score every module against the chassis velocity currently believed.
     *
     * @param measured                measured module states, in the same order as the kinematics
     * @param referenceRobotRelative  the chassis velocity to test against, <b>robot-relative</b>.
     *                                Pass the fused estimate if you have one; the raw
     *                                forward-kinematic speeds work too, and still catch the common
     *                                case where one module disagrees with the other three.
     * @return the aggregate {@link #slipFactor()} after this update, 0 to 1
     */
    public double update(SwerveModuleState[] measured, ChassisSpeeds referenceRobotRelative) {
        if (measured == null || measured.length != scores.length) {
            throw new IllegalArgumentException("expected " + scores.length + " module states, got "
                    + (measured == null ? "null" : measured.length));
        }
        SwerveModuleState[] predicted = kinematics.toSwerveModuleStates(referenceRobotRelative);
        boolean movingEnough = chassisSpeed(referenceRobotRelative) >= minSpeedForDetection;

        for (int i = 0; i < scores.length; i++) {
            double raw = 0.0;
            if (movingEnough) {
                // Project the measured wheel velocity onto the direction the module is predicted to
                // point. A module mid-rotation contributes its along-track component only, so
                // steering lag does not read as slip.
                double angleError = measured[i].angle.getRadians() - predicted[i].angle.getRadians();
                double alongTrack = measured[i].speedMetersPerSecond * Math.cos(angleError);
                double residual = Math.abs(alongTrack - predicted[i].speedMetersPerSecond);
                raw = Math.min(1.0, residual / slipThresholdMps);
            }
            scores[i] = filters[i].calculate(raw);
        }
        return slipFactor();
    }

    /**
     * Mean slip score across modules, 0 to 1 — how compromised the wheel-based velocity estimate is
     * as a whole. This is the weight the state estimator uses.
     */
    public double slipFactor() {
        double sum = 0.0;
        for (double score : scores) sum += score;
        return scores.length == 0 ? 0.0 : sum / scores.length;
    }

    /** Worst single module's score, 0 to 1 — the number to alert on. */
    public double peakSlip() {
        double peak = 0.0;
        for (double score : scores) peak = Math.max(peak, score);
        return peak;
    }

    /** Index of the worst module, or {@code -1} when nothing is slipping at all. */
    public int worstModule() {
        int worst = -1;
        double peak = 0.0;
        for (int i = 0; i < scores.length; i++) {
            if (scores[i] > peak) {
                peak = scores[i];
                worst = i;
            }
        }
        return worst;
    }

    /** A copy of the per-module scores, in kinematics order. */
    public double[] moduleScores() {
        return scores.clone();
    }

    /** How many modules this estimator was built for. */
    public int moduleCount() {
        return scores.length;
    }

    /** Clear all scores and filter history. */
    public void reset() {
        Arrays.fill(scores, 0.0);
        for (SignalProcessor.ExponentialMovingAverage filter : filters) filter.reset();
    }

    private static double chassisSpeed(ChassisSpeeds speeds) {
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    /** Start building a slip estimator. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link SlipEstimator}. */
    public static final class Builder {
        private SwerveDriveKinematics kinematics;
        private int moduleCount = 4;
        private double slipThresholdMps = 0.5;
        private double minSpeedForDetection = 0.25;
        private double smoothing = 0.3;

        /** The drivetrain's kinematics — required, and the module order everything else follows. */
        public Builder kinematics(SwerveDriveKinematics kinematics, int moduleCount) {
            this.kinematics = kinematics;
            this.moduleCount = moduleCount;
            return this;
        }

        /** Four-module convenience for {@link #kinematics(SwerveDriveKinematics, int)}. */
        public Builder kinematics(SwerveDriveKinematics kinematics) {
            return kinematics(kinematics, 4);
        }

        /**
         * The wheel-speed error that counts as fully slipped, in m/s. Defaults to 0.5 — comfortably
         * above encoder and steering noise on a typical swerve, well below a real wheel spin. Lower
         * it for a more sensitive detector at the cost of false positives.
         */
        public Builder slipThreshold(double slipThresholdMps) {
            this.slipThresholdMps = slipThresholdMps;
            return this;
        }

        /**
         * Chassis speed below which detection is suppressed, in m/s. Defaults to 0.25. At a standstill
         * the residuals are noise, and a robot that is not moving cannot be losing traction in any way
         * that matters.
         */
        public Builder minSpeedForDetection(double minSpeedForDetection) {
            this.minSpeedForDetection = minSpeedForDetection;
            return this;
        }

        /**
         * Smoothing on each module's score: {@code 1.0} is unfiltered, smaller is slower and steadier.
         * Defaults to 0.3, which settles in about three loops.
         */
        public Builder smoothing(double smoothing) {
            this.smoothing = smoothing;
            return this;
        }

        /** Validate and build. Throws if kinematics is missing or a parameter is out of range. */
        public SlipEstimator build() {
            if (kinematics == null) {
                throw new IllegalStateException("kinematics is required - slip is measured against "
                        + "the wheel speeds it predicts");
            }
            if (moduleCount < 1) {
                throw new IllegalStateException("moduleCount must be >= 1 (got " + moduleCount + ")");
            }
            if (!(slipThresholdMps > 0)) {
                throw new IllegalStateException("slipThreshold must be > 0 (got " + slipThresholdMps
                        + ") - scores are residuals divided by it");
            }
            if (minSpeedForDetection < 0) {
                throw new IllegalStateException("minSpeedForDetection must be >= 0 (got "
                        + minSpeedForDetection + ")");
            }
            return new SlipEstimator(this);
        }
    }
}
