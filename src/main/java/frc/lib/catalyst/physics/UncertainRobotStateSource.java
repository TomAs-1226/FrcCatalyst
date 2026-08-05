package frc.lib.catalyst.physics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N3;

/**
 * A {@link RobotStateSource} that also reports how much to believe itself.
 *
 * <p>The extra method is {@link #quality()}. Everything else — including the 3x3 pose covariance the
 * fancier consumers want — is derived from it by default, so an implementer only has to answer the
 * question they can actually answer.
 *
 * <pre>{@code
 * void driveAtSafeSpeed(UncertainRobotStateSource state) {
 *     double scale = switch (state.quality().level()) {
 *         case HIGH     -> 1.0;
 *         case MODERATE -> 0.6;
 *         case LOW      -> 0.3;
 *         case LOST     -> 0.0;
 *     };
 *     drive.setSpeedMultiplier(scale);
 * }
 * }</pre>
 *
 * @since 1.5.0
 */
public interface UncertainRobotStateSource extends RobotStateSource {

    /** How good the current estimate is, and what is limiting it. */
    LocalizationQuality quality();

    /**
     * Pose uncertainty as a 3x3 covariance matrix over {@code (x, y, theta)}, in metres and radians.
     *
     * <p>The default builds a diagonal matrix from the standard deviations in {@link #quality()},
     * which assumes the three axes are independent. That is an approximation — a robot that has been
     * dead-reckoning through a turn really does have correlated x/y/theta error — but it is the right
     * one for an estimator that only tracks per-axis uncertainty, and it is what
     * {@code addVisionMeasurement} style APIs expect. Override this if your estimator carries a real
     * covariance.
     */
    default Matrix<N3, N3> poseCovariance() {
        LocalizationQuality q = quality();
        double xy = q.translationStdDevMeters() * q.translationStdDevMeters();
        double theta = q.rotationStdDevRadians() * q.rotationStdDevRadians();
        Matrix<N3, N3> covariance = new Matrix<>(Nat.N3(), Nat.N3());
        covariance.set(0, 0, xy);
        covariance.set(1, 1, xy);
        covariance.set(2, 2, theta);
        return covariance;
    }
}
