package frc.lib.catalyst.physics.estimation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.lib.catalyst.physics.model.DrivetrainModel;
import frc.lib.catalyst.util.SignalProcessor;

/**
 * Measures the acceleration the wheels cannot account for — the physical signature of something
 * happening to the robot rather than because of it.
 *
 * <p>The residual is simple and needs no motor model at all: the drive wheels say the robot is
 * accelerating one way, the IMU says it is accelerating another, and the difference is everything
 * outside the drivetrain. Two very different faults fall out of the same number:
 *
 * <ul>
 *   <li><b>Wheels accelerate, robot does not.</b> The tyres are spinning. The residual points against
 *       the direction of travel.</li>
 *   <li><b>Robot accelerates, wheels do not.</b> Something hit it, or it hit something. The residual
 *       points in the direction of the impulse.</li>
 * </ul>
 *
 * <p>Both are reported as an acceleration vector in field coordinates, and — multiplied by mass — as
 * an external force in newtons. {@link #normalizedMagnitude()} scales that against what the carpet
 * could possibly deliver, so {@code 1.0} means "an acceleration the drivetrain physically could not
 * have produced".
 *
 * <p>The residual is smoothed per axis. Raw IMU acceleration on an FRC robot is genuinely noisy —
 * chassis flex and carpet seams both show up in it — and an unfiltered residual would spend its life
 * above any useful threshold.
 *
 * @since 1.5.0
 */
public final class DisturbanceEstimator {

    private final DrivetrainModel drivetrain;
    private final SignalProcessor.ExponentialMovingAverage filterX;
    private final SignalProcessor.ExponentialMovingAverage filterY;

    private Translation2d residual = Translation2d.kZero;

    /**
     * @param drivetrain  supplies the traction limit that {@link #normalizedMagnitude()} scales against
     * @param smoothing   EMA factor for the residual, {@code (0, 1]}; {@code 1.0} is unfiltered
     */
    public DisturbanceEstimator(DrivetrainModel drivetrain, double smoothing) {
        if (drivetrain == null) throw new IllegalArgumentException("drivetrain model must not be null");
        this.drivetrain = drivetrain;
        this.filterX = new SignalProcessor.ExponentialMovingAverage(smoothing);
        this.filterY = new SignalProcessor.ExponentialMovingAverage(smoothing);
    }

    /** A disturbance estimator with the default 0.4 smoothing — settles in roughly two loops. */
    public DisturbanceEstimator(DrivetrainModel drivetrain) {
        this(drivetrain, 0.4);
    }

    /**
     * Update the residual from this loop's two acceleration estimates.
     *
     * @param wheelAccelerationField  field-relative acceleration implied by the drive wheels, m/s^2 —
     *                                the time derivative of the forward-kinematic chassis velocity
     * @param measuredAccelerationField field-relative acceleration measured by the IMU, m/s^2
     * @return the smoothed residual after this update
     */
    public Translation2d update(Translation2d wheelAccelerationField, Translation2d measuredAccelerationField) {
        Translation2d wheels = wheelAccelerationField == null ? Translation2d.kZero : wheelAccelerationField;
        Translation2d measured = measuredAccelerationField == null ? Translation2d.kZero : measuredAccelerationField;
        Translation2d raw = measured.minus(wheels);
        residual = new Translation2d(filterX.calculate(raw.getX()), filterY.calculate(raw.getY()));
        return residual;
    }

    /** The drivetrain model supplying the traction limit and the mass these results are scaled by. */
    public DrivetrainModel drivetrain() {
        return drivetrain;
    }

    /** The current unexplained acceleration, field-relative, in m/s^2. */
    public Translation2d residualAcceleration() {
        return residual;
    }

    /** Magnitude of the unexplained acceleration, in m/s^2. */
    public double magnitudeMpsSq() {
        return residual.getNorm();
    }

    /**
     * The unexplained acceleration as a fraction of what the carpet could deliver. Roughly zero in
     * normal driving; approaching or above {@code 1.0} means the drivetrain cannot be the cause.
     */
    public double normalizedMagnitude() {
        return residual.getNorm() / drivetrain.maxTractionAccelerationMpsSq();
    }

    /**
     * Field-relative direction the disturbance is pushing. Meaningless when the residual is tiny, so
     * this returns {@link Rotation2d#kZero} below 0.05 m/s^2 rather than reporting the angle of noise.
     */
    public Rotation2d direction() {
        if (residual.getNorm() < 0.05) return Rotation2d.kZero;
        return residual.getAngle();
    }

    /**
     * The equivalent external force on the robot, in newtons — {@code m * a} on the residual. Useful
     * for judging whether a contact was a nudge or a collision.
     */
    public double externalForceNewtons() {
        return residual.getNorm() * drivetrain.robot().massKg();
    }

    /** Clear the residual and the filter history. */
    public void reset() {
        residual = Translation2d.kZero;
        filterX.reset();
        filterY.reset();
    }
}
