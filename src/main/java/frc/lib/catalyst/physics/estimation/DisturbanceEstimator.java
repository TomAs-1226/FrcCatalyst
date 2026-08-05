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
    private final SignalProcessor.ExponentialMovingAverage slipFilter;
    private final SignalProcessor.ExponentialMovingAverage impactFilter;

    private Translation2d residual = Translation2d.kZero;
    private double slipEvidence = 0.0;
    private double impactEvidence = 0.0;

    /**
     * @param drivetrain  supplies the traction limit that {@link #normalizedMagnitude()} scales against
     * @param smoothing   EMA factor for the residual, {@code (0, 1]}; {@code 1.0} is unfiltered
     */
    public DisturbanceEstimator(DrivetrainModel drivetrain, double smoothing) {
        if (drivetrain == null) throw new IllegalArgumentException("drivetrain model must not be null");
        this.drivetrain = drivetrain;
        this.filterX = new SignalProcessor.ExponentialMovingAverage(smoothing);
        this.filterY = new SignalProcessor.ExponentialMovingAverage(smoothing);
        this.slipFilter = new SignalProcessor.ExponentialMovingAverage(smoothing);
        this.impactFilter = new SignalProcessor.ExponentialMovingAverage(smoothing);
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
        decompose(wheels, measured);
        return residual;
    }

    /**
     * Splits the residual into the part slip can explain and the part it cannot.
     *
     * <p>Slip has a specific signature: it can only ever make the robot accelerate <em>less</em> than
     * the wheels claim, along the direction the wheels are pushing. It can take the achieved
     * acceleration anywhere from the full wheel value down to zero, and no further. So project the
     * measured acceleration onto the wheel direction and ask where it falls:
     *
     * <ul>
     *   <li>Inside {@code [0, |wheels|]} — consistent with slip. The shortfall is
     *       {@link #slipEvidence()}.</li>
     *   <li>Anything left over once that shortfall is accounted for is acceleration slip cannot
     *       produce, so something outside the robot did. That is {@link #impactEvidence()}.</li>
     * </ul>
     *
     * <p>This is what lets a wheel spinning on a slick patch be told apart from a defender arriving,
     * which the raw residual magnitude cannot do — both simply look large. Braking hard, where the
     * wheels and the IMU agree, produces neither.
     */
    private void decompose(Translation2d wheels, Translation2d measured) {
        double limit = drivetrain.maxTractionAccelerationMpsSq();
        double wheelMagnitude = wheels.getNorm();

        double rawSlip;
        Translation2d unexplained;
        if (wheelMagnitude < 1e-6) {
            // The wheels are not claiming anything, so nothing can be blamed on them slipping.
            rawSlip = 0.0;
            unexplained = measured;
        } else {
            Translation2d wheelDirection = wheels.div(wheelMagnitude);
            double along = measured.getX() * wheelDirection.getX() + measured.getY() * wheelDirection.getY();
            double explained = Math.max(0.0, Math.min(wheelMagnitude, along));
            rawSlip = wheelMagnitude - explained;
            unexplained = measured.minus(wheelDirection.times(explained));
        }

        slipEvidence = Math.max(0.0, slipFilter.calculate(rawSlip / limit));
        impactEvidence = Math.max(0.0, impactFilter.calculate(unexplained.getNorm() / limit));
    }

    /**
     * How much of the wheels' claimed acceleration never reached the robot, as a fraction of the
     * traction limit. This is the honest measure of <b>uniform</b> slip — the case per-module residuals
     * are blind to, because when every wheel over-reports together they all agree with each other.
     *
     * <p>{@link frc.lib.catalyst.physics.PhysicsCore} feeds this into the state estimator alongside
     * the per-module slip factor, so the wheels get distrusted in both failure modes rather than only
     * the one the modules can see.
     */
    public double slipEvidence() {
        return slipEvidence;
    }

    /**
     * Acceleration the robot experienced that no amount of wheel slip could account for, as a fraction
     * of the traction limit — something outside the robot did it.
     *
     * <p>This, not the raw residual, is what {@link frc.lib.catalyst.physics.diagnostics.CollisionDetector}
     * thresholds on. Hard acceleration that breaks traction produces a large residual and a large
     * {@link #slipEvidence()} while leaving this near zero, which is exactly the distinction between
     * "the wheels are spinning" and "we were hit".
     */
    public double impactEvidence() {
        return impactEvidence;
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
        slipEvidence = 0.0;
        impactEvidence = 0.0;
        filterX.reset();
        filterY.reset();
        slipFilter.reset();
        impactFilter.reset();
    }
}
