package frc.lib.catalyst.subsystems.swerve;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

/**
 * Light-weight chassis-aware setpoint generator. Clamps requested
 * {@link ChassisVelocities} by:
 * <ol>
 *   <li>Limiting the rate of change of the translational velocity vector
 *       so the wheels don't break friction (skid).</li>
 *   <li>Capping translation magnitude at the configured max wheel speed.</li>
 *   <li>Capping rotation rate at the configured max angular speed.</li>
 * </ol>
 *
 * <p>This is the cheap version of the "swerve setpoint generator" pattern
 * — it does not solve for wheel-by-wheel feasibility, but it stops the
 * most common driver-induced skid (jerking the stick in a new direction)
 * and is the kind of thing teams hand-roll every season.
 *
 * <p>Drop one into a {@link SwerveSubsystem} consumer:
 *
 * <pre>{@code
 * SwerveSetpointGenerator gen = new SwerveSetpointGenerator(
 *     drive.getMaxSpeedMPS(), drive.getMaxAngularRate(), 8.0); // 8 m/s² accel cap
 *
 * void drive(ChassisVelocities requested) {
 *     ChassisVelocities limited = gen.generate(requested);
 *     drivetrain.setControl(fieldCentricRequest
 *         .withVelocityX(limited.vx)
 *         .withVelocityY(limited.vy)
 *         .withRotationalRate(limited.omega));
 * }
 * }</pre>
 */
public final class SwerveSetpointGenerator {

    private final double maxTranslationMPS;
    private final double maxAngularRPS;
    private final double maxTranslationalAccel;
    private final double maxAngularAccel;

    private ChassisVelocities prev = new ChassisVelocities();
    private double lastTs = -1;

    /**
     * @param maxTranslationMPS     hard cap on translational velocity (m/s)
     * @param maxAngularRPS         hard cap on angular velocity (rad/s)
     * @param maxTranslationalAccel translational accel cap (m/s²)
     */
    public SwerveSetpointGenerator(double maxTranslationMPS, double maxAngularRPS,
                                   double maxTranslationalAccel) {
        this(maxTranslationMPS, maxAngularRPS, maxTranslationalAccel,
                /* maxAngularAccel = */ Math.PI * 8);
    }

    /** Full-control constructor. */
    public SwerveSetpointGenerator(double maxTranslationMPS, double maxAngularRPS,
                                   double maxTranslationalAccel, double maxAngularAccel) {
        this.maxTranslationMPS = maxTranslationMPS;
        this.maxAngularRPS = maxAngularRPS;
        this.maxTranslationalAccel = maxTranslationalAccel;
        this.maxAngularAccel = maxAngularAccel;
    }

    /** Reset the internal "previous" state. Call when re-enabling. */
    public void reset() {
        prev = new ChassisVelocities();
        lastTs = -1;
    }

    /**
     * Clamp the requested speeds against the configured limits, using
     * the elapsed wall time since the last call as {@code dt}.
     */
    public ChassisVelocities generate(ChassisVelocities desired) {
        double now = Timer.getTimestamp();
        double dt = (lastTs < 0) ? 0.02 : Math.max(0.001, now - lastTs);
        lastTs = now;
        return generate(desired, dt);
    }

    /** Variant where the caller provides {@code dt} explicitly (e.g. for tests). */
    public ChassisVelocities generate(ChassisVelocities desired, double dt) {
        // 1. Cap target translation magnitude.
        Translation2d targetV = new Translation2d(
                desired.vx, desired.vy);
        if (targetV.getNorm() > maxTranslationMPS) {
            targetV = targetV.times(maxTranslationMPS / targetV.getNorm());
        }

        // 2. Limit translational acceleration (delta-v cap).
        Translation2d prevV = new Translation2d(
                prev.vx, prev.vy);
        Translation2d deltaV = targetV.minus(prevV);
        double maxDeltaV = maxTranslationalAccel * dt;
        if (deltaV.getNorm() > maxDeltaV) {
            deltaV = deltaV.times(maxDeltaV / deltaV.getNorm());
        }
        Translation2d nextV = prevV.plus(deltaV);

        // 3. Cap rotation and rotational accel.
        double targetOmega = clamp(desired.omega,
                -maxAngularRPS, maxAngularRPS);
        double maxDeltaOmega = maxAngularAccel * dt;
        double nextOmega = clamp(
                targetOmega,
                prev.omega - maxDeltaOmega,
                prev.omega + maxDeltaOmega);

        ChassisVelocities out = new ChassisVelocities(nextV.getX(), nextV.getY(), nextOmega);
        prev = out;
        return out;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
