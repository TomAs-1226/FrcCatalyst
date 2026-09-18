package frc.lib.catalyst.subsystems.swerve;

import java.util.Objects;

import org.wpilib.math.geometry.Rotation2d;
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
 * — by default it does not solve for wheel-by-wheel feasibility, but it stops the
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
 *
 * <h2>Keeping the rotation: {@link Priority#ROTATION}</h2>
 *
 * <p>When translation and rotation together ask a module for more than its top speed, something has
 * to give. By default ({@link Priority#PROPORTIONAL}) the generator leaves that to the drivetrain,
 * whose desaturation shrinks every module in proportion — the turn as much as the translation. For a
 * robot aiming itself while it drives, the turn is the part that must not shrink: losing turn rate is
 * losing the target. {@link Priority#ROTATION} solves the one wheel-by-wheel question that matters
 * there: the largest share of the translation, direction kept, that fits beside the rotation in every
 * module. Give it the module positions:
 *
 * <pre>{@code
 * SwerveSetpointGenerator aimGen = new SwerveSetpointGenerator(
 *     drive.getMaxSpeedMPS(), 3.0, 8.0, Math.PI * 8,
 *     SwerveSetpointGenerator.Priority.ROTATION,
 *     drive.getDrivetrain().getModuleLocations());
 *
 * // Field-relative request: say which way the robot faces, so each module is judged where it is.
 * ChassisVelocities send = aimGen.generate(requested, dt, drive.getHeading());
 * }</pre>
 *
 * <p>The existing overloads, which take no heading, judge the request as robot-relative.
 */
public final class SwerveSetpointGenerator {

    /**
     * How the wheels' top speed is shared between translation and rotation when a request asks for
     * more than every module can deliver at once.
     *
     * @since 2.0.0
     */
    public enum Priority {
        /**
         * Left to the drivetrain, as the generator always has: translation and rotation are capped
         * separately, and the drivetrain's own desaturation (Phoenix's, or WPILib's
         * {@code desaturateWheelSpeeds}) shrinks every module — the translation and the rotation
         * together, in proportion. The default.
         */
        PROPORTIONAL,
        /**
         * The rotation is kept whole and the translation alone gives way: shrunk along its own
         * direction, by the largest factor that keeps every module at or under the max wheel speed
         * while the chassis also turns as asked, solved module by module. A rotation that alone takes a
         * module to the max wheel speed leaves no translation at all, so pair it with a rate cap below
         * that (the facing request's, when aiming). Needs the module positions.
         */
        ROTATION
    }

    private final double maxTranslationMPS;
    private final double maxAngularRPS;
    private final double maxTranslationalAccel;
    private final double maxAngularAccel;
    private final Priority priority;
    /** Wheel positions, robot frame {x, y}, m, for {@link Priority#ROTATION}. */
    private final double[][] modules;

    private ChassisVelocities prev = new ChassisVelocities();
    private double lastTs = -1;
    private double translationScale = 1.0;

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
        this(maxTranslationMPS, maxAngularRPS, maxTranslationalAccel, maxAngularAccel,
                Priority.PROPORTIONAL);
    }

    /**
     * Full control, with a {@link Priority}.
     *
     * @param maxTranslationMPS     hard cap on translational velocity, and the max wheel speed
     *                              {@link Priority#ROTATION} fills (m/s)
     * @param maxAngularRPS         hard cap on angular velocity (rad/s)
     * @param maxTranslationalAccel translational accel cap (m/s²)
     * @param maxAngularAccel       angular accel cap (rad/s²)
     * @param priority              how translation and rotation share the wheels
     * @param modulePositions       the wheel positions, robot frame (x forward, y left), m — as the
     *                              kinematics has them; required for {@link Priority#ROTATION}
     * @throws IllegalArgumentException if {@code priority} is ROTATION and no module positions are given
     * @since 2.0.0
     */
    public SwerveSetpointGenerator(double maxTranslationMPS, double maxAngularRPS,
                                   double maxTranslationalAccel, double maxAngularAccel,
                                   Priority priority, Translation2d... modulePositions) {
        this.maxTranslationMPS = maxTranslationMPS;
        this.maxAngularRPS = maxAngularRPS;
        this.maxTranslationalAccel = maxTranslationalAccel;
        this.maxAngularAccel = maxAngularAccel;
        this.priority = Objects.requireNonNull(priority, "priority");
        int n = modulePositions == null ? 0 : modulePositions.length;
        if (priority == Priority.ROTATION && n == 0) {
            throw new IllegalArgumentException(
                    "Priority.ROTATION needs the module positions to judge each wheel");
        }
        this.modules = new double[n][];
        for (int i = 0; i < n; i++) {
            Translation2d m = Objects.requireNonNull(modulePositions[i], "module position " + i);
            this.modules[i] = new double[] {m.getX(), m.getY()};
        }
    }

    /** Reset the internal "previous" state. Call when re-enabling. */
    public void reset() {
        prev = new ChassisVelocities();
        lastTs = -1;
        translationScale = 1.0;
    }

    /**
     * Clamp the requested speeds against the configured limits, using
     * the elapsed wall time since the last call as {@code dt}.
     */
    public ChassisVelocities generate(ChassisVelocities desired) {
        return generate(desired, wallDt());
    }

    /** Variant where the caller provides {@code dt} explicitly (e.g. for tests). */
    public ChassisVelocities generate(ChassisVelocities desired, double dt) {
        return generate(desired, dt, Rotation2d.kZero);
    }

    /**
     * Field-relative variant, timed by the wall clock: {@code desired} is in the field frame and
     * {@code robotHeading} is which way the robot faces in it. Only {@link Priority#ROTATION} uses the
     * heading.
     *
     * @since 2.0.0
     */
    public ChassisVelocities generate(ChassisVelocities desired, Rotation2d robotHeading) {
        return generate(desired, wallDt(), robotHeading);
    }

    /**
     * Field-relative variant with an explicit {@code dt}: {@code desired} is in the field frame and
     * {@code robotHeading} is which way the robot faces in it. Only {@link Priority#ROTATION} uses the
     * heading; {@code Rotation2d.kZero} makes it robot-relative.
     *
     * @since 2.0.0
     */
    public ChassisVelocities generate(ChassisVelocities desired, double dt, Rotation2d robotHeading) {
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

        // 4. Rotation priority: the translation alone gives way, direction kept, until every module
        // fits beside the rotation. Judged in the robot frame, where the modules are.
        translationScale = 1.0;
        if (priority == Priority.ROTATION && robotHeading != null) {
            double c = robotHeading.getCos();
            double s = robotHeading.getSin();
            double rx = nextV.getX() * c + nextV.getY() * s;
            double ry = -nextV.getX() * s + nextV.getY() * c;
            double scale = translationScale(rx, ry, nextOmega, maxTranslationMPS, modules);
            // Anything that is not a number here (a heading or a rotation that is not one) leaves the
            // translation as it was: the drivetrain's own desaturation still stands behind it.
            if (Double.isFinite(scale)) {
                translationScale = scale;
                nextV = nextV.times(scale);
            }
        }

        ChassisVelocities out = new ChassisVelocities(nextV.getX(), nextV.getY(), nextOmega);
        prev = out;
        return out;
    }

    /**
     * The share of the translation {@link Priority#ROTATION} kept on the last {@code generate}, 0 to 1:
     * below 1 means translation was given up to keep the turn. Always 1 under
     * {@link Priority#PROPORTIONAL}.
     *
     * @since 2.0.0
     */
    public double getTranslationScale() {
        return translationScale;
    }

    /**
     * The largest share of the robot-relative translation ({@code rx}, {@code ry}), 0 to 1, that keeps
     * every module at or under {@code limit} m/s while the chassis also turns at {@code omega}: module i
     * moves at s·v + omega × r_i, and |s·v + w_i|² = limit² is a quadratic in s whose larger root bounds
     * it. A rotation that alone takes a module to the limit leaves no translation at all.
     */
    static double translationScale(double rx, double ry, double omega, double limit, double[][] modules) {
        double a = rx * rx + ry * ry;
        if (a < 1e-12) {
            return 1.0;
        }
        double best = 1.0;
        for (double[] m : modules) {
            double wx = -omega * m[1];
            double wy = omega * m[0];
            double b = rx * wx + ry * wy;
            double cc = wx * wx + wy * wy - limit * limit;
            if (cc >= 0) {
                return 0.0;
            }
            double root = (-b + Math.sqrt(b * b - a * cc)) / a;
            best = Math.min(best, root);
        }
        return Math.max(0.0, best);
    }

    private double wallDt() {
        double now = Timer.getTimestamp();
        double dt = (lastTs < 0) ? 0.02 : Math.max(0.001, now - lastTs);
        lastTs = now;
        return dt;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
