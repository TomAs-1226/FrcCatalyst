package frc.lib.catalyst.util;

/**
 * Holds the driver's speed down while a swerve drivetrain aims itself at a target, where the target's
 * swing would outrun the turn the drivetrain has left — the constraint layer of shoot-on-the-move.
 *
 * <p>It is built the way team 581 builds theirs ({@code DriveConstraints} and {@code Swerve} in their
 * 2026 offseason code): a translation ceiling that applies while aiming, eased rather than stepped;
 * goal-centric caps on the velocity toward the target and across it, the smaller scale winning and the
 * direction kept; and a "moving beyond the safe shooting speed" gate for the shot. What differs is the
 * cap across the target: 581 set it by hand, and here it is worked out from what the drivetrain can
 * turn.
 *
 * <h2>Why speed, and why only part of it</h2>
 *
 * <p>Only the part of the velocity across the line to the target swings the aim; closing on the target
 * or backing away from it does not. Passing a target {@code d} metres off at {@code v} m/s swings its
 * bearing at {@code v/d} rad/s, and a drivetrain that aims itself can only turn so fast: the facing
 * request has a rate cap, and the modules share one top speed between translating and turning. Past
 * that no heading controller can keep up, and the only thing left to give is speed.
 *
 * <p>So the speed is capped where the swing would use more than {@code 1 - turnReserve} of the turn
 * that is left — the rate cap, or the modules' headroom above the speed, whichever is less. A cap on
 * the closing speed ({@link Config#radialCapMps(double)}, off by default) applies as well; the smaller
 * scale wins and the direction is kept. The cap moves at no more than {@link Config#capSlewMps2(double)},
 * so the robot eases down to it and back, never a jerk. On a model of the Catalyst X1 (simulated) the
 * governor alone took a 3 m/s arc round the target from 44° RMS of aim error to 8.7°, for 15-28% of the
 * mean speed; in straight strafes it cost almost nothing, and in a 4 m/s pass by the target about 6%.
 *
 * <h2>Using it</h2>
 *
 * <pre>{@code
 * AimSpeedGovernor governor = new AimSpeedGovernor(drive.getMaxSpeedMPS(), driveBaseRadiusMeters);
 *
 * // Every loop, on the driver's field velocity, before the heading tracker sees it:
 * double[] v = governor.govern(dt, vx, vy, pose.getX(), pose.getY(), target.getX(), target.getY());
 * tracker.update(now, pose.getX(), pose.getY(), heading, yawRate, v[0], v[1], target.getX(), target.getY());
 *
 * // And in the shot's gate, on the velocity the robot is actually moving at:
 * boolean ready = tracker.onTargetIn(0.25, Math.toRadians(2))
 *     && !governor.movingBeyondSafeSpeed(measured.vx, measured.vy, pose.getX(), pose.getY(),
 *             target.getX(), target.getY());
 * }</pre>
 *
 * <p>An acceleration limit belongs to the same family, and Catalyst already has one:
 * {@code SwerveSetpointGenerator}'s translational acceleration cap. Keep it generous while aiming. On the
 * X1's model a tight one halved the error in 3-4 m/s strafes mostly by making the robot slower, and
 * wrecked arcs: at 6 m/s² the 2 m/s arc went from 3.3° RMS to 8.3°.
 *
 * <p>Plain doubles — no WPILib, no Phoenix — so the same source builds against WPILib 2026 and 2027.
 * Field metres, blue origin; velocities m/s in the field frame; radians counter-clockwise; seconds.
 *
 * @since 2.0.0
 */
public final class AimSpeedGovernor {

    /** The settings. Every one can change between loops; each setter clamps it and ignores a value that is not a number. */
    public static final class Config {
        double maxModuleSpeedMps = 4.5;
        double moduleRadiusM = 0.4;
        double maxTurnRadps = 3.0;
        boolean enabled = true;
        double turnReserve = 0.25;
        double radialCapMps = 0.0;
        double capSlewMps2 = 4.0;

        /**
         * The modules' top speed, m/s — Phoenix's {@code kSpeedAt12Volts}, or
         * {@code SwerveSubsystem.getMaxSpeedMPS()}. Accepted 0.5-10.
         */
        public Config maxModuleSpeedMps(double v) {
            maxModuleSpeedMps = clampFinite(v, 0.5, 10.0, maxModuleSpeedMps);
            return this;
        }

        /** The farthest wheel from the centre of rotation, m (the drive-base radius). Accepted 0.05-1. */
        public Config moduleRadiusM(double v) {
            moduleRadiusM = clampFinite(v, 0.05, 1.0, moduleRadiusM);
            return this;
        }

        /**
         * The fastest turn the aim may ask for, rad/s: the facing request's rate cap, and
         * {@code HeadingTracker.Config.maxRateRadps}. Default 3, accepted 0.1-20.
         */
        public Config maxTurnRadps(double v) {
            maxTurnRadps = clampFinite(v, 0.1, 20.0, maxTurnRadps);
            return this;
        }

        /**
         * Whether the governor acts. Default on. Off, {@link #govern} passes the driver's velocity
         * through; {@link #movingBeyondSafeSpeed} still answers.
         */
        public Config enabled(boolean on) {
            enabled = on;
            return this;
        }

        /**
         * The share of the turn left that the governor keeps back for the aim's own corrections. Default
         * 0.25, accepted 0-0.9. Lower it and the governor caps less, and later.
         */
        public Config turnReserve(double v) {
            turnReserve = clampFinite(v, 0.0, 0.9, turnReserve);
            return this;
        }

        /**
         * The speed allowed toward or away from the target, m/s — 581's radial cap, for a shot whose
         * distance-dependent settings cannot keep up with a fast approach. 0, the default, is no cap.
         * Accepted 0-10.
         */
        public Config radialCapMps(double v) {
            radialCapMps = clampFinite(v, 0.0, 10.0, radialCapMps);
            return this;
        }

        /** How fast the cap may move, m/s per second. Default 4, accepted 0.5-50. */
        public Config capSlewMps2(double v) {
            capSlewMps2 = clampFinite(v, 0.5, 50.0, capSlewMps2);
            return this;
        }

        /** @see #maxModuleSpeedMps(double) */
        public double maxModuleSpeedMps() {
            return maxModuleSpeedMps;
        }

        /** @see #moduleRadiusM(double) */
        public double moduleRadiusM() {
            return moduleRadiusM;
        }

        /** @see #maxTurnRadps(double) */
        public double maxTurnRadps() {
            return maxTurnRadps;
        }

        /** @see #enabled(boolean) */
        public boolean enabled() {
            return enabled;
        }

        /** @see #turnReserve(double) */
        public double turnReserve() {
            return turnReserve;
        }

        /** @see #radialCapMps(double) */
        public double radialCapMps() {
            return radialCapMps;
        }

        /** @see #capSlewMps2(double) */
        public double capSlewMps2() {
            return capSlewMps2;
        }
    }

    /** The swing is worked out as if the target were never nearer than this, m. */
    static final double NEAR_FLOOR_M = 0.6;
    /**
     * How far over the safe speed {@link #movingBeyondSafeSpeed} lets the robot move, as a share: the
     * turn reserve covers that much, and a drivetrain that delivers all it is commanded would otherwise
     * flicker the gate as its speed dithers across the cap.
     */
    static final double SAFE_SPEED_MARGIN = 0.05;

    private final Config config;
    private boolean started;
    private double cap = Double.POSITIVE_INFINITY;
    private double capTarget = Double.POSITIVE_INFINITY;
    private boolean limiting;

    /**
     * @param maxModuleSpeedMps the modules' top speed, m/s
     * @param moduleRadiusM     the farthest wheel from the centre of rotation, m
     */
    public AimSpeedGovernor(double maxModuleSpeedMps, double moduleRadiusM) {
        this(maxModuleSpeedMps, moduleRadiusM, new Config());
    }

    /**
     * @param maxModuleSpeedMps the modules' top speed, m/s
     * @param moduleRadiusM     the farthest wheel from the centre of rotation, m
     * @param config            the rest of the settings, which stay live
     */
    public AimSpeedGovernor(double maxModuleSpeedMps, double moduleRadiusM, Config config) {
        if (!(maxModuleSpeedMps > 0) || !(moduleRadiusM > 0)) {
            throw new IllegalArgumentException("AimSpeedGovernor needs the modules' top speed and radius, got "
                    + maxModuleSpeedMps + " m/s and " + moduleRadiusM + " m");
        }
        this.config = config.maxModuleSpeedMps(maxModuleSpeedMps).moduleRadiusM(moduleRadiusM);
    }

    /** The live settings. */
    public Config config() {
        return config;
    }

    /** Start again: the next {@link #govern} passes its velocity through with no cap carried over. */
    public void reset() {
        started = false;
        cap = Double.POSITIVE_INFINITY;
        capTarget = Double.POSITIVE_INFINITY;
        limiting = false;
    }

    /**
     * One main loop: the driver's field velocity ({@code vx}, {@code vy}) held under the cap, for a robot
     * at ({@code x}, {@code y}) aiming at ({@code tx}, {@code ty}). Returns the field velocity to plan for
     * and send, {vx, vy}: the driver's own, or the same direction at the cap.
     *
     * <p>The cap starts at the driver's speed and eases toward {@link #safeSpeedMps} at no more than
     * {@link Config#capSlewMps2(double)}, so turning the governor on at speed does not brake the robot in
     * one loop. A loop period that is not a positive number of at most 0.1 s, or a pose or target that is
     * not a number, starts it again and passes the driver's velocity through; a velocity that is not a
     * number comes back as zero, never as itself.
     *
     * @param dt the time since the last loop, s
     */
    public double[] govern(double dt, double vx, double vy, double x, double y, double tx, double ty) {
        if (!(Double.isFinite(vx) && Double.isFinite(vy))) {
            return new double[] {0.0, 0.0};
        }
        if (!(Double.isFinite(dt) && dt > 0 && dt <= 0.1 && Double.isFinite(x) && Double.isFinite(y)
                && Double.isFinite(tx) && Double.isFinite(ty))) {
            reset();
            return new double[] {vx, vy};
        }
        double speed = Math.hypot(vx, vy);
        double target = config.enabled ? safeSpeedMps(vx, vy, x, y, tx, ty) : Double.POSITIVE_INFINITY;
        capTarget = target;
        if (!started) {
            started = true;
            cap = Math.max(target, speed);
        }
        // The cap eases toward its target; from above the driver's speed it comes down no faster than it may.
        double slew = config.capSlewMps2 * dt;
        double from = Double.isInfinite(cap) ? Math.max(speed, Double.isInfinite(target) ? speed : target) : cap;
        cap = Double.isInfinite(target) && from >= speed ? Double.POSITIVE_INFINITY
                : Math.max(Math.min(target, from + slew), from - slew);
        limiting = speed > cap;
        if (limiting) {
            return new double[] {vx * cap / speed, vy * cap / speed};
        }
        return new double[] {vx, vy};
    }

    /**
     * The fastest the robot may move in the direction of ({@code vx}, {@code vy}) at ({@code x},
     * {@code y}) and keep aiming at ({@code tx}, {@code ty}), m/s: where the swing across the line to the
     * target uses {@code 1 - turnReserve} of the turn left — the rate cap, or the modules' headroom above
     * the speed — or where the closing speed reaches the radial cap, whichever comes first. Infinite when
     * nothing binds: standing still, driving straight at the target with no radial cap, or on top of it.
     * Only the direction of the velocity matters, not its size.
     */
    public double safeSpeedMps(double vx, double vy, double x, double y, double tx, double ty) {
        double speed = Math.hypot(vx, vy);
        double dx = tx - x;
        double dy = ty - y;
        double dist = Math.hypot(dx, dy);
        if (!(speed > 1e-6) || !(dist > 1e-9)) {
            return Double.POSITIVE_INFINITY;
        }
        double d = Math.max(dist, NEAR_FLOOR_M);
        // The velocity across the line to the target, and along it, per unit of speed.
        double across = Math.abs(dy * vx - dx * vy) / dist / speed;
        double along = Math.abs(dx * vx + dy * vy) / dist / speed;
        double keep = 1.0 - config.turnReserve;
        double swingPerMps = across / d;
        double target = Double.POSITIVE_INFINITY;
        if (swingPerMps > 1e-9) {
            // Swing within the rate cap, and within the modules' headroom above the speed: a module at
            // speed s has (top - s) left to turn with, which is (top - s) / r of rotation.
            double byRate = keep * config.maxTurnRadps / swingPerMps;
            double byModules = keep * config.maxModuleSpeedMps / (config.moduleRadiusM * swingPerMps + keep);
            target = Math.min(byRate, byModules);
        }
        if (config.radialCapMps > 0 && along > 1e-9) {
            target = Math.min(target, config.radialCapMps / along);
        }
        return target;
    }

    /**
     * The shot's speed gate, as 581's {@code isMovingBeyondSafeSpeed}: whether the robot, measured moving
     * at ({@code vx}, {@code vy}) in the field frame, is going more than 5% faster than
     * {@link #safeSpeedMps} for its direction. Hold the shot while it is: the heading cannot be trusted to
     * follow the target's swing. Unlike {@link #limiting()}, which says the driver's command is being held
     * down, this asks whether the robot has actually slowed to the cap yet. Answers whether or not the
     * governor is {@link Config#enabled(boolean) enabled}; true for anything that is not a number, since a
     * speed it cannot judge is not a safe one.
     */
    public boolean movingBeyondSafeSpeed(double vx, double vy, double x, double y, double tx, double ty) {
        if (!(Double.isFinite(vx) && Double.isFinite(vy) && Double.isFinite(x) && Double.isFinite(y)
                && Double.isFinite(tx) && Double.isFinite(ty))) {
            return true;
        }
        return Math.hypot(vx, vy) > safeSpeedMps(vx, vy, x, y, tx, ty) * (1.0 + SAFE_SPEED_MARGIN);
    }

    /** The cap this loop, m/s; infinite when there is none. */
    public double capMps() {
        return cap;
    }

    /** What the cap is easing toward, m/s: {@link #safeSpeedMps} for the driver's direction, or infinite. */
    public double capTargetMps() {
        return capTarget;
    }

    /**
     * Whether the governor held the driver's speed down this loop. While it does, publish {@link #capMps()}
     * as {@code /Catalyst/Aim/SpeedCapMps} so the driver can see why the robot is slower.
     */
    public boolean limiting() {
        return limiting;
    }

    private static double clampFinite(double v, double lo, double hi, double fallback) {
        return Double.isFinite(v) ? Math.max(lo, Math.min(hi, v)) : fallback;
    }
}
