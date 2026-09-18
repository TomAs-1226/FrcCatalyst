package frc.lib.catalyst.util;

import java.util.function.DoubleUnaryOperator;

/**
 * Shoot-on-the-move heading control for a swerve drivetrain that aims itself: where the robot should
 * face to put a shot into a target while it drives, as a smooth reference for a heading loop that runs
 * where the heading is fresh — Phoenix's {@code FieldCentricFacingAngle}, in the drivetrain's odometry
 * thread.
 *
 * <p>The whole drivebase is the turret. The driver's sticks translate; this class says which way to face
 * and how fast that direction is turning. It is pure arithmetic on doubles — no WPILib, no Phoenix, no
 * NetworkTables — so the same source builds against WPILib 2026 and 2027 and every step can be tested at
 * a desk. For a real turret, {@link AimingSolver} and {@code TurretMechanism.track(...)} are the pieces;
 * this is the drivetrain's equivalent, with the drivetrain's own lag and disturbances handled.
 *
 * <h2>Wiring it to Phoenix</h2>
 *
 * <pre>{@code
 * HeadingTracker tracker = new HeadingTracker();
 * tracker.setTimeOfFlight(shotTime::get);            // distance (m) -> flight time (s); null = no lead
 * tracker.setShooterExit(-0.286, 0.0);               // where the ball leaves, robot frame (m)
 *
 * SwerveRequest.FieldCentricFacingAngle facing = new SwerveRequest.FieldCentricFacingAngle()
 *     .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
 *     .withTargetDirectionPerspective(SwerveRequest.TargetDirectionPerspectiveValue.BlueAlliance)
 *     .withHeadingPID(tracker.config().kP(), 0, 0)   // P only: the tracker models it
 *     .withMaxAbsRotationalRate(tracker.config().maxRateRadps());
 *
 * // At the start of the aiming command:
 * tracker.reset();
 *
 * // Every loop, with the field velocity the driver is commanding (vx, vy):
 * Pose2d pose = drive.getPose();
 * tracker.update(Timer.getTimestamp(), pose.getX(), pose.getY(), pose.getRotation().getRadians(),
 *         drive.getYawRateRadPerSec(), vx, vy, target.getX(), target.getY());
 * if (tracker.aim() != null) {
 *     drivetrain.setControl(facing.withVelocityX(vx).withVelocityY(vy)
 *         .withTargetDirection(Rotation2d.fromRadians(tracker.directionRad()))
 *         .withTargetRateFeedforward(tracker.feedforwardRadps()));
 * }
 * boolean ready = tracker.onTargetIn(0.25, Math.toRadians(2.0));
 * }</pre>
 *
 * <p>The facing request's gain and rate cap must match {@link Config#kP()} and
 * {@link Config#maxRateRadps()}: the disturbance observer models the request it is feeding. The
 * velocity handed to {@link #update} is the one the driver commands, in the blue-origin field frame —
 * the same one sent to the request. Pose, heading and target are the fused ones, and the turn rate is
 * the gyro's own, not the one the wheels report (see {@code VisionPoseSink.getYawRateRadPerSec()}).
 *
 * <h2>What it does, and why</h2>
 *
 * <ol>
 *   <li><b>The aim is solved for the shot, not the robot's centre.</b> The ball leaves from an exit that
 *       may sit off the centre ({@link #setShooterExit}), carrying the robot's velocity plus the exit's
 *       own speed round the turning robot, and flies for a time that depends on how far it has to go
 *       ({@link #setTimeOfFlight}). {@link #solve} finds the heading that lands it by fixed-point
 *       iteration — which needs no monotonic time-of-flight table, unlike a bisection — and says whether
 *       it converged.
 *   <li><b>The velocity is the commanded one, scaled to what the drivetrain delivers.</b> The command is
 *       smooth and a loop ahead of the robot; the measured velocity is neither. But a drivetrain need not
 *       make good all it is asked for — the Catalyst X1's pose moved 0.75-0.83 of its command — so the
 *       share the fused pose actually moves by is learned, slowly, and the command scaled by it.
 *   <li><b>A reference trajectory, propagated from its own state.</b> The heading to follow is not the
 *       noisy aim itself but a second-order reference that moves with the aim's swing — worked out from
 *       the velocity, so it carries no pose noise — and pulls itself onto the aim with bounded
 *       acceleration. It is never re-seeded from the measured heading, so the measurement's noise never
 *       gets into it.
 *   <li><b>Noise is ignored at the reference, not at the output.</b> Where the aim sits inside a band
 *       around the reference, the reference does not chase it; once outside, it closes all the way before
 *       it stops again. The band is wide standing still — where every hundredth of a radian per second of
 *       turn command re-steers all four modules, and the aim only moves because the pose is noisy — and
 *       narrows as the robot speeds up. The reference still moves with the aim's swing inside the band,
 *       so the band costs no lag while driving.
 *   <li><b>Feedforward leads by the drivetrain's delay.</b> The reference's rate is fed forward, advanced
 *       by the time the drivetrain takes to answer a turn ({@link Config#leadS()}) using the reference's
 *       own acceleration — a rate the heading loop would otherwise have to lag into existence.
 *   <li><b>A disturbance observer closes the gap between the turn asked for and the turn the gyro
 *       sees.</b> A drivetrain that turns less than it is told, or yaws on its own while it translates,
 *       leaves a steady heading error a proportional loop can only shrink. The observer compares the
 *       gyro's own rate with what the requests should have produced, and takes the difference out of the
 *       next feedforward. It is frozen, and lets go, when the robot is nearly still, so it cannot wind up
 *       against static friction.
 * </ol>
 *
 * <p>The heading loop itself — a gain on the heading error, applied at 100-250 Hz on the freshest gyro
 * reading — is the facing request's, with {@link Config#kP()}.
 *
 * <p>On a model of the Catalyst X1 fitted to its recordings (simulated, not yet driven), this held
 * 0.8-1.9° RMS in 0.5-1.5 m/s strafes where Phoenix's facing request on its own (kP 8 and a 0.12 s
 * lookahead) held 2.4-7.1°, while steering the modules a third to two thirds as much. At 2 m/s and up,
 * pair it with {@link AimSpeedGovernor} and {@code SwerveSetpointGenerator.Priority.ROTATION}. The
 * shoot-on-the-move page in the docs has the numbers, the wiring and what to watch on the robot.
 *
 * <p>Units throughout: field metres, blue origin; radians counter-clockwise; velocities m/s in the
 * field frame; times in seconds.
 *
 * @since 2.0.0
 */
public final class HeadingTracker {

    // ------------------------------------------------------------------ the aim solve

    /**
     * One aim: {@code headingRad} to face, {@code rateRadps} how fast that heading swings with the robot
     * moving as given, the virtual goal ({@code aimX}, {@code aimY}) the shot is aimed at so that it lands
     * in the target, how far the exit is from it ({@code distanceM}) — use this for shooter and hood
     * lookups — the ball's {@code timeOfFlightS}, and whether the solve {@code converged} and in how many
     * {@code iterations}.
     */
    public record Aim(double headingRad, double rateRadps, double aimX, double aimY, double distanceM,
            double timeOfFlightS, int iterations, boolean converged) {}

    /** The iteration cap for {@link #solve}. */
    public static final int MAX_ITERATIONS = 20;
    /** {@link #solve} has converged when the heading moves less than this between iterations, rad... */
    static final double CONVERGED_RAD = 1e-5;
    /** ...and the time of flight less than this, s. */
    static final double CONVERGED_S = 1e-4;
    /** The swing is worked out as if the aim point were never nearer than this, m. */
    public static final double NEAR_FLOOR_M = 0.6;

    /**
     * The heading that puts a shot into ({@code tx}, {@code ty}) from a robot at ({@code x}, {@code y})
     * moving at ({@code vx}, {@code vy}) and turning at {@code omega}, with the ball leaving from
     * ({@code exitX}, {@code exitY}) in the robot's frame (x forward, y left) and flying for
     * {@code tof(distance)} seconds; a null {@code tof} is no lead at all. {@code headingGuess} starts the
     * iteration — the robot's heading is a good one — and places the exit; without an exit offset it does
     * not matter.
     *
     * <p>The ball leaves the exit at the exit's own velocity — the robot's, plus {@code omega} times the
     * exit's lever arm — so it must be aimed at the target less that velocity times its flight: the
     * virtual goal. The flight depends on the distance to the virtual goal, which depends on the flight: a
     * fixed point, reached by iteration. Any time-of-flight function will do, including a measured table
     * that is not monotonic in distance (a bisection would need one that is). The iteration is a
     * contraction while the exit's speed times the table's slope (s/m) stays below one — 2 m/s against a
     * slope of 0.2 s/m, say — and is capped at {@link #MAX_ITERATIONS}; past that it reports
     * {@code converged == false} with its last answer.
     */
    public static Aim solve(double x, double y, double headingGuess, double vx, double vy, double omega,
            double tx, double ty, DoubleUnaryOperator tof, double exitX, double exitY) {
        double psi = headingGuess;
        double flight = 0.0;
        double gx = tx;
        double gy = ty;
        double ex = 0;
        double ey = 0;
        double evx = vx;
        double evy = vy;
        boolean converged = false;
        int iterations = 0;
        while (iterations < MAX_ITERATIONS) {
            iterations++;
            double c = Math.cos(psi);
            double s = Math.sin(psi);
            ex = exitX * c - exitY * s;
            ey = exitX * s + exitY * c;
            // The exit's velocity: the robot's, plus omega x the lever arm.
            evx = vx - omega * ey;
            evy = vy + omega * ex;
            gx = tx - evx * flight;
            gy = ty - evy * flight;
            double dx = gx - (x + ex);
            double dy = gy - (y + ey);
            double nextFlight = 0.0;
            if (tof != null) {
                double f = tof.applyAsDouble(Math.hypot(dx, dy));
                nextFlight = Double.isFinite(f) && f > 0 ? f : 0.0;
            }
            double nextPsi = Math.atan2(dy, dx);
            boolean settled = Math.abs(wrap(nextPsi - psi)) < CONVERGED_RAD
                    && Math.abs(nextFlight - flight) < CONVERGED_S;
            psi = nextPsi;
            flight = nextFlight;
            if (settled) {
                converged = true;
                break;
            }
        }
        // The goal as the last iteration left it, and its distance from the exit.
        gx = tx - evx * flight;
        gy = ty - evy * flight;
        double dx = gx - (x + ex);
        double dy = gy - (y + ey);
        double d2 = dx * dx + dy * dy;
        // How fast the heading to the goal swings as the exit moves: the goal stays put for a steady
        // velocity. (Its motion as the velocity changes - the acceleration times the flight - is left out
        // on purpose: counted, it asks for swings no drivetrain can make every time the driver reverses,
        // and aims worse.)
        double swing = (dy * evx - dx * evy) / Math.max(d2, NEAR_FLOOR_M * NEAR_FLOOR_M);
        return new Aim(Math.atan2(dy, dx), swing, gx, gy, Math.sqrt(d2), flight, iterations, converged);
    }

    // ------------------------------------------------------------------ settings

    /**
     * The tracker's settings. Every one can change between loops, so each can be a live tunable; each
     * setter clamps to the range given and ignores a value that is not a number. The defaults are what a
     * harness fitted to the Catalyst X1's recordings found best across two fits of its chassis, inside the
     * range where the reference is stable.
     */
    public static final class Config {
        double kP = 6.0;
        double leadS = 0.18;
        double bandwidthRadps = 7.0;
        double maxAccelRadps2 = 12.0;
        double velocitySmoothingS = 0.10;
        double bandStillRad = Math.toRadians(1.2);
        double bandMovingRad = Math.toRadians(0.3);
        double bandFadeMps = 0.6;
        double bandInnerShare = 0.15;
        double observerS = 0.10;
        double plantDelayS = 0.06;
        double plantLagS = 0.25;
        boolean learnDelivery = true;
        double deliveryS = 0.6;
        double maxRateRadps = 3.0;

        /**
         * The facing request's gain on the heading error, (rad/s) per rad. Default 6, accepted 0-15. Give
         * the facing request the same P: the observer models it. If the heading swings on the robot, try
         * 4 before anything else.
         */
        public Config kP(double v) {
            kP = clampFinite(v, 0.0, 15.0, kP);
            return this;
        }

        /**
         * How far the feedforward is advanced, s: about how long the drivetrain takes to answer a turn.
         * Default 0.18, accepted 0-0.3. A chassis that answers quickly (about 50 ms behind its wheels)
         * wants about 0.12.
         */
        public Config leadS(double v) {
            leadS = clampFinite(v, 0.0, 0.3, leadS);
            return this;
        }

        /**
         * How fast the reference closes on the aim, rad/s: its natural frequency, critically damped.
         * Default 7, accepted 1-12.
         */
        public Config bandwidthRadps(double v) {
            bandwidthRadps = clampFinite(v, 1.0, 12.0, bandwidthRadps);
            return this;
        }

        /**
         * The most the reference may accelerate, rad/s². Default 12, accepted 1-60. (Its acceleration
         * is not also rate-limited: a jerk limit inside this loop made it cycle at higher bandwidths in
         * the harness, and bought no smoothness.)
         */
        public Config maxAccelRadps2(double v) {
            maxAccelRadps2 = clampFinite(v, 1.0, 60.0, maxAccelRadps2);
            return this;
        }

        /**
         * How long the velocity the aim swings with is smoothed over, s: a thumb holding a creep
         * trembles. Default 0.10, accepted 0-0.5.
         */
        public Config velocitySmoothingS(double v) {
            velocitySmoothingS = clampFinite(v, 0.0, 0.5, velocitySmoothingS);
            return this;
        }

        /**
         * How far the aim may wander from the reference, standing still, before it is chased, rad.
         * Default 1.2° (0.021 rad), accepted 0-5°. Raise it if the modules twitch while the robot is
         * parked at the target.
         */
        public Config bandStillRad(double v) {
            bandStillRad = clampFinite(v, 0.0, Math.toRadians(5), bandStillRad);
            return this;
        }

        /** The same band at speed, rad: default 0.3° (0.005 rad), accepted 0-5°. */
        public Config bandMovingRad(double v) {
            bandMovingRad = clampFinite(v, 0.0, Math.toRadians(5), bandMovingRad);
            return this;
        }

        /**
         * The speed by which the band has narrowed from {@link #bandStillRad} to {@link #bandMovingRad},
         * m/s. Default 0.6, accepted 0.05-5.
         */
        public Config bandFadeMps(double v) {
            bandFadeMps = clampFinite(v, 0.05, 5.0, bandFadeMps);
            return this;
        }

        /**
         * Once chasing, the reference closes to within this share of the band before it stops. Default
         * 0.15, accepted 0-1.
         */
        public Config bandInnerShare(double v) {
            bandInnerShare = clampFinite(v, 0.0, 1.0, bandInnerShare);
            return this;
        }

        /**
         * The disturbance observer's time constant, s; 0 turns it off. Default 0.10, accepted 0-2. The
         * second thing to try, after {@link #kP}, if the heading swings.
         */
        public Config observerS(double v) {
            observerS = clampFinite(v, 0.0, 2.0, observerS);
            return this;
        }

        /** The drivetrain's own delay the observer expects, s. Default 0.06, accepted 0-0.3. */
        public Config plantDelayS(double v) {
            plantDelayS = clampFinite(v, 0.0, 0.3, plantDelayS);
            return this;
        }

        /**
         * The drivetrain's lag the observer expects, s. Default 0.25, accepted 0-0.3. A chassis that
         * trails its wheels by about 50 ms at speed rather than 250 wants about 0.07.
         */
        public Config plantLagS(double v) {
            plantLagS = clampFinite(v, 0.0, 0.3, plantLagS);
            return this;
        }

        /** Whether the share of the commanded velocity the drivetrain delivers is learned. Default on. */
        public Config learnDelivery(boolean v) {
            learnDelivery = v;
            return this;
        }

        /** How long that learning averages over, s. Default 0.6, accepted 0.1-10. */
        public Config deliveryS(double v) {
            deliveryS = clampFinite(v, 0.1, 10.0, deliveryS);
            return this;
        }

        /**
         * The largest turn rate the facing request may command, rad/s — the observer knows it cannot
         * exceed it, and the feedforward is capped at it. Default 3, accepted 0.1-20. Give the facing
         * request the same {@code withMaxAbsRotationalRate}.
         */
        public Config maxRateRadps(double v) {
            maxRateRadps = clampFinite(v, 0.1, 20.0, maxRateRadps);
            return this;
        }

        /** @see #kP(double) */
        public double kP() {
            return kP;
        }

        /** @see #leadS(double) */
        public double leadS() {
            return leadS;
        }

        /** @see #bandwidthRadps(double) */
        public double bandwidthRadps() {
            return bandwidthRadps;
        }

        /** @see #maxAccelRadps2(double) */
        public double maxAccelRadps2() {
            return maxAccelRadps2;
        }

        /** @see #velocitySmoothingS(double) */
        public double velocitySmoothingS() {
            return velocitySmoothingS;
        }

        /** @see #bandStillRad(double) */
        public double bandStillRad() {
            return bandStillRad;
        }

        /** @see #bandMovingRad(double) */
        public double bandMovingRad() {
            return bandMovingRad;
        }

        /** @see #bandFadeMps(double) */
        public double bandFadeMps() {
            return bandFadeMps;
        }

        /** @see #bandInnerShare(double) */
        public double bandInnerShare() {
            return bandInnerShare;
        }

        /** @see #observerS(double) */
        public double observerS() {
            return observerS;
        }

        /** @see #plantDelayS(double) */
        public double plantDelayS() {
            return plantDelayS;
        }

        /** @see #plantLagS(double) */
        public double plantLagS() {
            return plantLagS;
        }

        /** @see #learnDelivery(boolean) */
        public boolean learnDelivery() {
            return learnDelivery;
        }

        /** @see #deliveryS(double) */
        public double deliveryS() {
            return deliveryS;
        }

        /** @see #maxRateRadps(double) */
        public double maxRateRadps() {
            return maxRateRadps;
        }
    }

    // ------------------------------------------------------------------ state

    private final Config config;
    private DoubleUnaryOperator timeOfFlight;
    private double exitX;
    private double exitY;

    private boolean started;
    private double lastT;
    /** The reference: heading (continuous, not wrapped), rate and acceleration. */
    private double refRad;
    private double refRate;
    private double refAccel;
    private boolean chasing;
    /** The velocity the aim swings with, smoothed. */
    private double aimVx;
    private double aimVy;
    /** The aim's rate last loop, for its acceleration. */
    private double lastGoalRate = Double.NaN;
    private double goalAccel;
    /** The aim this loop. */
    private Aim aim;
    private double goalRad;
    /** The share of the commanded velocity the drivetrain makes good, as learned. */
    private double delivered = 1.0;
    private double lastPoseX = Double.NaN;
    private double lastPoseY;
    private final double[] cmdHistX = new double[64];
    private final double[] cmdHistY = new double[64];
    private int cmdCount;
    /** The disturbance observer: the requests sent, one per loop, and what they should have produced. */
    private final double[] requestHist = new double[64];
    private int requestCount;
    private double modelRate;
    private double disturbance;
    /** This loop's outputs. */
    private double direction;
    private double feedforward;
    private double leadTerm;
    private double measuredHeading;

    /** A tracker with the default settings. */
    public HeadingTracker() {
        this(new Config());
    }

    /** A tracker with the given settings, which stay live: change them between loops as you like. */
    public HeadingTracker(Config config) {
        this.config = config;
    }

    /** The live settings. */
    public Config config() {
        return config;
    }

    /**
     * The shot's time of flight by distance: metres from the exit to the virtual goal in, seconds out.
     * Null (the default) aims straight at the target with no lead. Anything will do, including a measured
     * table that is not monotonic — {@code InterpolatingTable::get} fits. A value that is not a positive
     * number is taken as no flight.
     */
    public void setTimeOfFlight(DoubleUnaryOperator tof) {
        timeOfFlight = tof;
    }

    /**
     * Where the ball leaves the robot, robot frame (x forward, y left), m. Zero (the default) for a
     * shooter at the centre. Off the centre, the exit sits away from the line the robot faces along and
     * moves sideways as the robot turns — 0.29 m behind the centre, turning at 2 rad/s, it moves at
     * 0.58 m/s, which the ball carries — and {@link #solve} aims for both.
     */
    public void setShooterExit(double xM, double yM) {
        exitX = Double.isFinite(xM) ? xM : 0.0;
        exitY = Double.isFinite(yM) ? yM : 0.0;
    }

    /**
     * Start again from the robot as it is at the next {@link #update}: at the start of each aiming
     * command. The learned delivered share outlives it — it is the drivetrain's, not the aim's.
     */
    public void reset() {
        started = false;
        chasing = false;
        lastGoalRate = Double.NaN;
        goalAccel = 0;
        requestCount = 0;
        disturbance = 0;
        modelRate = 0;
        lastPoseX = Double.NaN;
        cmdCount = 0;
        aim = null;
        aimVx = 0;
        aimVy = 0;
        refAccel = 0;
        feedforward = 0;
        // The share the drivetrain makes good is the robot's, not the aim's, so it outlives a reset - but
        // never as a value that is not a number.
        if (!Double.isFinite(delivered)) {
            delivered = 1.0;
        }
    }

    /**
     * One main loop. A loop with any argument but the gyro rate not a number is not used: the tracker
     * starts again from the next good one, and {@link #aim()} reads null until then. A gap of more than
     * 0.1 s since the last loop starts it again too.
     *
     * @param now            time, s
     * @param x              fused pose, m
     * @param y              fused pose, m
     * @param headingRad     fused heading, rad — the one the facing request closes its loop on
     * @param gyroRateRadps  the gyro's own turn rate, rad/s; NaN when there is none (the observer then
     *                       rests)
     * @param vx             the field velocity the driver commands, m/s
     * @param vy             the field velocity the driver commands, m/s
     * @param tx             the target, m
     * @param ty             the target, m
     */
    public void update(double now, double x, double y, double headingRad, double gyroRateRadps, double vx,
            double vy, double tx, double ty) {
        // A value that is not a number - a pose the estimator lost, a target never set - would otherwise ride
        // the reference and the learned share into every loop after it. Such a loop is not used: the tracker
        // starts again from the next good one, and aim() reads null until then. (The gyro rate may be NaN;
        // the observer rests without it.)
        if (!(Double.isFinite(now) && Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(headingRad)
                && Double.isFinite(vx) && Double.isFinite(vy) && Double.isFinite(tx) && Double.isFinite(ty))) {
            reset();
            return;
        }
        double dt = now - lastT;
        boolean gap = !started || !(dt > 0 && dt <= 0.1);
        if (gap) {
            // Nothing carries over a gap: the robot has moved on without the tracker.
            if (started) {
                reset();
            }
            dt = 0.02;
        }
        lastT = now;
        measuredHeading = headingRad;

        learnDelivery(x, y, vx, vy, dt, gap);
        double k = gap || config.velocitySmoothingS <= 0 ? 1.0 : 1.0 - Math.exp(-dt / config.velocitySmoothingS);
        aimVx += k * (delivered * vx - aimVx);
        aimVy += k * (delivered * vy - aimVy);
        double avx = aimVx;
        double avy = aimVy;
        double speed = Math.hypot(avx, avy);

        // ------------------------------------------------ the aim, for the shot
        double guess = started ? refRad : headingRad;
        aim = solve(x, y, guess, avx, avy, started ? refRate : 0.0, tx, ty, timeOfFlight, exitX, exitY);
        double goalRate = aim.rateRadps();
        // The aim's acceleration is a difference of two rates, so it is smoothed, and kept to what a robot can do.
        double rawAccel = Double.isNaN(lastGoalRate) || gap ? 0.0 : (goalRate - lastGoalRate) / dt;
        goalAccel += (1.0 - Math.exp(-dt / 0.08)) * (clamp(rawAccel, -config.maxAccelRadps2, config.maxAccelRadps2)
                - goalAccel);
        lastGoalRate = goalRate;

        if (!started) {
            started = true;
            refRad = headingRad;
            refRate = Double.isFinite(gyroRateRadps) ? gyroRateRadps : 0.0;
            refAccel = 0;
            chasing = true;
        }
        goalRad = refRad + wrap(aim.headingRad() - refRad);

        // ------------------------------------------------ the reference
        double error = refRad - goalRad;
        double band = config.bandMovingRad + (config.bandStillRad - config.bandMovingRad)
                * Math.max(0.0, 1.0 - speed / config.bandFadeMps);
        if (chasing) {
            chasing = Math.abs(error) > band * config.bandInnerShare;
        } else {
            chasing = Math.abs(error) > band;
        }
        double wn = config.bandwidthRadps;
        double pull = chasing ? wn * wn * error : 0.0;
        double accel = goalAccel - pull - 2.0 * wn * (refRate - goalRate);
        accel = clamp(accel, -config.maxAccelRadps2, config.maxAccelRadps2);
        refRate += accel * dt;
        refRad += refRate * dt;
        refAccel = accel;

        // ------------------------------------------------ what the facing request is given
        // Faced halfway through the loop this direction is held for, so the heading loop's error is centred.
        direction = refRad + refRate * 0.5 * dt;
        leadTerm = config.leadS * refAccel;
        double ff = refRate + leadTerm;
        observe(gyroRateRadps, headingRad, speed, ff, dt);
        // Capped here as well as by the facing request's own limit: the lead and the observer can add up to
        // about twice the rate limit on a hard swing, and nothing else stands between that and the modules.
        feedforward = clamp(ff - disturbance, -config.maxRateRadps, config.maxRateRadps);
    }

    /**
     * The disturbance observer. What the facing request asks for is about the feedforward plus the gain on
     * the heading error; the drivetrain should turn at that, delayed and lagged. Whatever the gyro sees beyond
     * it is a disturbance - the drivetrain turning more or less than it is told, or yawing on its own - and
     * the feedforward takes it out.
     */
    private void observe(double gyroRate, double heading, double speed, double ff, double dt) {
        double request = clamp(ff - disturbance + config.kP * wrap(direction - heading), -config.maxRateRadps,
                config.maxRateRadps);
        int n = requestHist.length;
        requestHist[requestCount % n] = request;
        requestCount++;
        int delayLoops = (int) Math.round(config.plantDelayS / dt);
        int idx = requestCount - 1 - delayLoops;
        double delayed = idx >= 0 ? requestHist[idx % n] : 0.0;
        modelRate += (config.plantLagS > 0 ? 1.0 - Math.exp(-dt / config.plantLagS) : 1.0) * (delayed - modelRate);
        boolean moving = speed > 0.25 || Math.abs(refRate) > 0.4;
        if (config.observerS <= 0 || !Double.isFinite(gyroRate) || requestCount <= delayLoops + 2) {
            disturbance = 0;
            return;
        }
        if (moving) {
            double k = 1.0 - Math.exp(-dt / config.observerS);
            disturbance += k * ((gyroRate - modelRate) - disturbance);
            disturbance = clamp(disturbance, -1.5, 1.5);
        } else {
            // Nearly still: static friction owns the drivetrain here, and an observer would wind up against it.
            disturbance *= Math.exp(-dt / 0.2);
        }
    }

    /**
     * Learn how much of the commanded velocity the drivetrain makes good: the fused pose's movement over the
     * last loop against the command from about the drivetrain's delay ago, averaged over
     * {@link Config#deliveryS()} and kept between 0.5 and 1.2 so that a vision jump or a reseed cannot run
     * away with it.
     */
    private void learnDelivery(double x, double y, double vx, double vy, double dt, boolean gap) {
        int n = cmdHistX.length;
        cmdHistX[cmdCount % n] = vx;
        cmdHistY[cmdCount % n] = vy;
        cmdCount++;
        if (!config.learnDelivery) {
            delivered = 1.0;
        } else if (!gap && !Double.isNaN(lastPoseX) && cmdCount > 4) {
            int back = cmdCount - 1 - 2;
            double cx = cmdHistX[back % n];
            double cy = cmdHistY[back % n];
            double c2 = cx * cx + cy * cy;
            if (c2 > 0.3 * 0.3) {
                double px = (x - lastPoseX) / dt;
                double py = (y - lastPoseY) / dt;
                double ratio = clampFinite((px * cx + py * cy) / c2, 0.3, 1.5, delivered);
                delivered += (1.0 - Math.exp(-dt / config.deliveryS)) * (ratio - delivered);
                delivered = clampFinite(delivered, 0.5, 1.2, 1.0);
            }
        }
        lastPoseX = x;
        lastPoseY = y;
    }

    // ------------------------------------------------------------------ outputs

    /** The direction for the facing request to face, rad (continuous; wrap it if you need to). */
    public double directionRad() {
        return direction;
    }

    /**
     * The turn rate for the facing request to feed forward, rad/s: the reference's, led, less the
     * disturbance, and never more than {@link Config#maxRateRadps()}.
     */
    public double feedforwardRadps() {
        return feedforward;
    }

    /** The reference heading, rad (continuous). */
    public double referenceRad() {
        return refRad;
    }

    /** The reference's turn rate, rad/s. Standing still at the target it should read 0. */
    public double referenceRateRadps() {
        return refRate;
    }

    /** The reference's acceleration, rad/s². */
    public double referenceAccelRadps2() {
        return refAccel;
    }

    /** The feedforward's lead, rad/s: the reference's acceleration times {@link Config#leadS()}. */
    public double leadTermRadps() {
        return leadTerm;
    }

    /**
     * The disturbance observer's estimate, rad/s: how much faster the gyro turns than the requests
     * explain. Near 0 below 0.25 m/s; while strafing it should flip with the direction.
     */
    public double disturbanceRadps() {
        return disturbance;
    }

    /** The learned share of the commanded velocity the drivetrain makes good, 0.5-1.2. */
    public double deliveredShare() {
        return delivered;
    }

    /** Whether the reference is closing on the aim (outside its noise band) or only moving with its swing. */
    public boolean chasing() {
        return chasing;
    }

    /** This loop's aim, or null before the first {@link #update} and after a loop that could not be used. */
    public Aim aim() {
        return aim;
    }

    /** How far the measured heading is from the aim, rad, positive to turn left; NaN without an aim. */
    public double aimErrorRad() {
        return aim == null ? Double.NaN : wrap(aim.headingRad() - measuredHeading);
    }

    /** How far the measured heading is from the reference, rad: what the facing request is closing. */
    public double trackingErrorRad() {
        return aim == null ? Double.NaN : wrap(refRad - measuredHeading);
    }

    /**
     * Whether the heading will be within {@code toleranceRad} of the aim {@code lookaheadS} from now: the
     * ready-to-shoot answer for a feeder that takes that long to get a ball to the shooter (0.25 s on 5805's
     * robots). A ball fed now leaves then, so the question is where the heading will be, not where it is.
     * The reference and the aim are each carried forward on their rates and accelerations; the error the
     * heading loop still has to close is added as it stands. False while there is no aim, while the solve
     * has not converged, and for a tolerance that is not positive.
     */
    public boolean onTargetIn(double lookaheadS, double toleranceRad) {
        if (aim == null || !aim.converged() || !(toleranceRad > 0)) {
            return false;
        }
        double l = Math.max(0.0, Double.isFinite(lookaheadS) ? lookaheadS : 0.0);
        double ref = refRad + refRate * l + 0.5 * refAccel * l * l;
        double goal = goalRad + aim.rateRadps() * l + 0.5 * goalAccel * l * l;
        return Math.abs(wrap(ref - goal)) + Math.abs(trackingErrorRad()) <= toleranceRad;
    }

    // ------------------------------------------------------------------ helpers

    /** An angle brought into [-pi, pi). */
    public static double wrap(double a) {
        double m = (a + Math.PI) % (2 * Math.PI);
        if (m < 0) {
            m += 2 * Math.PI;
        }
        return m - Math.PI;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double clampFinite(double v, double lo, double hi, double fallback) {
        return Double.isFinite(v) ? clamp(v, lo, hi) : fallback;
    }
}
