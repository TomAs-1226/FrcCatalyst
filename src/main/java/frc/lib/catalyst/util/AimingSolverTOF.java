package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Hardware-independent aiming math for a turreted shooter, including
 * Shoot-On-The-Fly (SOTF) motion compensation.
 *
 * <p>This class touches no motors and no NetworkTables — it's pure
 * geometry, so it can be unit-tested without a robot. Feed it the fused
 * robot pose and the field-relative chassis velocity; it returns a
 * {@link Solution} containing the field-relative angle the turret should
 * point, the distance to use for shooter/hood lookups, and (if you
 * supplied the tables) the shooter RPM and hood angle.
 *
 * <h2>The SOTF method (virtual goal)</h2>
 *
 * <p>When the robot is moving, the game piece inherits the robot's
 * velocity. To hit a stationary target you aim at a <em>virtual goal</em>
 * shifted opposite to the robot's motion by one time-of-flight:
 *
 * <pre>
 *   virtualGoal = target − v_field · timeOfFlight
 * </pre>
 *
 * <p>Time-of-flight depends on the distance to the <em>virtual</em> goal,
 * not the real one, so the solve iterates a few times to converge
 * (typically 2–3 iterations is plenty). This is the same approach used
 * by top FRC teams for shoot-while-moving.
 *
 * <h2>Usage</h2>
 *
 * <pre>{@code
 * // One-time setup. Fill the tables from your own range testing.
 * AimingSolver solver = AimingSolver.builder()
 *     .target(FieldConstants.SPEAKER_CENTER)          // alliance-resolved Translation2d
 *     .shotTime(new InterpolatingTable()
 *         .add(1.5, 0.28).add(3.0, 0.42).add(5.0, 0.61))   // distance(m) -> flight time(s)
 *     .shooterRpm(new InterpolatingTable()
 *         .add(1.5, 2600).add(3.0, 3400).add(5.0, 4200))
 *     .hoodAngle(new InterpolatingTable()
 *         .add(1.5, 12).add(3.0, 28).add(5.0, 40))
 *     .iterations(3)
 *     .build();
 *
 * // Each loop:
 * AimingSolver.Solution s = solver.solve(drive.getPose(), drive.getFieldRelativeSpeeds());
 * turret.aimAtFieldAngle(s.turretFieldAngleDeg(), drive.getHeading().getDegrees());
 * shooter.spinUp(s.shooterRpm());
 * hood.goTo(s.hoodDegrees());
 * boolean readyToShoot = turret.atSetpoint() && shooter.atSpeed() && s.feasible();
 * }</pre>
 */
public final class AimingSolverTOF {

    /**
     * The result of an aiming solve.
     *
     * @param turretFieldAngleDeg  field-relative bearing the turret bore should
     *                             point (degrees, CCW positive, 0 = +X field axis).
     *                             Convert to a turret command with
     *                             {@code turretFieldAngleDeg - robotHeadingDeg}.
     * @param distanceMeters       distance to the virtual goal — use this for
     *                             shooter / hood lookups.
     * @param shotTimeSeconds      estimated time of flight at that distance.
     * @param shooterRpm           looked-up shooter speed (0 if no table supplied).
     * @param hoodDegrees          looked-up hood angle (0 if no table supplied).
     * @param virtualGoal          the motion-compensated aim point in field
     *                             coordinates (the real target when stationary).
     * @param turretFieldRateDps   rate of change of the field bearing
     *                             (degrees/second), for turret velocity
     *                             feedforward. Subtract the robot's yaw rate to
     *                             get the robot-relative turret rate.
     * @param feasible             false when the solve produced a degenerate
     *                             result (robot on the target, or beyond max range).
     */
    public record Solution(
            double turretFieldAngleDeg,
            double distanceMeters,
            double shotTimeSeconds,
            double shooterRpm,
            double hoodDegrees,
            Translation2d virtualGoal,
            double turretFieldRateDps,
            boolean feasible) {}

    private final InterpolatingTable shotTime;   // distance(m) -> flight time(s); may be null
    private final InterpolatingTable shooterRpm; // distance(m) -> rpm; may be null
    private final InterpolatingTable hoodAngle;  // distance(m) -> degrees; may be null
    private final int iterations;
    private final double minFeasibleDistance;
    private final double maxRange;               // metres; Double.MAX_VALUE = unbounded
    private final ShotCompensation comp;         // may be null

    private volatile Translation2d target;

    private AimingSolverTOF(Builder b) {
        this.shotTime = b.shotTime;
        this.shooterRpm = b.shooterRpm;
        this.hoodAngle = b.hoodAngle;
        this.iterations = Math.max(1, b.iterations);
        this.minFeasibleDistance = b.minFeasibleDistance;
        this.maxRange = b.maxRange;
        this.comp = b.comp;
        this.target = b.target;
    }

    /**
     * Update the target at runtime — call this on alliance change so the
     * solver aims at your alliance's goal.
     */
    public void setTarget(Translation2d target) {
        this.target = target;
    }

    /** The current aim target in field coordinates. */
    public Translation2d getTarget() {
        return target;
    }

    /**
     * Static aim — ignores robot motion. Use for stationary shots or as a
     * fallback when you don't trust the velocity estimate.
     */
    public Solution solveStatic(Pose2d robotPose) {
        return solve(robotPose, new ChassisSpeeds());
    }

    /**
     * Full Shoot-On-The-Fly solve.
     *
     * @param robotPose            fused field pose (latency-compensated upstream
     *                             via {@link PoseHistory} if you have it).
     * @param fieldRelativeSpeeds  chassis speeds in the FIELD frame. If you only
     *                             have robot-relative speeds, rotate them first
     *                             with {@link ChassisSpeeds#fromRobotRelativeSpeeds}.
     */
    public Solution solve(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
        Translation2d robotXY = robotPose.getTranslation();
        Translation2d goal = target;

        // Condition the velocity used for motion comp: deadband out noise,
        // clamp collision spikes, scale by the SOTF aggressiveness knob. With
        // no compensation object this is the raw field velocity.
        double vx = (comp != null) ? comp.conditionVelocity(fieldRelativeSpeeds.vxMetersPerSecond)
                                   : fieldRelativeSpeeds.vxMetersPerSecond;
        double vy = (comp != null) ? comp.conditionVelocity(fieldRelativeSpeeds.vyMetersPerSecond)
                                   : fieldRelativeSpeeds.vyMetersPerSecond;

        // A NaN/Inf velocity (e.g. a momentary pose-estimator glitch) must never
        // poison the aim. Fall back to a stationary solve for that axis.
        if (!Double.isFinite(vx)) vx = 0.0;
        if (!Double.isFinite(vy)) vy = 0.0;

        // Solve the virtual-goal fixed point:  virtualGoal = goal - v * tof(|virtualGoal - robot|).
        // The shot-time table clamps to its endpoints, so the map is a
        // contraction and this converges quickly. We iterate to convergence
        // (capped) rather than a fixed count, so tof, dist and virtualGoal are
        // all mutually consistent — that is what makes an ideal shot exact and
        // the shooter/hood lookup distance agree with the flight time.
        double dist = goal.getDistance(robotXY);
        double tof = lookup(shotTime, dist, 0.0);

        Translation2d virtualGoal = goal;
        int maxIters = Math.max(iterations, 20);
        for (int i = 0; i < maxIters && tof > 0.0; i++) {
            Translation2d next = new Translation2d(goal.getX() - vx * tof, goal.getY() - vy * tof);
            double nextDist = next.getDistance(robotXY);
            double nextTof = lookup(shotTime, nextDist, 0.0);
            boolean converged = Math.abs(nextDist - dist) < 1e-12;
            virtualGoal = next;
            dist = nextDist;
            tof = nextTof;
            if (converged) break;
        }

        // Safety pin: if the iteration hit its cap without fully converging,
        // re-pin the virtual goal to the reported tof so an ideal shot is still
        // exact by construction (landing = virtualGoal + v*tof = goal). At a
        // true fixed point this is a no-op.
        if (tof > 0.0) {
            virtualGoal = new Translation2d(goal.getX() - vx * tof, goal.getY() - vy * tof);
            dist = virtualGoal.getDistance(robotXY);
        }

        Translation2d aim = virtualGoal.minus(robotXY);
        double aimNorm = aim.getNorm();
        boolean feasible = aimNorm >= minFeasibleDistance && dist <= maxRange;

        // atan2 of a zero vector is 0; guard so a robot sitting on the target
        // doesn't spin the turret to a meaningless angle.
        double fieldAngleDeg = aimNorm >= minFeasibleDistance
                ? Math.toDegrees(Math.atan2(aim.getY(), aim.getX()))
                : 0.0;

        // Analytic field-bearing rate for turret velocity feedforward.
        // bearing = atan2(ry, rx) with r = virtualGoal - robot; the robot moves
        // at (vx, vy) so d(r)/dt ≈ -(vx, vy) to first order. Then
        //   d(bearing)/dt = (ry·vx − rx·vy) / |r|²   (rad/s).
        double rateDps = 0.0;
        if (aimNorm >= minFeasibleDistance) {
            double n2 = aimNorm * aimNorm;
            rateDps = Math.toDegrees((aim.getY() * vx - aim.getX() * vy) / n2);
        }

        // Shooter / hood read off the lookup distance with the operator's
        // distance bias folded in (shoot longer/shorter without moving aim).
        double lookupDist = dist + (comp != null ? comp.distanceBiasMeters() : 0.0);

        double turretBias = (comp != null) ? comp.turretBiasDeg() : 0.0;
        double rpmBias    = (comp != null) ? comp.rpmBias() : 0.0;
        double hoodBias   = (comp != null) ? comp.hoodBiasDeg() : 0.0;

        return new Solution(
                fieldAngleDeg + turretBias,
                lookupDist,
                tof,
                lookup(shooterRpm, lookupDist, 0.0) + rpmBias,
                lookup(hoodAngle, lookupDist, 0.0) + hoodBias,
                virtualGoal,
                rateDps,
                feasible);
    }

    private static double lookup(InterpolatingTable table, double key, double fallback) {
        if (table == null || table.size() == 0) return fallback;
        return table.get(key);
    }

    // ============================================================
    //                        BUILDER
    // ============================================================

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private Translation2d target = new Translation2d();
        private InterpolatingTable shotTime;
        private InterpolatingTable shooterRpm;
        private InterpolatingTable hoodAngle;
        private int iterations = 3;
        private double minFeasibleDistance = 0.10; // metres
        private double maxRange = Double.MAX_VALUE;
        private ShotCompensation comp;

        /** Field-coordinate aim point. Resolve alliance before passing it in. */
        public Builder target(Translation2d target) {
            this.target = target;
            return this;
        }

        /**
         * Distance (metres) → time of flight (seconds). Required for SOTF —
         * without it {@link #solve} behaves like {@link #solveStatic}.
         */
        public Builder shotTime(InterpolatingTable shotTimeByDistance) {
            this.shotTime = shotTimeByDistance;
            return this;
        }

        /** Distance (metres) → shooter RPM. Optional. */
        public Builder shooterRpm(InterpolatingTable rpmByDistance) {
            this.shooterRpm = rpmByDistance;
            return this;
        }

        /** Distance (metres) → hood angle (degrees). Optional. */
        public Builder hoodAngle(InterpolatingTable hoodByDistance) {
            this.hoodAngle = hoodByDistance;
            return this;
        }

        /** Number of virtual-goal refinement passes. Default 3. */
        public Builder iterations(int n) {
            this.iterations = n;
            return this;
        }

        /**
         * Below this robot-to-goal distance the solve is treated as
         * infeasible (turret holds, {@code Solution.feasible()} is false).
         * Default 0.10 m.
         */
        public Builder minFeasibleDistance(double meters) {
            this.minFeasibleDistance = meters;
            return this;
        }

        /**
         * Beyond this distance the shot is marked infeasible. Use your
         * tested effective range so the "ready to fire" gate refuses
         * hopeless long shots. Default unbounded.
         */
        public Builder maxRange(double meters) {
            this.maxRange = meters;
            return this;
        }

        /**
         * Attach a live {@link ShotCompensation} — operator aim biases plus
         * the velocity deadband / clamp / scale used for collision-robust
         * motion compensation. Optional; without it the solver uses raw
         * field velocity and zero bias.
         */
        public Builder compensation(ShotCompensation comp) {
            this.comp = comp;
            return this;
        }

        public AimingSolverTOF build() {
            return new AimingSolverTOF(this);
        }
    }
}
