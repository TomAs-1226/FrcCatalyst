package frc.lib.catalyst.autonomy;

/**
 * Limits how fast a commanded field velocity may change, as a vector. New in Autonomy 2.1.
 *
 * <p>A slammed stick asks for the whole top speed in one loop. The wheels cannot deliver it: they
 * slip, the robot lurches, and a tall robot tips. This hands the drivetrain a velocity that changes
 * no faster than the limit it is given - which should be what the carpet and the robot's stability
 * allow ({@code PhysicsConstraints.maxAccelerationMpsSq()}), or less for a driver who wants a
 * gentler robot.
 *
 * <p>It limits the <em>size of the change of the velocity vector</em>, not each axis on its own.
 * Per-axis limiters, which {@code SwerveSubsystem.enableSlewRateLimiting} uses, let x and y each
 * change at the full rate, so a diagonal push accelerates at 1.4 times the limit and a change of
 * direction bends through a path nobody asked for; and a limiter that calls a falling value
 * "deceleration" treats speeding up in -x as braking.
 *
 * <p>Braking here is what it is on the carpet: a change of velocity <em>against the way the robot is
 * moving</em>. A change straight against the motion takes the braking limit, one along it or from
 * rest takes the launch limit, and one across it - a change of direction at speed - takes a blend of
 * the two by how much of it opposes the motion. So a driver who stops by throwing the stick the other
 * way brakes exactly as hard as one who lets go, until the robot has stopped; then it launches.
 * (Through 2.0.0-rc.3 braking meant "the speed asked is lower than the speed now", which a full
 * reversal never is: it braked at the launch limit, softer than letting go.)
 *
 * <p>Pure: it holds the last velocity it returned and nothing else.
 */
public final class VelocityLimiter {
    private double vx;
    private double vy;

    /** Start from a velocity - the robot's measured one, when a drive command starts. */
    public void reset(double vx, double vy) {
        this.vx = Double.isFinite(vx) ? vx : 0.0;
        this.vy = Double.isFinite(vy) ? vy : 0.0;
    }

    /**
     * One loop.
     *
     * @param dt the loop period, s
     * @param wantVx the velocity asked, m/s
     * @param wantVy the velocity asked, m/s
     * @param maxAccelMpsSq the fastest the velocity may change along the robot's motion or from
     *     rest (a launch), m/s^2; not finite or not positive means no limit
     * @param maxDecelMpsSq the fastest it may change against the robot's motion (braking), m/s^2;
     *     not finite or not positive means no limit
     * @return {vx, vy} to send
     */
    public double[] limit(double dt, double wantVx, double wantVy, double maxAccelMpsSq, double maxDecelMpsSq) {
        if (!Double.isFinite(wantVx) || !Double.isFinite(wantVy)) {
            wantVx = 0.0;
            wantVy = 0.0;
        }
        double dvx = wantVx - vx;
        double dvy = wantVy - vy;
        double change = Math.hypot(dvx, dvy);
        double speed = Math.hypot(vx, vy);
        // How much of the change opposes the motion: 1 straight against it, 0 along it or from rest.
        double braking = change > 1e-9 && speed > 1e-9
                ? Math.max(0.0, -(dvx * vx + dvy * vy) / (change * speed))
                : 0.0;
        double accel = Double.isFinite(maxAccelMpsSq) && maxAccelMpsSq > 0.0 ? maxAccelMpsSq : Double.POSITIVE_INFINITY;
        double decel = Double.isFinite(maxDecelMpsSq) && maxDecelMpsSq > 0.0 ? maxDecelMpsSq : Double.POSITIVE_INFINITY;
        double rate;
        if (Double.isInfinite(accel) || Double.isInfinite(decel)) {
            // No blend with "no limit": whichever the change is more of.
            rate = braking >= 0.5 ? decel : accel;
        } else {
            rate = accel + (decel - accel) * braking;
        }
        if (Double.isFinite(rate) && dt > 0.0) {
            double step = rate * dt;
            if (change > step) {
                dvx *= step / change;
                dvy *= step / change;
            }
        }
        vx += dvx;
        vy += dvy;
        return new double[] {vx, vy};
    }

    /** The velocity last returned, m/s. */
    public double[] velocity() {
        return new double[] {vx, vy};
    }
}
