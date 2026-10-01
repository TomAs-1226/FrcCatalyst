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
 * "deceleration" treats speeding up in -x as braking. Here slowing down means the commanded
 * <em>speed</em> is falling, whichever way the robot points.
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
     * @param maxAccelMpsSq the fastest the velocity may change while the speed is not falling,
     *     m/s^2; not finite or not positive means no limit
     * @param maxDecelMpsSq the fastest it may change while the speed asked is lower than the speed
     *     now (the robot is being slowed); not finite or not positive means no limit
     * @return {vx, vy} to send
     */
    public double[] limit(double dt, double wantVx, double wantVy, double maxAccelMpsSq, double maxDecelMpsSq) {
        if (!Double.isFinite(wantVx) || !Double.isFinite(wantVy)) {
            wantVx = 0.0;
            wantVy = 0.0;
        }
        boolean slowing = Math.hypot(wantVx, wantVy) < Math.hypot(vx, vy);
        double rate = slowing ? maxDecelMpsSq : maxAccelMpsSq;
        double dvx = wantVx - vx;
        double dvy = wantVy - vy;
        double change = Math.hypot(dvx, dvy);
        if (Double.isFinite(rate) && rate > 0.0 && dt > 0.0) {
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
