package frc.lib.catalyst.subsystems.swerve;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;

/**
 * What the rotation channel does when the driver is not touching it.
 *
 * <p>The old rule in {@code advancedDrive} was: rotation stick at rest, so lock the heading -
 * snapped to the nearest cardinal - and run the heading loop against it, always. On a robot on the
 * field that is a nice feature. On a robot on blocks it is a robot that cannot turn being told to
 * turn: every module goes tangential and the wheels spin as hard as the loop asks, from the moment
 * of enable, with nobody touching anything. The Catalyst X1's first two enables were exactly that,
 * and they were read as a steer-direction problem because that is what module runaway usually is.
 *
 * <p>The rule now: hold the heading only while the driver is translating. A parked robot has no
 * heading to keep straight, and one that is being carried, tested on a stand, or nudged in the pit
 * must not fight. While translating, the hold engages on the current heading (snapped to a preset
 * if one is within tolerance), a small error is ignored rather than chased, and the correction is
 * clamped to the rate the sticks would allow.
 *
 * <p>A static decision so it can be tested without a drivetrain.
 */
final class HeadingHold {
    private HeadingHold() {}

    /** What to do this loop: the rotation rate to command and the heading now locked (or null). */
    record Decision(double rotRadPerSec, Rotation2d locked, boolean driverRotating) {}

    /**
     * @param rawRot        rotation stick after deadband, -1..1; nonzero means the driver is turning
     * @param translating   whether the translation sticks are off centre after deadband
     * @param heading       the robot's current heading
     * @param locked        the heading held since the driver last let go of rotation, or null
     * @param snapAngles    presets to prefer when a lock is first taken, degrees, or null
     * @param snapTolDeg    how close to a preset the heading must be to snap to it
     * @param pid           the heading loop, with continuous input already enabled
     * @param maxRate       full-stick rotation rate, rad/s
     * @param multiplier    the drive's speed multiplier (slow mode, physics scale)
     */
    static Decision decide(double rawRot, boolean translating, Rotation2d heading, Rotation2d locked,
                           double[] snapAngles, double snapTolDeg, PIDController pid,
                           double maxRate, double multiplier) {
        double limit = Math.abs(maxRate * multiplier);
        if (Math.abs(rawRot) > 0.0) {
            pid.reset();
            return new Decision(rawRot * maxRate * multiplier, null, true);
        }
        if (!translating) {
            // Parked: nothing to hold straight, and possibly nothing that can turn.
            pid.reset();
            return new Decision(0.0, null, false);
        }
        Rotation2d target = locked != null ? locked : snapped(heading, snapAngles, snapTolDeg);
        double rot = pid.calculate(heading.getRadians(), target.getRadians());
        if (pid.atSetpoint()) {
            rot = 0.0;
        }
        rot = Math.max(-limit, Math.min(limit, rot));
        return new Decision(rot, target, false);
    }

    /** The nearest preset within tolerance, else the heading itself. */
    static Rotation2d snapped(Rotation2d heading, double[] snapAngles, double snapTolDeg) {
        if (snapAngles == null) {
            return heading;
        }
        double best = heading.getDegrees();
        double minDiff = Double.MAX_VALUE;
        for (double snap : snapAngles) {
            double diff = Math.abs(normalize(heading.getDegrees() - snap));
            if (diff < minDiff && diff < snapTolDeg) {
                minDiff = diff;
                best = snap;
            }
        }
        return Rotation2d.fromDegrees(best);
    }

    private static double normalize(double degrees) {
        double d = degrees % 360.0;
        if (d > 180.0) {
            d -= 360.0;
        } else if (d < -180.0) {
            d += 360.0;
        }
        return d;
    }
}
