package frc.lib.catalyst.hardware;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A source of robot heading.
 *
 * <p>Catalyst had exactly one of these for years — {@link CatalystGyro}, wrapping a Pigeon 2 — and
 * that was fine, because a roboRIO robot had exactly one IMU worth using. Systemcore has an IMU of
 * its own, so there are now two, and code that wants "the heading" should be able to say so without
 * naming a part number.
 *
 * <p>{@code CatalystGyro} still exists unchanged and is still what a swerve drivetrain should use:
 * the Pigeon is the better sensor for that job, and nothing here changes which one a team picks.
 * What this adds is the ability to hold both — a Pigeon driving odometry and the onboard IMU as an
 * independent second opinion, which is exactly the sort of disagreement Physics Core is built to
 * notice. Two sensors that agree tell you little; two that stop agreeing tell you something broke.
 *
 * <p>Angles are degrees and rates are degrees per second throughout, matching what
 * {@code CatalystGyro} has always returned. Yaw is CCW-positive.
 *
 * @since 2.0.0
 */
public interface CatalystIMU {

    /** Heading as a {@link Rotation2d} — yaw, CCW positive. */
    Rotation2d getHeading();

    /** Yaw in degrees, CCW positive and continuous (it does not wrap at 180). */
    double getYaw();

    /** Pitch in degrees. */
    double getPitch();

    /** Roll in degrees. */
    double getRoll();

    /** Yaw rate in degrees per second, CCW positive. */
    double getYawRate();

    /** Set the current heading to zero. */
    void zeroYaw();

    /** Set the current heading to a specific value, in degrees. */
    void setYaw(double degrees);

    /**
     * Measured linear acceleration in the robot's XY plane, in m/s², or empty if this device does
     * not report it.
     *
     * <p>Default empty so every existing implementation stays valid — a heading source that only
     * knows heading is a perfectly good heading source, and this must not become a method everyone
     * has to write.
     *
     * <p>What makes it worth having is {@link DualIMU}: the difference between two accelerometers at
     * known points on a rigid body measures angular acceleration directly, instead of differentiating
     * a gyro and amplifying its noise. Without this on the interface, a caller has to fetch that
     * from a Pigeon and from Systemcore's onboard IMU by hand, through two unrelated APIs, in two
     * different unit conventions — which is enough friction that nobody does it.
     *
     * <p>Gravity is <em>not</em> removed. On a level robot the Z component carries it and X/Y do
     * not, which is the case this is for; on a robot climbing a ramp some of it leaks into X. Say so
     * rather than silently subtracting an assumed 9.81, which would be wrong in exactly the moment
     * the reading mattered.
     */
    default java.util.Optional<Translation2d> getAcceleration() {
        return java.util.Optional.empty();
    }

    /**
     * Human-readable name for the underlying device, for logs and the spec sheet.
     *
     * <p>Defaults to the implementing class's simple name, which is right often enough that most
     * implementations need not override it.
     */
    default String deviceType() {
        return getClass().getSimpleName();
    }
}
