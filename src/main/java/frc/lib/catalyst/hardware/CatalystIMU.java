package frc.lib.catalyst.hardware;

import org.wpilib.math.geometry.Rotation2d;

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
     * Human-readable name for the underlying device, for logs and the spec sheet.
     *
     * <p>Defaults to the implementing class's simple name, which is right often enough that most
     * implementations need not override it.
     */
    default String deviceType() {
        return getClass().getSimpleName();
    }
}
