package frc.lib.catalyst.physics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Everything Physics Core knows about the robot's motion at one instant.
 *
 * <p>This is {@code Pose2d} plus the things a pose alone cannot tell you: which way the robot is
 * actually travelling (not just pointing), how hard it is accelerating, and how much of that you
 * should believe. It is an immutable snapshot — {@link PhysicsCore#state()} hands back the latest
 * one, and it stays valid even as the next loop runs.
 *
 * <p>All vectors are <b>field-relative</b>. Acceleration is the derivative of {@link #fieldVelocity()},
 * so it reflects what the robot is really doing, not what the wheels were told to do.
 *
 * @param timestampSeconds                 FPGA-clock time this snapshot describes
 * @param pose                             field-relative pose, owned by the drivetrain estimator
 * @param fieldVelocity                    fused field-relative chassis velocity
 * @param fieldAcceleration                field-relative translational acceleration, m/s^2
 * @param angularAccelerationRadPerSecSq   heading acceleration, rad/s^2
 * @param quality                          how much to believe all of the above
 * @since 1.5.0
 */
public record PhysicalRobotState(
        double timestampSeconds,
        Pose2d pose,
        ChassisSpeeds fieldVelocity,
        Translation2d fieldAcceleration,
        double angularAccelerationRadPerSecSq,
        LocalizationQuality quality) implements UncertainRobotStateSource {

    /** Ground speed in metres per second, ignoring rotation. */
    public double speedMetersPerSecond() {
        return Math.hypot(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    }

    /** Magnitude of {@link #fieldAcceleration()} in metres per second squared. */
    public double accelerationMetersPerSecSq() {
        return fieldAcceleration.getNorm();
    }

    /**
     * Direction of travel as a field-relative bearing. Meaningless when the robot is nearly stopped,
     * so this returns {@link Rotation2d#kZero} below 1 cm/s rather than amplifying noise.
     */
    public Rotation2d headingOfTravel() {
        if (speedMetersPerSecond() < 0.01) return Rotation2d.kZero;
        return new Rotation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    }

    /** The same snapshot with a different quality attached — used when a prediction degrades it. */
    public PhysicalRobotState withQuality(LocalizationQuality newQuality) {
        return new PhysicalRobotState(timestampSeconds, pose, fieldVelocity, fieldAcceleration,
                angularAccelerationRadPerSecSq, newQuality);
    }

    /**
     * A zeroed state with {@link LocalizationQuality#unknown()} quality, returned before the first
     * update. It is deliberately not null: callers get a usable object whose quality tells them not
     * to trust it, rather than a {@code NullPointerException} on the first loop.
     */
    public static PhysicalRobotState unknown() {
        return new PhysicalRobotState(0.0, Pose2d.kZero, new ChassisSpeeds(),
                Translation2d.kZero, 0.0, LocalizationQuality.unknown());
    }
}
