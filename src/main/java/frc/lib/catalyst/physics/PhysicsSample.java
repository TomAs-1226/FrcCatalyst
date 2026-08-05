package frc.lib.catalyst.physics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * One loop's worth of measurements, gathered at a single instant.
 *
 * <p>This is the input to {@link PhysicsCore#update(PhysicsSample)}, and it exists so that Physics
 * Core has a pure, hardware-free entry point. On a robot you rarely build one by hand — configure
 * the suppliers on the builder and call {@code physics.update()} instead. But because this type
 * carries everything the estimators need, the same code path can be driven from a unit test or
 * replayed from a log with no HAL, no NetworkTables, and no robot.
 *
 * @param timestampSeconds        FPGA-clock time these measurements describe
 * @param pose                    pose from the drivetrain estimator; Physics Core reads it, never writes it
 * @param robotRelativeSpeeds     forward-kinematic chassis velocity, robot-relative
 * @param moduleStates            measured module states in kinematics order, or {@code null} if slip
 *                                scoring is not in use
 * @param robotRelativeAcceleration IMU translational acceleration, robot-relative, in m/s^2 with
 *                                gravity removed, or {@code null} if the robot has no accelerometer
 * @param yawRateRadPerSec        gyro yaw rate, rad/s
 * @since 1.5.0
 */
public record PhysicsSample(
        double timestampSeconds,
        Pose2d pose,
        ChassisSpeeds robotRelativeSpeeds,
        SwerveModuleState[] moduleStates,
        Translation2d robotRelativeAcceleration,
        double yawRateRadPerSec) {

    /** Compact constructor: the pose and chassis speeds are required; the optional inputs may be null. */
    public PhysicsSample {
        if (pose == null) throw new IllegalArgumentException("pose must not be null");
        if (robotRelativeSpeeds == null) {
            throw new IllegalArgumentException("robotRelativeSpeeds must not be null");
        }
    }

    /** True when module states were supplied, so slip scoring can run. */
    public boolean hasModuleStates() {
        return moduleStates != null && moduleStates.length > 0;
    }

    /** True when an accelerometer reading was supplied, so the IMU fusion path is live. */
    public boolean hasAcceleration() {
        return robotRelativeAcceleration != null;
    }

    /**
     * A sample with no module states and no accelerometer — the minimum Physics Core will accept.
     * Confidence tracking and prediction still work; slip and collision detection do not.
     */
    public static PhysicsSample of(double timestampSeconds, Pose2d pose,
                                   ChassisSpeeds robotRelativeSpeeds, double yawRateRadPerSec) {
        return new PhysicsSample(timestampSeconds, pose, robotRelativeSpeeds, null, null, yawRateRadPerSec);
    }
}
