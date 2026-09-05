package frc.lib.catalyst.subsystems.vision;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

/**
 * Where {@link VisionSubsystem} sends the poses it accepts, and where it reads the robot's current
 * pose and motion from.
 *
 * <h2>Why this is an interface</h2>
 *
 * <p>Vision fusion used to require a Catalyst {@code SwerveSubsystem}, which meant a team on a tank
 * drive, a team with its own swerve code, or anyone with four cameras on a bench and no drivetrain at
 * all got no fusion, no per-camera telemetry and no health reporting - {@code periodic()} returned on
 * its first line. The three things fusion actually needs from a drivetrain are the three methods
 * here, and {@code SwerveSubsystem} already had all of them, so it implements this and nothing about
 * it changed. Anything else can implement it too. {@link StandaloneVisionPose} is the no-drivetrain
 * case.
 *
 * <h2>hasPose()</h2>
 *
 * <p>Two of the filter's gates compare a camera's estimate against the current pose: too far from
 * it, or heading too different. A sink that has not been given a pose yet - a standalone estimator
 * before its first measurement - has nothing to compare against, and comparing against a default
 * origin would reject every real estimate forever. A sink answers {@code false} here until its pose
 * means something, and those two gates stand down until it does. A drivetrain's odometry always
 * means something, so the default is {@code true}.
 *
 * <h2>resetPose()</h2>
 *
 * <p>A drivetrain's odometry always means <em>something</em>, but at boot it means "the origin",
 * which is wherever the robot was switched on, and the too-far gate measured against that rejected
 * every real estimate on a robot that had been carried onto the field. So {@link VisionSubsystem}
 * now <em>seeds</em>: the first good estimate replaces the pose outright through this method, and
 * only after that do the gates compare against it. It also re-anchors the same way when every
 * camera has disagreed with the pose for a while. The default is a measurement the estimator cannot
 * argue with; a sink that can reset properly overrides it, as {@code SwerveSubsystem} does.
 *
 * @since 2.0.0
 */
public interface VisionPoseSink {

    /** The robot's current best estimate of its own pose, field-relative. */
    Pose2d getPose();

    /** The robot's current velocity, robot-relative. Used to reject estimates taken while blurred. */
    ChassisVelocities getChassisSpeeds();

    /**
     * Fuse an accepted vision estimate.
     *
     * @param visionPose       field-relative pose the camera measured
     * @param timestampSeconds when the frame was captured, on the robot's clock
     * @param stdDevs          measurement standard deviations [x, y, theta]
     */
    void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs);

    /** Whether {@link #getPose()} is a real estimate rather than a placeholder. See the class note. */
    default boolean hasPose() {
        return true;
    }

    /**
     * Replace the pose with a vision estimate outright - seeding at boot, or re-anchoring after the
     * pose has gone wrong. See the class note.
     *
     * @param pose             field-relative pose the camera measured
     * @param timestampSeconds when the frame was captured, on the robot's clock
     */
    default void resetPose(Pose2d pose, double timestampSeconds) {
        addVisionMeasurement(pose, timestampSeconds, VecBuilder.fill(0.001, 0.001, 0.001));
    }
}
