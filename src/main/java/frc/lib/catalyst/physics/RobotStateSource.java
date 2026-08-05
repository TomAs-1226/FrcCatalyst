package frc.lib.catalyst.physics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The one thing every Catalyst component needs from "where is the robot": a pose, a field-relative
 * velocity, and the timestamp both were true at.
 *
 * <p>This exists so that consumers — {@code AimingSolver}, {@code Autopilot}, a state-machine guard —
 * can be written against a contract instead of against one concrete drivetrain. A plain
 * {@code SwerveSubsystem} satisfies it (it already tracks all three), and so does
 * {@link PhysicsCore}. Code that takes a {@code RobotStateSource} works with either, and works
 * unchanged if a team later adds Physics Core or takes it away again.
 *
 * <pre>{@code
 * // Without Physics Core:
 * RobotStateSource source = drive;
 *
 * // With it — same call sites, better estimate:
 * RobotStateSource source = physics;
 * }</pre>
 *
 * <p>Implementations that can also say <em>how much</em> to believe the answer should implement
 * {@link UncertainRobotStateSource} instead.
 *
 * @since 1.5.0
 */
public interface RobotStateSource {

    /** Best current estimate of the robot's field-relative pose. */
    Pose2d pose();

    /**
     * Best current estimate of the robot's velocity in <b>field-relative</b> terms — {@code vx} points
     * down the field's +X axis regardless of which way the robot is facing.
     */
    ChassisSpeeds fieldVelocity();

    /**
     * The timestamp, on the same clock as {@code Timer.getFPGATimestamp()}, that {@link #pose()} and
     * {@link #fieldVelocity()} describe. Callers use this to reject stale state and to line the
     * estimate up with a timestamped measurement.
     */
    double timestampSeconds();
}
