package frc.lib.catalyst.subsystems.swerve;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Bend a PathPlanner path toward a live target <em>without</em> leaving it —
 * the modern answer to "the plan says go here, but I can see the goal/piece
 * slightly off the planned point."
 *
 * <h2>Why this exists</h2>
 *
 * <p>A PathPlanner path is a time-parameterized plan: at time <i>t</i> the
 * robot should be at a specific (x, y, θ) moving at a specific velocity. The
 * controller splits its output into <b>feedforward</b> (the planned velocity —
 * the gross motion) and <b>feedback</b> (a PID correction that nudges the robot
 * back to the planned point). PathPlanner removed automatic replanning in 2025
 * because no general "robot went off course" recovery satisfies everyone.
 *
 * <p>Instead, PathPlanner 2026 lets you <b>override the feedback</b> while
 * keeping the feedforward. So you keep the path's momentum and route, but bend
 * the correction toward what you actually see:
 *
 * <ul>
 *   <li>{@link #facing(Supplier, Command)} / {@link #facingPoint} — override
 *       the rotation target so the robot <b>faces a goal while driving the
 *       path's XY</b>. This is the clean, safe one — shoot-on-the-move without
 *       leaving the route. PathPlanner still does its own rotation feedback,
 *       just toward your target.</li>
 *   <li>{@link #nudgingXY} — override the X/Y feedback to bias the robot toward
 *       a detected target. ⚠️ This genuinely deviates the position and
 *       <b>replaces</b> PathPlanner's own XY correction, so when the target is
 *       lost the robot follows pure feedforward (it can drift). Use it only
 *       while a target is actually visible.</li>
 * </ul>
 *
 * <p>For a hard chase that fully leaves the path (drive onto a scattered game
 * piece), don't try to blend it in — run a discrete reactive action
 * ({@code SwerveSubsystem.driveToPiece}) and then
 * {@code SwerveSubsystem.pathfindThenFollowPath(...)} to rejoin the planned
 * route from wherever you ended up. See {@code docs/advanced/autonomous.md}.
 *
 * <p>Every wrapper here sets its override when the command starts and
 * <b>clears it when the command ends or is interrupted</b>, so a stale
 * override can never leak into the next path.
 */
public final class PathCorrection {

    private static final Supplier<Optional<Rotation2d>> NO_ROTATION_OVERRIDE = Optional::empty;

    private PathCorrection() {}

    /**
     * Wrap a path-follow command so the robot faces {@code heading} instead of
     * the path's planned rotation, while still following the planned XY.
     * Return {@code Optional.empty()} from the supplier on any loop to fall
     * back to the planned heading. Override is cleared when the command ends.
     *
     * @param heading desired robot heading supplier (empty = use planned)
     * @param follow  the path-follow command (e.g. {@code swerve.followPath("X")})
     */
    public static Command facing(Supplier<Optional<Rotation2d>> heading, Command follow) {
        return follow
                .beforeStarting(() -> PPHolonomicDriveController.setRotationTargetOverride(heading))
                .finallyDo(interrupted ->
                        PPHolonomicDriveController.setRotationTargetOverride(NO_ROTATION_OVERRIDE))
                .withName("PathCorrection.facing(" + follow.getName() + ")");
    }

    /**
     * Convenience: face a fixed field point (a goal) while following a path —
     * the shoot-on-the-move pattern. Computes the heading from the live robot
     * pose to {@code goal} each loop.
     *
     * @param goal      field point to keep the front (or bore) pointed at
     * @param robotPose live robot pose supplier
     * @param follow    the path-follow command
     */
    public static Command facingPoint(Translation2d goal, Supplier<Pose2d> robotPose, Command follow) {
        return facing(() -> {
            Pose2d p = robotPose.get();
            Translation2d v = goal.minus(p.getTranslation());
            if (v.getNorm() < 1e-6) return Optional.empty();
            return Optional.of(new Rotation2d(Math.atan2(v.getY(), v.getX())));
        }, follow);
    }

    /**
     * Wrap a path-follow command so an X/Y velocity correction bends the robot
     * toward a target while keeping the path's feedforward. ⚠️ Replaces
     * PathPlanner's own XY feedback — supply 0 when no target so you fall back
     * to pure feedforward (and clear quickly). Override is cleared on end.
     *
     * @param xCorrection velocity correction in m/s on field X
     * @param yCorrection velocity correction in m/s on field Y
     * @param follow      the path-follow command
     */
    public static Command nudgingXY(DoubleSupplier xCorrection, DoubleSupplier yCorrection, Command follow) {
        return follow
                .beforeStarting(() -> PPHolonomicDriveController.overrideXYFeedback(xCorrection, yCorrection))
                .finallyDo(interrupted -> PPHolonomicDriveController.clearXYFeedbackOverride())
                .withName("PathCorrection.nudgingXY(" + follow.getName() + ")");
    }

    /** Clear every active path correction override. A safety reset. */
    public static void clearAll() {
        PPHolonomicDriveController.clearFeedbackOverrides();
        PPHolonomicDriveController.setRotationTargetOverride(NO_ROTATION_OVERRIDE);
    }
}
