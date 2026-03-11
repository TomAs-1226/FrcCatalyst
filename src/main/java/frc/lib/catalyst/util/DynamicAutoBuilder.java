package frc.lib.catalyst.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Dynamic autonomous path generation using PathPlanner's on-the-fly capabilities.
 *
 * <p>Generates paths at runtime from the robot's current position to target poses.
 * This is the approach used by top teams (254, 3061) for adaptive autonomous routines
 * that respond to game conditions (e.g., piece availability, scoring position).
 *
 * <p>Key features:
 * <ul>
 *   <li>On-the-fly path generation from current pose to any target</li>
 *   <li>Multi-waypoint paths with configurable constraints</li>
 *   <li>Alliance-aware mirroring</li>
 *   <li>Scoring position presets</li>
 * </ul>
 *
 * <p>Requires PathPlanner's AutoBuilder to be configured first (via SwerveSubsystem).
 *
 * <p>Example:
 * <pre>{@code
 * PathConstraints constraints = DynamicAutoBuilder.defaultConstraints(3.0, 2.0);
 *
 * // Simple drive-to-pose
 * Command driveTo = DynamicAutoBuilder.pathfindToPose(
 *     new Pose2d(5.0, 2.0, Rotation2d.fromDegrees(0)), constraints);
 *
 * // Drive to pose then follow a pre-made path
 * Command autoScore = DynamicAutoBuilder.pathfindThenFollowPath("ScorePath", constraints);
 *
 * // Generate path through multiple waypoints
 * Command complexPath = DynamicAutoBuilder.generatePath(
 *     currentPoseSupplier,
 *     List.of(
 *         new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45)),
 *         new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(90))
 *     ),
 *     constraints,
 *     Rotation2d.fromDegrees(90), // end heading
 *     0.0 // end velocity
 * );
 * }</pre>
 */
public final class DynamicAutoBuilder {

    private DynamicAutoBuilder() {}

    /**
     * Create default path constraints.
     *
     * @param maxVelocityMPS max velocity in m/s
     * @param maxAccelerationMPSS max acceleration in m/s^2
     * @return PathConstraints for on-the-fly paths
     */
    public static PathConstraints defaultConstraints(double maxVelocityMPS, double maxAccelerationMPSS) {
        return new PathConstraints(
                maxVelocityMPS,
                maxAccelerationMPSS,
                2 * Math.PI, // max angular velocity (rad/s)
                4 * Math.PI  // max angular acceleration (rad/s^2)
        );
    }

    /**
     * Create path constraints with custom angular limits.
     */
    public static PathConstraints constraints(double maxVelMPS, double maxAccelMPSS,
                                               double maxAngVelRadPerSec, double maxAngAccelRadPerSecSq) {
        return new PathConstraints(maxVelMPS, maxAccelMPSS, maxAngVelRadPerSec, maxAngAccelRadPerSecSq);
    }

    /**
     * Pathfind to a target pose using PathPlanner's pathfinding.
     * Automatically avoids obstacles defined in PathPlanner.
     *
     * @param targetPose the target field pose
     * @param constraints path constraints
     * @return command that pathfinds to the target
     */
    public static Command pathfindToPose(Pose2d targetPose, PathConstraints constraints) {
        try {
            return AutoBuilder.pathfindToPose(targetPose, constraints);
        } catch (Exception e) {
            DriverStation.reportError("DynamicAutoBuilder: Failed to pathfind - " + e.getMessage(), false);
            return Commands.none();
        }
    }

    /**
     * Pathfind to a target pose and stop with zero end velocity.
     *
     * @param targetPose target pose
     * @param constraints path constraints
     * @param endVelocityMPS desired velocity at the target (0 = stop)
     * @return command that pathfinds to the target
     */
    public static Command pathfindToPose(Pose2d targetPose, PathConstraints constraints,
                                          double endVelocityMPS) {
        try {
            return AutoBuilder.pathfindToPose(targetPose, constraints, endVelocityMPS);
        } catch (Exception e) {
            DriverStation.reportError("DynamicAutoBuilder: Failed to pathfind - " + e.getMessage(), false);
            return Commands.none();
        }
    }

    /**
     * Pathfind to a pre-made PathPlanner path and then follow it.
     * Useful for approaching a scoring position with pathfinding,
     * then following a precise pre-made path for the final alignment.
     *
     * @param pathName name of the PathPlanner path file
     * @param constraints pathfinding constraints
     * @return command that pathfinds to and follows the path
     */
    public static Command pathfindThenFollowPath(String pathName, PathConstraints constraints) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
        } catch (Exception e) {
            DriverStation.reportError("DynamicAutoBuilder: Failed to load path '" + pathName
                    + "' - " + e.getMessage(), false);
            return Commands.none();
        }
    }

    /**
     * Follow a pre-made PathPlanner path (no pathfinding, starts immediately).
     *
     * @param pathName name of the PathPlanner path file
     * @return command that follows the path
     */
    public static Command followPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("DynamicAutoBuilder: Failed to load path '" + pathName
                    + "' - " + e.getMessage(), false);
            return Commands.none();
        }
    }

    /**
     * Generate and follow a path through multiple waypoints from the current position.
     * The path is generated at command initialization time using the robot's current pose.
     *
     * @param currentPoseSupplier supplies the robot's current pose
     * @param waypoints list of poses to visit (in order)
     * @param constraints path constraints
     * @param endHeading desired robot heading at the end
     * @param endVelocityMPS desired velocity at the end (0 = stop)
     * @return command that generates and follows the path
     */
    public static Command generatePath(Supplier<Pose2d> currentPoseSupplier,
                                         List<Pose2d> waypoints,
                                         PathConstraints constraints,
                                         Rotation2d endHeading,
                                         double endVelocityMPS) {
        return Commands.defer(() -> {
            try {
                Pose2d currentPose = currentPoseSupplier.get();

                // Build all poses including current position
                List<Pose2d> allPoses = new ArrayList<>();
                allPoses.add(currentPose);
                allPoses.addAll(waypoints);

                // Use PathPlanner's public API for creating waypoints from poses
                List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
                        allPoses.toArray(new Pose2d[0]));

                PathPlannerPath path = new PathPlannerPath(
                        bezierPoints,
                        constraints,
                        null, // ideal starting state (auto-computed)
                        new GoalEndState(endVelocityMPS, endHeading));

                return AutoBuilder.followPath(path);
            } catch (Exception e) {
                DriverStation.reportError(
                        "DynamicAutoBuilder: Failed to generate path - " + e.getMessage(), false);
                return Commands.none();
            }
        }, java.util.Set.of()).withName("DynamicAutoBuilder.GeneratePath");
    }

    /**
     * Mirror a pose for red alliance.
     * Standard FRC field mirroring: X is mirrored, heading is flipped.
     *
     * @param bluePose pose in blue alliance coordinates
     * @return mirrored pose for red alliance
     */
    public static Pose2d mirrorForRed(Pose2d bluePose) {
        return mirrorForRed(bluePose, 16.54);
    }

    /**
     * Mirror a pose for red alliance with custom field length.
     */
    public static Pose2d mirrorForRed(Pose2d bluePose, double fieldLengthMeters) {
        return new Pose2d(
                fieldLengthMeters - bluePose.getX(),
                bluePose.getY(),
                new Rotation2d(Math.PI).minus(bluePose.getRotation()));
    }

    /**
     * Get a pose that is alliance-correct.
     * Returns the blue pose for blue alliance, mirrored for red.
     *
     * @param bluePose the pose in blue alliance coordinates
     * @return alliance-correct pose
     */
    public static Pose2d alliancePose(Pose2d bluePose) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return mirrorForRed(bluePose);
        }
        return bluePose;
    }
}
