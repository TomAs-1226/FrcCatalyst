package frc.lib.catalyst.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

/**
 * Camera source implementation for Limelight cameras.
 * Uses NetworkTables directly (no external LimelightLib dependency required).
 *
 * <p>Supports MegaTag2 pose estimation when robot orientation is set.
 *
 * <p>Example:
 * <pre>{@code
 * LimelightSource cam = new LimelightSource("limelight-front",
 *     new Transform3d(0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-15), 0)));
 * }</pre>
 */
public class LimelightSource implements CameraSource {

    private final String name;
    private final Transform3d robotToCamera;
    private final NetworkTable limelightTable;

    // Robot orientation for MegaTag2
    private double robotYaw = 0;
    private double robotYawRate = 0;
    private double robotPitch = 0;
    private double robotRoll = 0;

    public LimelightSource(String name, Transform3d robotToCamera) {
        this.name = name;
        this.robotToCamera = robotToCamera;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(name);

        // Set camera pose relative to robot
        limelightTable.getEntry("camerapose_robotspace_set").setDoubleArray(new double[]{
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                Math.toDegrees(robotToCamera.getRotation().getX()),
                Math.toDegrees(robotToCamera.getRotation().getY()),
                Math.toDegrees(robotToCamera.getRotation().getZ())
        });
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Optional<PoseEstimate> getEstimatedPose() {
        // Set robot orientation for MegaTag2
        limelightTable.getEntry("robot_orientation_set").setDoubleArray(new double[]{
                robotYaw, robotYawRate, robotPitch, 0, robotRoll, 0
        });

        // Try MegaTag2 first (more reliable with robot orientation)
        double[] botposeBlue = limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);

        if (botposeBlue.length < 7) {
            // Fall back to MegaTag1
            botposeBlue = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        }

        if (botposeBlue.length < 7) {
            return Optional.empty();
        }

        // Check if we have valid tag data
        double tv = limelightTable.getEntry("tv").getDouble(0);
        if (tv < 1) {
            return Optional.empty();
        }

        double x = botposeBlue[0];
        double y = botposeBlue[1];
        double yaw = botposeBlue[5];
        double latency = botposeBlue[6]; // total latency in ms

        // Tag count and average distance from additional fields
        int tagCount = botposeBlue.length > 7 ? (int) botposeBlue[7] : 1;
        double avgDist = botposeBlue.length > 9 ? botposeBlue[9] : 3.0;

        // Timestamp
        double timestampSeconds = (edu.wpi.first.wpilibj.Timer.getFPGATimestamp())
                - (latency / 1000.0);

        Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));

        return Optional.of(new PoseEstimate(
                pose,
                timestampSeconds,
                tagCount,
                avgDist,
                0.0 // Limelight doesn't directly expose ambiguity in same way
        ));
    }

    @Override
    public void setRobotOrientation(double yawDegrees, double yawRate,
                                     double pitchDegrees, double rollDegrees) {
        this.robotYaw = yawDegrees;
        this.robotYawRate = yawRate;
        this.robotPitch = pitchDegrees;
        this.robotRoll = rollDegrees;
    }
}
