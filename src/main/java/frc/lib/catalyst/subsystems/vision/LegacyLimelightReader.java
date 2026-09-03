package frc.lib.catalyst.subsystems.vision;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;

import java.util.Optional;

/**
 * Reads a Limelight over the classic per-key NetworkTables API.
 *
 * <h2>Why this exists again</h2>
 *
 * <p>Catalyst read these keys directly until 2027, then moved to LimelightLib 2 because Limelight OS
 * 2027 publishes results as a single {@code results_msgpack} topic and turns the per-key API off by
 * default. That was the right move and it is still the preferred path.
 *
 * <p>It is not, however, a path any shipping camera can take today. Measured on two Limelight 4s:
 * the newest camera image Limelight publishes is <b>2026.1</b>, there is no 2027 image for any
 * camera — LL2, LL3, LL3G, LL3A or LL4 — and a 2026 camera publishes fifty per-key topics and no
 * {@code results_msgpack}. LimelightLib subscribes to exactly that one topic, so against every
 * camera a team can actually buy and flash right now, it reads nothing at all: status
 * {@code NO_DATA}, empty queue, no pose, no error. The 2027 line is Limelight Systemcore's.
 *
 * <p>So Catalyst reads whichever API the camera in front of it actually speaks. This class is the
 * older half. {@link LimelightSource} chooses between them and says which it chose.
 *
 * <h2>What is lost on this path</h2>
 *
 * <p>Worth knowing, because it is the reason the modern path is still preferred:
 *
 * <ul>
 *   <li><b>No camera-side rejection.</b> LimelightLib's accepted-estimate queue applies the camera's
 *       own ambiguity, distance and field-bounds gates. Here every estimate the camera publishes is
 *       taken at face value, so a pose estimator downstream needs its own sanity limits.
 *   <li><b>No queue.</b> The per-key API holds only the newest value, so estimates produced between
 *       robot loops are lost rather than delivered as a batch.
 *   <li><b>Standard deviations are not read.</b> They exist as a {@code stddevs} topic but their
 *       layout is not pinned here, and guessing at it would produce confident wrong weights.
 * </ul>
 *
 * @since 2.0.0
 */
final class LegacyLimelightReader {

    /**
     * Layout of the {@code botpose_*} arrays, which is why the indices below are named.
     *
     * <p>{@code [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgTagDist, avgTagArea]}.
     * An array shorter than {@link #MIN_LENGTH} is not a partial reading to salvage, it is a topic
     * that has never been written.
     */
    private static final int I_X = 0, I_Y = 1, I_YAW = 5, I_LATENCY_MS = 6;
    private static final int I_TAG_COUNT = 7, I_AVG_DIST = 9;
    private static final int MIN_LENGTH = 7;

    private final String name;
    private final NetworkTable table;

    LegacyLimelightReader(String name, Transform3d robotToCamera) {
        this.name = name;
        this.table = NetworkTableInstance.getDefault().getTable(name);

        table.getEntry("camerapose_robotspace_set").setDoubleArray(new double[] {
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                Math.toDegrees(robotToCamera.getRotation().getX()),
                Math.toDegrees(robotToCamera.getRotation().getY()),
                Math.toDegrees(robotToCamera.getRotation().getZ())
        });
    }

    /** Whether this camera is publishing the per-key API at all. */
    boolean isPublishing() {
        return table.getEntry("tv").exists();
    }

    /** Whether the camera currently reports a valid target. */
    boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) >= 1.0;
    }

    /**
     * Push the robot's yaw, which MegaTag2 needs to resolve tag ambiguity.
     *
     * <p>Written straight through rather than cached and written on read, which is what the
     * pre-2027 version did. Caching meant the camera only learned the robot's heading on loops where
     * somebody happened to ask for a pose — so a robot that read vision every other loop fed the
     * camera a heading at half the rate it thought it did.
     */
    void setRobotOrientation(double yawDegrees, double yawRate,
                             double pitchDegrees, double rollDegrees) {
        table.getEntry("robot_orientation_set").setDoubleArray(new double[] {
                yawDegrees, yawRate, pitchDegrees, 0, rollDegrees, 0
        });
    }

    /**
     * The newest pose the camera is reporting, or empty.
     *
     * @param useMegaTag2 read the MegaTag2 array, falling back to MegaTag1 if it is absent
     */
    Optional<CameraSource.PoseEstimate> read(boolean useMegaTag2) {
        double[] botpose = new double[0];
        if (useMegaTag2) {
            botpose = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
        }
        if (botpose.length < MIN_LENGTH) {
            botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        }
        if (botpose.length < MIN_LENGTH) {
            return Optional.empty();
        }
        if (!hasTarget()) {
            return Optional.empty();
        }

        double x = botpose[I_X];
        double y = botpose[I_Y];
        double yaw = botpose[I_YAW];
        double latencyMs = botpose[I_LATENCY_MS];

        // A camera with nothing in view publishes an all-zero array rather than clearing the topic,
        // so "the origin, right now" is what no-detection looks like on this API. Treating it as a
        // real measurement teleports a pose estimator to the blue-alliance corner.
        if (x == 0.0 && y == 0.0 && yaw == 0.0) {
            return Optional.empty();
        }

        // Every guard here is a comparison, and NaN passes all of them. A non-finite pose reaching a
        // pose estimator poisons the fused pose permanently, which is worse than no vision at all.
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(yaw)
                || !Double.isFinite(latencyMs) || latencyMs < 0) {
            return Optional.empty();
        }

        int tagCount = botpose.length > I_TAG_COUNT ? (int) botpose[I_TAG_COUNT] : 1;
        double avgDist = botpose.length > I_AVG_DIST ? botpose[I_AVG_DIST] : Double.NaN;
        if (tagCount <= 0) {
            // The camera saw something it could not place on the field. MegaTag has no answer, and
            // the array it published is whatever it computed from nothing.
            return Optional.empty();
        }

        return Optional.of(new CameraSource.PoseEstimate(
                new Pose2d(x, y, Rotation2d.fromDegrees(yaw)),
                Timer.getTimestamp() - (latencyMs / 1000.0),
                tagCount,
                Double.isFinite(avgDist) ? avgDist : 0.0,
                // This API exposes no per-estimate ambiguity, and unlike the modern path there is no
                // camera-side gate that has already applied one. Zero here means "not reported",
                // not "checked and fine" - which is exactly why the modern path is preferred.
                0.0));
    }

    /** The camera's name, for messages. */
    String cameraName() {
        return name;
    }
}
