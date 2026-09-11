package frc.lib.catalyst.subsystems.vision;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.networktables.TimestampedDouble;
import org.wpilib.networktables.TimestampedDoubleArray;
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
 *   <li><b>No batch.</b> Only the newest frame is delivered each loop; estimates the camera
 *       produced between robot loops are dropped rather than handed over together. They are at
 *       least no longer mistaken for new ones - see below.
 *   <li><b>Standard deviations are not read.</b> They exist as a {@code stddevs} topic but their
 *       layout is not pinned here, and guessing at it would produce confident wrong weights.
 * </ul>
 *
 * <h2>Frame identity, and why it is not optional</h2>
 *
 * <p>NetworkTables holds the last value written until it is overwritten, so re-reading a topic
 * returns the same frame again. The robot loop runs at 50 Hz and an AprilTag pipeline does not, so
 * the same {@code botpose} array is read several times per unique frame. Stamped with the clock at
 * read time - which is what this class did - each re-read looks like a fresh, independent
 * measurement, and the pose estimator weights it as one. The fused pose ends up several times more
 * confident in vision than the evidence supports, on every camera at once.
 *
 * <p>The same flaw made staleness undetectable. VisionSubsystem computes a frame's age as
 * {@code now - timestamp}; with the timestamp built as {@code now - latency}, that expression
 * reduces to the camera's own self-reported latency, a small constant, no matter how old the data
 * really is. Its {@code StaleData} gate could not fire on this path at all. A camera that keeps its
 * NetworkTables session while its pipeline stalls leaves its last pose sitting in the table, and
 * Catalyst would have re-served it forever, always stamped "now", dragging the fused pose back to
 * wherever the camera froze.
 *
 * <p>Both are fixed by taking the value and its NetworkTables publish time together, atomically,
 * and refusing to hand over a frame whose timestamp has not advanced. That timestamp is also what
 * the estimate is stamped from, so age is now a real measurement and the staleness gate works.
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

    /** Never-published sentinel. NetworkTables stamps a real value with a non-zero time. */
    private static final long NEVER = 0L;

    /** A heartbeat older than this means the camera has stopped, whatever its last value says. */
    private static final double ALIVE_SECONDS = 1.0;

    private final String name;
    private final NetworkTable table;

    /**
     * Subscribers rather than entries, because only a subscriber can hand back the value and its
     * publish time in one call. Reading them separately would let a frame land between the two and
     * pair one read timestamp with the other read array.
     *
     * <p>No duplicate-handling option is set. Frame identity is decided by the publish timestamp
     * rather than by comparing values, so whether NetworkTables collapses an identical republish is
     * not something this has to care about - and a camera that genuinely re-publishes the same pose
     * has produced a new frame, whatever it contains.
     */
    private final DoubleArraySubscriber megaTag2;
    private final DoubleArraySubscriber megaTag1;

    /**
     * {@code hb} counts frames and {@code hw} is {@code [fps, cpu temperature C, ram %, temp]}, both
     * published by every Limelight OS since 2024. The heartbeat is the liveness signal: a camera
     * whose session is up but whose pipeline has died stops advancing it, and a camera that is
     * unplugged leaves its last value in the table forever. Either way its publish time stops moving,
     * which is what {@link #secondsSinceHeartbeat()} measures.
     */
    private final DoubleSubscriber heartbeat;
    private final DoubleArraySubscriber hardware;
    /**
     * The same two poses with the field's centre as the origin - what the camera actually solves.
     * {@code botpose_wpiblue} is derived from it by adding half the field, so a tag the camera can
     * see but cannot place (not in its field map, or MegaTag2 with no heading) is all zeros here
     * and lands exactly on the field's centre there, with {@code tv} = 1 and a tag counted. Seen
     * on four Limelight 4s at once: a robot on a bench, reported at (8.27, 4.03, 0) to the
     * millimetre. Only the centre-origin array can tell that apart from a real fix.
     */
    private final DoubleArraySubscriber megaTag2Centre;
    private final DoubleArraySubscriber megaTag1Centre;

    /** Publish time of the newest frame already handed over, in NetworkTables microseconds. */
    private long lastConsumedNt = NEVER;
    /**
     * Per-source frame markers. A single shared one meant that examining - and rejecting - a
     * MegaTag2 frame also marked the MegaTag1 frame published alongside it as already consumed, so
     * the fallback below could never fire even once.
     */
    private long lastMegaTag2Nt = NEVER;
    private long lastMegaTag1Nt = NEVER;
    /**
     * The camera saw a tag and could not place it on the field: not in its map, or no MegaTag fix.
     * Distinct from seeing nothing, and the two look identical from outside without this.
     */
    private boolean unplaceable;

    LegacyLimelightReader(String name, Transform3d robotToCamera) {
        this.name = name;
        this.table = NetworkTableInstance.getDefault().getTable(name);
        this.megaTag2 = table.getDoubleArrayTopic("botpose_orb_wpiblue")
                .subscribe(new double[0]);
        this.megaTag1 = table.getDoubleArrayTopic("botpose_wpiblue")
                .subscribe(new double[0]);
        this.heartbeat = table.getDoubleTopic("hb").subscribe(0.0);
        this.hardware = table.getDoubleArrayTopic("hw").subscribe(new double[0]);
        this.megaTag2Centre = table.getDoubleArrayTopic("botpose_orb").subscribe(new double[0]);
        this.megaTag1Centre = table.getDoubleArrayTopic("botpose").subscribe(new double[0]);

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
    /**
     * The camera is seeing a tag it cannot place on the field.
     *
     * <p>Worth reporting separately from "sees nothing", because the two are indistinguishable from
     * the outside and have completely different fixes. Seeing nothing means point the camera at a
     * tag. This means the tag is not in the camera's field map, or the tag is one no map contains -
     * id 0, say - or MegaTag has no fix for it. The camera looks perfectly healthy either way:
     * every per-tag 3D solve is correct, {@code tv} is 1, and a tag is counted.
     */
    boolean isUnplaceable() {
        return unplaceable;
    }

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
     * <p>MegaTag2 first when asked for, MegaTag1 when MegaTag2 has no answer - and "no answer" means
     * more than "the topic is missing". A Limelight that cannot solve MegaTag2 still publishes a
     * full-length {@code botpose_orb_wpiblue}: six zeros with half a field added, which is the exact
     * centre of the field and passes every length and finite check there is. Preferring it because
     * it arrived, then rejecting it as unplaceable, threw away a MegaTag1 fix that was sitting in the
     * same frame.
     *
     * <p>Measured on a Limelight 4, firmware reporting {@code pipe_fiducial}, with a mapped tag in
     * view across 1,540 frames in four configurations - no heading fed, heading fed at 50 Hz,
     * internal IMU, internal IMU seeded. MegaTag1 solved in every frame of all four; MegaTag2 solved
     * in none of them. A robot that only ever reads MegaTag2 on that camera is blind with a working
     * camera, and the only symptom is vision quietly never contributing.
     *
     * @param useMegaTag2 try the MegaTag2 array first, falling back to MegaTag1 when it has no fix
     */
    Optional<CameraSource.PoseEstimate> read(boolean useMegaTag2) {
        if (useMegaTag2) {
            Optional<CameraSource.PoseEstimate> fix =
                    consider(megaTag2.getAtomic(), megaTag2Centre.get(), true);
            if (fix.isPresent()) {
                return fix;
            }
        }
        return consider(megaTag1.getAtomic(), megaTag1Centre.get(), false);
    }

    /**
     * Judge one of the two pose arrays.
     *
     * @param frame        the array and the time it was published
     * @param centre       the same solve with the field centre as the origin, for the unplaceable test
     * @param fromMegaTag2 which array this is, for the per-source de-duplication
     */
    private Optional<CameraSource.PoseEstimate> consider(
            TimestampedDoubleArray frame, double[] centre, boolean fromMegaTag2) {
        if (frame == null || frame.value.length < MIN_LENGTH || frame.timestamp == NEVER) {
            return Optional.empty();
        }

        // Nothing new since the last hand-over. Not an error and not a fault - the camera simply has
        // not produced a frame since this was last asked, which at 50 Hz against a slower pipeline
        // is most loops. Serving the old one again would be inventing evidence.
        //
        // Tracked per source. One shared marker meant that rejecting a MegaTag2 frame also marked
        // the MegaTag1 frame beside it as consumed, so the fallback could never be read.
        long last = fromMegaTag2 ? lastMegaTag2Nt : lastMegaTag1Nt;
        if (frame.timestamp <= last) {
            return Optional.empty();
        }

        // Consumed regardless of what happens below. A frame rejected for tv=0 or a bad pose is not
        // new evidence next loop either, and re-examining it forever would be its own bug.
        if (fromMegaTag2) {
            lastMegaTag2Nt = frame.timestamp;
        } else {
            lastMegaTag1Nt = frame.timestamp;
        }
        lastConsumedNt = Math.max(lastConsumedNt, frame.timestamp);

        double[] botpose = frame.value;
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

        // The other way a camera says "I could not place this": the centre-origin solve is zero and
        // the blue-origin copy of it is therefore the exact centre of the field. tv is 1 and a tag is
        // counted, because the tag was seen - it just is not in the camera's map, or MegaTag2 has no
        // fix for it. Not every camera publishes the centre-origin key; when it is absent this cannot
        // tell and the estimate stands on the other checks.
        if (centre.length > I_YAW && centre[I_X] == 0.0 && centre[I_Y] == 0.0 && centre[I_YAW] == 0.0) {
            unplaceable = true;
            return Optional.empty();
        }
        unplaceable = false;

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
            unplaceable = true;
            return Optional.empty();
        }

        return Optional.of(new CameraSource.PoseEstimate(
                new Pose2d(x, y, Rotation2d.fromDegrees(yaw)),
                captureTime(frame.timestamp, latencyMs),
                tagCount,
                Double.isFinite(avgDist) ? avgDist : 0.0,
                // This API exposes no per-estimate ambiguity, and unlike the modern path there is no
                // camera-side gate that has already applied one. Zero here means "not reported",
                // not "checked and fine" - which is exactly why the modern path is preferred.
                0.0));
    }

    /**
     * When the frame was captured, on the robot's clock.
     *
     * <p>Two subtractions from now: how long ago NetworkTables received the value, and the pipeline
     * latency the camera reports on top of that. Written as a difference of two NetworkTables times
     * rather than by converting one directly, so it holds whatever the two clocks' origins are.
     */
    private static double captureTime(long ntPublishMicros, double latencyMs) {
        double ageSeconds = (NetworkTablesJNI.now() - ntPublishMicros) / 1_000_000.0;
        return Timer.getTimestamp() - ageSeconds - (latencyMs / 1000.0);
    }

    /**
     * How long since this camera published a new pose, or empty if it never has.
     *
     * <p>The signal the per-key path could not previously produce. LimelightLib calls the equivalent
     * state {@code STALE} - a camera holding its NetworkTables connection while its data stops
     * advancing - but that detection routes through the modern API, which no shipping camera speaks,
     * so on this path it can never fire. This is the honest substitute.
     */
    java.util.OptionalDouble secondsSinceLastFrame() {
        if (lastConsumedNt == NEVER) {
            return java.util.OptionalDouble.empty();
        }
        return java.util.OptionalDouble.of(
                (NetworkTablesJNI.now() - lastConsumedNt) / 1_000_000.0);
    }

    /**
     * How long since the camera's heartbeat last advanced, or empty if it never has.
     *
     * <p>Measured from the NetworkTables publish time rather than by comparing values, so a camera
     * that republishes the same count still reads as alive and one that stops publishing reads as
     * dead even though its last value is still sitting in the table.
     */
    java.util.OptionalDouble secondsSinceHeartbeat() {
        TimestampedDouble hb = heartbeat.getAtomic();
        if (hb.timestamp == NEVER) {
            return java.util.OptionalDouble.empty();
        }
        return java.util.OptionalDouble.of((NetworkTablesJNI.now() - hb.timestamp) / 1_000_000.0);
    }

    /** Whether the heartbeat advanced within the last second. */
    boolean isAlive() {
        java.util.OptionalDouble age = secondsSinceHeartbeat();
        return age.isPresent() && age.getAsDouble() < ALIVE_SECONDS;
    }

    /** The camera's reported frame rate, or empty if it has not published {@code hw}. */
    java.util.OptionalDouble fps() {
        return hardwareField(0);
    }

    /** The camera's reported CPU temperature in C, or empty. Limelight 4s throttle from ~85 C. */
    java.util.OptionalDouble cpuTemperatureC() {
        return hardwareField(1);
    }

    private java.util.OptionalDouble hardwareField(int index) {
        double[] hw = hardware.get();
        if (hw == null || hw.length <= index || !Double.isFinite(hw[index])) {
            return java.util.OptionalDouble.empty();
        }
        return java.util.OptionalDouble.of(hw[index]);
    }

    /** The camera's name, for messages. */
    String cameraName() {
        return name;
    }
}
