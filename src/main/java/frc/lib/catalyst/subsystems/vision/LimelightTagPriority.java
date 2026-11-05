package frc.lib.catalyst.subsystems.vision;

import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTablesJNI;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

/**
 * {@link TargetTagPriority} for Limelights: reads each camera's {@code rawfiducials} and writes its
 * {@code fiducial_id_filters_set}.
 *
 * <pre>{@code
 * // Shooter camera first: it leads whenever it can see the target.
 * LimelightTagPriority priority = new LimelightTagPriority(
 *         List.of("limelight-shooter", "limelight-left", "limelight-right"),
 *         Set.of("limelight-shooter", "limelight-left", "limelight-right"),
 *         fieldLayout.getTags().stream().mapToInt(t -> t.ID).toArray());
 *
 * // Every loop:
 * priority.update(scoring, isRed ? RED_GOAL_TAGS : BLUE_GOAL_TAGS);
 * }</pre>
 *
 * <p>A frame counts as a sighting only while it is at most {@value #FRESH_FRAME_SECONDS} s old by its
 * NetworkTables timestamp: a camera that drops off leaves its last array on the server, and that is
 * not evidence it still sees anything. The filters are written only when what they should say
 * changes; the value stays on the NetworkTables server, so a camera that reboots picks it up again.
 * A paused camera gets the filter {@code [0]}, which no FRC field tag matches.
 *
 * <p>Nothing else in Catalyst writes {@code fiducial_id_filters_set}. A filter configured in the
 * Limelight's own web interface is overridden while this runs.
 *
 * <h2>Keys</h2>
 *
 * <pre>
 *   /Catalyst/Vision/TagPriority/Mode            string   ALL_TAGS, WAITING, TARGET_TAGS, TARGET_SEARCH
 *   /Catalyst/Vision/TagPriority/Leader          double   the leading camera's index, or -1
 *   /Catalyst/Vision/TagPriority/LeaderName      string
 *   /Catalyst/Vision/TagPriority/VisibleTagIds   double[] target tags in any camera's fresh raw frame
 * </pre>
 *
 * @since 2.0.0
 */
public final class LimelightTagPriority {

    /** A raw frame older than this is not a sighting, s. */
    public static final double FRESH_FRAME_SECONDS = 0.25;
    /** Limelight's {@code rawfiducials}: seven numbers per tag, the id first. */
    static final int RAW_FIDUCIAL_STRIDE = 7;
    /** A filter no field tag matches. */
    static final double[] NO_TAGS = {0};
    private static final String KEY = "Vision/TagPriority/";

    private final TargetTagPriority core;
    private final List<String> names;
    private final List<DoubleArrayPublisher> filters = new ArrayList<>();
    private final List<DoubleArraySubscriber> raw = new ArrayList<>();
    private final double[] allTags;
    private double[][] written;

    /**
     * @param cameraNames     every camera's NetworkTables name, in priority order
     * @param mayLead         the cameras that take part; the rest keep every tag throughout
     * @param allTags         every tag on the field, for the unfiltered state
     */
    public LimelightTagPriority(List<String> cameraNames, Set<String> mayLead, int[] allTags) {
        this(cameraNames, mayLead, allTags, new TargetTagPriority.Config());
    }

    public LimelightTagPriority(List<String> cameraNames, Set<String> mayLead, int[] allTags,
                                TargetTagPriority.Config config) {
        this.names = List.copyOf(cameraNames);
        this.allTags = Arrays.stream(allTags).sorted().distinct().mapToDouble(i -> i).toArray();
        boolean[] leads = new boolean[names.size()];
        for (int i = 0; i < names.size(); i++) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(names.get(i));
            filters.add(table.getDoubleArrayTopic("fiducial_id_filters_set").publish());
            raw.add(table.getDoubleArrayTopic("rawfiducials").subscribe(new double[0]));
            leads[i] = mayLead.contains(names.get(i));
        }
        this.core = new TargetTagPriority(leads, config);
    }

    /**
     * One loop.
     *
     * @param wanted     the robot wants priority now
     * @param targetTags the tags on the target being scored on, for this alliance
     */
    public TargetTagPriority.Mode update(boolean wanted, Set<Integer> targetTags) {
        long nowNanos = NetworkTablesJNI.now();
        boolean[] sees = new boolean[raw.size()];
        Set<Integer> visible = new TreeSet<>();
        for (int camera = 0; camera < raw.size(); camera++) {
            var frame = raw.get(camera).getAtomic();
            if (freshFrame(nowNanos, frame.timestamp)) {
                sees[camera] = seesAny(frame.value, targetTags);
                for (double id : visibleTagIds(frame.value, targetTags)) visible.add((int) id);
            }
        }
        TargetTagPriority.Mode mode = core.update(nowNanos / 1e9, wanted, sees);
        write(targetTags);

        int leader = core.leader();
        CatalystLog.log(KEY + "Mode", mode.name());
        CatalystLog.log(KEY + "Leader", (double) leader);
        CatalystLog.log(KEY + "LeaderName", leader >= 0 ? names.get(leader) : "");
        CatalystLog.log(KEY + "VisibleTagIds", toArray(visible));
        return mode;
    }

    public TargetTagPriority.Mode mode() {
        return core.mode();
    }

    /** The leading camera's index in the constructor's list, or -1. */
    public int leader() {
        return core.leader();
    }

    private void write(Set<Integer> targetTags) {
        double[] target = toArray(targetTags);
        double[][] next = new double[filters.size()][];
        for (int camera = 0; camera < filters.size(); camera++) {
            next[camera] = switch (core.filterFor(camera)) {
                case ALL -> allTags;
                case TARGET -> target;
                case NONE -> NO_TAGS;
            };
        }
        if (Arrays.deepEquals(next, written)) return;
        written = next;
        for (int camera = 0; camera < filters.size(); camera++) filters.get(camera).set(next[camera]);
    }

    /**
     * In WPILib 2027 NetworkTables timestamps and {@code now()} share the local <em>nanosecond</em>
     * clock (2026 used microseconds; read as microseconds every frame would look fresh for 250 s).
     */
    static boolean freshFrame(long nowNanos, long timestampNanos) {
        double age = (nowNanos - timestampNanos) / 1e9;
        return timestampNanos > 0 && age >= 0 && age <= FRESH_FRAME_SECONDS;
    }

    /** Whether a {@code rawfiducials} array holds one of {@code tags}. */
    static boolean seesAny(double[] rawFiducials, Set<Integer> tags) {
        return visibleTagIds(rawFiducials, tags).length > 0;
    }

    /** The distinct ids in complete records that are in {@code tags}, ascending. */
    static double[] visibleTagIds(double[] rawFiducials, Set<Integer> tags) {
        Set<Integer> visible = new TreeSet<>();
        for (int i = 0; i + RAW_FIDUCIAL_STRIDE <= rawFiducials.length; i += RAW_FIDUCIAL_STRIDE) {
            double id = rawFiducials[i];
            if (Double.isFinite(id) && id == Math.rint(id) && tags.contains((int) id)) visible.add((int) id);
        }
        return toArray(visible);
    }

    static double[] toArray(Set<Integer> tags) {
        return tags.stream().sorted().mapToDouble(Integer::doubleValue).toArray();
    }
}
