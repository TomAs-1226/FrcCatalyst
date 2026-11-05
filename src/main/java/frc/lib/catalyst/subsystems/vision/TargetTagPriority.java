package frc.lib.catalyst.subsystems.vision;

import java.util.Arrays;

/**
 * Which camera the pose should come from while the robot is scoring: one leader camera, solving from
 * the target's own tags, with the others paused.
 *
 * <h2>The failure this exists for</h2>
 *
 * <p>Two cameras that are each steady on their own can still make the pose jump when both are fused.
 * Each solves from different tags, each carries its own small mounting and tag-placement error, and
 * the estimator flips between them. On team 5805's robot at CAMIS 2026 each camera was steady to
 * 1.5-3.5 cm from fix to fix, but the fused pose jumped 30 cm (p90) whenever cameras on different tags
 * were mixed. Right next to the target, the tags on the target are the ones whose placement matters,
 * and a single camera solving from them alone is the steadiest pose there is.
 *
 * <h2>What it does</h2>
 *
 * <ul>
 *   <li><b>{@link Mode#ALL_TAGS}</b> while the robot is not asking for priority: every camera, every tag.
 *   <li><b>{@link Mode#WAITING}</b> while it is asking but no camera has seen a target tag: still every
 *       tag. Narrowing a camera that sees no target tag to target tags would only blind it.
 *   <li><b>{@link Mode#TARGET_TAGS}</b> once a camera has seen a target tag for {@link Config#debounceSeconds}:
 *       the first such camera in the list leads and solves from the target's tags; every other camera
 *       that may lead is paused. List the shooter's camera first and it wins ties. A leader is replaced
 *       only once it has lost the target, never mid-shot because another camera also sees it.
 *   <li><b>{@link Mode#TARGET_SEARCH}</b> once the leader has not seen a target tag for
 *       {@link Config#holdSeconds}: every camera on target tags only, and the first to see one leads.
 *       A handover therefore never mixes other field tags in. After {@link Config#searchSeconds} more
 *       with nothing, back to every tag.
 * </ul>
 *
 * <p>The hold and the search came from the same event's logs. With a 0.75 s hold and an immediate
 * return to every tag, none of 39 leader losses recovered within the hold, and the all-tag windows
 * that followed stepped the pose by up to 108 cm. With 0.3 s and a search on target tags the handovers
 * stay on the target.
 *
 * <p>Pure: the caller says, each loop, which cameras have a target tag in a fresh frame, and reads
 * back each camera's {@link Filter}. {@link LimelightTagPriority} is that wiring for Limelights.
 * Cameras that may not lead (a camera that never faces the target) keep every tag throughout.
 *
 * @since 2.0.0
 */
public final class TargetTagPriority {

    /** What the cameras are doing. */
    public enum Mode {
        /** Not asked for: every camera on every tag. */
        ALL_TAGS,
        /** Asked for, but no camera has seen a target tag yet: every camera on every tag. */
        WAITING,
        /** One camera leads on the target's tags; the others that may lead are paused. */
        TARGET_TAGS,
        /** The leader lost the target: every camera that may lead on target tags, the first to see one leads. */
        TARGET_SEARCH
    }

    /** What one camera should solve from. */
    public enum Filter {
        /** Every field tag. */
        ALL,
        /** The target's tags only. */
        TARGET,
        /** No tag: the camera contributes no pose. */
        NONE
    }

    /** Timing. The defaults are the ones that ran on a robot. */
    public static final class Config {
        double debounceSeconds = 0.1;
        double holdSeconds = 0.3;
        double searchSeconds = 1.0;

        /** How long a target tag must be in view before priority starts, s. Default 0.1. */
        public Config debounceSeconds(double s) {
            debounceSeconds = nonNegative(s, "debounceSeconds");
            return this;
        }

        /** How long a leader keeps its place without seeing the target, s. Default 0.3. */
        public Config holdSeconds(double s) {
            holdSeconds = nonNegative(s, "holdSeconds");
            return this;
        }

        /** How long every camera searches on target tags after the hold, s. Default 1.0. */
        public Config searchSeconds(double s) {
            searchSeconds = nonNegative(s, "searchSeconds");
            return this;
        }

        private static double nonNegative(double s, String name) {
            if (!(s >= 0) || !Double.isFinite(s)) {
                throw new IllegalArgumentException(name + " must be finite and not negative");
            }
            return s;
        }
    }

    private final Config config;
    private final boolean[] mayLead;
    private Mode mode = Mode.ALL_TAGS;
    private int leader = -1;
    private double seeingSince = Double.NaN;
    private double lastSeen = Double.NaN;

    /** @param mayLead one entry per camera, in priority order: whether it may lead (and be paused) */
    public TargetTagPriority(boolean[] mayLead) {
        this(mayLead, new Config());
    }

    public TargetTagPriority(boolean[] mayLead, Config config) {
        this.mayLead = mayLead.clone();
        this.config = config;
    }

    /** {@code cameras} cameras, every one of which may lead. */
    public static TargetTagPriority allMayLead(int cameras) {
        boolean[] all = new boolean[cameras];
        Arrays.fill(all, true);
        return new TargetTagPriority(all);
    }

    /**
     * One loop.
     *
     * @param now        s, any monotonic clock
     * @param wanted     the robot wants priority now (scoring, and the feature is switched on)
     * @param seesTarget per camera, whether its newest fresh frame holds a target tag. A stale frame
     *                   is not a sighting.
     * @return the mode after this loop
     */
    public Mode update(double now, boolean wanted, boolean[] seesTarget) {
        if (seesTarget.length != mayLead.length) {
            throw new IllegalArgumentException("expected " + mayLead.length + " cameras, got " + seesTarget.length);
        }
        int firstSeeing = -1;
        boolean leaderSees = false;
        for (int camera = 0; camera < seesTarget.length; camera++) {
            boolean sees = mayLead[camera] && seesTarget[camera];
            if (sees && firstSeeing < 0) firstSeeing = camera;
            if (sees && camera == leader) leaderSees = true;
        }
        boolean leading = mode == Mode.TARGET_TAGS && leader >= 0;
        // Only the leader's own sightings keep its hold; a paused camera cannot take over mid-shot.
        boolean seeing = leading ? leaderSees : firstSeeing >= 0;
        if (!seeing) {
            seeingSince = Double.NaN;
        } else if (Double.isNaN(seeingSince)) {
            seeingSince = now;
        }
        boolean rising = seeing && now - seeingSince >= config.debounceSeconds;
        if (seeing) lastSeen = now;
        double sinceSeen = Double.isNaN(lastSeen) ? Double.POSITIVE_INFINITY : now - lastSeen;
        boolean held = leading && sinceSeen <= config.holdSeconds;
        boolean searching = (leading || mode == Mode.TARGET_SEARCH)
                && sinceSeen <= config.holdSeconds + config.searchSeconds;

        Mode previous = mode;
        if (!wanted) {
            mode = Mode.ALL_TAGS;
        } else if (held) {
            mode = Mode.TARGET_TAGS;
        } else if (rising && firstSeeing >= 0) {
            mode = Mode.TARGET_TAGS;
            if (!leading) leader = firstSeeing;   // a new leader only when none holds
        } else if (searching) {
            mode = Mode.TARGET_SEARCH;
        } else {
            mode = Mode.WAITING;
        }
        if (mode != Mode.TARGET_TAGS) {
            leader = -1;
        } else if (leader < 0 || previous != Mode.TARGET_TAGS && firstSeeing >= 0) {
            leader = firstSeeing >= 0 ? firstSeeing : leader;
        }
        return mode;
    }

    public Mode mode() {
        return mode;
    }

    /** The leading camera's index, or -1 outside {@link Mode#TARGET_TAGS}. */
    public int leader() {
        return leader;
    }

    public int cameraCount() {
        return mayLead.length;
    }

    /** What camera {@code camera} should solve from now. */
    public Filter filterFor(int camera) {
        if (!mayLead[camera] || mode == Mode.ALL_TAGS || mode == Mode.WAITING) return Filter.ALL;
        if (mode == Mode.TARGET_SEARCH) return Filter.TARGET;
        return camera == leader ? Filter.TARGET : Filter.NONE;
    }
}
