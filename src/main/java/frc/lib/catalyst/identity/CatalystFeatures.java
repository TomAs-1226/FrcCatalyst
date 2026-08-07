package frc.lib.catalyst.identity;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Which parts of Catalyst a robot actually runs.
 *
 * <p>A spec sheet lists what the robot is made of; this lists what it is made to do. Two robots with
 * the same drivetrain and the same motor count can be completely different machines because one of
 * them scores on its own and the other does not, and nothing else on the sheet says so.
 *
 * <p>Nothing here is declared. Every entry is written by the component that created it, at the
 * moment it was created — {@code Autopilot.build()} records an autopilot because an autopilot was
 * just built. That matters more than it looks: a feature list a team maintains by hand is a list of
 * what they meant to use, and this one is a list of what came up. When they differ, the second is
 * the one worth seeing on a driver station.
 *
 * <p>Registration usually happens <em>after</em> {@code RobotIdentity.declare()}, because teams
 * declare at the top of {@code RobotContainer} and build their behaviours further down. Each record
 * therefore refreshes the sheet, the same way a drivetrain coming up does. NT4 keeps the last value
 * of a topic and hands it to whoever subscribes next, so a dashboard sees the finished list whether
 * it connected before the robot finished waking up or an hour into a match.
 *
 * @since 1.11.0
 */
public final class CatalystFeatures {

    /** A robot that acquires and scores a game piece without being told each step. */
    public static final String AUTOPILOT = "Autopilot";
    /** A scored chooser: several behaviours, and the one the situation favours wins. */
    public static final String STRATEGIST = "Strategist";
    /** A sequence with fallbacks, deadlines and bail conditions. */
    public static final String SEQUENCE = "Sequence";
    /**
     * Superstructure goals pursued through a coordinator.
     *
     * <p>Capitalised on both words on purpose: the published key is this name with its spaces
     * removed, so "Goal director" would go out as {@code Goaldirector} and a dashboard reading
     * {@code GoalDirector} would silently find nothing.
     */
    public static final String GOAL_DIRECTOR = "Goal Director";
    /** The physics estimator, with the profile it was built at. */
    public static final String PHYSICS_CORE = "Physics Core";

    /* Sorted so the published order is the same on every boot. A dashboard diffing two robots, or
     * one robot across two builds, should see differences that are real rather than differences in
     * whichever order the constructors happened to run. */
    private static final Map<String, List<String>> USED = new TreeMap<>();

    private CatalystFeatures() {}

    /**
     * Note that a piece of Catalyst is in use on this robot.
     *
     * <p>Called for you by the component being built. Teams do not call this.
     *
     * @param feature which part, one of the constants on this class
     * @param label   what this instance is called, or null when it has no name worth publishing
     */
    public static void record(String feature, String label) {
        if (feature == null || feature.isBlank()) return;

        boolean changed;
        synchronized (USED) {
            List<String> labels = USED.computeIfAbsent(feature, key -> new ArrayList<>());
            String clean = label == null ? "" : label.trim();
            /* An unnamed instance still counts, so the feature is recorded either way — but a blank
             * label is not added to the name list, because an empty string in a list of names reads
             * as a robot that named something "". */
            changed = clean.isEmpty() ? labels.isEmpty() : !labels.contains(clean);
            if (changed && !clean.isEmpty()) labels.add(clean);
        }

        /* Outside the lock. refresh() takes RobotIdentity's monitor, and taking two locks in one
         * order here while something else takes them in the other is how a robot deadlocks during
         * construction. Refreshing on an unchanged registry is only wasted work, never wrong. */
        if (changed) RobotIdentity.refresh();
    }

    /**
     * What has been recorded so far, in a stable order.
     *
     * @return feature name to the labels seen under it, which may be empty for an unnamed instance
     */
    public static Map<String, List<String>> snapshot() {
        synchronized (USED) {
            Map<String, List<String>> copy = new LinkedHashMap<>();
            USED.forEach((feature, labels) -> copy.put(feature, List.copyOf(labels)));
            return copy;
        }
    }

    /** Forget everything recorded. For tests; a robot boots once. */
    public static void reset() {
        synchronized (USED) {
            USED.clear();
        }
    }
}
