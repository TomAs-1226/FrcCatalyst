package frc.lib.catalyst.statemachine;

/**
 * How the machine turns a request into a path through the legal-transition graph.
 *
 * @since 1.2.0
 */
public enum Routing {

    /**
     * A request is legal only if a declared edge connects the current state to the target.
     * An omitted edge is a hard {@link RejectReason#NO_EDGE} rejection.
     *
     * <p>This is the default, and it is what makes the graph a genuine safety constraint rather
     * than a hint: if you never declared {@code CLIMB -> SCORE_HIGH}, the robot will not do it,
     * no matter what else is reachable.
     */
    DIRECT_ONLY,

    /**
     * Route through waypoints automatically: if {@code STOW -> SCORE_HIGH} is not declared but
     * {@code STOW -> MID -> SCORE_HIGH} is, the machine visits {@code MID} on the way and
     * confirms arrival there before continuing.
     *
     * <p>Ergonomically this is the "press Y from anywhere" mode, but shortest is not the same as
     * safest, and adding one edge in week six can silently reroute an existing binding. To keep
     * that visible, enabling this publishes <em>every</em> multi-hop route the graph can produce
     * to {@code Graph/Routes} and into {@code validate().warnings()}, and breadth-first search is
     * deterministic (lowest enum ordinal first), so a route never changes without a graph change.
     */
    SHORTEST_PATH
}
