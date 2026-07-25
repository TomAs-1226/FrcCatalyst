package frc.lib.catalyst.statemachine;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;

/**
 * The legal-transition graph: which states may follow which.
 *
 * <p>This is the object that turns a bag of position presets into an actual state machine. An edge
 * that was never declared is not a transition the robot can make, which is what lets a team encode
 * "the arm may only deploy from a raised elevator" as structure rather than as a comment.
 *
 * <p>Breadth-first search here is <b>deterministic</b>: successors are visited in enum-ordinal
 * order and ties are broken the same way every time. A route therefore never changes unless the
 * graph changes — important, because a route that silently reshuffles between matches is worse
 * than no routing at all.
 *
 * @param <S> the machine's state enum
 * @since 1.2.0
 */
public final class StateGraph<S extends Enum<S>> {

    private final Class<S> stateType;
    private final S[] constants;
    private final Map<S, EnumSet<S>> edges;
    private final Map<String, Double> costs;
    private final Map<String, String> guardNames;

    StateGraph(Class<S> stateType, S[] constants, Map<S, EnumSet<S>> edges,
               Map<String, Double> costs, Map<String, String> guardNames) {
        this.stateType = stateType;
        this.constants = constants;
        this.edges = edges;
        this.costs = costs;
        this.guardNames = guardNames;
    }

    /** The state enum this graph is over. */
    public Class<S> stateType() {
        return stateType;
    }

    /** States directly reachable from {@code from}. Never null; possibly empty. */
    public EnumSet<S> successors(S from) {
        EnumSet<S> s = edges.get(from);
        return s == null ? EnumSet.noneOf(stateType) : EnumSet.copyOf(s);
    }

    /** Is there a declared edge {@code from -> to}? */
    public boolean hasEdge(S from, S to) {
        EnumSet<S> s = edges.get(from);
        return s != null && s.contains(to);
    }

    /**
     * Shortest path from {@code from} to {@code to}, excluding the origin.
     *
     * <p>Returns {@link Route#empty()} when unreachable. A one-element route means a direct edge
     * exists. Deterministic: equal-length candidates always resolve to the same answer.
     *
     * @param passable optional per-edge filter — return {@code false} to treat an edge as
     *                 temporarily impassable (a guard that is false right now). May be null.
     */
    public Route<S> route(S from, S to, EdgeFilter<S> passable) {
        if (from == to) return Route.empty();

        Map<S, S> previous = new java.util.EnumMap<>(stateType);
        EnumSet<S> seen = EnumSet.noneOf(stateType);
        Deque<S> queue = new ArrayDeque<>();
        queue.add(from);
        seen.add(from);

        while (!queue.isEmpty()) {
            S at = queue.poll();
            for (S next : orderedSuccessors(at)) {
                if (seen.contains(next)) continue;
                if (passable != null && !passable.passable(at, next)) continue;
                seen.add(next);
                previous.put(next, at);
                if (next == to) return rebuild(previous, from, to);
                queue.add(next);
            }
        }
        return Route.empty();
    }

    /** Shortest path with no edge filtering. */
    public Route<S> route(S from, S to) {
        return route(from, to, null);
    }

    /** States that cannot be reached from {@code start} by any number of hops. */
    public EnumSet<S> unreachableFrom(S start) {
        EnumSet<S> reached = EnumSet.of(start);
        Deque<S> queue = new ArrayDeque<>();
        queue.add(start);
        while (!queue.isEmpty()) {
            S at = queue.poll();
            for (S next : orderedSuccessors(at)) {
                if (reached.add(next)) queue.add(next);
            }
        }
        EnumSet<S> all = EnumSet.allOf(stateType);
        all.removeAll(reached);
        return all;
    }

    /** Total declared edges. */
    public int edgeCount() {
        int n = 0;
        for (EnumSet<S> s : edges.values()) n += s.size();
        return n;
    }

    /** One line per edge: {@code "STOW->AIM|cost=1.0|guard=endgame"}. */
    public String[] describeEdges() {
        List<String> out = new ArrayList<>();
        for (S from : constants) {
            for (S to : orderedSuccessors(from)) {
                String key = from.name() + "->" + to.name();
                double cost = costs.getOrDefault(key, 1.0);
                String guard = guardNames.getOrDefault(key, "");
                out.add(key + "|cost=" + cost + (guard.isEmpty() ? "" : "|guard=" + guard));
            }
        }
        return out.toArray(new String[0]);
    }

    /**
     * Graphviz DOT source for the legal graph.
     *
     * <p>Published as {@code Graph/Dot} so a team can paste it into any Graphviz viewer and see
     * their superstructure as a picture — which matters most for the graph you <em>thought</em>
     * you declared versus the one you actually did.
     */
    public String toDot(String machineName) {
        StringBuilder sb = new StringBuilder();
        sb.append("digraph ").append(sanitize(machineName)).append(" {\n");
        sb.append("  rankdir=LR;\n");
        sb.append("  node [shape=box, style=rounded];\n");
        for (S s : constants) sb.append("  ").append(s.name()).append(";\n");
        for (S from : constants) {
            for (S to : orderedSuccessors(from)) {
                String key = from.name() + "->" + to.name();
                String guard = guardNames.getOrDefault(key, "");
                sb.append("  ").append(from.name()).append(" -> ").append(to.name());
                if (!guard.isEmpty()) sb.append(" [label=\"").append(sanitize(guard)).append("\"]");
                sb.append(";\n");
            }
        }
        sb.append("}\n");
        return sb.toString();
    }

    private List<S> orderedSuccessors(S from) {
        EnumSet<S> s = edges.get(from);
        if (s == null || s.isEmpty()) return List.of();
        // EnumSet iterates in ordinal order, which is what makes BFS deterministic.
        return new ArrayList<>(s);
    }

    private Route<S> rebuild(Map<S, S> previous, S from, S to) {
        List<S> hops = new ArrayList<>();
        S at = to;
        while (at != from) {
            hops.add(at);
            at = previous.get(at);
            if (at == null) return Route.empty();
        }
        java.util.Collections.reverse(hops);
        return new Route<>(hops);
    }

    private static String sanitize(String s) {
        return s.replaceAll("[^A-Za-z0-9_]", "_");
    }

    @Override
    public String toString() {
        return "StateGraph(" + stateType.getSimpleName() + ", " + Arrays.toString(constants).length()
                + " states, " + edgeCount() + " edges)";
    }

    /** Per-edge passability test used to exclude edges whose guards are currently false. */
    @FunctionalInterface
    public interface EdgeFilter<S extends Enum<S>> {
        /** Return {@code false} to treat the edge {@code from -> to} as impassable right now. */
        boolean passable(S from, S to);
    }
}
