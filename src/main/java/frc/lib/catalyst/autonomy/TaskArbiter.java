package frc.lib.catalyst.autonomy;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Picks as many non-conflicting tasks as will fit, best first.
 *
 * <p>Everything in this library that chooses what to do picks exactly one thing. {@code Strategist}
 * scores its behaviours and runs the winner; {@code GoalDirector} pursues one goal. So a robot whose
 * intake behaviour and whose shooter behaviour share no mechanisms still ran one of them and left
 * the other waiting, for no reason other than that nothing could express "both".
 *
 * <p>This is that missing primitive. Given scored candidates that each declare what they need, it
 * takes them in score order and keeps any whose requirements do not collide with something already
 * taken. It is the same shape as the command scheduler's requirement model and deliberately so -
 * this decides, and the scheduler still enforces.
 *
 * <h2>Why greedy, and why that is the right answer here</h2>
 *
 * <p>Maximising total score over non-conflicting sets is the weighted independent set problem, which
 * is NP-hard, and a robot has 20 ms. Greedy-by-score is not optimal in general. It is optimal often
 * enough, it is <em>explicable</em> - "it took the best one, then the best one that still fit" is
 * something a student can reason about at a competition - and it is stable: adding a candidate can
 * never displace a higher-scoring one. An arbiter nobody can predict is worse than one that
 * occasionally leaves a point on the table.
 *
 * <p>Generic in the resource type so it can be tested with plain strings, and used with
 * {@code Mechanism} on a robot. Pure, deterministic, and free of any clock.
 *
 * @param <T> what a task is - a behaviour, a goal, an action, whatever the caller has
 * @param <R> what a resource is - {@code Mechanism} in production
 * @since 2.1.0
 */
public final class TaskArbiter {
    private TaskArbiter() {}

    /**
     * One thing that could run.
     *
     * @param task         the caller's own object, handed back untouched in the selection
     * @param name         for telemetry and for the deterministic tie-break
     * @param score        higher wins; compared against {@code minScore}
     * @param requirements what it needs; two candidates conflict when these intersect
     * @param canStart     its precondition, already evaluated and guarded by the caller
     */
    public record Candidate<T, R>(T task, String name, double score, Set<R> requirements,
                                  boolean canStart) {}

    /** Why a candidate did not make it, so the dashboard can say rather than just omit. */
    public record Skipped(String name, String reason) {}

    /**
     * @param winners the tasks to run, in the order they were taken
     * @param skipped everything else, with a reason each
     */
    public record Selection<T, R>(List<Candidate<T, R>> winners, List<Skipped> skipped) {

        /** The winning tasks alone. */
        public List<T> tasks() {
            List<T> out = new ArrayList<>(winners.size());
            for (Candidate<T, R> c : winners) {
                out.add(c.task());
            }
            return out;
        }

        /** A one-line account of the decision, for {@code Autonomy/Explain}. */
        public String explain() {
            if (winners.isEmpty()) {
                return skipped.isEmpty() ? "nothing to choose from"
                        : "nothing ran: " + skipped.get(0).name() + " " + skipped.get(0).reason()
                          + (skipped.size() > 1 ? " (and " + (skipped.size() - 1) + " more)" : "");
            }
            StringBuilder sb = new StringBuilder("running ");
            for (int i = 0; i < winners.size(); i++) {
                sb.append(i > 0 ? ", " : "").append(winners.get(i).name());
            }
            if (!skipped.isEmpty()) {
                sb.append("; held ").append(skipped.size());
            }
            return sb.toString();
        }
    }

    /**
     * Choose.
     *
     * @param candidates what could run; not modified
     * @param minScore   a candidate scoring below this is not worth running at all
     */
    public static <T, R> Selection<T, R> select(List<Candidate<T, R>> candidates, double minScore) {
        List<Candidate<T, R>> ordered = new ArrayList<>(candidates);
        // Score descending, then name, so an arbiter given the same inputs twice answers the same
        // way twice. Two behaviours that tie must not swap between loops.
        ordered.sort(Comparator.<Candidate<T, R>>comparingDouble(Candidate::score).reversed()
                .thenComparing(Candidate::name));

        List<Candidate<T, R>> winners = new ArrayList<>(ordered.size());
        List<Skipped> skipped = new ArrayList<>();
        Set<R> claimed = new HashSet<>();

        for (Candidate<T, R> c : ordered) {
            if (!c.canStart()) {
                skipped.add(new Skipped(c.name(), "cannot start"));
                continue;
            }
            if (c.score() < minScore) {
                skipped.add(new Skipped(c.name(),
                        String.format("scored %.2f, below the %.2f floor", c.score(), minScore)));
                continue;
            }
            R clash = firstClash(claimed, c.requirements());
            if (clash != null) {
                skipped.add(new Skipped(c.name(), "needs " + clash + ", already taken"));
                continue;
            }
            winners.add(c);
            claimed.addAll(c.requirements());
        }
        return new Selection<>(List.copyOf(winners), List.copyOf(skipped));
    }

    /**
     * The first resource in {@code wanted} that is already {@code claimed}, or null.
     *
     * <p>Iterates the wanted set rather than intersecting: a candidate needs one or two mechanisms,
     * and this runs every loop over every candidate, so it stops at the first collision and
     * allocates nothing.
     */
    private static <R> R firstClash(Set<R> claimed, Set<R> wanted) {
        if (claimed.isEmpty() || wanted == null || wanted.isEmpty()) {
            return null;
        }
        for (R r : wanted) {
            if (claimed.contains(r)) {
                return r;
            }
        }
        return null;
    }
}
