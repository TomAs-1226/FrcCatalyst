package frc.lib.catalyst.autonomy;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

/**
 * Which of several things to go after, when going after one costs time you could spend on another.
 *
 * <p>A robot choosing between game pieces is not choosing the nearest, and it is not choosing the
 * most valuable. It is choosing the best <em>rate</em>: value divided by the time to get there and
 * use it. A piece worth twice as much but four times as far away is a worse cycle, and a scoring
 * location worth slightly less but reachable in half the time is usually the right answer.
 *
 * <p>So this ranks by value per second, refuses anything that cannot be reached in the time left,
 * and says why it chose. Nothing here knows about the field, the game, or how a target is reached -
 * the caller supplies value and reachability, typically from
 * {@code CapabilityEvaluator.evaluateDriveTo(...)}, which already computes time-to-arrive from the
 * drivetrain model and says whether the answer is trustworthy.
 *
 * <p>Pure and generic, so a season's worth of scoring rules can be tested at a desk.
 *
 * <p>{@code T} throughout is whatever a target is to the caller - a detected game piece, a scoring
 * location, a record of your own. This never looks inside it.
 *
 * @since 2.1.0
 */
public final class ChaseCore {
    private ChaseCore() {}

    /**
     * One thing that could be chased.
     *
     * @param target    the caller's own object
     * @param name      for telemetry and the deterministic tie-break
     * @param value     what getting it is worth; any consistent unit, points is the natural one
     * @param seconds   how long to reach and use it; {@code NaN} when unknown
     * @param reachable false when the caller's feasibility check says no
     * @param note      why it is not reachable, or any other detail worth surfacing
     */
    public record Target<T>(T target, String name, double value, double seconds, boolean reachable,
                            String note) {

        /**
         * Value per second: the number that actually decides a cycle.
         *
         * <p>A target with no time estimate is ranked on value alone rather than dropped - not
         * knowing how long something takes is a reason to be cautious, not a reason to ignore a
         * piece sitting in front of the robot.
         */
        public double rate() {
            if (Double.isNaN(seconds) || seconds <= 0) {
                return value;
            }
            return value / seconds;
        }
    }

    /**
     * @param chosen    what to go after, or empty when nothing qualifies
     * @param rejected  everything else, best first, each with the reason it lost
     * @param reason    a sentence for {@code Autonomy/Chase/Why}
     */
    public record Choice<T>(Optional<Target<T>> chosen, List<Target<T>> rejected, String reason) {}

    /**
     * Choose.
     *
     * @param candidates       what could be chased; not modified
     * @param secondsRemaining time left in the period, or {@code NaN} when unknown
     * @param minValue         a target worth less than this is not worth a cycle
     */
    public static <T> Choice<T> choose(List<Target<T>> candidates, double secondsRemaining,
                                       double minValue) {
        List<Target<T>> ranked = new ArrayList<>(candidates);
        // Rate descending, then name, so the same field twice gives the same answer twice. A chaser
        // that reorders equal targets between loops drives at the midpoint of both.
        ranked.sort(Comparator.<Target<T>>comparingDouble(Target::rate).reversed()
                .thenComparing(Target::name));

        List<Target<T>> rejected = new ArrayList<>();
        for (Target<T> t : ranked) {
            if (!t.reachable()) {
                rejected.add(t);
                continue;
            }
            if (t.value() < minValue) {
                rejected.add(t);
                continue;
            }
            // Do not start something the match will not let you finish. Driving most of the way to
            // a piece and stopping is worth exactly nothing, and it costs the cycle you could have
            // had instead.
            if (!Double.isNaN(secondsRemaining) && !Double.isNaN(t.seconds())
                    && t.seconds() > secondsRemaining) {
                rejected.add(t);
                continue;
            }
            return new Choice<>(Optional.of(t), List.copyOf(rejected),
                    String.format("%s: %.1f value in %.1fs (%.2f/s)",
                            t.name(), t.value(), t.seconds(), t.rate()));
        }
        return new Choice<>(Optional.empty(), List.copyOf(rejected),
                candidates.isEmpty() ? "nothing in sight" : "nothing worth chasing");
    }
}
