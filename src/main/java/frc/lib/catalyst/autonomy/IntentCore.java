package frc.lib.catalyst.autonomy;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

/**
 * Guesses what the driver is about to do, and keeps score of how often it was right.
 *
 * <h2>It cannot act, and that is the point</h2>
 *
 * <p>An intention system that acts on a guess is a robot that does something the driver did not ask
 * for, at a moment the driver was busy. There is no training data for this, no validation set, and
 * the prediction horizon available is about a second - over which "what is the driver about to do"
 * is mostly a restatement of what they are already doing. So this publishes a guess and a running
 * hit rate and commands nothing.
 *
 * <p>That is not a placeholder. The hit rate is the deliverable: it is the evidence that decides
 * whether inference is ever worth wiring to anything. Ship it, drive a season, and the number says
 * whether the idea works on this robot with these drivers. Building the acting half first would be
 * deciding that question by assumption.
 *
 * <h2>How the guess is made</h2>
 *
 * <p>The caller supplies candidate intents and a score for each, exactly as
 * {@code Strategist} does for behaviours - this owns none of the game knowledge. What it adds is
 * hysteresis, so the guess does not flap, and {@link #observe} for recording what actually happened.
 *
 * @since 2.1.0
 */
public final class IntentCore {

    /**
     * One thing the driver might be about to do.
     *
     * @param name       what it is called, matched against {@link #observe}
     * @param confidence 0..1; the caller's own scoring
     * @param because    why the caller thinks so, for the dashboard
     */
    public record Guess(String name, double confidence, String because) {}

    /**
     * @param best    the leading guess, or empty when nothing scored above the floor
     * @param ranked  every candidate, best first
     * @param hitRate the fraction of past guesses that matched what happened, or NaN before there
     *                is enough history to mean anything
     * @param samples how many outcomes have been observed
     */
    public record Reading(Optional<Guess> best, List<Guess> ranked, double hitRate, int samples) {

        /** A sentence for {@code Autonomy/Intent}. */
        public String explain() {
            if (best.isEmpty()) {
                return "no idea";
            }
            Guess g = best.get();
            String rate = Double.isNaN(hitRate) ? "not enough history"
                    : String.format("right %.0f%% of %d", hitRate * 100, samples);
            return String.format("%s (%.0f%%%s) - %s",
                    g.name(), g.confidence() * 100,
                    g.because().isEmpty() ? "" : ", " + g.because(), rate);
        }
    }

    /** Below this many outcomes the hit rate is noise and is reported as unknown. */
    private static final int MIN_SAMPLES = 10;

    private final double minConfidence;
    private final double stickiness;

    private String held = "";
    private int hits;
    private int samples;

    /**
     * @param minConfidence a guess below this is not worth making
     * @param stickiness    how much a guess must beat the one being held by before it replaces it;
     *                      0 means always take the leader, which flaps
     */
    public IntentCore(double minConfidence, double stickiness) {
        this.minConfidence = minConfidence;
        this.stickiness = Math.max(0.0, stickiness);
    }

    /** Rank the candidates and update the held guess. */
    public Reading read(List<Guess> candidates) {
        List<Guess> ranked = new ArrayList<>(candidates == null ? List.of() : candidates);
        ranked.sort(Comparator.<Guess>comparingDouble(Guess::confidence).reversed()
                .thenComparing(Guess::name));

        Guess leader = ranked.isEmpty() ? null : ranked.get(0);
        if (leader == null || leader.confidence() < minConfidence) {
            held = "";
            return new Reading(Optional.empty(), List.copyOf(ranked), rate(), samples);
        }

        if (!leader.name().equals(held)) {
            // Only displace the held guess if the leader is clearly better. Two intents trading
            // places every loop would make the dashboard useless and the hit rate meaningless.
            double heldConfidence = ranked.stream()
                    .filter(g -> g.name().equals(held))
                    .mapToDouble(Guess::confidence)
                    .findFirst()
                    .orElse(Double.NEGATIVE_INFINITY);
            if (leader.confidence() >= heldConfidence + stickiness) {
                held = leader.name();
            }
        }

        String current = held;
        Optional<Guess> best = ranked.stream().filter(g -> g.name().equals(current)).findFirst();
        return new Reading(best.isPresent() ? best : Optional.of(leader),
                List.copyOf(ranked), rate(), samples);
    }

    /**
     * Record what the driver actually did, so the guess can be marked.
     *
     * <p>Call this when a real intent becomes unambiguous - a scoring sequence started, a piece was
     * acquired, a climb began. The name must match a candidate's.
     */
    public void observe(String actual) {
        if (actual == null || actual.isEmpty()) {
            return;
        }
        samples++;
        if (actual.equals(held)) {
            hits++;
        }
    }

    /** The fraction of guesses that were right, or NaN while there is too little history. */
    public double hitRate() {
        return rate();
    }

    /** How many outcomes have been observed. */
    public int samples() {
        return samples;
    }

    /** The guess currently held, or empty. */
    public String held() {
        return held;
    }

    private double rate() {
        return samples < MIN_SAMPLES ? Double.NaN : (double) hits / samples;
    }

    /** Forget the history, for a new match. */
    public void reset() {
        held = "";
        hits = 0;
        samples = 0;
    }
}
