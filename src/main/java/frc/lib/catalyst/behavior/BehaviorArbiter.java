package frc.lib.catalyst.behavior;

import java.util.List;

/**
 * Decides which behaviour should be running, and remembers how often that answer changes.
 *
 * <h2>Why this is a separate class</h2>
 *
 * <p>The decision used to live inside the selector's coroutine, where it could not be tested without
 * a scheduler and where the only record of it was the current winner's name on NetworkTables. Pulled
 * out, it is a pure function of (scores, incumbent, clock) and every rule below can be exercised in
 * a unit test.
 *
 * <h2>The problem it exists for</h2>
 *
 * <p>Utility selection has no memory. Every loop the highest scorer wins, so two behaviours whose
 * scores cross repeatedly hand control back and forth at the loop rate — and each handover cancels a
 * command and schedules a new one, so the mechanism restarts fifty times a second and never
 * finishes anything. From the driver's seat the robot twitches and does nothing. Nothing in the
 * scores looks wrong, because nothing is wrong with any individual score.
 *
 * <p>The likelier trigger is not two behaviours trading places but one behaviour flickering in and
 * out of eligibility — a vision-gated scorer with a Limelight whose {@code tv} drops for a frame
 * takes the selector to "nothing wants to run", cancels, and re-schedules on the next loop. That is
 * counted here as a switch too, because it costs exactly the same restart.
 *
 * <h2>Both knobs default to zero, and that is load-bearing</h2>
 *
 * <p>A non-zero default would change what every existing robot does on a library upgrade, silently,
 * which is the failure the additive-only rule exists to prevent. At zero the arbiter reproduces the
 * old behaviour with one deliberate exception, stated here rather than buried: an exact score tie
 * now keeps the incumbent. The old scan took the first behaviour in registration order on a tie, so
 * a tie between two behaviours could switch or not depending on which was registered first. Keeping
 * the incumbent is both stabler and deterministic.
 *
 * <p>{@code switchMargin} is denominated in the team's own score units, which is why it cannot have
 * a safe default: nobody but the team knows whether their scores run 0–1 or 0–100. Set too high it
 * turns a thrashing robot into a frozen one, which is equally silent — so {@code heldSeconds} and
 * {@code lastSwitchReason} are published whether or not the knobs are used, and a stuck incumbent
 * shows up as a large held time with a stale reason. Both failure directions are visible on the
 * same keys.
 *
 * @since 2.0.0
 */
final class BehaviorArbiter {

    /** One behaviour's standing this loop. */
    record Candidate(String name, double score, boolean eligible) {}

    /**
     * What to do this loop.
     *
     * @param winner  the behaviour that should be running, or null for none
     * @param switched whether that differs from the incumbent, and so costs a restart
     * @param reason  human-readable, for a dashboard; free-form and not parsed
     */
    record Decision(String winner, boolean switched, String reason) {}

    private final double minScore;
    private final double switchMargin;
    private final double minDwellSeconds;

    private int switches = 0;
    private double lastSwitchTs = Double.NaN;
    private double firstDecisionTs = Double.NaN;
    private String lastSwitchReason = "(none yet)";

    BehaviorArbiter(double minScore, double switchMargin, double minDwellSeconds) {
        this.minScore = minScore;
        this.switchMargin = Math.max(0.0, switchMargin);
        this.minDwellSeconds = Math.max(0.0, minDwellSeconds);
    }

    /**
     * Pick a winner.
     *
     * @param candidates every registered behaviour with this loop's score and eligibility
     * @param incumbent  the behaviour currently running, or null/empty if none
     * @param now        seconds, monotonic
     */
    Decision decide(List<Candidate> candidates, String incumbent, double now) {
        if (Double.isNaN(firstDecisionTs)) {
            firstDecisionTs = now;
        }
        String held = (incumbent == null || incumbent.isEmpty()) ? null : incumbent;

        Candidate best = null;
        for (Candidate c : candidates) {
            if (!c.eligible() || !(c.score() > minScore)) {
                continue;
            }
            if (best == null || c.score() > best.score()) {
                best = c;
            }
        }

        if (best == null) {
            // Nothing wants to run. If something was running, this costs a restart when it comes
            // back - which is the flickering-eligibility case, and the one the original framing of
            // this problem missed.
            if (held != null) {
                return switchTo(null, now, "no candidate above minScore " + minScore
                        + "; released '" + held + "'");
            }
            return new Decision(null, false, "nothing eligible");
        }

        if (best.name().equals(held)) {
            return new Decision(best.name(), false, "incumbent still winning");
        }

        if (held == null) {
            return switchTo(best.name(), now, "started '" + best.name() + "'");
        }

        // The incumbent is still around. Is it still a real option? If it has gone ineligible or
        // fallen below minScore there is nothing to protect and the switch is immediate - a
        // behaviour that cannot run must never be held on to by a dwell timer.
        Candidate incumbentNow = null;
        for (Candidate c : candidates) {
            if (c.name().equals(held)) {
                incumbentNow = c;
                break;
            }
        }
        boolean incumbentViable = incumbentNow != null
                && incumbentNow.eligible() && incumbentNow.score() > minScore;
        if (!incumbentViable) {
            return switchTo(best.name(), now, "'" + held + "' is no longer runnable");
        }

        double margin = best.score() - incumbentNow.score();
        if (!(margin > switchMargin)) {
            return new Decision(held, false, String.format(
                    "'%s' leads '%s' by %.3f, under the %.3f margin", best.name(), held,
                    margin, switchMargin));
        }

        double heldFor = heldSeconds(now);
        if (heldFor < minDwellSeconds) {
            return new Decision(held, false, String.format(
                    "'%s' held %.2fs of %.2fs minimum", held, heldFor, minDwellSeconds));
        }

        return switchTo(best.name(), now, String.format(
                "'%s' beat '%s' by %.3f after %.2fs", best.name(), held, margin, heldFor));
    }

    private Decision switchTo(String winner, double now, String reason) {
        switches++;
        lastSwitchTs = now;
        lastSwitchReason = reason;
        return new Decision(winner, true, reason);
    }

    /** Total handovers, including releases to nothing. Each one restarts a command. */
    int switches() {
        return switches;
    }

    /** Handovers per second since the first decision. Zero before any time has passed. */
    double switchesPerSecond(double now) {
        if (Double.isNaN(firstDecisionTs)) {
            return 0.0;
        }
        double elapsed = now - firstDecisionTs;
        return elapsed > 0 ? switches / elapsed : 0.0;
    }

    /** How long the current winner has been held. Zero before the first switch. */
    double heldSeconds(double now) {
        return Double.isNaN(lastSwitchTs) ? 0.0 : now - lastSwitchTs;
    }

    /** Why the last handover happened, for a dashboard. */
    String lastSwitchReason() {
        return lastSwitchReason;
    }
}
