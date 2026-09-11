package frc.lib.catalyst.autonomy;

import java.util.ArrayList;
import java.util.List;

/**
 * Combines every limit on the robot's output into one number, and says which one is binding.
 *
 * <p>A robot accumulates limiters. Physics Core scales for slip and localisation confidence, the
 * driver holds slow mode, a brownout monitor throttles, an autonomy layer eases off near a wall.
 * Each is reasonable and each is written separately, so what a team actually gets is several
 * uncoordinated writers of one field - and the last one to write in a given loop wins. This library
 * has already shipped that bug in the other direction: a governor six of eight drive commands
 * ignored entirely.
 *
 * <p>The rule here is the only one that is safe when limiters disagree: <b>take the smallest, and
 * say whose it was</b>. A limiter can always slow the robot and can never speed it up, so adding a
 * limiter is always safe and removing one is the only way to go faster. That property is what makes
 * this composable - a team can add their own limit without auditing the others.
 *
 * <p>Attribution is the other half. "The robot feels sluggish" is a support question; "traction is
 * holding you at 60%" is an answer, and it costs one string.
 *
 * <p>Pure. No clock, no hardware, no state.
 *
 * @since 2.1.0
 */
public final class AuthorityCore {
    private AuthorityCore() {}

    /**
     * One limiter's opinion.
     *
     * @param source who is asking, e.g. {@code "physics"}, {@code "slowMode"}, {@code "power"}
     * @param scale  the most output this limiter will allow, in [0, 1]; values outside are clamped
     * @param reason why, for the dashboard; may be empty when it is not limiting
     */
    public record Limit(String source, double scale, String reason) {

        /** A limiter that is not currently limiting anything. */
        public static Limit none(String source) {
            return new Limit(source, 1.0, "");
        }

        /** Clamped on the way in, so no limiter can grant authority it does not have. */
        public Limit {
            scale = Double.isNaN(scale) ? 0.0 : Math.clamp(scale, 0.0, 1.0);
            reason = reason == null ? "" : reason;
            source = source == null ? "?" : source;
        }
    }

    /**
     * @param scale   the authority to apply, in [0, 1]
     * @param binding the source of the smallest limit, or {@code "none"} when nothing is limiting
     * @param explain a sentence naming what is holding the robot back
     */
    public record Authority(double scale, String binding, String explain) {

        /** True when something is meaningfully holding the robot back. */
        public boolean limited() {
            return scale < 0.999;
        }
    }

    /**
     * The smallest limit wins.
     *
     * @param limits every limiter's opinion this loop; an empty list means full authority
     */
    public static Authority combine(List<Limit> limits) {
        if (limits == null || limits.isEmpty()) {
            return new Authority(1.0, "none", "no limits");
        }
        Limit lowest = null;
        for (Limit l : limits) {
            if (lowest == null || l.scale() < lowest.scale()) {
                lowest = l;
            }
        }
        if (lowest.scale() >= 0.999) {
            return new Authority(1.0, "none", "no limits");
        }

        // Anything else close to the floor is worth naming too: a driver told only about slow mode
        // will keep wondering why it is still slow after they release it.
        List<String> alsoNear = new ArrayList<>(2);
        for (Limit l : limits) {
            if (l != lowest && l.scale() <= lowest.scale() + 0.05) {
                alsoNear.add(l.source());
            }
        }

        StringBuilder explain = new StringBuilder();
        explain.append(String.format("%s holding at %.0f%%", lowest.source(), lowest.scale() * 100));
        if (!lowest.reason().isEmpty()) {
            explain.append(" (").append(lowest.reason()).append(')');
        }
        if (!alsoNear.isEmpty()) {
            explain.append("; also ").append(String.join(", ", alsoNear));
        }
        return new Authority(lowest.scale(), lowest.source(), explain.toString());
    }

    /**
     * The limit a {@link Situation}'s traction and power facets imply, or {@link Limit#none} when
     * neither is measured.
     *
     * <p>Deliberately gentle, and deliberately not a substitute for {@code PhysicsConstraints}: this
     * exists so a robot with a situation but no physics constraints still gets the obvious
     * protections, not so it can replace a model that knows the drivetrain.
     */
    public static Limit fromSituation(Situation s) {
        if (s == null) {
            return Limit.none("situation");
        }
        double scale = 1.0;
        String reason = "";
        if (s.traction().valid() && s.traction().slipping()) {
            // Wheels already slipping will not deliver more, so asking for more only spoils odometry.
            double slipScale = 1.0 - 0.5 * Math.min(1.0, s.traction().slipFactor());
            if (slipScale < scale) {
                scale = slipScale;
                reason = String.format("slip %.2f", s.traction().slipFactor());
            }
        }
        if (s.power().valid() && s.power().overBudget()) {
            if (0.6 < scale) {
                scale = 0.6;
                reason = String.format("%.0f A over budget", -s.power().headroomAmps());
            }
        }
        return new Limit("situation", scale, reason);
    }
}
