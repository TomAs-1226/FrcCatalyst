package frc.lib.catalyst.statemachine;

import java.util.List;

/**
 * An ordered list of states to visit, excluding the origin.
 *
 * <p>A direct transition has exactly one hop; a routed one has several. An empty route means
 * "unreachable", which is why {@code plan(target)} can be used as a zero-side-effect reachability
 * preview before a button is ever pressed.
 *
 * @param hops the states to visit in order; the last element is the target
 * @param <S>  the machine's state enum
 * @since 1.2.0
 */
public record Route<S extends Enum<S>>(List<S> hops) {

    /** Defensively copies the hop list so a route is genuinely immutable once handed out. */
    public Route {
        hops = hops == null ? List.of() : List.copyOf(hops);
    }

    /** The final destination, or {@code null} when this route is empty. */
    public S target() {
        return hops.isEmpty() ? null : hops.get(hops.size() - 1);
    }

    /** Number of hops. Zero means unreachable. */
    public int length() {
        return hops.size();
    }

    /** True when no path exists. */
    public boolean isEmpty() {
        return hops.isEmpty();
    }

    /** Renders as {@code "MID>SCORE_HIGH"}, or {@code ""} when empty. */
    public String render() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < hops.size(); i++) {
            if (i > 0) sb.append('>');
            sb.append(hops.get(i).name());
        }
        return sb.toString();
    }

    /** The empty (unreachable) route. */
    public static <S extends Enum<S>> Route<S> empty() {
        return new Route<S>(List.<S>of());
    }

    @Override
    public String toString() {
        return isEmpty() ? "(unreachable)" : render();
    }
}
