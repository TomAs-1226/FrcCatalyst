package frc.lib.catalyst.statemachine;

import java.util.List;

/**
 * The permanent record of one transition attempt, emitted exactly once when the attempt ends for
 * any reason — including rejection.
 *
 * <p>This is the answer to the complaint that motivated the whole package: a state machine you
 * wrote yourself works until it doesn't, and then there is nothing to read. A ring of these,
 * published as {@code Transition/History}, tells you what was asked for, by whom, which way it
 * went, how long it took, whether it made it, and which mechanism was responsible if it did not —
 * for the last fifty attempts, without a laptop attached.
 *
 * @param seq            monotonic sequence number; matches {@code Transition/Seq}
 * @param startTimestamp clock reading when the request was accepted
 * @param endTimestamp   clock reading when it ended
 * @param durationSeconds wall time of the attempt
 * @param from           origin state
 * @param to             requested target
 * @param route          hops actually planned
 * @param trigger        free-text source of the request ({@code "op.y"}, {@code "auto"}, …)
 * @param outcome        how it ended
 * @param reason         rejection reason, or {@link RejectReason#NONE}
 * @param detail         human detail naming the culprit
 * @param arrivals       per-binding arrival reports; empty for rejections
 * @param <S>            the machine's state enum
 * @since 1.2.0
 */
public record TransitionRecord<S extends Enum<S>>(
        long seq, double startTimestamp, double endTimestamp, double durationSeconds,
        S from, S to, List<S> route, String trigger,
        Outcome outcome, RejectReason reason, String detail,
        List<ArrivalReport> arrivals) {

    /** How a transition attempt ended. */
    public enum Outcome {
        /** Every gating binding reached its goal and the state was confirmed. */
        ARRIVED,
        /** Refused before anything moved. */
        REJECTED,
        /** A hop blew its deadline. */
        TIMED_OUT,
        /** Cancelled by {@code abort()} or by the driving command being interrupted. */
        ABORTED,
        /** Replaced by a newer request before it finished. */
        SUPERSEDED,
        /** Ended by a fault policy. */
        FAULTED,
        /** Not a real transition — {@code seed()} injecting a known starting state. */
        SEEDED
    }

    /** Defensively copies the collections and normalises nulls. */
    public TransitionRecord {
        route = route == null ? List.of() : List.copyOf(route);
        arrivals = arrivals == null ? List.of() : List.copyOf(arrivals);
        trigger = trigger == null ? "" : trigger;
        detail = detail == null ? "" : detail;
        reason = reason == null ? RejectReason.NONE : reason;
    }

    /** Renders the route as {@code "MID>SCORE_HIGH"}, or {@code ""} when direct or rejected. */
    public String routeString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < route.size(); i++) {
            if (i > 0) sb.append('>');
            sb.append(route.get(i).name());
        }
        return sb.toString();
    }

    /** True when this attempt did not end in {@link Outcome#ARRIVED}. */
    public boolean failed() {
        return outcome != Outcome.ARRIVED && outcome != Outcome.SEEDED;
    }

    /**
     * Pipe-delimited single line for a {@code String[]} log entry, in the order
     * {@code timestamp|seq|from|to|route|trigger|outcome|reason|duration|detail}.
     *
     * <p>Pipes inside any field become underscores, so the line always splits into exactly ten
     * columns and a dashboard table can rely on the shape.
     */
    public String serialize() {
        return String.join("|",
                String.format("%.3f", endTimestamp),
                Long.toString(seq),
                from == null ? "" : from.name(),
                to == null ? "" : to.name(),
                sanitize(routeString()),
                sanitize(trigger),
                outcome.name(),
                reason.name(),
                String.format("%.3f", durationSeconds),
                sanitize(detail));
    }

    private static String sanitize(String s) {
        return s == null ? "" : s.replace('|', '_');
    }

    @Override
    public String toString() {
        String via = route.size() > 1 ? " via " + routeString() : "";
        String because = detail.isEmpty() ? "" : "  " + detail;
        return String.format("[%7.2fs] %s->%s%s  %s  %.2fs%s",
                endTimestamp,
                from == null ? "?" : from.name(),
                to == null ? "?" : to.name(),
                via, outcome, durationSeconds, because);
    }
}
