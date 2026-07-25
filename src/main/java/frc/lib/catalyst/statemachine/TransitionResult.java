package frc.lib.catalyst.statemachine;

import java.util.List;

/**
 * The immediate answer to a transition request: accepted with a route, or refused with a reason.
 *
 * <p>{@code request(...)} is a pure decision function — it never builds a command and never
 * throws — so this record is the complete outcome of asking. A rejected result carries both a
 * machine-readable {@link RejectReason} and a human {@code detail} naming the specific guard,
 * interlock or mechanism responsible.
 *
 * @param accepted whether the request was granted
 * @param from     the state the machine was in when asked
 * @param to       the requested target
 * @param route    the hops that will be visited; empty on rejection
 * @param reason   {@link RejectReason#NONE} when accepted
 * @param detail   free text naming the culprit, or {@code ""}
 * @param seq      the transition sequence number when accepted, or {@code -1}
 * @param <S>      the machine's state enum
 * @since 1.2.0
 */
public record TransitionResult<S extends Enum<S>>(
        boolean accepted, S from, S to, List<S> route,
        RejectReason reason, String detail, long seq) {

    /** Defensively copies the route and normalises nulls. */
    public TransitionResult {
        route = route == null ? List.of() : List.copyOf(route);
        reason = reason == null ? RejectReason.NONE : reason;
        detail = detail == null ? "" : detail;
    }

    /** Convenience inverse of {@link #accepted()}. */
    public boolean rejected() {
        return !accepted;
    }

    /** Build an acceptance. */
    public static <S extends Enum<S>> TransitionResult<S> accept(long seq, S from, S to, List<S> route) {
        return new TransitionResult<>(true, from, to, route, RejectReason.NONE, "", seq);
    }

    /** Build a rejection. */
    public static <S extends Enum<S>> TransitionResult<S> reject(S from, S to, RejectReason reason, String detail) {
        return new TransitionResult<>(false, from, to, List.of(), reason, detail, -1L);
    }

    @Override
    public String toString() {
        String fromName = from == null ? "?" : from.name();
        String toName = to == null ? "?" : to.name();
        if (accepted) {
            return fromName + "->" + toName + " ACCEPTED seq=" + seq;
        }
        return fromName + "->" + toName + ": " + reason + (detail.isEmpty() ? "" : " (" + detail + ")");
    }
}
