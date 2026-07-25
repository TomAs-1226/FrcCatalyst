package frc.lib.catalyst.statemachine;

import java.util.List;

/**
 * Everything the machine knows about itself right now, in one immutable value.
 *
 * <p>Handed to dashboards, tests and any code that wants a coherent picture rather than a dozen
 * getters read at slightly different moments.
 *
 * @param machine         machine name
 * @param current         last state whose arrival was proven
 * @param stateConfirmed  {@code false} after a timeout, abort, interrupt or fault
 * @param target          requested destination, or {@code current} when idle
 * @param nextHop         the hop being executed now
 * @param phase           what the machine is doing
 * @param faulted         whether a fault is latched
 * @param faultReason     fault text, or {@code ""}
 * @param blocker         stable, low-cardinality holdup summary
 * @param blockerDetail   holdup detail with live numbers
 * @param summary         one-line human summary
 * @param progress        fraction of gating bindings at goal, in {@code [0,1]}
 * @param elapsedSeconds  time spent on the current hop
 * @param timeoutSeconds  the current hop's deadline
 * @param stageIndex      active stage of a staged edge, or {@code -1}
 * @param stageCount      number of stages on the active edge, or {@code 0}
 * @param waitingOn       gating binding keys not yet at goal
 * @param bindings        per-binding samples
 * @param history         newest-first transition history
 * @param ticks           step counter, the heartbeat
 * @param uptimeSeconds   seconds since the machine was built
 * @param transitions     completed transition count
 * @param rejections      rejected request count
 * @param timeouts        blown deadline count
 * @param aborts          abort count
 * @param yields          ownership-loss count
 * @param <S>             the machine's state enum
 * @since 1.2.0
 */
public record Snapshot<S extends Enum<S>>(
        String machine, S current, boolean stateConfirmed, S target, S nextHop,
        Phase phase, boolean faulted, String faultReason,
        String blocker, String blockerDetail, String summary,
        double progress, double elapsedSeconds, double timeoutSeconds,
        int stageIndex, int stageCount, List<String> waitingOn,
        List<BindingSample> bindings, List<TransitionRecord<S>> history,
        long ticks, double uptimeSeconds,
        long transitions, long rejections, long timeouts, long aborts, long yields) {

    /** Defensively copies the collections and normalises nulls. */
    public Snapshot {
        waitingOn = waitingOn == null ? List.of() : List.copyOf(waitingOn);
        bindings = bindings == null ? List.of() : List.copyOf(bindings);
        history = history == null ? List.of() : List.copyOf(history);
        machine = machine == null ? "" : machine;
        faultReason = faultReason == null ? "" : faultReason;
        blocker = blocker == null ? "" : blocker;
        blockerDetail = blockerDetail == null ? "" : blockerDetail;
        summary = summary == null ? "" : summary;
    }
}
