package frc.lib.catalyst.statemachine;

import java.util.List;

/**
 * The engine's only output channel.
 *
 * <p>Change detection happens in {@link StateMachineCore}, not here, so every sink — the shipped
 * NetworkTables/WPILOG one, a test recorder, a future AdvantageKit bridge — receives an identical
 * already-diffed stream. Sinks are dumb on purpose: a sink that decided for itself when to write
 * would make the log cadence depend on which sink was installed, and cadence bugs are the reason
 * dashboards fall over at competition.
 *
 * @param <S> the machine's state enum
 * @since 1.2.0
 */
public interface StateMachineTelemetry<S extends Enum<S>> {

    /** Once at build: the static shape of the machine. */
    void graph(String machine, String[] states, String[] edges, String[] bindings,
               String[] routes, String dot, List<String> errors, List<String> warnings);

    /** Only when one of these changed. */
    void state(S current, boolean confirmed, S target, S nextHop, Phase phase, int stageIndex);

    /** Only when one of these changed. {@code blockerDetail} is already throttled by the core. */
    void blocker(String blocker, String blockerDetail, String summary, List<String> waitingOn);

    /** Every tick while transitioning, plus once more on completion. */
    void progress(long seq, double fraction, double elapsedSeconds, double timeoutSeconds,
                  String route, String trigger);

    /** Per binding. Scalars every tick while transitioning; strings and booleans on change. */
    void binding(BindingSample sample);

    /** Once when a transition ends for any reason, including rejection. */
    void transition(TransitionRecord<S> record);

    /** The newest-first serialized ring, rewritten whenever it changes. */
    void history(String[] newestFirst);

    /** States that can be requested right now, when the set changes. */
    void legalTargets(String[] targets);

    /** Only when one changed. */
    void counters(long transitions, long rejections, long timeouts, long aborts, long yields);

    /** Heartbeat, every tick. A machine that has stopped stepping is otherwise invisible. */
    void heartbeat(long ticks, double uptimeSeconds, boolean enabled);

    /** On change. */
    void fault(boolean faulted, String reason);

    /** A sink that discards everything. The default, so the core never depends on a logging stack. */
    static <S extends Enum<S>> StateMachineTelemetry<S> noop() {
        return new StateMachineTelemetry<S>() {
            @Override public void graph(String m, String[] s, String[] e, String[] b,
                                        String[] r, String d, List<String> er, List<String> w) {}
            @Override public void state(S c, boolean cf, S t, S n, Phase p, int si) {}
            @Override public void blocker(String b, String bd, String s, List<String> w) {}
            @Override public void progress(long q, double f, double el, double to, String r, String t) {}
            @Override public void binding(BindingSample sample) {}
            @Override public void transition(TransitionRecord<S> record) {}
            @Override public void history(String[] newestFirst) {}
            @Override public void legalTargets(String[] targets) {}
            @Override public void counters(long t, long r, long to, long a, long y) {}
            @Override public void heartbeat(long ticks, double uptime, boolean enabled) {}
            @Override public void fault(boolean faulted, String reason) {}
        };
    }

    /** Fan out to several sinks. A sink that throws does not stop the others. */
    @SafeVarargs
    static <S extends Enum<S>> StateMachineTelemetry<S> multi(StateMachineTelemetry<S>... sinks) {
        final List<StateMachineTelemetry<S>> all = List.of(sinks);
        return new StateMachineTelemetry<S>() {
            @Override public void graph(String m, String[] s, String[] e, String[] b,
                                        String[] r, String d, List<String> er, List<String> w) {
                for (var k : all) k.graph(m, s, e, b, r, d, er, w);
            }
            @Override public void state(S c, boolean cf, S t, S n, Phase p, int si) {
                for (var k : all) k.state(c, cf, t, n, p, si);
            }
            @Override public void blocker(String b, String bd, String s, List<String> w) {
                for (var k : all) k.blocker(b, bd, s, w);
            }
            @Override public void progress(long q, double f, double el, double to, String r, String t) {
                for (var k : all) k.progress(q, f, el, to, r, t);
            }
            @Override public void binding(BindingSample sample) {
                for (var k : all) k.binding(sample);
            }
            @Override public void transition(TransitionRecord<S> record) {
                for (var k : all) k.transition(record);
            }
            @Override public void history(String[] newestFirst) {
                for (var k : all) k.history(newestFirst);
            }
            @Override public void legalTargets(String[] targets) {
                for (var k : all) k.legalTargets(targets);
            }
            @Override public void counters(long t, long r, long to, long a, long y) {
                for (var k : all) k.counters(t, r, to, a, y);
            }
            @Override public void heartbeat(long ticks, double uptime, boolean enabled) {
                for (var k : all) k.heartbeat(ticks, uptime, enabled);
            }
            @Override public void fault(boolean faulted, String reason) {
                for (var k : all) k.fault(faulted, reason);
            }
        };
    }
}
