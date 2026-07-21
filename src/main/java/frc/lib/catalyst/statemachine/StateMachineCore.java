package frc.lib.catalyst.statemachine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

/**
 * The state machine engine: a real, guarded, logged finite state machine over an enum of states
 * and an arbitrary set of {@link Binding}s.
 *
 * <p>This class imports <b>nothing</b> from WPILib. It has no {@code Command}, no {@code Subsystem},
 * no {@code Trigger}, no {@code Timer}, no NetworkTables — it is driven by {@link #step()} and told
 * the time by a {@link DoubleSupplier}. That is what lets the whole of the machine's logic be
 * unit-tested on a laptop with no HAL, and it is also what will make the eventual port to the 2027
 * command framework a change to the thin robot-side adapter rather than to the engine.
 *
 * <p>For robot code, use {@link frc.lib.catalyst.statemachine.robot.Superstructure}, which wraps
 * this in a {@code SubsystemBase}, steps it once per loop, wires the logging, and exposes
 * {@code Command} and {@code Trigger} factories.
 *
 * <h2>The invariant that matters</h2>
 *
 * <p>{@link #current()} is <b>only ever</b> a state whose arrival was proven by every gating
 * binding reporting {@link Binding#atGoal}. A timeout, an abort, an interrupt or a fault sets
 * {@link #stateConfirmed()} to {@code false} and leaves {@code current()} at the last
 * <em>proven</em> value. The machine never claims to be somewhere it merely tried to go.
 *
 * <p>That sounds obvious and is the single most common defect in hand-rolled superstructure code —
 * including the one this package replaces, where an interrupted transition set the current state to
 * its target, so the <em>next</em> transition planned its route from a state the robot was not in.
 *
 * @param <S> the enum of states
 * @since 1.2.0
 */
public final class StateMachineCore<S extends Enum<S>> {

    // ------------------------------------------------------------------
    // Immutable configuration, fixed at build()
    // ------------------------------------------------------------------

    private final Class<S> stateType;
    private final String machineName;
    private final S[] constants;
    private final List<Bound> bindings;
    private final Map<Handle<?>, Bound> byHandle;
    private final EnumMap<S, Map<Handle<?>, Object>> stateGoals;
    private final EnumMap<S, Set<Handle<?>>> stateReleased;
    private final EnumMap<S, Set<Handle<?>>> stateGating;
    private final EnumMap<S, StateSpec<S>> specs;
    private final Map<String, EdgeSpec<S>> edgeSpecs;
    private final List<Interlock<S>> interlocks;
    private final StateGraph<S> graph;
    private final DoubleSupplier clock;
    private final StateMachineTelemetry<S> telemetry;
    private final double defaultTimeout;
    private final Routing routing;
    private final FaultPolicy defaultFaultPolicy;
    private final boolean strict;
    private final boolean abortOnOverride;
    private final Map<String, List<S>> pinnedRoutes;

    // ------------------------------------------------------------------
    // Mutable runtime belief
    // ------------------------------------------------------------------

    private S current;
    private boolean stateConfirmed;
    /**
     * Whether the machine has <em>ever</em> confirmed a state. Distinct from
     * {@link #stateConfirmed}, which goes false after any timeout or abort: a machine that was
     * confirmed at MID and then timed out on the way to HIGH still knows perfectly well that its
     * last proven state was MID, and must stay operable. Only a machine that has never been
     * seeded genuinely does not know where to route from.
     */
    private boolean everConfirmed;
    /** Origin of the in-flight transition, so its record reports where it came from. */
    private S transitionOrigin;
    /**
     * When the whole transition began, as opposed to {@link #hopStartSeconds}, which is reset per hop
     * because it doubles as the per-hop deadline base. A multi-hop record must span the whole
     * transition, not just its final leg.
     */
    private double transitionStartSeconds;
    private S requestedTarget;
    private List<S> route = List.of();
    private int routeIndex;
    private Phase phase = Phase.IDLE;
    private long seq;
    private long rejectPhaseTick = Long.MIN_VALUE;
    private long rejectPulseTick = Long.MIN_VALUE;
    /**
     * When a transition ends abnormally, the hop it was pursuing. While this is set, bindings keep
     * the goals they already had rather than snapping back to the origin state — which is what
     * {@link FaultPolicy#HOLD_AND_REPORT} means, and what stops a timed-out elevator from being
     * commanded back down through whatever it was stuck on.
     */
    private S holdGoalsFrom;
    private double hopStartSeconds;
    private double settledSinceSeconds = -1.0;
    private int stageIndex = -1;
    private List<List<Handle<?>>> activeStages = List.of();
    private String activeTrigger = "";
    private boolean faulted;
    private String faultReason = "";
    private boolean recovering;
    private boolean releasedByFault;
    private boolean enabled = true;
    private double disabledSinceSeconds = Double.NaN;
    private long ticks;
    private final double builtAtSeconds;

    private final TransitionHistory<S> history;
    private TransitionResult<S> lastResult;
    private long transitionCount;
    private long rejectionCount;
    private long timeoutCount;
    private long abortCount;
    private long yieldCount;

    // Change-detection shadows, so sinks receive an already-diffed stream.
    private S lastPubState;
    private boolean lastPubConfirmed;
    private S lastPubTarget;
    private S lastPubHop;
    private Phase lastPubPhase;
    private int lastPubStage = Integer.MIN_VALUE;
    private String lastPubBlocker;
    private String lastPubSummary;
    private String lastPubDetail;
    private double lastDetailPublishSeconds = Double.NEGATIVE_INFINITY;
    private String lastPubLegal;
    private boolean lastPubFaulted;
    private String lastPubFaultReason;
    private long[] lastPubCounters = {-1, -1, -1, -1, -1};

    private static final double DETAIL_PERIOD_SECONDS = 0.2;

    // ------------------------------------------------------------------

    private StateMachineCore(Builder<S> b) {
        this.stateType = b.stateType;
        this.machineName = b.machineName;
        this.constants = b.stateType.getEnumConstants();
        this.bindings = List.copyOf(b.bindings);
        this.byHandle = Map.copyOf(b.byHandle);
        this.specs = b.specs;
        this.edgeSpecs = Map.copyOf(b.edgeSpecs);
        this.interlocks = List.copyOf(b.interlocks);
        this.clock = b.clock;
        this.telemetry = b.telemetry;
        this.defaultTimeout = b.defaultTimeout;
        this.routing = b.routing;
        this.defaultFaultPolicy = b.defaultFaultPolicy;
        this.strict = b.strict;
        this.abortOnOverride = b.abortOnOverride;
        this.pinnedRoutes = Map.copyOf(b.pinnedRoutes);
        this.history = new TransitionHistory<>(b.historyCapacity);
        this.graph = b.buildGraph();

        this.stateGoals = new EnumMap<>(stateType);
        this.stateReleased = new EnumMap<>(stateType);
        this.stateGating = new EnumMap<>(stateType);
        for (S s : constants) {
            StateSpec<S> spec = specs.get(s);
            Map<Handle<?>, Object> goals = new HashMap<>();
            Set<Handle<?>> released = new HashSet<>();
            Set<Handle<?>> nonGating = new HashSet<>();
            if (b.defaults != null) {
                goals.putAll(b.defaults.goals);
                released.addAll(b.defaults.released);
                nonGating.addAll(b.defaults.nonGating);
            }
            if (spec != null) {
                goals.putAll(spec.goals);
                for (Handle<?> h : spec.goals.keySet()) released.remove(h);
                released.addAll(spec.released);
                for (Handle<?> h : spec.released) goals.remove(h);
                nonGating.addAll(spec.nonGating);
            }
            Set<Handle<?>> gating = new HashSet<>(goals.keySet());
            gating.removeAll(nonGating);
            gating.removeIf(h -> byHandle.get(h).advisory);
            stateGoals.put(s, Map.copyOf(goals));
            stateReleased.put(s, Set.copyOf(released));
            stateGating.put(s, Set.copyOf(gating));
        }

        this.current = b.initialState;
        this.requestedTarget = b.initialState;
        this.stateConfirmed = false;
        this.builtAtSeconds = clock.getAsDouble();
        this.hopStartSeconds = builtAtSeconds;
        this.lastResult = TransitionResult.accept(0L, current, current, List.of());

        publishGraph(b.lastValidation);
    }

    /** Start building a machine over {@code stateType}, named {@code machineName} in the logs. */
    public static <S extends Enum<S>> Builder<S> builder(Class<S> stateType, String machineName) {
        return new Builder<>(stateType, machineName);
    }

    // ==================================================================
    // Belief and truth
    // ==================================================================

    /**
     * The last state whose every gating binding was proven to be at goal.
     *
     * <p>Never speculative. After a timeout or an abort this stays at the last <em>proven</em>
     * state and {@link #stateConfirmed()} goes false.
     */
    public S current() {
        return current;
    }

    /** False after a timeout, abort, interrupt or fault, and before the first {@link #seed}. */
    public boolean stateConfirmed() {
        return stateConfirmed;
    }

    /** The requested destination. Equals {@link #current()} when idle. */
    public S target() {
        return requestedTarget;
    }

    /** The hop currently being executed — equals {@link #target()} for a direct transition. */
    public S nextHop() {
        if (route.isEmpty() || routeIndex >= route.size()) return requestedTarget;
        return route.get(routeIndex);
    }

    /** What the machine is doing right now. */
    public Phase phase() {
        return phase;
    }

    /** True while a transition is in flight. */
    public boolean isTransitioning() {
        return phase == Phase.MOVING || phase == Phase.SETTLING;
    }

    /** True while a fault is latched. Cleared by {@link #clearFault()}. */
    public boolean isFaulted() {
        return faulted;
    }

    /** Fault text, or {@code ""} when healthy. */
    public String faultReason() {
        return faultReason;
    }

    /**
     * Is every gating binding of {@code state} at that state's goal <b>right now</b>?
     *
     * <p>A measurement, not a latch. Safe to call for a state the machine is not in and has never
     * been in — the goals are evaluated fresh against live sensors, and elapsed-time arrival tests
     * see zero elapsed time unless that exact goal happens to be the one currently applied. That
     * is what stops a settle-timer binding from reporting "arrived" for every state at once.
     */
    public boolean isAt(S state) {
        Set<Handle<?>> gating = stateGating.get(state);
        if (gating == null || gating.isEmpty()) return true;
        Map<Handle<?>, Object> goals = stateGoals.get(state);
        for (Handle<?> h : gating) {
            Object goal = goals.get(h);
            if (goal == null) continue;
            Bound bound = byHandle.get(h);
            if (!bound.atGoal(goal, clock.getAsDouble())) return false;
        }
        return true;
    }

    /** {@link #current()} is {@code state}, the belief is confirmed, and it still measures true. */
    public boolean isSettledAt(S state) {
        return current == state && stateConfirmed && isAt(state);
    }

    // ==================================================================
    // Diagnosis
    // ==================================================================

    /**
     * A stable, low-cardinality one-liner naming what is holding the machine up:
     * {@code "waiting:elevator,arm"}, {@code "guard:endgame"}, {@code "yielded:elevator"},
     * {@code "reject:NO_EDGE"}, {@code "fault"}, or {@code ""}.
     *
     * <p>Deliberately carries no live numbers, so it can be logged on change instead of every
     * loop. The numbers live in {@link #blockerDetail()}.
     */
    public String blocker() {
        if (faulted) return "fault";
        if (rejectPulsing() && lastResult != null && lastResult.rejected()) {
            return "reject:" + lastResult.reason();
        }
        List<String> yielded = new ArrayList<>();
        for (Bound b : bindings) {
            if (!b.advisory && !b.owned && b.appliedGoal != null) yielded.add(b.key);
        }
        if (!yielded.isEmpty()) {
            Collections.sort(yielded);
            return "yielded:" + String.join(",", yielded);
        }
        List<String> waiting = waitingOn();
        if (!waiting.isEmpty()) return "waiting:" + String.join(",", waiting);
        return "";
    }

    /**
     * The same diagnosis with live numbers — {@code "elevator err 0.520 m > 0.020 m"} — plus any
     * {@link Binding#note} the blocking mechanisms have to offer.
     *
     * <p>High cardinality by design, so the telemetry layer throttles it to 5 Hz.
     */
    public String blockerDetail() {
        if (faulted) return faultReason;
        if (rejectPulsing() && lastResult != null && lastResult.rejected()) {
            return lastResult.toString();
        }
        S ref = referenceState();
        Map<Handle<?>, Object> goals = stateGoals.get(ref);
        Set<Handle<?>> gating = stateGating.get(ref);
        if (goals == null || gating == null) return "";
        List<String> parts = new ArrayList<>();
        double now = clock.getAsDouble();
        for (Handle<?> h : sorted(gating)) {
            Object goal = goals.get(h);
            if (goal == null) continue;
            Bound b = byHandle.get(h);
            if (b.atGoal(goal, now)) continue;
            parts.add(b.describeShortfall(goal));
        }
        return String.join("; ", parts);
    }

    /**
     * One human-readable line covering everything:
     * {@code "MOVING STOW->AIM stage 1/3 2.1/4.0s waiting elevator"}.
     */
    public String summary() {
        StringBuilder sb = new StringBuilder();
        sb.append(phase);
        if (!stateConfirmed) sb.append('?');
        sb.append(' ').append(current == null ? "?" : current.name());
        if (isTransitioning()) {
            sb.append("->").append(nextHop().name());
            if (nextHop() != requestedTarget) sb.append(" (of ").append(requestedTarget.name()).append(')');
            if (stageIndex >= 0 && !activeStages.isEmpty()) {
                sb.append(" stage ").append(Math.min(stageIndex + 1, activeStages.size()))
                  .append('/').append(activeStages.size());
            }
            sb.append(String.format(" %.1f/%.1fs", elapsedSeconds(), timeoutSeconds()));
        }
        String b = blocker();
        if (!b.isEmpty()) sb.append(' ').append(b);
        return sb.toString();
    }

    /** Keys of gating bindings not yet at goal, sorted. Empty when settled. */
    public List<String> waitingOn() {
        S ref = referenceState();
        Map<Handle<?>, Object> goals = stateGoals.get(ref);
        Set<Handle<?>> gating = stateGating.get(ref);
        if (goals == null || gating == null) return List.of();
        List<String> out = new ArrayList<>();
        double now = clock.getAsDouble();
        for (Handle<?> h : gating) {
            Object goal = goals.get(h);
            if (goal == null) continue;
            Bound b = byHandle.get(h);
            if (!b.atGoal(goal, now)) out.add(b.key);
        }
        Collections.sort(out);
        return out;
    }

    /** Fraction of the active hop's gating bindings that are at goal, in {@code [0,1]}. */
    public double progress() {
        S ref = referenceState();
        Set<Handle<?>> gating = stateGating.get(ref);
        Map<Handle<?>, Object> goals = stateGoals.get(ref);
        if (gating == null || gating.isEmpty()) return 1.0;
        int total = 0;
        int arrived = 0;
        double now = clock.getAsDouble();
        for (Handle<?> h : gating) {
            Object goal = goals.get(h);
            if (goal == null) continue;
            total++;
            if (byHandle.get(h).atGoal(goal, now)) arrived++;
        }
        return total == 0 ? 1.0 : (double) arrived / total;
    }

    /** Seconds spent on the current hop. */
    public double elapsedSeconds() {
        return clock.getAsDouble() - hopStartSeconds;
    }

    /** The deadline in force for the current hop. Always finite. */
    public double timeoutSeconds() {
        return effectiveTimeout(current, nextHop());
    }

    /** Active stage of a staged edge, or {@code -1} when the edge is unstaged. */
    public int stageIndex() {
        return activeStages.isEmpty() ? -1 : stageIndex;
    }

    /** Number of stages on the active edge, or {@code 0}. */
    public int stageCount() {
        return activeStages.size();
    }

    /** Step counter — the heartbeat. A machine that has stopped stepping is otherwise invisible. */
    public long ticks() {
        return ticks;
    }

    /** Seconds since this machine was built. */
    public double uptimeSeconds() {
        return clock.getAsDouble() - builtAtSeconds;
    }

    // ==================================================================
    // Graph
    // ==================================================================

    /** The state enum. */
    public Class<S> stateType() {
        return stateType;
    }

    /** Machine name, used as the log prefix and in messages. */
    public String name() {
        return machineName;
    }

    /** The legal-transition graph. */
    public StateGraph<S> graph() {
        return graph;
    }

    /** States that would be accepted right now, with guards and interlocks applied. */
    public EnumSet<S> legalTargets() {
        EnumSet<S> out = EnumSet.noneOf(stateType);
        for (S s : constants) {
            if (s == current) continue;
            if (evaluate(s) == RejectReason.NONE) out.add(s);
        }
        return out;
    }

    /** Would a request for {@code target} be accepted right now? */
    public boolean canReach(S target) {
        return evaluate(target) == RejectReason.NONE;
    }

    /**
     * The route a request for {@code target} would take, with no side effects at all.
     *
     * <p>Automatic routing is only defensible if the chosen path is visible before it runs; this
     * is how a team checks that "press Y from anywhere" will not take the arm through the chassis.
     */
    public Route<S> plan(S target) {
        return planRoute(current, target);
    }

    /** Every bound handle, in bind order. */
    public List<Handle<?>> handles() {
        List<Handle<?>> out = new ArrayList<>(bindings.size());
        for (Bound b : bindings) out.add(b.handle);
        return out;
    }

    /** The goal assigned to {@code handle} in {@code state}, or {@code null} if released there. */
    @SuppressWarnings("unchecked")
    public <G> G goalOf(S state, Handle<G> handle) {
        Map<Handle<?>, Object> goals = stateGoals.get(state);
        return goals == null ? null : (G) goals.get(handle);
    }

    /**
     * The goal this binding should be pursuing right now, or {@code null} when it is released or
     * is being held back by a later actuation stage.
     *
     * <p>This is the single method {@code GoalRunner} consults, which is what keeps actuation
     * policy in the engine rather than smeared across the robot layer.
     */
    @SuppressWarnings("unchecked")
    public <G> G activeGoalOf(Handle<G> handle) {
        if (!enabled) return null;
        Bound b = byHandle.get(handle);
        if (b == null) return null;
        if (releasedByFault) return null;

        S ref;
        if (isTransitioning()) {
            ref = nextHop();
            if (!activeStages.isEmpty()) {
                int stage = stageOf(handle);
                // A handle in a later stage stays parked at the state we departed from.
                if (stage > stageIndex) ref = current;
            }
        } else {
            ref = holdGoalsFrom != null ? holdGoalsFrom : current;
        }
        if (stateReleased.get(ref).contains(handle)) return null;
        return (G) stateGoals.get(ref).get(handle);
    }

    // ==================================================================
    // Control
    // ==================================================================

    /** Request a transition, attributing it to {@code "code"}. */
    public TransitionResult<S> request(S target) {
        return request(target, "code");
    }

    /**
     * Request a transition to {@code target}.
     *
     * <p>A pure decision function: it never builds a command, never moves anything by itself, and
     * <b>never throws</b>. Every rejection is counted, logged with a reason, and reflected in
     * {@link #blocker()} for one loop.
     *
     * @param triggerSource free text recorded with the transition — {@code "op.y"},
     *                      {@code "auto:leftThree"} — so the history says who asked
     */
    public TransitionResult<S> request(S target, String triggerSource) {
        RejectReason reason = evaluate(target);
        if (reason != RejectReason.NONE) {
            return refuse(target, reason, describeRejection(target, reason));
        }

        Route<S> planned = planRoute(current, target);
        if (planned.isEmpty()) {
            RejectReason r = routing == Routing.DIRECT_ONLY ? RejectReason.NO_EDGE : RejectReason.NO_ROUTE;
            return refuse(target, r, describeRejection(target, r));
        }

        if (isTransitioning()) {
            finishTransition(TransitionRecord.Outcome.SUPERSEDED,
                    "superseded by request for " + target.name(), false);
        } else if (stateConfirmed) {
            runAll(specs.get(current) == null ? List.of() : specs.get(current).onExit);
        }

        seq++;
        holdGoalsFrom = null;
        transitionOrigin = current;
        transitionStartSeconds = clock.getAsDouble();
        requestedTarget = target;
        route = planned.hops();
        routeIndex = 0;
        activeTrigger = triggerSource == null ? "" : triggerSource;
        phase = Phase.MOVING;
        hopStartSeconds = clock.getAsDouble();
        settledSinceSeconds = -1.0;
        loadStages(current, nextHop());
        runAll(edgeOnTransit(current, nextHop()));

        lastResult = TransitionResult.accept(seq, current, target, route);
        return lastResult;
    }

    /**
     * Request {@code target} ignoring multi-hop routing — the edge must exist directly.
     * Guards, interlocks and entry guards still apply.
     */
    public TransitionResult<S> requestDirect(S target, String triggerSource) {
        if (!graph.hasEdge(current, target)) {
            return refuse(target, RejectReason.NO_EDGE,
                    "no direct edge " + current.name() + "->" + target.name());
        }
        RejectReason reason = evaluate(target);
        if (reason != RejectReason.NONE) {
            return refuse(target, reason, describeRejection(target, reason));
        }
        if (isTransitioning()) {
            finishTransition(TransitionRecord.Outcome.SUPERSEDED,
                    "superseded by direct request for " + target.name(), false);
        } else if (stateConfirmed) {
            runAll(specs.get(current) == null ? List.of() : specs.get(current).onExit);
        }
        seq++;
        holdGoalsFrom = null;
        transitionOrigin = current;
        transitionStartSeconds = clock.getAsDouble();
        requestedTarget = target;
        route = List.of(target);
        routeIndex = 0;
        activeTrigger = triggerSource == null ? "" : triggerSource;
        phase = Phase.MOVING;
        hopStartSeconds = clock.getAsDouble();
        settledSinceSeconds = -1.0;
        loadStages(current, target);
        runAll(edgeOnTransit(current, target));
        lastResult = TransitionResult.accept(seq, current, target, route);
        return lastResult;
    }

    /** Cancel any in-flight transition. Leaves {@link #current()} at the last proven state. */
    public void abort(String reason) {
        if (!isTransitioning()) return;
        abortCount++;
        stateConfirmed = false;
        finishTransition(TransitionRecord.Outcome.ABORTED, reason == null ? "aborted" : reason, true);
    }

    /** Clear a latched fault so the machine will accept requests again. */
    public void clearFault() {
        faulted = false;
        faultReason = "";
        recovering = false;
        releasedByFault = false;
        if (phase == Phase.FAULTED) phase = stateConfirmed ? Phase.HOLDING : Phase.IDLE;
    }

    /**
     * Tell the machine where it is, without moving anything.
     *
     * <p>Call this once at robot init for the state the robot is physically built into — usually
     * {@code STOW}. Until it is called, the machine does not know its origin and refuses requests
     * with {@link RejectReason#NOT_SEEDED} rather than guessing and routing from the wrong place.
     */
    public void seed(S assumedState) {
        if (isTransitioning()) return;
        double now = clock.getAsDouble();
        current = assumedState;
        requestedTarget = assumedState;
        holdGoalsFrom = null;
        stateConfirmed = true;
        everConfirmed = true;
        phase = Phase.HOLDING;
        route = List.of();
        routeIndex = 0;
        hopStartSeconds = now;
        history.record(new TransitionRecord<>(seq, now, now, 0.0, assumedState, assumedState,
                List.of(), "seed", TransitionRecord.Outcome.SEEDED, RejectReason.NONE,
                "seeded without motion", List.of()));
        telemetry.history(history.serialized());
    }

    /** Enable or disable actuation. While disabled, deadlines are frozen and no goals are issued. */
    public void setEnabled(boolean enabled) {
        if (this.enabled == enabled) return;
        double now = clock.getAsDouble();
        if (!enabled) {
            disabledSinceSeconds = now;
        } else if (!Double.isNaN(disabledSinceSeconds)) {
            // Deadlines must not run while the robot cannot move, or every in-flight transition
            // would report a timeout the instant it is re-enabled.
            hopStartSeconds += now - disabledSinceSeconds;
            disabledSinceSeconds = Double.NaN;
        }
        this.enabled = enabled;
        if (!enabled) phase = Phase.DISABLED;
        else if (phase == Phase.DISABLED) phase = stateConfirmed ? Phase.HOLDING : Phase.IDLE;
    }

    /** Whether actuation is enabled. */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Advance one loop: evaluate arrival, advance actuation stages, enforce deadlines, publish.
     *
     * <p>Called once per scheduler loop by {@code Superstructure.periodic()}, or by a test loop
     * with a fake clock.
     */
    public void step() {
        ticks++;
        double now = clock.getAsDouble();

        if (phase == Phase.REJECTED && ticks > rejectPhaseTick + 1) {
            phase = stateConfirmed ? Phase.HOLDING : Phase.IDLE;
        }

        syncGoalApplication(now);
        publishBindings(now);

        if (!enabled) {
            phase = Phase.DISABLED;
            publishTimeline(now);
            return;
        }

        if (abortOnOverride && isTransitioning() && anyNonAdvisoryYielded()) {
            abort("mechanism ownership lost");
        }

        if (isTransitioning()) {
            advanceTransition(now);
        }

        publishTimeline(now);
    }

    /** Called by {@code GoalRunner} when a mechanism is taken over by, or handed back from, another command. */
    public void noteOwned(String bindingKey, boolean owned) {
        for (Bound b : bindings) {
            if (b.key.equals(bindingKey)) {
                if (b.owned && !owned) yieldCount++;
                b.owned = owned;
                return;
            }
        }
    }

    // ==================================================================
    // Observability
    // ==================================================================

    /** Newest-first transition history. */
    public List<TransitionRecord<S>> history() {
        return history.snapshot();
    }

    /** The transition history ring itself, for capacity changes. */
    public TransitionHistory<S> historyBuffer() {
        return history;
    }

    /** The newest transition record, or {@code null}. */
    public TransitionRecord<S> lastTransition() {
        return history.newest();
    }

    /** The result of the most recent request. */
    public TransitionResult<S> lastResult() {
        return lastResult;
    }

    /** The sequence number of the in-flight transition, or {@code -1} when idle. */
    public long activeSeq() {
        return isTransitioning() ? seq : -1L;
    }

    /**
     * The sequence number of the most recently <b>accepted</b> request.
     *
     * <p>Distinct from {@code lastResult().seq()}, which a rejection overwrites with {@code -1}: a
     * command that started an accepted transition must be able to tell "a newer transition superseded
     * me" (a real supersession) apart from "some unrelated request was refused" (which leaves this
     * command's transition running). Only the former changes this value.
     */
    public long lastAcceptedSeq() {
        return seq;
    }

    /** A coherent picture of everything, in one immutable value. */
    public Snapshot<S> snapshot() {
        double now = clock.getAsDouble();
        List<BindingSample> samples = new ArrayList<>(bindings.size());
        for (Bound b : bindings) samples.add(b.sample(now, isGatingNow(b)));
        return new Snapshot<>(machineName, current, stateConfirmed, requestedTarget, nextHop(),
                phase, faulted, faultReason, blocker(), blockerDetail(), summary(),
                progress(), elapsedSeconds(), timeoutSeconds(), stageIndex(), stageCount(),
                waitingOn(), samples, history.snapshot(), ticks, uptimeSeconds(),
                transitionCount, rejectionCount, timeoutCount, abortCount, yieldCount);
    }

    /** One binding's live sample, or {@code null} for an unknown key. */
    public BindingSample sample(String bindingKey) {
        for (Bound b : bindings) {
            if (b.key.equals(bindingKey)) return b.sample(clock.getAsDouble(), isGatingNow(b));
        }
        return null;
    }

    /** Completed transitions. */
    public long transitionCount() { return transitionCount; }
    /** Refused requests. */
    public long rejectionCount() { return rejectionCount; }
    /** Blown deadlines. */
    public long timeoutCount() { return timeoutCount; }
    /** Aborts. */
    public long abortCount() { return abortCount; }
    /** Times a bound mechanism was taken over by another command. */
    public long yieldCount() { return yieldCount; }

    // ------------------------------------------------------------------
    // Condition suppliers. Trigger construction needs the HAL, so the robot layer wraps these.
    // ------------------------------------------------------------------

    /** True while the machine believes it is in {@code state}. */
    public BooleanSupplier inState(S state) {
        return () -> current == state;
    }

    /** True while {@link #isSettledAt(Enum)} holds. */
    public BooleanSupplier settledIn(S state) {
        return () -> isSettledAt(state);
    }

    /** True while a transition is in flight. */
    public BooleanSupplier transitioning() {
        return this::isTransitioning;
    }

    /** True while a fault is latched. */
    public BooleanSupplier faultedSupplier() {
        return this::isFaulted;
    }

    /** Pulses true for exactly one loop after each refused request. */
    public BooleanSupplier rejectedSupplier() {
        return this::rejectPulsing;
    }

    private boolean rejectPulsing() {
        return rejectPulseTick != Long.MIN_VALUE && ticks <= rejectPulseTick + 1;
    }

    /** True while any non-advisory bound mechanism is held by another command. */
    public BooleanSupplier overridden() {
        return this::anyNonAdvisoryYielded;
    }

    // ==================================================================
    // Internals
    // ==================================================================

    private RejectReason evaluate(S target) {
        if (target == null) return RejectReason.UNKNOWN_STATE;
        if (!enabled) return RejectReason.DISABLED;
        if (faulted) return RejectReason.FAULTED;
        if (!everConfirmed && !isRecoveryState(target)) return RejectReason.NOT_SEEDED;
        if (isTransitioning() && target == requestedTarget) return RejectReason.ALREADY_THERE;

        // Re-seeking the state we are already PROVEN to be in is a no-op; re-seeking a state whose
        // occupancy is unconfirmed (after a timeout or abort left us holding a failed target's goals)
        // is a legal, necessary recovery — it is the only way to command the machine back to a safe
        // state it can actually reach. Without this, request(current) below would fall through to
        // NO_EDGE (there is no self-edge) and the machine could never be told "go back where you are".
        if (target == current) {
            return stateConfirmed ? RejectReason.ALREADY_THERE : RejectReason.NONE;
        }

        for (Interlock<S> lock : interlocks) {
            if (blockedByInterlock(lock, target)) return RejectReason.INTERLOCK_BLOCKED;
        }

        StateSpec<S> spec = specs.get(target);
        if (spec != null && spec.entryGuard != null && !safeGuard(spec.entryGuard, false)) {
            // Fail closed: a guard that throws is treated as blocking, never as an escape that takes
            // the robot loop down with it.
            return RejectReason.ENTRY_GUARD_BLOCKED;
        }

        Set<Handle<?>> gating = stateGating.get(target);
        if (gating != null) {
            for (Handle<?> h : gating) {
                Bound b = byHandle.get(h);
                if (!safeZeroed(b)) return RejectReason.NOT_ZEROED;
            }
        }

        if (routing == Routing.DIRECT_ONLY) {
            if (!graph.hasEdge(current, target)) return RejectReason.NO_EDGE;
            if (!edgePassable(current, target)) return RejectReason.GUARD_BLOCKED;
            return RejectReason.NONE;
        }
        return planRoute(current, target).isEmpty() ? RejectReason.NO_ROUTE : RejectReason.NONE;
    }

    /**
     * Evaluate a user {@link BooleanSupplier} without ever letting it take down the scheduler.
     *
     * <p>A guard that throws — a CANcoder that dropped off the bus, a closure over a nulled field —
     * must not propagate out of {@link #step()} and kill the robot loop. It is treated as returning
     * {@code fallbackWhenThrowing}, chosen at each call site so a throwing guard fails <em>closed</em>
     * (blocks the transition) rather than open.
     */
    private boolean safeGuard(BooleanSupplier supplier, boolean fallbackWhenThrowing) {
        try {
            return supplier.getAsBoolean();
        } catch (RuntimeException ex) {
            reportGuardFailure(ex);
            return fallbackWhenThrowing;
        }
    }

    private boolean safeZeroed(Bound b) {
        try {
            return b.binding.zeroed();
        } catch (RuntimeException ex) {
            reportGuardFailure(ex);
            return false;   // fail closed: an unknowable zero state is treated as not zeroed
        }
    }

    /** True when {@code lock} is unsatisfied and blocks {@code target}, with the supplier guarded. */
    private boolean blockedByInterlock(Interlock<S> lock, S target) {
        return !safeGuard(lock.satisfied, false) && lock.blocksState.test(target);
    }

    private void reportGuardFailure(RuntimeException ex) {
        System.err.println("[Catalyst] state machine '" + machineName + "' guard threw (treated as "
                + "blocking): " + ex);
    }

    private String describeRejection(S target, RejectReason reason) {
        switch (reason) {
            case INTERLOCK_BLOCKED:
                for (Interlock<S> lock : interlocks) {
                    if (blockedByInterlock(lock, target)) return lock.name;
                }
                return "interlock";
            case ENTRY_GUARD_BLOCKED: {
                StateSpec<S> spec = specs.get(target);
                return spec == null || spec.entryGuardReason.isEmpty() ? "entry guard" : spec.entryGuardReason;
            }
            case GUARD_BLOCKED: {
                EdgeSpec<S> e = edgeSpecs.get(edgeKey(current, target));
                return e == null || e.guardReason.isEmpty() ? "edge guard" : e.guardReason;
            }
            case NOT_ZEROED: {
                Set<Handle<?>> gating = stateGating.get(target);
                if (gating != null) {
                    for (Handle<?> h : gating) {
                        Bound b = byHandle.get(h);
                        if (!b.binding.zeroed()) return b.key + " has not been zeroed";
                    }
                }
                return "a mechanism has not been zeroed";
            }
            case NO_EDGE:
                return "no edge " + current.name() + "->" + target.name()
                        + "; declared successors of " + current.name() + " are " + graph.successors(current);
            case NO_ROUTE:
                return "no legal route from " + current.name() + " to " + target.name() + " right now";
            case NOT_SEEDED:
                return "the machine has never confirmed a state; call seed(state) at robot init";
            case DISABLED:
                return "robot disabled";
            case FAULTED:
                return "faulted: " + faultReason;
            case ALREADY_THERE:
                return "already transitioning to " + target.name();
            default:
                return reason.name();
        }
    }

    private TransitionResult<S> refuse(S target, RejectReason reason, String detail) {
        rejectionCount++;
        rejectPulseTick = ticks;
        // A refused request must never disturb a transition that is already in flight. Overwriting
        // the phase here would silently abandon the running transition without so much as a record.
        if (!isTransitioning()) {
            phase = Phase.REJECTED;
            rejectPhaseTick = ticks;
        }
        lastResult = TransitionResult.reject(current, target, reason, detail);
        double now = clock.getAsDouble();
        TransitionRecord<S> rec = new TransitionRecord<>(-1L, now, now, 0.0, current, target,
                List.of(), "request", TransitionRecord.Outcome.REJECTED, reason, detail, List.of());
        history.record(rec);
        telemetry.transition(rec);
        telemetry.history(history.serialized());
        return lastResult;
    }

    private Route<S> planRoute(S from, S to) {
        if (from == to) {
            // A confirmed self-request is genuinely a no-op; an unconfirmed one is a recovery re-seek
            // that must actually re-drive to `to` (see the target == current branch in evaluate()).
            return (from == current && !stateConfirmed) ? new Route<>(List.of(to)) : Route.empty();
        }
        List<S> pinned = pinnedRoutes.get(edgeKey(from, to));
        if (pinned != null) return new Route<>(pinned);
        if (routing == Routing.DIRECT_ONLY) {
            return graph.hasEdge(from, to) && edgePassable(from, to)
                    ? new Route<>(List.of(to)) : Route.<S>empty();
        }
        return graph.route(from, to, this::edgePassable);
    }

    private boolean edgePassable(S from, S to) {
        EdgeSpec<S> e = edgeSpecs.get(edgeKey(from, to));
        if (e != null && e.guard != null && !safeGuard(e.guard, false)) return false;
        for (Interlock<S> lock : interlocks) {
            if (blockedByInterlock(lock, to)) return false;
        }
        return true;
    }

    private void advanceTransition(double now) {
        S hop = nextHop();

        if (!activeStages.isEmpty() && stageIndex < activeStages.size()) {
            if (stageGatingArrived(stageIndex, hop, now)) stageIndex++;
        }

        boolean stagesDone = activeStages.isEmpty() || stageIndex >= activeStages.size();
        boolean arrived = stagesDone && isAt(hop);

        if (arrived) {
            StateSpec<S> spec = specs.get(hop);
            double settle = spec == null ? 0.0 : spec.settleSeconds;
            if (settle > 0.0) {
                if (settledSinceSeconds < 0) settledSinceSeconds = now;
                if (now - settledSinceSeconds < settle) {
                    phase = Phase.SETTLING;
                    return;
                }
            }
            // The one and only place `current` is written on the strength of a measurement.
            current = hop;
            stateConfirmed = true;
            everConfirmed = true;
            settledSinceSeconds = -1.0;
            if (spec != null) runAll(spec.onEnter);

            if (hop == requestedTarget) {
                transitionCount++;
                finishTransition(TransitionRecord.Outcome.ARRIVED, "", false);
            } else {
                routeIndex++;
                hopStartSeconds = now;
                stageIndex = 0;
                loadStages(current, nextHop());
                runAll(edgeOnTransit(current, nextHop()));
                phase = Phase.MOVING;
            }
            return;
        }

        settledSinceSeconds = -1.0;
        phase = Phase.MOVING;

        if (now - hopStartSeconds > effectiveTimeout(current, hop)) {
            timeoutCount++;
            stateConfirmed = false;
            String detail = blockerDetail();
            if (detail.isEmpty()) detail = "deadline expired with no gating binding reporting a shortfall";
            finishTransition(TransitionRecord.Outcome.TIMED_OUT, detail, false);

            if (strict) {
                applyFaultPolicy(hop, detail);
            } else {
                // Dry-run mode: the deadline still fires and is still recorded, but the machine
                // simply gives up on the transition instead of latching a fault.
                phase = Phase.TIMED_OUT;
            }
        }
    }

    private void applyFaultPolicy(S hop, String detail) {
        StateSpec<S> spec = specs.get(hop);
        FaultPolicy policy = spec != null && spec.faultPolicy != null ? spec.faultPolicy : defaultFaultPolicy;
        if (recovering) policy = FaultPolicy.HOLD_AND_REPORT;

        faulted = true;
        faultReason = detail;
        phase = Phase.FAULTED;

        if (policy == FaultPolicy.RELEASE_ALL) {
            releasedByFault = true;
        } else if (policy == FaultPolicy.RECOVER_TO && spec != null && spec.recoveryState != null) {
            S recovery = spec.recoveryState;
            recovering = true;
            faulted = false;
            faultReason = "";
            TransitionResult<S> r = request(recovery, "recover");
            faulted = true;
            faultReason = detail;
            if (r.rejected()) {
                phase = Phase.FAULTED;
            }
        }
    }

    private void finishTransition(TransitionRecord.Outcome outcome, String detail, boolean fromAbort) {
        double now = clock.getAsDouble();
        List<ArrivalReport> arrivals = new ArrayList<>();
        for (Bound b : bindings) {
            Object goal = b.appliedGoal;
            if (goal == null) continue;
            arrivals.add(b.arrivalReport(goal, now));
        }
        TransitionRecord<S> rec = new TransitionRecord<>(seq, transitionStartSeconds, now,
                now - transitionStartSeconds, transitionOrigin == null ? current : transitionOrigin,
                requestedTarget, route, activeTrigger,
                outcome, RejectReason.NONE, detail, arrivals);
        history.record(rec);
        telemetry.transition(rec);
        telemetry.history(history.serialized());

        S abandonedHop = nextHop();
        route = List.of();
        routeIndex = 0;
        activeStages = List.of();
        stageIndex = -1;
        settledSinceSeconds = -1.0;
        if (outcome == TransitionRecord.Outcome.ARRIVED) {
            holdGoalsFrom = null;
            phase = Phase.HOLDING;
            requestedTarget = current;
        } else if (outcome != TransitionRecord.Outcome.SUPERSEDED) {
            // Keep pursuing the goals we already had. Snapping back to the origin state's goals
            // would mean a timed-out or aborted transition actively drives the robot backwards,
            // through whatever it just failed to get past.
            holdGoalsFrom = abandonedHop;
            requestedTarget = current;
            if (fromAbort) phase = stateConfirmed ? Phase.HOLDING : Phase.IDLE;
        }
    }

    private void loadStages(S from, S to) {
        EdgeSpec<S> e = edgeSpecs.get(edgeKey(from, to));
        activeStages = e == null ? List.of() : List.copyOf(e.stages);
        stageIndex = activeStages.isEmpty() ? -1 : 0;
    }

    private List<Runnable> edgeOnTransit(S from, S to) {
        EdgeSpec<S> e = edgeSpecs.get(edgeKey(from, to));
        return e == null ? List.of() : e.onTransit;
    }

    private boolean stageGatingArrived(int stage, S hop, double now) {
        if (stage < 0 || stage >= activeStages.size()) return true;
        Map<Handle<?>, Object> goals = stateGoals.get(hop);
        Set<Handle<?>> gating = stateGating.get(hop);
        for (Handle<?> h : activeStages.get(stage)) {
            if (!gating.contains(h)) continue;
            Object goal = goals.get(h);
            if (goal == null) continue;
            if (!byHandle.get(h).atGoal(goal, now)) return false;
        }
        return true;
    }

    private int stageOf(Handle<?> handle) {
        for (int i = 0; i < activeStages.size(); i++) {
            if (activeStages.get(i).contains(handle)) return i;
        }
        return -1;
    }

    private double effectiveTimeout(S from, S to) {
        EdgeSpec<S> e = edgeSpecs.get(edgeKey(from, to));
        if (e != null && !Double.isNaN(e.timeoutSeconds) && e.timeoutSeconds > 0) return e.timeoutSeconds;
        StateSpec<S> spec = specs.get(to);
        if (spec != null && !Double.isNaN(spec.timeoutSeconds) && spec.timeoutSeconds > 0) {
            return spec.timeoutSeconds;
        }
        return defaultTimeout;
    }

    private boolean isRecoveryState(S target) {
        for (S s : constants) {
            StateSpec<S> spec = specs.get(s);
            if (spec != null && spec.recoveryState == target) return true;
        }
        return false;
    }

    private boolean anyNonAdvisoryYielded() {
        for (Bound b : bindings) {
            if (!b.advisory && !b.owned && b.appliedGoal != null) return true;
        }
        return false;
    }

    private boolean isGatingNow(Bound b) {
        Set<Handle<?>> gating = stateGating.get(referenceState());
        return gating != null && gating.contains(b.handle);
    }

    /**
     * The state whose goals are actually being pursued right now — the hop while transitioning,
     * otherwise the held state after a timeout or abort, otherwise the confirmed current state.
     *
     * <p>Every diagnostic ({@link #blockerDetail}, {@link #waitingOn}, {@link #progress}, the
     * per-binding samples) measures against this, so after a timeout they report against the goals
     * the mechanisms are truly holding — the same goals {@link #activeGoalOf} hands the runners —
     * rather than against the confirmed state the robot is no longer being driven toward.
     */
    private S referenceState() {
        if (isTransitioning()) return nextHop();
        return holdGoalsFrom != null ? holdGoalsFrom : current;
    }

    /**
     * Track, per binding, when the goal it should be pursuing last changed. The engine owns this
     * rather than the robot layer so that elapsed-time arrival tests work identically in a unit
     * test with a fake clock and on a real robot.
     */
    private void syncGoalApplication(double now) {
        for (Bound b : bindings) {
            Object want = activeGoalOf(b.handle);
            if (!Objects.equals(want, b.appliedGoal)) {
                b.appliedGoal = want;
                b.appliedAtSeconds = now;
                b.arrived = false;
                b.arrivalSeconds = Double.NaN;
            } else if (want != null && !b.arrived && b.atGoal(want, now)) {
                b.arrived = true;
                b.arrivalSeconds = now - b.appliedAtSeconds;
            } else if (want != null && b.arrived && !b.atGoal(want, now)) {
                b.arrived = false;
            }
        }
    }

    private static void runAll(List<Runnable> actions) {
        for (Runnable r : actions) {
            try {
                r.run();
            } catch (RuntimeException ex) {
                // A user action must never take the robot loop down with it.
                System.err.println("[Catalyst] state machine action threw: " + ex);
            }
        }
    }

    private List<Handle<?>> sorted(Set<Handle<?>> set) {
        List<Handle<?>> out = new ArrayList<>(set);
        out.sort(java.util.Comparator.comparing(Handle::key));
        return out;
    }

    private String edgeKey(S from, S to) {
        return from.name() + "->" + to.name();
    }

    // ------------------------------------------------------------------
    // Publishing
    // ------------------------------------------------------------------

    private void publishGraph(ValidationReport report) {
        String[] states = new String[constants.length];
        for (int i = 0; i < constants.length; i++) states[i] = constants[i].name();

        List<String> bindingLines = new ArrayList<>();
        for (Bound b : bindings) {
            bindingLines.add(b.key + "|" + b.binding.kind() + "|" + b.binding.unit()
                    + "|goal=" + b.handle.goalTypeName() + "|advisory=" + b.advisory);
        }

        List<String> routeLines = new ArrayList<>();
        if (routing == Routing.SHORTEST_PATH) {
            for (S from : constants) {
                for (S to : constants) {
                    if (from == to) continue;
                    Route<S> r = graph.route(from, to);
                    if (r.length() > 1) routeLines.add(from.name() + "->" + to.name() + " via " + r.render());
                }
            }
        }

        telemetry.graph(machineName, states, graph.describeEdges(),
                bindingLines.toArray(new String[0]), routeLines.toArray(new String[0]),
                graph.toDot(machineName),
                report == null ? List.of() : report.errors(),
                report == null ? List.of() : report.warnings());
    }

    private void publishBindings(double now) {
        boolean moving = isTransitioning();
        for (Bound b : bindings) {
            if (moving || b.sampleChanged(now, isGatingNow(b))) {
                telemetry.binding(b.sample(now, isGatingNow(b)));
            }
        }
    }

    private void publishTimeline(double now) {
        S hop = nextHop();
        if (current != lastPubState || stateConfirmed != lastPubConfirmed
                || requestedTarget != lastPubTarget || hop != lastPubHop
                || phase != lastPubPhase || stageIndex() != lastPubStage) {
            telemetry.state(current, stateConfirmed, requestedTarget, hop, phase, stageIndex());
            lastPubState = current;
            lastPubConfirmed = stateConfirmed;
            lastPubTarget = requestedTarget;
            lastPubHop = hop;
            lastPubPhase = phase;
            lastPubStage = stageIndex();
        }

        String blocker = blocker();
        String summary = summary();
        boolean detailDue = now - lastDetailPublishSeconds >= DETAIL_PERIOD_SECONDS;
        if (!Objects.equals(blocker, lastPubBlocker) || !Objects.equals(summary, lastPubSummary) || detailDue) {
            String detail = detailDue ? blockerDetail() : (lastPubDetail == null ? "" : lastPubDetail);
            if (!Objects.equals(blocker, lastPubBlocker) || !Objects.equals(summary, lastPubSummary)
                    || !Objects.equals(detail, lastPubDetail)) {
                telemetry.blocker(blocker, detail, summary, waitingOn());
                lastPubBlocker = blocker;
                lastPubSummary = summary;
                lastPubDetail = detail;
            }
            if (detailDue) lastDetailPublishSeconds = now;
        }

        if (isTransitioning() || phase == Phase.HOLDING) {
            Route<S> r = new Route<>(route);
            telemetry.progress(seq, progress(), elapsedSeconds(), timeoutSeconds(),
                    r.render(), activeTrigger);
        }

        String legal = legalTargets().toString();
        if (!Objects.equals(legal, lastPubLegal)) {
            EnumSet<S> set = legalTargets();
            String[] names = new String[set.size()];
            int i = 0;
            for (S s : set) names[i++] = s.name();
            telemetry.legalTargets(names);
            lastPubLegal = legal;
        }

        if (faulted != lastPubFaulted || !Objects.equals(faultReason, lastPubFaultReason)) {
            telemetry.fault(faulted, faultReason);
            lastPubFaulted = faulted;
            lastPubFaultReason = faultReason;
        }

        if (lastPubCounters[0] != transitionCount || lastPubCounters[1] != rejectionCount
                || lastPubCounters[2] != timeoutCount || lastPubCounters[3] != abortCount
                || lastPubCounters[4] != yieldCount) {
            telemetry.counters(transitionCount, rejectionCount, timeoutCount, abortCount, yieldCount);
            lastPubCounters = new long[]{transitionCount, rejectionCount, timeoutCount, abortCount, yieldCount};
        }

        telemetry.heartbeat(ticks, uptimeSeconds(), enabled);
    }

    // ------------------------------------------------------------------
    // Bound — one binding plus its runtime bookkeeping
    // ------------------------------------------------------------------

    private static final class Bound {
        final Handle<?> handle;
        final String key;
        final Binding<Object> binding;
        final boolean advisory;

        Object appliedGoal;
        double appliedAtSeconds;
        boolean arrived;
        double arrivalSeconds = Double.NaN;
        boolean owned = true;

        private String lastGoalLabel = " ";
        private boolean lastArrived;
        private boolean lastOwned = true;
        private boolean lastGating;
        private String lastNote = " ";

        @SuppressWarnings("unchecked")
        Bound(Handle<?> handle, Binding<?> binding, boolean advisory) {
            this.handle = handle;
            this.key = handle.key();
            this.binding = (Binding<Object>) binding;
            this.advisory = advisory;
        }

        /**
         * Elapsed time is only offered for the goal that is actually applied. Any other goal sees
         * zero, which is what keeps {@code isAt(otherState)} from reporting true purely because
         * some settle window happened to have elapsed.
         */
        boolean atGoal(Object goal, double now) {
            double since = Objects.equals(goal, appliedGoal) ? now - appliedAtSeconds : 0.0;
            try {
                return binding.atGoal(goal, since);
            } catch (RuntimeException ex) {
                return false;
            }
        }

        double since(Object goal, double now) {
            return Objects.equals(goal, appliedGoal) ? now - appliedAtSeconds : 0.0;
        }

        BindingSample sample(double now, boolean gating) {
            Object goal = appliedGoal;
            String label = goal == null ? "" : safe(() -> binding.label(goal));
            String detail = goal == null ? "" : safe(() -> binding.detail(goal));
            String note = goal == null ? "" : safe(() -> binding.note(goal));
            double measured = safeD(binding::measured);
            double error = goal == null ? Double.NaN : safeD(() -> binding.error(goal));
            double tol = goal == null ? Double.NaN : safeD(() -> binding.tolerance(goal));
            boolean observable = goal == null || safeB(() -> binding.observable(goal));
            boolean at = goal != null && atGoal(goal, now);
            return new BindingSample(key, binding.kind(), binding.unit(), label, detail,
                    at, owned, observable, gating, measured, error, tol, arrivalSeconds, note);
        }

        boolean sampleChanged(double now, boolean gating) {
            Object goal = appliedGoal;
            String label = goal == null ? "" : safe(() -> binding.label(goal));
            String note = goal == null ? "" : safe(() -> binding.note(goal));
            boolean at = goal != null && atGoal(goal, now);
            boolean changed = !label.equals(lastGoalLabel) || at != lastArrived || owned != lastOwned
                    || gating != lastGating || !note.equals(lastNote);
            if (changed) {
                lastGoalLabel = label;
                lastArrived = at;
                lastOwned = owned;
                lastGating = gating;
                lastNote = note;
            }
            return changed;
        }

        ArrivalReport arrivalReport(Object goal, double now) {
            return new ArrivalReport(key, binding.kind(), safe(() -> binding.label(goal)),
                    atGoal(goal, now), safeB(() -> binding.observable(goal)),
                    safeD(() -> binding.error(goal)), safeD(() -> binding.tolerance(goal)),
                    binding.unit(), arrivalSeconds);
        }

        String describeShortfall(Object goal) {
            String note = safe(() -> binding.note(goal));
            if (!note.isEmpty()) return key + " " + note;
            double err = safeD(() -> binding.error(goal));
            double tol = safeD(() -> binding.tolerance(goal));
            if (Double.isNaN(err)) {
                return key + " not at " + safe(() -> binding.label(goal));
            }
            return String.format("%s err %.3f %s > %.3f %s", key, Math.abs(err), binding.unit(),
                    Math.abs(tol), binding.unit());
        }

        private static String safe(java.util.function.Supplier<String> s) {
            try {
                String v = s.get();
                return v == null ? "" : v;
            } catch (RuntimeException ex) {
                return "";
            }
        }

        private static double safeD(DoubleSupplier s) {
            try {
                return s.getAsDouble();
            } catch (RuntimeException ex) {
                return Double.NaN;
            }
        }

        private static boolean safeB(BooleanSupplier s) {
            try {
                return s.getAsBoolean();
            } catch (RuntimeException ex) {
                return true;
            }
        }
    }

    /** A global precondition, named positively: {@code satisfied() == true} means safe to proceed. */
    private static final class Interlock<S extends Enum<S>> {
        final String name;
        final BooleanSupplier satisfied;
        final Predicate<S> blocksState;

        Interlock(String name, BooleanSupplier satisfied, Predicate<S> blocksState) {
            this.name = name;
            this.satisfied = satisfied;
            this.blocksState = blocksState;
        }
    }

    // ==================================================================
    // Builder
    // ==================================================================

    /**
     * Assembles a machine and validates it before it can exist.
     *
     * <p>Every problem found is collected and reported together, because a deploy cycle in a pit
     * costs the better part of a minute and six typos in one message beats six crashes.
     *
     * @param <S> the enum of states
     */
    public static final class Builder<S extends Enum<S>> {

        private final Class<S> stateType;
        private final String machineName;
        private final List<Bound> bindings = new ArrayList<>();
        private final Map<Handle<?>, Bound> byHandle = new HashMap<>();
        private final EnumMap<S, StateSpec<S>> specs;
        private final Map<String, EdgeSpec<S>> edgeSpecs = new HashMap<>();
        private final Map<String, List<S>> pinnedRoutes = new HashMap<>();
        private final List<Interlock<S>> interlocks = new ArrayList<>();
        private final Set<S> allowUnreachable;
        private final Map<S, EnumSet<S>> edges;
        private StateSpec<S> defaults;
        private DoubleSupplier clock;
        private StateMachineTelemetry<S> telemetry = StateMachineTelemetry.noop();
        private double defaultTimeout = 4.0;
        private int historyCapacity = 50;
        private S initialState;
        private Routing routing = Routing.DIRECT_ONLY;
        private FaultPolicy defaultFaultPolicy = FaultPolicy.HOLD_AND_REPORT;
        private boolean strict = true;
        private boolean abortOnOverride;
        private ValidationReport lastValidation;

        Builder(Class<S> stateType, String machineName) {
            this.stateType = stateType;
            this.machineName = machineName == null ? stateType.getSimpleName() : machineName;
            this.specs = new EnumMap<>(stateType);
            this.allowUnreachable = EnumSet.noneOf(stateType);
            this.edges = new EnumMap<>(stateType);
            S[] cs = stateType.getEnumConstants();
            this.initialState = cs != null && cs.length > 0 ? cs[0] : null;
        }

        /**
         * Register a mechanism and get back a type-carrying handle.
         *
         * @param key     stable, log-safe identifier
         * @param binding the mechanism adapter
         * @return a handle whose goal type the compiler will enforce at every {@code set(...)}
         */
        public <G> Handle<G> bind(String key, Binding<G> binding) {
            return bindInternal(key, binding, false);
        }

        /**
         * Register a mechanism that is driven but never gates arrival and never counts as an
         * override — LEDs, dashboards, anything whose "arrival" is meaningless.
         */
        public <G> Handle<G> bindAdvisory(String key, Binding<G> binding) {
            return bindInternal(key, binding, true);
        }

        private <G> Handle<G> bindInternal(String key, Binding<G> binding, boolean advisory) {
            if (binding == null) throw new IllegalArgumentException("binding for '" + key + "' is null");
            String resolved = key == null || key.isEmpty() ? binding.key() : key;
            Handle<G> handle = new Handle<>(bindings.size(), resolved,
                    binding.getClass().getSimpleName());
            Bound bound = new Bound(handle, binding, advisory);
            bindings.add(bound);
            byHandle.put(handle, bound);
            return handle;
        }

        /** The time source. Required — there is deliberately no default, so tests cannot get a real clock. */
        public Builder<S> clock(DoubleSupplier secondsSupplier) {
            this.clock = secondsSupplier;
            return this;
        }

        /** Where the machine publishes. Defaults to {@link StateMachineTelemetry#noop()}. */
        public Builder<S> telemetry(StateMachineTelemetry<S> telemetry) {
            this.telemetry = telemetry == null ? StateMachineTelemetry.noop() : telemetry;
            return this;
        }

        /** Deadline for any hop with no more specific timeout. Default 4.0 s. */
        public Builder<S> defaultTimeout(double seconds) {
            this.defaultTimeout = seconds;
            return this;
        }

        /** How many transition records to retain. Default 50. */
        public Builder<S> historyCapacity(int entries) {
            this.historyCapacity = entries;
            return this;
        }

        /** The state the machine assumes before {@code seed(...)}. Default: the first constant. */
        public Builder<S> initialState(S state) {
            this.initialState = state;
            return this;
        }

        /** Direct-only (default) or automatic multi-hop routing. */
        public Builder<S> routing(Routing routing) {
            this.routing = routing == null ? Routing.DIRECT_ONLY : routing;
            return this;
        }

        /** Fault policy for states that do not declare their own. */
        public Builder<S> defaultFaultPolicy(FaultPolicy policy) {
            this.defaultFaultPolicy = policy == null ? FaultPolicy.HOLD_AND_REPORT : policy;
            return this;
        }

        /**
         * When {@code false}, deadlines still fire, still count and still log, but downgrade from
         * a fault to a warning.
         *
         * <p>The migration path: run a practice match with {@code strict(false)}, read
         * {@code Transition/History}, fix the three tolerances it names, then turn strict back on.
         */
        public Builder<S> strict(boolean strict) {
            this.strict = strict;
            return this;
        }

        /**
         * Abort the in-flight transition the moment any non-advisory mechanism is taken over by
         * another command. Default {@code false}, which lets a driver jog one mechanism without
         * cancelling the whole superstructure move.
         */
        public Builder<S> abortOnOverride(boolean abort) {
            this.abortOnOverride = abort;
            return this;
        }

        /**
         * The safe posture, stated once and inherited by every state that does not override it.
         *
         * <p>This is what keeps a ten-mechanism, nine-state robot at roughly thirty-five lines
         * instead of ninety assignments — and it means the mechanism you forget to mention in one
         * state inherits a deliberate default instead of staying wherever the last state parked it.
         */
        public Builder<S> defaults(Consumer<StateSpec<S>> spec) {
            if (defaults == null) defaults = new StateSpec<>(initialState);
            spec.accept(defaults);
            return this;
        }

        /** Declare one state. Every enum constant must be declared exactly once. */
        public Builder<S> state(S state, Consumer<StateSpec<S>> spec) {
            StateSpec<S> s = new StateSpec<>(state);
            spec.accept(s);
            specs.put(state, s);
            return this;
        }

        /** Declare edges from {@code from} to each of the listed states. */
        @SafeVarargs
        public final Builder<S> allow(S from, S first, S... rest) {
            edges.computeIfAbsent(from, k -> EnumSet.noneOf(stateType)).add(first);
            for (S s : rest) edges.get(from).add(s);
            return this;
        }

        /** Declare edges in both directions. */
        public Builder<S> allowBoth(S a, S b) {
            edges.computeIfAbsent(a, k -> EnumSet.noneOf(stateType)).add(b);
            edges.computeIfAbsent(b, k -> EnumSet.noneOf(stateType)).add(a);
            return this;
        }

        /**
         * Make {@code hub} reachable from every state and able to reach every state.
         *
         * <p>Creates real edges, so under {@link Routing#SHORTEST_PATH} a hub also makes almost
         * everything reachable in two hops. That is usually what you want for a {@code STOW}
         * state, and is exactly what you do not want to discover by accident — which is why
         * {@code validate()} lists every route a hub creates.
         */
        public Builder<S> hub(S hub) {
            for (S s : stateType.getEnumConstants()) {
                if (s == hub) continue;
                edges.computeIfAbsent(s, k -> EnumSet.noneOf(stateType)).add(hub);
                edges.computeIfAbsent(hub, k -> EnumSet.noneOf(stateType)).add(s);
            }
            return this;
        }

        /** Declare one edge and configure its guard, deadline, cost and actuation stages. */
        public Builder<S> edge(S from, S to, Consumer<EdgeSpec<S>> spec) {
            edges.computeIfAbsent(from, k -> EnumSet.noneOf(stateType)).add(to);
            EdgeSpec<S> e = new EdgeSpec<>(from, to);
            spec.accept(e);
            edgeSpecs.put(from.name() + "->" + to.name(), e);
            return this;
        }

        /** Pin the route for {@code from -> to} instead of trusting breadth-first search. */
        @SafeVarargs
        public final Builder<S> via(S from, S to, S... waypoints) {
            List<S> hops = new ArrayList<>(Arrays.asList(waypoints));
            hops.add(to);
            pinnedRoutes.put(from.name() + "->" + to.name(), List.copyOf(hops));
            return this;
        }

        /** Suppress the "state is unreachable" error for these states. */
        @SafeVarargs
        public final Builder<S> allowUnreachable(S... states) {
            allowUnreachable.addAll(Arrays.asList(states));
            return this;
        }

        /**
         * A global precondition, named positively: {@code satisfied} returning {@code true} means
         * it is safe to proceed.
         *
         * <p>"The climber is deployed, so nothing but CLIMB and STOW is allowed" is one line here
         * and is inexpressible as a per-edge guard without touching every edge.
         */
        public Builder<S> interlock(String name, BooleanSupplier satisfied, Predicate<S> blocksState) {
            interlocks.add(new Interlock<>(name, satisfied, blocksState));
            return this;
        }

        /**
         * Check the configuration without building it. Touches no hardware, so this is the hook a
         * unit test uses to catch a superstructure misconfiguration at commit time.
         */
        public ValidationReport validate() {
            List<String> errors = new ArrayList<>();
            List<String> warnings = new ArrayList<>();

            if (clock == null) errors.add("no clock supplied — call .clock(...)");
            if (stateType.getEnumConstants() == null || stateType.getEnumConstants().length == 0) {
                errors.add(stateType.getSimpleName() + " has no enum constants");
                return new ValidationReport(errors, warnings);
            }
            if (bindings.isEmpty()) warnings.add("no mechanisms are bound; the machine will do nothing");

            Set<String> keys = new HashSet<>();
            for (Bound b : bindings) {
                if (!keys.add(b.key)) errors.add("duplicate binding key '" + b.key + "'");
            }

            for (S s : stateType.getEnumConstants()) {
                if (!specs.containsKey(s)) {
                    errors.add("state " + s.name() + " is never declared — add .state(" + s.name()
                            + ", spec -> ...), even if it only takes the defaults");
                }
            }

            // Per-goal validation, so an out-of-range setpoint or an unknown preset is a build
            // error on a laptop rather than an exception thrown inside a command factory.
            for (Map.Entry<S, StateSpec<S>> e : specs.entrySet()) {
                StateSpec<S> spec = e.getValue();
                Map<Handle<?>, Object> merged = new HashMap<>();
                if (defaults != null) merged.putAll(defaults.goals);
                merged.putAll(spec.goals);
                for (Handle<?> h : spec.released) merged.remove(h);
                for (Map.Entry<Handle<?>, Object> g : merged.entrySet()) {
                    Bound b = byHandle.get(g.getKey());
                    if (b == null) {
                        errors.add("state " + e.getKey().name() + " sets a handle that was never bound");
                        continue;
                    }
                    final String where = "state " + e.getKey().name() + ", mechanism '" + b.key + "': ";
                    try {
                        b.binding.validate(g.getValue(), p -> errors.add(where + p));
                    } catch (RuntimeException ex) {
                        errors.add(where + "validation threw " + ex);
                    }
                }
                if (spec.faultPolicy == FaultPolicy.RECOVER_TO && spec.recoveryState == null) {
                    errors.add("state " + e.getKey().name()
                            + " declares FaultPolicy.RECOVER_TO but no recoverTo(state)");
                }
            }

            StateGraph<S> g = buildGraph();
            if (initialState != null) {
                EnumSet<S> unreachable = g.unreachableFrom(initialState);
                unreachable.removeAll(allowUnreachable);
                unreachable.remove(initialState);
                if (!unreachable.isEmpty()) {
                    errors.add("no path from " + initialState.name() + " to " + unreachable
                            + " — add edges, or call .allowUnreachable(...) if that is intentional");
                }
            }
            if (g.edgeCount() == 0) {
                errors.add("no transitions are declared — use .allow(...), .allowBoth(...), "
                        + ".hub(...) or .edge(...)");
            }

            if (routing == Routing.SHORTEST_PATH) {
                for (S from : stateType.getEnumConstants()) {
                    for (S to : stateType.getEnumConstants()) {
                        if (from == to) continue;
                        Route<S> r = g.route(from, to);
                        if (r.length() > 1) {
                            warnings.add("SHORTEST_PATH will route " + from.name() + "->" + to.name()
                                    + " via " + r.render());
                        }
                    }
                }
            }

            for (Map.Entry<String, EdgeSpec<S>> e : edgeSpecs.entrySet()) {
                for (List<Handle<?>> stage : e.getValue().stages) {
                    for (Handle<?> h : stage) {
                        if (!byHandle.containsKey(h)) {
                            errors.add("edge " + e.getKey() + " stages a handle that was never bound");
                        }
                    }
                }
            }

            lastValidation = new ValidationReport(errors, warnings);
            return lastValidation;
        }

        /** Validate, then construct. Throws {@link StateMachineConfigException} listing every error. */
        public StateMachineCore<S> build() {
            ValidationReport report = validate();
            report.throwIfInvalid(machineName);
            return new StateMachineCore<>(this);
        }

        StateGraph<S> buildGraph() {
            Map<S, EnumSet<S>> copy = new EnumMap<>(stateType);
            for (Map.Entry<S, EnumSet<S>> e : edges.entrySet()) {
                copy.put(e.getKey(), EnumSet.copyOf(e.getValue()));
            }
            Map<String, Double> costs = new HashMap<>();
            Map<String, String> guards = new HashMap<>();
            for (Map.Entry<String, EdgeSpec<S>> e : edgeSpecs.entrySet()) {
                costs.put(e.getKey(), e.getValue().cost);
                if (!e.getValue().guardReason.isEmpty()) guards.put(e.getKey(), e.getValue().guardReason);
            }
            return new StateGraph<>(stateType, stateType.getEnumConstants(), copy, costs, guards);
        }
    }
}
