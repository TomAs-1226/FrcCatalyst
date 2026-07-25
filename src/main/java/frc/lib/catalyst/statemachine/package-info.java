/**
 * The hardware-free heart of the Catalyst state machine: the engine that decides what a
 * superstructure should do, and the small immutable value types it speaks in.
 *
 * <p>The one class that matters here is {@link frc.lib.catalyst.statemachine.StateMachineCore},
 * a real guarded, logged finite state machine over an enum of states. What makes this package
 * unusual is what it does <b>not</b> import: nothing from WPILib. There is no {@code Command}, no
 * {@code Subsystem}, no {@code Timer}, no NetworkTables anywhere below this line. The engine is
 * driven by {@code step()} and told the time by a {@code DoubleSupplier}, which is exactly what
 * lets the whole of its logic — routing, guards, arrival, deadlines, faults — be unit-tested on a
 * laptop with no HAL. The invariant it exists to protect is that {@code current()} is only ever a
 * state whose arrival was actually measured; a timeout or abort leaves the machine where it truly
 * is rather than where it was trying to go, which is the single most common defect in hand-rolled
 * superstructure code.
 *
 * <p><b>If you are writing robot code, you are almost certainly in the wrong package.</b> Build
 * your machine through {@link frc.lib.catalyst.statemachine.robot.Superstructure}, which wraps this
 * engine in a {@code SubsystemBase}, steps it once per loop, and hands you {@code Command} and
 * {@code Trigger} factories. Come here only to understand what that facade is standing on.
 *
 * <h2>The engine and how you configure it</h2>
 *
 * <ul>
 *   <li>{@link frc.lib.catalyst.statemachine.StateMachineCore} — the engine itself, built through
 *       its inner {@code Builder}.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.Binding} — the only thing the engine knows about
 *       hardware: a ten-line, WPILib-free plug-in contract a test can implement with one mutable
 *       {@code double}.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.StateSpec} — declares what one state means (where each
 *       mechanism goes, what must be true to enter, what runs on the way in and out).</li>
 *   <li>{@link frc.lib.catalyst.statemachine.EdgeSpec} — declares what one transition means (when
 *       it is allowed, how long it may take, and in what order the mechanisms move).</li>
 *   <li>{@link frc.lib.catalyst.statemachine.StateGraph} — the legal-transition graph; an edge you
 *       never declared is a move the robot cannot make.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.Handle} — an opaque, goal-typed token for one bound
 *       mechanism, and the piece that makes the builder refuse to compile a wrong-goal mistake.</li>
 * </ul>
 *
 * <h2>The value types it speaks in</h2>
 *
 * <p>All value-based (records or enums), all immutable, so a snapshot handed to a dashboard or a
 * test can never change underneath it:
 *
 * <ul>
 *   <li>{@link frc.lib.catalyst.statemachine.Phase} — what the machine is doing right now.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.RejectReason} — why a request was refused, so a button
 *       that does nothing is never indistinguishable from a broken one.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.Routing} — direct-only versus automatic multi-hop
 *       routing.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.FaultPolicy} — what to do when a hop blows its
 *       deadline.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.Route} — the ordered hops a request would take, usable
 *       as a zero-side-effect reachability preview.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.TransitionResult} — the immediate answer to a request:
 *       accepted with a route, or refused with a reason.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.TransitionRecord} — the permanent record of one
 *       attempt, emitted once when it ends for any reason.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.ArrivalReport} — one binding's contribution to a
 *       completed transition (did it arrive, how close, how long).</li>
 *   <li>{@link frc.lib.catalyst.statemachine.BindingSample} — one binding's live situation,
 *       published every loop.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.Snapshot} — everything the machine knows about itself,
 *       coherent in one value.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.ValidationReport} — the result of checking a
 *       configuration without building it, so a mistake becomes a failing test rather than a
 *       {@code robotInit} crash on the field.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.StateMachineConfigException} — thrown by
 *       {@code build()} carrying every problem found, not just the first.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.TransitionHistory} — the bounded, newest-first ring of
 *       records behind {@code Transition/History}.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.StateMachineTelemetry} — the engine's one output
 *       channel; sinks are dumb because the core already diffs the stream.</li>
 * </ul>
 *
 * <p>For how to <em>use</em> a machine, see {@code docs/advanced/statemachine.md}. For how it works
 * under the hood and how to debug it — one loop start to finish, and a symptom-to-log-key map — see
 * {@code docs/advanced/statemachine-internals.md}. {@code StateMachineCore.explain()} (and
 * {@code Superstructure.explain()}) print a plain-language dump of what you built and why it is stuck.
 */
package frc.lib.catalyst.statemachine;
