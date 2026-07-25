/**
 * One typed, value-equal goal per mechanism: the small "what should this mechanism be doing"
 * records the state machine stores in its state table.
 *
 * <p>Every goal here is a {@code record} (or a sealed hierarchy of records), and that is not a
 * stylistic choice — it is a correctness requirement. The engine in
 * {@link frc.lib.catalyst.statemachine.StateMachineCore} compares the goal it wants against the
 * goal currently applied with {@link java.util.Objects#equals} on every single loop, and only
 * re-issues actuation when they differ. Records give value-based {@code equals}/{@code hashCode}
 * for free; a goal type with identity equality would compare unequal to itself fifty times a
 * second, tear down and rebuild its pursue command every loop, and show up on the field as a
 * mechanism that stutters and never settles. That is why "just use a plain class" is a trap this
 * package deliberately closes.
 *
 * <p>The second reason there is one goal type <em>per mechanism</em> rather than a single shared
 * one is the compiler. Because {@code Handle<G>} carries its goal type and {@code StateSpec.set}
 * is declared {@code <G> set(Handle<G>, G)}, sending a {@code RotationalGoal} to an elevator handle
 * does not compile — the mistake is caught at build on a laptop, not discovered as an elevator
 * trying to drive to 90 degrees on the field.
 *
 * <h2>The goal types</h2>
 *
 * <ul>
 *   <li>{@link frc.lib.catalyst.statemachine.goals.LinearGoal} — a position for an elevator,
 *       telescoping arm, or slide, named numerically or by preset.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.goals.RotationalGoal} — an angle for an arm, wrist,
 *       or hood, as explicit degrees or a named preset.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.goals.WristGoal} — a pitch, a roll, and the arrival
 *       band for a two-axis differential wrist.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.goals.FlywheelGoal} — fixed wheel speeds, a live
 *       supplier-fed speed, or an explicit idle (which needs its own branch because a stopped
 *       flywheel reports "not at speed" forever).</li>
 *   <li>{@link frc.lib.catalyst.statemachine.goals.TurretGoal} — where a turret should point:
 *       fixed angles, or continuous aiming modes that recompute every loop.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.goals.ClawGoal} — grip, close, open and the other
 *       claw actions, some carrying a settle time or timeout.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.goals.RollerGoal} — run, hold, and intake-until-piece
 *       commands for an intake or indexer roller.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.goals.WinchGoal} — open-loop drive commands for a
 *       climber winch, each paired with a rule for when to stop (there is deliberately no positional
 *       variant, because the mechanism has no honest position control).</li>
 *   <li>{@link frc.lib.catalyst.statemachine.goals.PneumaticGoal} — a commanded solenoid state plus
 *       the settle time and re-assert period a feedback-free piston needs.</li>
 * </ul>
 *
 * <p>The bindings that actually pursue these goals live in
 * {@link frc.lib.catalyst.statemachine.mech}; the design reasoning is in
 * {@code docs/advanced/statemachine.md}.
 */
package frc.lib.catalyst.statemachine.goals;
