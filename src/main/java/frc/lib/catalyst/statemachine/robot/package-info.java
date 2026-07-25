/**
 * The thin WPILib layer that bolts the hardware-free engine onto a real robot: the facade you
 * build, the default command that moves each mechanism, and the sink that logs it all.
 *
 * <p>Everything below {@link frc.lib.catalyst.statemachine.StateMachineCore} is deliberately
 * WPILib-free so it can be unit-tested on a laptop. This package is where that abstinence ends and
 * the {@code Command}, {@code Subsystem} and {@code Trigger} types finally appear. Keeping all of
 * that glue in one small layer is what will make the eventual port to the 2027 command framework a
 * change here rather than a rewrite of the engine.
 *
 * <h2>What lives here</h2>
 *
 * <ul>
 *   <li>{@link frc.lib.catalyst.statemachine.robot.Superstructure} — the facade you actually build.
 *       It wraps the engine in a {@code SubsystemBase}, steps it once per loop in
 *       {@code periodic()}, wires the logging, and hands you {@code Command} and {@code Trigger}
 *       factories. Unlike the old {@code SuperstructureCoordinator}, it accepts every Catalyst
 *       mechanism type and any subsystem you wrote yourself.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.robot.GoalRunner} — the per-mechanism worker installed
 *       as each bound subsystem's <b>default command</b>. That one choice buys driver override for
 *       free: a driver's {@code whileTrue(elevator.jogUp(2))} interrupts the runner on the elevator
 *       alone, the engine notices and logs the lost ownership, and when the button is released the
 *       machine's goal is re-applied on the very next loop.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.robot.Actuator} — the robot-side extension of
 *       {@link frc.lib.catalyst.statemachine.Binding} that adds the {@code Command} plumbing. Every
 *       binding in {@link frc.lib.catalyst.statemachine.mech} is an ordinary implementation of it.
 *       Its commands are <b>hosted</b> by {@code GoalRunner} rather than scheduled, which is why its
 *       contract is strict.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.robot.CatalystStateMachineLog} — the telemetry sink
 *       that routes the whole state-machine log schema into {@code CatalystLog}, including the
 *       channels (AlertManager, the Driver Station console) that still work with no dashboard
 *       attached at a regional.</li>
 *   <li>{@link frc.lib.catalyst.statemachine.robot.SuperstructureLike} — the string-keyed seam that
 *       {@code GoalDirector} consumes, so a team can move from the old coordinator to
 *       {@code Superstructure} without touching their goal layer.</li>
 * </ul>
 *
 * <p>Start with {@code Superstructure}; the rest of this package is the machinery it stands up for
 * you. The full worked example and log schema are in {@code docs/advanced/statemachine.md}.
 */
package frc.lib.catalyst.statemachine.robot;
