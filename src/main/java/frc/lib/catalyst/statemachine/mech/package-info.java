/**
 * The adapters that let the hardware-free engine actually drive hardware: one
 * {@link frc.lib.catalyst.statemachine.robot.Actuator} per Catalyst mechanism, plus the
 * {@link frc.lib.catalyst.statemachine.mech.Mechanisms} factory you call to build them.
 *
 * <p>The engine understands only the {@link frc.lib.catalyst.statemachine.Binding} contract, which
 * imports no WPILib. Each class here implements the robot-side {@code Actuator} extension of that
 * contract — it knows how to build a {@code Command} that drives one specific Catalyst mechanism
 * toward a goal from {@link frc.lib.catalyst.statemachine.goals}, and how to answer the one question
 * the engine keeps asking: "is it there yet?" Every one of these bindings is an ordinary
 * implementation with no special-casing inside the engine, which is precisely what makes a team's
 * own subsystem exactly as first-class as {@code LinearMechanism}.
 *
 * <h2>The nine typed bindings</h2>
 *
 * <p>Each wraps the matching Catalyst mechanism and pursues the matching sibling goal. You rarely
 * name these directly — the factory below constructs them — but they are public so a team that
 * wants the behaviour without the factory's naming loses nothing by using them:
 * {@link frc.lib.catalyst.statemachine.mech.LinearBinding},
 * {@link frc.lib.catalyst.statemachine.mech.RotationalBinding},
 * {@link frc.lib.catalyst.statemachine.mech.WristBinding},
 * {@link frc.lib.catalyst.statemachine.mech.FlywheelBinding},
 * {@link frc.lib.catalyst.statemachine.mech.TurretBinding},
 * {@link frc.lib.catalyst.statemachine.mech.ClawBinding},
 * {@link frc.lib.catalyst.statemachine.mech.RollerBinding},
 * {@link frc.lib.catalyst.statemachine.mech.WinchBinding}, and
 * {@link frc.lib.catalyst.statemachine.mech.PneumaticBinding}.
 *
 * <h2>The factory facade — the entry point you call</h2>
 *
 * <p>{@link frc.lib.catalyst.statemachine.mech.Mechanisms} is the single place you turn a subsystem
 * into something the state machine can drive. Its nine typed factories ({@code linear},
 * {@code rotational}, {@code wrist}, {@code flywheel}, {@code turret}, {@code claw}, {@code roller},
 * {@code winch}, {@code pneumatic}) are one-line wrappers over the bindings above. The important
 * half, though, is the four custom escape-hatch tiers, which let <em>any</em> subsystem — your
 * swerve drive, an LED controller, a vendor-SDK wrapper — reach the same fidelity (arrival gating,
 * blocker reporting, re-assertion, build-time validation) by saying only as much as it needs to:
 *
 * <ol>
 *   <li>{@code instant} — fire and forget; there is no arrival to wait for.</li>
 *   <li>{@code custom} — call a method, and answer a real arrival question.</li>
 *   <li>{@code commands} — the subsystem already exposes {@code Command} factories.</li>
 *   <li>{@code build} — everything: units, tolerance, notes, validation, re-assertion.</li>
 * </ol>
 *
 * <p>The custom-mechanism tiers are worked through end to end in
 * {@code docs/advanced/statemachine.md}.
 */
package frc.lib.catalyst.statemachine.mech;
