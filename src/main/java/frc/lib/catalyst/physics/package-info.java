/**
 * Catalyst Physics Core — the optional layer that watches how the robot actually behaves.
 *
 * <p>Everything else in Catalyst tells the robot what to do. This package is the part that notices
 * what the robot <em>did</em>, compares it against what the physics says should have happened, and
 * hands the difference back to the rest of the library as information.
 *
 * <h2>The one-paragraph version</h2>
 * Wheel odometry is excellent until a tyre breaks loose. An IMU does not care about traction but
 * drifts within seconds. Vision is absolute truth right up until it has not seen a tag for four
 * seconds. Physics Core fuses the three into a velocity with an honest confidence attached, scores
 * which wheel is lying, notices when something hit the robot, and predicts where the robot will be
 * when a shot actually leaves it.
 *
 * <h2>It is optional and it is advisory</h2>
 * Every Catalyst API works identically without this package on the classpath. Physics Core takes no
 * control authority: it writes no pose, schedules no command, blocks no transition, and changes no
 * setpoint. It produces estimates and explanations; the state machine, {@code BehaviorEngine}, and
 * {@code Autopilot} keep deciding what the robot does with them.
 *
 * <h2>Layout</h2>
 * <ul>
 *   <li>{@link frc.lib.catalyst.physics.PhysicsCore} — the one object a robot project touches.</li>
 *   <li>{@link frc.lib.catalyst.physics.RobotStateSource} /
 *       {@link frc.lib.catalyst.physics.UncertainRobotStateSource} — the contract a plain
 *       {@code SwerveSubsystem} and Physics Core both satisfy, so consumers depend on neither.</li>
 *   <li>{@code data} — timestamped signal buffers and the synchronizer that lines them up onto one
 *       clock, because comparing measurements taken at different instants is how residuals lie.</li>
 *   <li>{@code observation} — timestamped evidence arriving from outside the drivetrain.</li>
 *   <li>{@code model} — the handful of measured physical facts, and the traction and tipping limits
 *       that follow from them.</li>
 *   <li>{@code estimation} — the fused state, per-module slip scoring, and the unexplained-acceleration
 *       residual.</li>
 *   <li>{@code prediction} — short-horizon propagation, and the robot's state at shot release.</li>
 *   <li>{@code diagnostics} — discrete events, currently collisions.</li>
 * </ul>
 *
 * <h2>Scope</h2>
 * This is <b>not</b> a rigid-body physics engine: there is no collision solver and no contact model.
 * For simulation with game pieces, Catalyst integrates with maple-sim. Physics Core is about the real
 * robot's measured behaviour.
 *
 * <p>What ships today is Phase 1 — shadow mode. Every service is observation-only and every algorithm
 * is HAL-free and unit tested. Fusing pose, applying dynamic constraints, and online parameter
 * identification are later, explicitly opt-in phases; see the project roadmap.
 *
 * @since 1.5.0
 */
package frc.lib.catalyst.physics;
