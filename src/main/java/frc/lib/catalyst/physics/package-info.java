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
 *   <li>{@code observation} — timestamped evidence arriving from outside the drivetrain: poses,
 *       velocities, ranges, bearings, and known contacts.</li>
 *   <li>{@code model} — the measured physical facts and what follows from them: traction and tipping
 *       limits, a live mass tree, stability from the zero-moment point, and closed-form ballistics.</li>
 *   <li>{@code estimation} — the fused state, per-module slip scoring, the unexplained-acceleration
 *       residual, and online identification of feedforward gains and battery resistance.</li>
 *   <li>{@code prediction} — short-horizon propagation, shot-release state, whole-robot power
 *       planning, and pre-flight capability evaluation.</li>
 *   <li>{@code diagnostics} — discrete events and fault reasoning: collisions, jams, residual
 *       monitoring, and cause ranking.</li>
 *   <li>{@code constraints} — the opt-in Phase 3 layer. Computes limits; applies none of them.</li>
 *   <li>{@code replay} — run recorded samples through a different configuration to ask what it would
 *       have done.</li>
 *   <li>{@code sim} — inject faults on purpose, so the detectors can be tested without a field.</li>
 * </ul>
 *
 * <h2>Scope</h2>
 * This is <b>not</b> a rigid-body physics engine: there is no collision solver and no contact model.
 * For simulation with game pieces, Catalyst integrates with maple-sim. Physics Core is about the real
 * robot's measured behaviour.
 *
 * <h2>Two things it will not do on its own</h2>
 * <ul>
 *   <li><b>It never writes your pose.</b> Your drivetrain estimator stays the single writer of
 *       {@code Pose2d}. Absolute observations reach Physics Core as evidence about confidence, and an
 *       outlier is rejected and counted rather than believed.</li>
 *   <li><b>It never applies a limit.</b> {@code constraints} computes speed and acceleration caps;
 *       putting one into effect is a line of your code. A team that never writes that line gets
 *       exactly the robot they had before.</li>
 * </ul>
 *
 * <p>Learned parameters follow the same rule: {@code FeedforwardIdentifier} and
 * {@code BatteryResistanceIdentifier} report what the robot's behaviour says its constants are, and
 * have no way to change them.
 *
 * @since 1.5.0
 */
package frc.lib.catalyst.physics;
