package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.CatalystMechanism;
import frc.lib.catalyst.mechanisms.ClawMechanism;
import frc.lib.catalyst.mechanisms.DifferentialWristMechanism;
import frc.lib.catalyst.mechanisms.FlywheelMechanism;
import frc.lib.catalyst.mechanisms.LinearMechanism;
import frc.lib.catalyst.mechanisms.PneumaticMechanism;
import frc.lib.catalyst.mechanisms.RollerMechanism;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.lib.catalyst.mechanisms.TurretMechanism;
import frc.lib.catalyst.mechanisms.WinchMechanism;
import frc.lib.catalyst.statemachine.Binding;
import frc.lib.catalyst.statemachine.goals.ClawGoal;
import frc.lib.catalyst.statemachine.goals.FlywheelGoal;
import frc.lib.catalyst.statemachine.goals.LinearGoal;
import frc.lib.catalyst.statemachine.goals.PneumaticGoal;
import frc.lib.catalyst.statemachine.goals.RollerGoal;
import frc.lib.catalyst.statemachine.goals.RotationalGoal;
import frc.lib.catalyst.statemachine.goals.TurretGoal;
import frc.lib.catalyst.statemachine.goals.WinchGoal;
import frc.lib.catalyst.statemachine.goals.WristGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.BiPredicate;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

/**
 * The single entry point for turning a subsystem — anyone's subsystem — into an
 * {@link Actuator} the state machine can drive.
 *
 * <p>This class exists to answer one complaint directly: a state machine that only understands
 * the mechanisms its own library ships is a state machine that half the teams cannot use. So
 * there are two halves here, and the second half is the important one.
 *
 * <h2>The nine typed factories</h2>
 *
 * <p>{@link #linear}, {@link #rotational}, {@link #wrist}, {@link #flywheel}, {@link #turret},
 * {@link #claw}, {@link #roller}, {@link #winch} and {@link #pneumatic} each wrap the matching
 * Catalyst mechanism in the matching sibling binding. They are thin on purpose: every one is a
 * one-line delegation to a public constructor, so a team that wants the binding's behaviour but
 * not this class's naming can construct {@link LinearBinding} and friends directly and lose
 * nothing. Each factory has an overload without a key, which uses
 * {@code mechanism.getMechanismName()} — the same string the mechanism already logs under, which
 * keeps binding telemetry sitting next to mechanism telemetry in a log viewer.
 *
 * <h2>The four escape hatches</h2>
 *
 * <p>A team's own swerve drive, LED controller, climber-of-unusual-shape or vendor-SDK wrapper is
 * not second-class. The engine knows nothing about the nine typed bindings; it knows only
 * {@link Actuator}. So a custom subsystem reaches exactly the same fidelity — arrival gating,
 * blocker reporting, re-assertion, build-time validation — by picking whichever of these four
 * tiers matches how much the subsystem actually needs to say:
 *
 * <ol>
 *   <li>{@link #instant} — fire and forget. There is no arrival to wait for.</li>
 *   <li>{@link #custom} — call a method, and answer a real arrival question.</li>
 *   <li>{@link #commands} — the subsystem already exposes {@code Command} factories.</li>
 *   <li>{@link #build} — everything: units, tolerance, notes, validation, re-assertion.</li>
 * </ol>
 *
 * <h2>Worked example — all four tiers at once</h2>
 *
 * <pre>{@code
 * // Tier 1: fire-and-forget. An LED pattern has no arrival concept, so the binding
 * // reports at-goal immediately and the state never waits on it.
 * Actuator<LedPattern> leds =
 *     Mechanisms.instant("Leds", leds::setPattern, leds);
 *
 * // Tier 2: a direct method call plus a genuine sensor test. This is the simplest
 * // binding that can actually gate a transition. HoodPreset is an enum, so it has
 * // the value-based equals the engine needs.
 * Actuator<HoodPreset> hood =
 *     Mechanisms.custom("Hood",
 *         want -> hood.setAngleDeg(want.degrees()),
 *         want -> Math.abs(hood.getAngleDeg() - want.degrees()) < 1.0,
 *         hood);
 *
 * // Tier 3: the subsystem already speaks Commands. The factory must hand back a
 * // fresh instance every call -- a hosted command is initialised many times a match.
 * Actuator<ClimbStep> climber =
 *     Mechanisms.commands("Climber",
 *         climber::stepCommand,
 *         climber::stepComplete,
 *         climber);
 *
 * // Tier 4: full control. Everything the nine typed bindings publish is available here.
 * // Note the explicit <ShotGoal> witness -- it is required, not decoration. See build().
 * Actuator<ShotGoal> shooter =
 *     Mechanisms.<ShotGoal>build("Shooter", shooter)
 *         .kind("flywheel")
 *         .unit("rps")
 *         .pursue(goal -> shooter.spinUpCommand(goal.rps()))
 *         .hold(goal -> shooter.maintainCommand(goal.rps()))
 *         .atGoal(goal -> Math.abs(shooter.getRps() - goal.rps()) < 2.0)
 *         .measured(shooter::getRps)
 *         .error(goal -> shooter.getRps() - goal.rps())
 *         .tolerance(2.0)
 *         .range(0.0, 90.0, ShotGoal::rps)
 *         .label(ShotGoal::name)
 *         .note(goal -> shooter.isStalled() ? "stalled" : "")
 *         .reassertEvery(25)
 *         .done();
 * }</pre>
 *
 * <p>Note what tier 4 buys over tier 2. {@code measured} and {@code error} make the shooter
 * plottable in a log. {@code tolerance} makes the plot readable, because the band is drawn next to
 * the trace. {@code range} turns an impossible setpoint into a build failure on a laptop instead of
 * a mechanism that silently never arrives in a match. {@code note} is what appears in
 * {@code BlockerDetail} when a state refuses to complete, which is the difference between "the
 * robot is stuck" and "the shooter is stalled". None of that is reserved for library mechanisms.
 *
 * <h2>Goal types</h2>
 *
 * <p>Whatever {@code G} a team chooses must have value-based {@code equals} and {@code hashCode} —
 * a record, an enum, or a sealed hierarchy of records, and never an array. The engine compares the
 * wanted goal against the active one with {@link Objects#equals} every loop to decide whether to
 * rebuild actuation; an identity-equality goal type would therefore rebuild its command fifty times
 * a second forever. This is the one rule a custom binding cannot get away with breaking, and it is
 * not enforceable at compile time, so it is stated here as loudly as possible.
 *
 * @since 1.2.0
 */
public final class Mechanisms {

    /**
     * Not instantiable. This is a factory holder; there is no state worth carrying.
     *
     * @throws AssertionError always, so reflection cannot quietly produce an instance
     */
    private Mechanisms() {
        throw new AssertionError("Mechanisms is a static factory holder and cannot be instantiated");
    }

    // ==================================================================
    // The nine typed factories
    //
    // Every sibling binding constructor takes (mechanism, key) in that order, so each
    // factory below transposes its own (key, mechanism) argument order on the way in.
    // The factory order is the one that reads correctly at a call site, where the key is
    // the thing being named and the mechanism is the thing being named after it.
    // ==================================================================

    /**
     * Wraps a {@link LinearMechanism} — elevator, telescoping arm, linear slide — as an actuator.
     *
     * @param key       stable, unique, log-safe telemetry key; becomes {@code Bindings/<key>/...}
     * @param mechanism the mechanism to drive; becomes the binding's sole requirement
     * @return a {@link LinearBinding} over {@code mechanism}
     * @throws NullPointerException if either argument is {@code null}
     */
    public static Actuator<LinearGoal> linear(String key, LinearMechanism mechanism) {
        return new LinearBinding(mechanism, key);
    }

    /**
     * Wraps a {@link LinearMechanism}, keyed by its own mechanism name.
     *
     * <p>The key becomes {@code mechanism.getMechanismName()}, which is the same string the
     * mechanism already publishes its own telemetry under, so binding and mechanism data land
     * beside each other in a log viewer.
     *
     * @param mechanism the mechanism to drive
     * @return a {@link LinearBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<LinearGoal> linear(LinearMechanism mechanism) {
        return linear(nameOf(mechanism), mechanism);
    }

    /**
     * Wraps a {@link RotationalMechanism} — a pivoting arm, a shoulder, a single-axis wrist.
     *
     * @param key       stable, unique, log-safe telemetry key
     * @param mechanism the mechanism to drive
     * @return a {@link RotationalBinding} over {@code mechanism}
     * @throws NullPointerException if either argument is {@code null}
     */
    public static Actuator<RotationalGoal> rotational(String key, RotationalMechanism mechanism) {
        return new RotationalBinding(mechanism, key);
    }

    /**
     * Wraps a {@link RotationalMechanism}, keyed by its own mechanism name.
     *
     * @param mechanism the mechanism to drive
     * @return a {@link RotationalBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<RotationalGoal> rotational(RotationalMechanism mechanism) {
        return rotational(nameOf(mechanism), mechanism);
    }

    /**
     * Wraps a {@link DifferentialWristMechanism} — two motors producing coupled pitch and roll.
     *
     * @param key       stable, unique, log-safe telemetry key
     * @param mechanism the wrist to drive
     * @return a {@link WristBinding} over {@code mechanism}
     * @throws NullPointerException if either argument is {@code null}
     */
    public static Actuator<WristGoal> wrist(String key, DifferentialWristMechanism mechanism) {
        return new WristBinding(mechanism, key);
    }

    /**
     * Wraps a {@link DifferentialWristMechanism} with an explicit homing test.
     *
     * @param key            stable, unique, log-safe telemetry key
     * @param mechanism      the wrist to drive
     * @param zeroedSupplier reports whether the wrist encoders have been referenced
     * @return a {@link WristBinding} over {@code mechanism}
     * @throws NullPointerException if any argument is {@code null}
     */
    public static Actuator<WristGoal> wrist(String key, DifferentialWristMechanism mechanism,
                                           java.util.function.BooleanSupplier zeroedSupplier) {
        return new WristBinding(mechanism, key, zeroedSupplier);
    }

    /**
     * Wraps a {@link DifferentialWristMechanism} with an explicit homing test, keyed by its own name.
     *
     * <p>A differential wrist exposes no {@code hasBeenZeroed()} of its own, so without this the
     * binding reports itself always-homed and a state that depends on an unreferenced wrist is
     * attempted rather than refused. Supply the test and an unhomed wrist becomes a clean
     * {@code NOT_ZEROED} rejection naming the mechanism.
     *
     * @param mechanism      the wrist to drive
     * @param zeroedSupplier reports whether the wrist encoders have been referenced; polled every loop
     * @return a {@link WristBinding} over {@code mechanism}
     * @throws NullPointerException if either argument is {@code null}
     */
    public static Actuator<WristGoal> wrist(DifferentialWristMechanism mechanism,
                                           java.util.function.BooleanSupplier zeroedSupplier) {
        return wrist(nameOf(mechanism), mechanism, zeroedSupplier);
    }

    /**
     * Wraps a {@link DifferentialWristMechanism}, keyed by its own mechanism name.
     *
     * @param mechanism the wrist to drive
     * @return a {@link WristBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<WristGoal> wrist(DifferentialWristMechanism mechanism) {
        return wrist(nameOf(mechanism), mechanism);
    }

    /**
     * Wraps a {@link FlywheelMechanism} — a shooter wheel, a feeder wheel, any velocity-controlled
     * spinner.
     *
     * <p>The logged tolerance defaults to {@link FlywheelBinding#DEFAULT_VELOCITY_TOLERANCE_RPS}. Note
     * that this value is only ever <em>reported</em>: the actual arrival test is the mechanism's own
     * {@code atSpeed()}, using whatever band was configured on the mechanism. Use
     * {@link #flywheel(String, FlywheelMechanism, double)} to mirror that band here, so a plot of
     * {@code Error} against {@code Tolerance} shows the line the mechanism is really applying.
     *
     * @param key       stable, unique, log-safe telemetry key
     * @param mechanism the flywheel to drive
     * @return a {@link FlywheelBinding} over {@code mechanism}
     * @throws IllegalArgumentException if {@code mechanism} is {@code null}
     */
    public static Actuator<FlywheelGoal> flywheel(String key, FlywheelMechanism mechanism) {
        return new FlywheelBinding(mechanism, key);
    }

    /**
     * Wraps a {@link FlywheelMechanism} with an explicit logged tolerance.
     *
     * @param key                  stable, unique, log-safe telemetry key
     * @param mechanism            the flywheel to drive
     * @param velocityToleranceRPS the band mirrored into the log
     * @return a {@link FlywheelBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} or {@code key} is {@code null}
     */
    public static Actuator<FlywheelGoal> flywheel(String key, FlywheelMechanism mechanism,
                                                 double velocityToleranceRPS) {
        return new FlywheelBinding(mechanism, key, velocityToleranceRPS);
    }

    /**
     * Wraps a {@link FlywheelMechanism} with an explicit logged tolerance, keyed by its own name.
     *
     * @param mechanism            the flywheel to drive
     * @param velocityToleranceRPS the band the mechanism's own {@code atSpeed()} applies, mirrored
     *                             here so the logged tolerance matches the one actually in force
     * @return a {@link FlywheelBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<FlywheelGoal> flywheel(FlywheelMechanism mechanism,
                                                 double velocityToleranceRPS) {
        return flywheel(nameOf(mechanism), mechanism, velocityToleranceRPS);
    }

    /**
     * Wraps a {@link FlywheelMechanism}, keyed by its own mechanism name.
     *
     * @param mechanism the flywheel to drive
     * @return a {@link FlywheelBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<FlywheelGoal> flywheel(FlywheelMechanism mechanism) {
        return flywheel(nameOf(mechanism), mechanism);
    }

    /**
     * Wraps a {@link TurretMechanism} with no heading source.
     *
     * <p>Only robot-relative goals can be pursued by a turret bound this way. A field-relative goal
     * — {@link TurretGoal.FieldAngle}, {@link TurretGoal.FieldPoint}, {@link TurretGoal.Solved} —
     * needs to know where the robot is pointing before it can be converted into a turret angle, and
     * this overload has no way to find out. Use
     * {@link #turret(String, TurretMechanism, DoubleSupplier)} for those.
     *
     * @param key       stable, unique, log-safe telemetry key
     * @param mechanism the turret to drive
     * @return a {@link TurretBinding} over {@code mechanism} with no heading supplier
     * @throws IllegalArgumentException if {@code mechanism} is {@code null}
     */
    public static Actuator<TurretGoal> turret(String key, TurretMechanism mechanism) {
        return new TurretBinding(mechanism, key);
    }

    /**
     * Wraps a {@link TurretMechanism} with a robot heading source, unlocking field-relative aiming.
     *
     * <p>{@code robotHeadingDeg} is read live, every loop, on both the pursuit and the arrival
     * path — it is the robot's current field-relative heading in degrees, normally
     * {@code drivetrain::getHeadingDegrees} or the pose estimator's rotation. It is a supplier
     * rather than a value precisely because it changes while the turret is aiming: a field-relative
     * goal has to counter-rotate against the chassis, so a stale heading is a miss.
     *
     * @param key             stable, unique, log-safe telemetry key
     * @param mechanism       the turret to drive
     * @param robotHeadingDeg live robot heading in degrees, CCW-positive, field-relative
     * @return a {@link TurretBinding} over {@code mechanism} that can convert field-relative goals
     * @throws IllegalArgumentException if {@code mechanism} is {@code null}
     */
    public static Actuator<TurretGoal> turret(String key, TurretMechanism mechanism,
                                              DoubleSupplier robotHeadingDeg) {
        return new TurretBinding(mechanism, key, robotHeadingDeg);
    }

    /**
     * Wraps a {@link TurretMechanism} with an explicit travel range, so clamped targets are detected.
     *
     * @param key             stable, unique, log-safe telemetry key
     * @param mechanism       the turret to drive
     * @param robotHeadingDeg robot heading in degrees for field-relative aiming
     * @param minAngleDeg     minimum robot-relative angle in degrees
     * @param maxAngleDeg     maximum robot-relative angle in degrees
     * @return a {@link TurretBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} or {@code key} is {@code null}
     */
    public static Actuator<TurretGoal> turret(String key, TurretMechanism mechanism,
                                             DoubleSupplier robotHeadingDeg,
                                             double minAngleDeg, double maxAngleDeg) {
        return new TurretBinding(mechanism, key, robotHeadingDeg, minAngleDeg, maxAngleDeg);
    }

    /**
     * Wraps a {@link TurretMechanism} with an explicit travel range, keyed by its own name.
     *
     * <p>The range is what lets the binding notice that a requested angle was <em>clamped</em>.
     * {@code TurretMechanism.atSetpoint()} reports true once the turret reaches the clamped angle —
     * so it reports true while pointing somewhere the shot will miss from. Knowing the limits turns
     * that into a blocker note naming the unreachable target and the angle actually taken.
     *
     * @param mechanism       the turret to drive
     * @param robotHeadingDeg robot heading in degrees, needed by the field-relative aiming goals
     * @param minAngleDeg     minimum robot-relative angle in degrees
     * @param maxAngleDeg     maximum robot-relative angle in degrees
     * @return a {@link TurretBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<TurretGoal> turret(TurretMechanism mechanism,
                                             DoubleSupplier robotHeadingDeg,
                                             double minAngleDeg, double maxAngleDeg) {
        return turret(nameOf(mechanism), mechanism, robotHeadingDeg, minAngleDeg, maxAngleDeg);
    }

    /**
     * Wraps a {@link TurretMechanism} with no heading source, keyed by its own mechanism name.
     *
     * @param mechanism the turret to drive
     * @return a {@link TurretBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<TurretGoal> turret(TurretMechanism mechanism) {
        return turret(nameOf(mechanism), mechanism);
    }

    /**
     * Wraps a {@link TurretMechanism} with a heading source, keyed by its own mechanism name.
     *
     * @param mechanism       the turret to drive
     * @param robotHeadingDeg live robot heading in degrees, CCW-positive, field-relative
     * @return a {@link TurretBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<TurretGoal> turret(TurretMechanism mechanism, DoubleSupplier robotHeadingDeg) {
        return turret(nameOf(mechanism), mechanism, robotHeadingDeg);
    }

    /**
     * Wraps a {@link ClawMechanism} — a gripper, a pincher, any hold-or-release end effector.
     *
     * @param key       stable, unique, log-safe telemetry key
     * @param mechanism the claw to drive
     * @return a {@link ClawBinding} over {@code mechanism}
     * @throws NullPointerException if either argument is {@code null}
     */
    public static Actuator<ClawGoal> claw(String key, ClawMechanism mechanism) {
        return new ClawBinding(mechanism, key);
    }

    /**
     * Wraps a {@link ClawMechanism}, keyed by its own mechanism name.
     *
     * @param mechanism the claw to drive
     * @return a {@link ClawBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<ClawGoal> claw(ClawMechanism mechanism) {
        return claw(nameOf(mechanism), mechanism);
    }

    /**
     * Wraps a {@link RollerMechanism} — an intake, an indexer, a feeder.
     *
     * <p>Built with {@link RollerBinding.PieceDetection#UNDECLARED}, which is the honest default:
     * {@code RollerMechanism} does not expose whether a beam break or stall threshold was
     * configured, so a binding made here cannot fail the build on an unreachable
     * {@link RollerGoal.IntakeUntilPiece} and warns in its note instead. A team that knows the
     * answer should construct {@link RollerBinding} directly with the three-argument constructor
     * and get the build-time check.
     *
     * @param key       stable, unique, log-safe telemetry key
     * @param mechanism the roller to drive
     * @return a {@link RollerBinding} over {@code mechanism}
     * @throws NullPointerException if either argument is {@code null}
     */
    public static Actuator<RollerGoal> roller(String key, RollerMechanism mechanism) {
        return new RollerBinding(mechanism, key);
    }

    /**
     * Wraps a {@link RollerMechanism}, keyed by its own mechanism name.
     *
     * @param mechanism the roller to drive
     * @return a {@link RollerBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<RollerGoal> roller(RollerMechanism mechanism) {
        return roller(nameOf(mechanism), mechanism);
    }

    /**
     * Wraps a {@link RollerMechanism} and declares whether it can actually sense a game piece.
     *
     * <p>Worth the extra argument. A {@link RollerGoal.IntakeUntilPiece} goal on a roller built with
     * neither a beam break nor stall detection can only ever expire on its timeout — the roller will
     * spin, the transition will stall, and the log will say the roller never arrived, with nothing to
     * suggest that arrival was impossible by construction. Declaring
     * {@link RollerBinding.PieceDetection#ABSENT} turns that into a build error on a laptop.
     *
     * <p>The two-argument form defaults to {@link RollerBinding.PieceDetection#UNDECLARED}, which
     * assumes detection may be present and carries the caveat in the binding's note instead — because
     * guessing the other way would fail builds on perfectly healthy robots.
     *
     * @param key       stable, unique, log-safe telemetry key
     * @param mechanism the roller to drive
     * @param detection whether the mechanism has piece detection configured
     * @return a {@link RollerBinding} over {@code mechanism}
     * @throws NullPointerException if any argument is {@code null}
     */
    public static Actuator<RollerGoal> roller(String key, RollerMechanism mechanism,
                                              RollerBinding.PieceDetection detection) {
        return new RollerBinding(mechanism, key, detection);
    }

    /**
     * Wraps a {@link RollerMechanism} with declared piece detection, keyed by its own mechanism name.
     *
     * @param mechanism the roller to drive
     * @param detection whether the mechanism has piece detection configured
     * @return a {@link RollerBinding} over {@code mechanism}
     * @throws NullPointerException if either argument is {@code null}
     */
    public static Actuator<RollerGoal> roller(RollerMechanism mechanism,
                                              RollerBinding.PieceDetection detection) {
        return roller(nameOf(mechanism), mechanism, detection);
    }

    /**
     * Wraps a {@link WinchMechanism} — a climber spool, a rope winch, any wind-in/wind-out drum.
     *
     * @param key       stable, unique, log-safe telemetry key
     * @param mechanism the winch to drive
     * @return a {@link WinchBinding} over {@code mechanism}
     * @throws NullPointerException if either argument is {@code null}
     */
    public static Actuator<WinchGoal> winch(String key, WinchMechanism mechanism) {
        return new WinchBinding(mechanism, key);
    }

    /**
     * Wraps a {@link WinchMechanism}, keyed by its own mechanism name.
     *
     * @param mechanism the winch to drive
     * @return a {@link WinchBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<WinchGoal> winch(WinchMechanism mechanism) {
        return winch(nameOf(mechanism), mechanism);
    }

    /**
     * Wraps a {@link PneumaticMechanism} — a single or double solenoid.
     *
     * <p>No minimum-pressure gate is applied. Construct {@link PneumaticBinding} directly with its
     * three-argument constructor to refuse actuation below a pressure threshold.
     *
     * @param key       stable, unique, log-safe telemetry key
     * @param mechanism the solenoid to drive
     * @return a {@link PneumaticBinding} over {@code mechanism}
     * @throws NullPointerException if either argument is {@code null}
     */
    public static Actuator<PneumaticGoal> pneumatic(String key, PneumaticMechanism mechanism) {
        return new PneumaticBinding(mechanism, key);
    }

    /**
     * Wraps a {@link PneumaticMechanism} and mirrors its pressure gate into the blocker message.
     *
     * @param key            stable, unique, log-safe telemetry key
     * @param mechanism      the pneumatic mechanism to drive
     * @param minPressurePSI the mirrored threshold in psi, or {@link Double#NaN} if there is none
     * @return a {@link PneumaticBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} or {@code key} is {@code null}
     */
    public static Actuator<PneumaticGoal> pneumatic(String key, PneumaticMechanism mechanism,
                                                   double minPressurePSI) {
        return new PneumaticBinding(mechanism, key, minPressurePSI);
    }

    /**
     * Wraps a {@link PneumaticMechanism} and mirrors its pressure gate, keyed by its own name.
     *
     * <p>Pass the same number given to {@code Config.Builder.requirePressureAbove(psi)}. It affects
     * no actuation decision — it only lets the blocker message name the actual threshold instead of
     * something vaguer, which is the difference between a diagnosable stall and a mystery.
     *
     * @param mechanism      the pneumatic mechanism to drive
     * @param minPressurePSI the mirrored threshold in psi, or {@link Double#NaN} if there is none
     * @return a {@link PneumaticBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<PneumaticGoal> pneumatic(PneumaticMechanism mechanism,
                                                   double minPressurePSI) {
        return pneumatic(nameOf(mechanism), mechanism, minPressurePSI);
    }

    /**
     * Wraps a {@link PneumaticMechanism}, keyed by its own mechanism name.
     *
     * @param mechanism the solenoid to drive
     * @return a {@link PneumaticBinding} over {@code mechanism}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    public static Actuator<PneumaticGoal> pneumatic(PneumaticMechanism mechanism) {
        return pneumatic(nameOf(mechanism), mechanism);
    }

    // ==================================================================
    // Escape hatches
    // ==================================================================

    /**
     * Tier 1 — a fire-and-forget output that is always at goal.
     *
     * <p>For LEDs, dashboard writes, a candle pattern, a servo nudge: anything whose effect is
     * instantaneous and whose completion there is nothing to wait for. {@code apply} runs once per
     * goal change, inside a {@code runOnce}, and {@link Binding#atGoal} is hard-wired {@code true}
     * so a state carrying only this binding completes on the loop it is entered.
     *
     * <p>{@link Binding#observable} stays {@code true} here, and that is deliberate rather than an
     * oversight. {@code observable == false} means "arrival is a timer pretending to be a sensor",
     * which is a warning to whoever reads the log. This binding is not pretending: there genuinely
     * is nothing to sense, because setting an LED pattern is complete the instant it is set.
     *
     * @param <G>      the goal type; must have value-based {@code equals}/{@code hashCode}
     * @param key      stable, unique, log-safe telemetry key
     * @param apply    invoked once each time the goal changes
     * @param requires subsystems this actuator owns; may be empty for a pure logging sink
     * @return an actuator that applies {@code apply} and immediately reports arrival
     * @throws NullPointerException if {@code key} or {@code apply} is {@code null}
     */
    public static <G> Actuator<G> instant(String key, Consumer<G> apply, Subsystem... requires) {
        Objects.requireNonNull(apply, "apply");
        return Mechanisms.<G>build(key, requires)
                .pursue(goal -> Commands.runOnce(() -> apply.accept(goal), requires))
                .atGoal(goal -> true)
                .done();
    }

    /**
     * Tier 2 — direct method calls plus a real arrival test. The simplest binding that can gate a
     * transition.
     *
     * <p>{@code apply} is wrapped in a {@code Commands.run}, so it is called every loop for as long
     * as the goal is active rather than once at the start. That is the right default for a setter
     * that writes a setpoint into a controller, and it is harmless for an idempotent setter. A
     * subsystem that must be poked exactly once should use {@link #commands} with a
     * {@code runOnce} factory instead.
     *
     * <p>{@code atGoal} here takes only the goal, so it cannot express a settle timer — which is
     * the point of this tier. If arrival is really "half a second has passed", say so explicitly
     * with {@link CustomBuilder#settle(double)}, which also flips {@link Binding#observable} to
     * {@code false} so nobody reading a log mistakes the timer for a sensor.
     *
     * @param <G>      the goal type; must have value-based {@code equals}/{@code hashCode}
     * @param key      stable, unique, log-safe telemetry key
     * @param apply    invoked every loop while the goal is active
     * @param atGoal   live arrival test; must be a pure function of sensors and the goal
     * @param requires subsystems this actuator owns
     * @return an actuator driving {@code apply} and gating on {@code atGoal}
     * @throws NullPointerException if {@code key}, {@code apply} or {@code atGoal} is {@code null}
     */
    public static <G> Actuator<G> custom(String key, Consumer<G> apply, Predicate<G> atGoal,
                                         Subsystem... requires) {
        return Mechanisms.<G>build(key, requires)
                .apply(apply)
                .atGoal(atGoal)
                .done();
    }

    /**
     * Tier 3 — the subsystem already exposes {@link Command} factories.
     *
     * <p>This is the natural tier for a subsystem written in ordinary WPILib style, where the
     * public surface is {@code Command intakeCommand()} rather than {@code void setIntake()}.
     *
     * <p>{@code factory} <b>must return a fresh command instance on every call</b>. The state
     * machine hosts commands rather than scheduling them, calling {@code initialize} directly, and
     * it will do so many times over a match — re-entering the state, re-asserting after the command
     * ended early, recovering from an override. A factory that caches and returns the same instance
     * produces a command that is initialised twice without being ended in between, which in WPILib
     * is undefined behaviour and in practice is a mechanism that stops responding partway through a
     * match. A method reference to a {@code Commands.run(...)} factory is fresh; a field holding a
     * command is not.
     *
     * <p>For the same reason the factory must not be a {@code goToAndWait}-style composition.
     * Arrival is decided by {@code atGoal}, never by a command finishing; a command that ends on
     * arrival simply stops driving, and the mechanism sags.
     *
     * @param <G>      the goal type; must have value-based {@code equals}/{@code hashCode}
     * @param key      stable, unique, log-safe telemetry key
     * @param factory  produces a fresh command for a goal on every call
     * @param atGoal   live arrival test; must be a pure function of sensors and the goal
     * @param requires subsystems this actuator owns; must cover the commands' own requirements
     * @return an actuator that hosts {@code factory}'s commands and gates on {@code atGoal}
     * @throws NullPointerException if {@code key}, {@code factory} or {@code atGoal} is {@code null}
     */
    public static <G> Actuator<G> commands(String key, Function<G, Command> factory,
                                           Predicate<G> atGoal, Subsystem... requires) {
        return Mechanisms.<G>build(key, requires)
                .pursue(factory)
                .atGoal(atGoal)
                .done();
    }

    /**
     * Tier 4 — full control over every part of the {@link Actuator} contract.
     *
     * <p>Everything the nine typed bindings publish is reachable from the returned builder:
     * measurement and units for plotting, tolerance for a readable plot, range checks that fail the
     * build instead of the match, notes that explain a stuck state, re-assertion for actuations
     * that can be silently refused. Nothing is reserved for library mechanisms.
     *
     * <p><b>Write the type argument explicitly:</b> {@code Mechanisms.<MyGoal>build("Key", sub)}.
     * This is a requirement rather than a style preference, and it is worth understanding why so the
     * error message makes sense when it appears. {@code G} does not occur in this method's parameter
     * list, and Java resolves the type arguments of a call at the head of a chain from that call
     * alone — the assignment on the far side of {@code done()} is too late to participate. So a bare
     * {@code build("Key", sub)} infers {@code G} as {@code Object}, and the failure surfaces one
     * call later as "cannot find symbol" on whatever the first lambda tries to read off its goal,
     * which is a confusing place to be told about a missing type argument. The three
     * shorthand factories above do not have this problem: they return {@code Actuator<G>} directly,
     * with no chain in between, so ordinary target typing infers {@code G} from the variable being
     * assigned to.
     *
     * @param <G>      the goal type; must have value-based {@code equals}/{@code hashCode}
     * @param key      stable, unique, log-safe telemetry key
     * @param requires subsystems this actuator owns
     * @return a fresh builder; finish it with {@link CustomBuilder#done()}
     * @throws NullPointerException if {@code key} is {@code null}, or {@code requires} contains
     *                              {@code null}
     */
    public static <G> CustomBuilder<G> build(String key, Subsystem... requires) {
        return new CustomBuilder<>(key, requires);
    }

    /**
     * Reads a mechanism's own name for use as a binding key.
     *
     * <p>Extracted so the {@code NullPointerException} from a null mechanism names the parameter
     * rather than surfacing as an opaque failure inside {@code getMechanismName()}.
     *
     * @param mechanism the mechanism to name
     * @return {@code mechanism.getMechanismName()}
     * @throws NullPointerException if {@code mechanism} is {@code null}
     */
    private static String nameOf(CatalystMechanism mechanism) {
        return Objects.requireNonNull(mechanism, "mechanism").getMechanismName();
    }

    // ==================================================================
    // Builder
    // ==================================================================

    /**
     * Assembles a fully-featured {@link Actuator} for a subsystem the library has never heard of.
     *
     * <p>Every method returns {@code this} for chaining and every one is optional except that the
     * configuration must end up with a way to actuate ({@link #pursue(Function)} or
     * {@link #apply(Consumer)}) and a way
     * to decide arrival ({@link #atGoal(Predicate)}, {@link #atGoal(BiPredicate)} or
     * {@link #settle}). {@link #done()} enforces exactly that and nothing more.
     *
     * <p>Repeated calls overwrite, last one wins, with one exception: {@link #range} accumulates, so
     * several independent range checks can be layered onto one goal type.
     *
     * <p>The builder is not thread-safe and is not meant to be. It is used once, on the main thread,
     * while the robot is still initialising.
     *
     * @param <G> the goal type; must have value-based {@code equals}/{@code hashCode}
     * @since 1.2.0
     */
    public static final class CustomBuilder<G> {

        /** Telemetry key for the finished actuator. Never {@code null}. */
        private final String key;

        /** Owned subsystems, in declaration order, deduplicated. Never {@code null}. */
        private final Set<Subsystem> requirements;

        /**
         * The same subsystems as an array, kept because {@code Commands.run} and friends take
         * varargs and rebuilding the array on every {@code pursueCommand} call would allocate on a
         * path that runs whenever a goal changes.
         */
        private final Subsystem[] requiresArray;

        /** {@code MechanismView} kind string. Defaults to {@code "custom"}. */
        private String kind = "custom";

        /** Unit label for {@link #measured(DoubleSupplier)}. Defaults to none. */
        private String unit = "";

        /** Produces the pursue command. {@code null} until {@link #pursue(Function)} or {@link #apply(Consumer)}. */
        private Function<G, Command> pursue;

        /** Produces the post-arrival hold command, or {@code null} for "keep pursuing". */
        private Function<G, Command> hold;

        /** Arrival test. {@code null} until {@link #atGoal(Predicate)} or {@link #settle}. */
        private BiPredicate<G, Double> atGoal;

        /** Live measurement source, or {@code null} for {@code NaN}. */
        private DoubleSupplier measured;

        /** Signed error toward a goal, or {@code null} for {@code NaN}. */
        private ToDoubleFunction<G> error;

        /** Tolerance band, or {@code NaN} when unstated. */
        private double tolerance = Double.NaN;

        /** Explicit observability test. {@code null} means "use {@link #observableDefault}". */
        private Predicate<G> observable;

        /**
         * What {@link Binding#observable} answers when no explicit test was given. {@link #settle}
         * flips this to {@code false}; an explicit {@link #observable(Predicate)} outranks it, in
         * either call order.
         */
        private boolean observableDefault = true;

        /** Low-cardinality label source, or {@code null} for {@code String.valueOf(goal)}. */
        private Function<G, String> label;

        /** Runtime note source, or {@code null} for {@code ""}. */
        private Function<G, String> note;

        /** Homing test, or {@code null} for "always zeroed". */
        private BooleanSupplier zeroed;

        /** Re-assertion period in loops. {@code 0} disables. */
        private int reassertPeriodLoops;

        /** Release hook, or {@code null} for no-op. */
        private Runnable onRelease;

        /** Accumulated build-time range checks. Empty until {@link #range} is called. */
        private final List<RangeCheck<G>> ranges = new ArrayList<>();

        /**
         * Starts a builder. Use {@link Mechanisms#build(String, Subsystem...)} rather than calling
         * this directly.
         *
         * @param key      stable, unique, log-safe telemetry key
         * @param requires subsystems this actuator owns
         * @throws NullPointerException if {@code key} is {@code null}, or any element of
         *                              {@code requires} is {@code null}
         */
        private CustomBuilder(String key, Subsystem... requires) {
            this.key = Objects.requireNonNull(key, "key");
            Subsystem[] source = requires == null ? new Subsystem[0] : requires;
            Set<Subsystem> unique = new LinkedHashSet<>();
            for (int i = 0; i < source.length; i++) {
                unique.add(Objects.requireNonNull(source[i], "requires[" + i + "]"));
            }
            this.requirements = Collections.unmodifiableSet(unique);
            this.requiresArray = unique.toArray(new Subsystem[0]);
        }

        /**
         * Sets the {@code MechanismView}-compatible kind string.
         *
         * <p>Worth setting to one of the standard kinds — {@code "linear"}, {@code "rotational"},
         * {@code "flywheel"}, {@code "roller"} — when the custom subsystem behaves like one, since
         * a dashboard will then render it with the type-specific gauge instead of a generic
         * readout.
         *
         * @param kind the kind string; {@code null} is treated as {@code "custom"}
         * @return {@code this}
         */
        public CustomBuilder<G> kind(String kind) {
            this.kind = kind == null ? "custom" : kind;
            return this;
        }

        /**
         * Sets the unit label that goes with {@link #measured(DoubleSupplier)}.
         *
         * @param unit the unit, such as {@code "m"}, {@code "deg"} or {@code "rps"}; {@code null}
         *             is treated as no unit
         * @return {@code this}
         */
        public CustomBuilder<G> unit(String unit) {
            this.unit = unit == null ? "" : unit;
            return this;
        }

        /**
         * Sets the command factory that drives the mechanism toward a goal.
         *
         * <p>The factory must produce a <b>fresh instance on every call</b>, must not be a
         * {@code goToAndWait}-style composition, and must either never end or end with a persistent
         * effect. See {@link Mechanisms#commands} for why each of those matters.
         *
         * @param pursue produces a fresh command per call
         * @return {@code this}
         * @throws NullPointerException if {@code pursue} is {@code null}
         */
        public CustomBuilder<G> pursue(Function<G, Command> pursue) {
            this.pursue = Objects.requireNonNull(pursue, "pursue");
            return this;
        }

        /**
         * Sets actuation as a plain method call, wrapped into a {@code Commands.run} against this
         * builder's requirements.
         *
         * <p>{@code apply} therefore runs <em>every loop</em> while the goal is active, not once.
         * The resulting command never ends, which satisfies the pursue contract without the caller
         * having to think about it. This and {@link #pursue(Function)} write the same field, so whichever is
         * called last wins.
         *
         * @param apply invoked every loop while the goal is active
         * @return {@code this}
         * @throws NullPointerException if {@code apply} is {@code null}
         */
        public CustomBuilder<G> apply(Consumer<G> apply) {
            Objects.requireNonNull(apply, "apply");
            this.pursue = goal -> Commands.run(() -> apply.accept(goal), requiresArray);
            return this;
        }

        /**
         * Sets the command to run once arrival first becomes {@code true}.
         *
         * <p>Leave this unset for a closed-loop mechanism whose pursuit already holds position —
         * that is the {@code null} case, meaning "keep pursuing", and it is what an elevator wants.
         * Set it for anything open-loop that would otherwise keep driving into a hard stop after it
         * arrives: a winch that has finished extending, a roller that has finished ejecting, any
         * duty-cycle goal. The factory returns a fresh instance per call, same as {@link #pursue(Function)}.
         *
         * @param hold produces a fresh hold command per call; may return {@code null} per goal to
         *             keep pursuing for that goal specifically
         * @return {@code this}
         * @throws NullPointerException if {@code hold} is {@code null}
         */
        public CustomBuilder<G> hold(Function<G, Command> hold) {
            this.hold = Objects.requireNonNull(hold, "hold");
            return this;
        }

        /**
         * Sets an arrival test that can consult elapsed time.
         *
         * <p>The second argument is {@code secondsSinceApplied}, exactly as described on
         * {@link Binding#atGoal} — seconds since this goal was first applied, or {@code 0.0} when
         * it is not currently applied. It is the only sanctioned source of elapsed time; calling
         * {@code Timer.getFPGATimestamp()} from inside the predicate makes the binding untestable
         * and breaks disabled-mode accounting.
         *
         * <p>The test must be a <b>pure function</b> of live sensors, the goal and the elapsed
         * time. It is called for goals belonging to states the machine is not in, which is what
         * keeps {@code isAt} a measurement rather than a latch, so a predicate that consults a
         * "last applied goal" field answers the wrong question.
         *
         * <p>Prefer {@link #atGoal(Predicate)} when time is not needed: this overload boxes the
         * elapsed time into a {@link Double} on every call, and every call means every binding
         * every loop.
         *
         * @param atGoal live arrival test over goal and elapsed seconds
         * @return {@code this}
         * @throws NullPointerException if {@code atGoal} is {@code null}
         */
        public CustomBuilder<G> atGoal(BiPredicate<G, Double> atGoal) {
            this.atGoal = Objects.requireNonNull(atGoal, "atGoal");
            return this;
        }

        /**
         * Sets an arrival test that depends only on live sensors and the goal.
         *
         * <p>The common case, and the allocation-free one. Same purity requirement as
         * {@link #atGoal(BiPredicate)}.
         *
         * @param atGoal live arrival test over the goal
         * @return {@code this}
         * @throws NullPointerException if {@code atGoal} is {@code null}
         */
        public CustomBuilder<G> atGoal(Predicate<G> atGoal) {
            Objects.requireNonNull(atGoal, "atGoal");
            this.atGoal = (goal, seconds) -> atGoal.test(goal);
            return this;
        }

        /**
         * Declares that arrival cannot be sensed and is really a fixed settle time.
         *
         * <p>Arrival becomes {@code secondsSinceApplied >= seconds}, and {@link Binding#observable}
         * becomes {@code false} — which is the honest half of this method. A settle timer and a
         * sensor look identical in a log until something goes wrong, at which point the difference
         * is the whole diagnosis, so the binding says out loud which one it is. An explicit
         * {@link #observable(Predicate)} still outranks this, in either call order, for the case
         * where arrival is sensed for some goals and timed for others.
         *
         * <p>{@code seconds <= 0} yields a binding that is at goal immediately, since
         * {@code secondsSinceApplied} starts at zero. That is legal and occasionally what is wanted,
         * but {@link Mechanisms#instant} says it more clearly.
         *
         * @param seconds settle time in seconds
         * @return {@code this}
         */
        public CustomBuilder<G> settle(double seconds) {
            this.atGoal = (goal, elapsed) -> elapsed >= seconds;
            this.observableDefault = false;
            return this;
        }

        /**
         * Sets the live measured value published as {@code Bindings/<key>/Measured}.
         *
         * <p>Pair it with {@link #unit(String)}; a number with no unit in a log is a number nobody
         * trusts.
         *
         * @param measured live measurement source
         * @return {@code this}
         * @throws NullPointerException if {@code measured} is {@code null}
         */
        public CustomBuilder<G> measured(DoubleSupplier measured) {
            this.measured = Objects.requireNonNull(measured, "measured");
            return this;
        }

        /**
         * Sets the signed error toward a goal, for logging.
         *
         * <p>Sign convention is the binding's own, but it should be consistent — conventionally
         * measured minus target, so a positive error means overshoot.
         *
         * @param error signed error function
         * @return {@code this}
         * @throws NullPointerException if {@code error} is {@code null}
         */
        public CustomBuilder<G> error(ToDoubleFunction<G> error) {
            this.error = Objects.requireNonNull(error, "error");
            return this;
        }

        /**
         * Sets the tolerance band reported alongside {@link #error(ToDoubleFunction)}, for logging.
         *
         * <p>This is a reporting value only — it does not feed the arrival test, which stays
         * whatever {@link #atGoal(Predicate)} or {@link #settle} was given. Set it to the same number the
         * arrival test uses, or the log will draw a band the mechanism is not actually judged
         * against.
         *
         * @param tolerance the tolerance band in {@link #unit(String)}
         * @return {@code this}
         */
        public CustomBuilder<G> tolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        /**
         * Adds a build-time range check on some component of the goal.
         *
         * <p>The extracted value must lie within {@code [min, max]} inclusive, or
         * {@link Binding#validate} reports it and the builder aggregates it into the single
         * {@code StateMachineConfigException} raised at build. This is the whole point of
         * {@code validate}: an out-of-range setpoint becomes a laptop-side failure listing every
         * bad goal at once, rather than a mechanism that sits perfectly still in a match while
         * {@code atGoal} reports false forever.
         *
         * <p>Calls accumulate, so a goal with several numeric components gets several checks.
         *
         * @param min       inclusive lower bound
         * @param max       inclusive upper bound
         * @param extractor pulls the value to check out of a goal
         * @return {@code this}
         * @throws NullPointerException if {@code extractor} is {@code null}
         */
        public CustomBuilder<G> range(double min, double max, ToDoubleFunction<G> extractor) {
            ranges.add(new RangeCheck<>(min, max, Objects.requireNonNull(extractor, "extractor")));
            return this;
        }

        /**
         * Sets an explicit observability test, overriding whatever {@link #settle} implied.
         *
         * <p>For a binding whose arrival is genuinely sensed for some goals and timed for others —
         * a claw that can sense a grip but only time an open, say.
         *
         * @param observable {@code true} when arrival for this goal is really sensed
         * @return {@code this}
         * @throws NullPointerException if {@code observable} is {@code null}
         */
        public CustomBuilder<G> observable(Predicate<G> observable) {
            this.observable = Objects.requireNonNull(observable, "observable");
            return this;
        }

        /**
         * Sets the low-cardinality label for a goal.
         *
         * <p>The label is logged and edge-detected, so it must not interpolate a live value — a
         * label carrying a changing float writes to the log at 50 Hz for the whole match. Put the
         * live numbers in {@link #note(Function)} or leave them to {@code detail}, which is never
         * edge-detected. For a goal type that carries a supplier, this should return the goal's
         * explicit label component rather than anything derived from the supplier.
         *
         * @param label produces a stable, low-cardinality name for a goal
         * @return {@code this}
         * @throws NullPointerException if {@code label} is {@code null}
         */
        public CustomBuilder<G> label(Function<G, String> label) {
            this.label = Objects.requireNonNull(label, "label");
            return this;
        }

        /**
         * Sets the free-form runtime note surfaced in {@code BlockerDetail}.
         *
         * <p>This is what turns "the robot is stuck in SCORE" into "the shooter is stalled". Return
         * {@code ""} when there is nothing to say; a note that always has content is noise.
         *
         * @param note produces a note for a goal, or {@code ""}
         * @return {@code this}
         * @throws NullPointerException if {@code note} is {@code null}
         */
        public CustomBuilder<G> note(Function<G, String> note) {
            this.note = Objects.requireNonNull(note, "note");
            return this;
        }

        /**
         * Sets the homing test.
         *
         * <p>A state whose gating bindings are not all zeroed is rejected outright rather than
         * attempted, which is what stops an unhomed arm from being commanded to a position it will
         * interpret in the wrong frame.
         *
         * @param zeroed {@code true} when the mechanism is homed
         * @return {@code this}
         * @throws NullPointerException if {@code zeroed} is {@code null}
         */
        public CustomBuilder<G> zeroed(BooleanSupplier zeroed) {
            this.zeroed = Objects.requireNonNull(zeroed, "zeroed");
            return this;
        }

        /**
         * Re-runs the pursue command's {@code initialize()} every {@code loops} loops for as long as
         * the mechanism has not arrived.
         *
         * <p>For actuations that can be silently refused and need retrying rather than quietly
         * never happening — the canonical case is a solenoid that would not fire because pressure
         * was low. The pneumatic binding uses 25, about half a second at 50 Hz.
         *
         * @param loops period in robot loops; {@code 0} or negative disables
         * @return {@code this}
         */
        public CustomBuilder<G> reassertEvery(int loops) {
            this.reassertPeriodLoops = Math.max(0, loops);
            return this;
        }

        /**
         * Sets a hook run when the machine relinquishes this binding, on state release or override.
         *
         * <p>The place to stop an open-loop output that would otherwise keep running after the
         * state machine has stopped caring about it.
         *
         * @param onRelease the hook
         * @return {@code this}
         * @throws NullPointerException if {@code onRelease} is {@code null}
         */
        public CustomBuilder<G> onRelease(Runnable onRelease) {
            this.onRelease = Objects.requireNonNull(onRelease, "onRelease");
            return this;
        }

        /**
         * Finishes the builder and produces the actuator.
         *
         * <p>Throws when the configuration cannot work, naming the call that is missing. Throwing is
         * right here and wrong almost everywhere else in this package: this runs during robot
         * construction, before the command scheduler exists, so an exception is a stack trace on a
         * laptop rather than a dead robot loop. The alternative — degrading to a binding that never
         * actuates or never arrives — would turn a typo into a mechanism that mysteriously does
         * nothing, discovered in a pit.
         *
         * @return the finished actuator; a fresh instance, safe to hold and reuse
         * @throws IllegalStateException if neither {@link #pursue(Function)} nor
         *                               {@link #apply(Consumer)} was called, or if neither
         *                               {@link #atGoal(Predicate)} nor {@link #settle} was called
         */
        public Actuator<G> done() {
            if (pursue == null) {
                throw new IllegalStateException(
                        "Binding \"" + key + "\" has no way to actuate: call pursue(Function<G,Command>) "
                                + "or apply(Consumer<G>) before done().");
            }
            if (atGoal == null) {
                throw new IllegalStateException(
                        "Binding \"" + key + "\" has no way to decide arrival: call atGoal(Predicate<G>), "
                                + "atGoal(BiPredicate<G,Double>) or settle(double) before done().");
            }
            return new Custom<>(this);
        }

        /**
         * Alias for {@link #done()}, for callers who read the chain as a build rather than a
         * completion. Identical behaviour, identical exceptions.
         *
         * @return the finished actuator
         * @throws IllegalStateException under exactly the conditions {@link #done()} documents
         */
        public Actuator<G> build() {
            return done();
        }
    }

    /**
     * One accumulated build-time range check: the bounds, and how to pull the value out of a goal.
     *
     * @param <G> the goal type
     */
    private static final class RangeCheck<G> {

        /** Inclusive lower bound. */
        private final double min;

        /** Inclusive upper bound. */
        private final double max;

        /** Pulls the checked value out of a goal. */
        private final ToDoubleFunction<G> extractor;

        /**
         * @param min       inclusive lower bound
         * @param max       inclusive upper bound
         * @param extractor pulls the checked value out of a goal
         */
        private RangeCheck(double min, double max, ToDoubleFunction<G> extractor) {
            this.min = min;
            this.max = max;
            this.extractor = extractor;
        }

        /**
         * Runs the check against one goal, reporting at most one problem.
         *
         * <p>A {@code NaN} extraction is reported too. It is almost always an unresolved preset or
         * an uninitialised field, and silently passing it through would command {@code NaN} into a
         * motor controller.
         *
         * @param goal     the goal to check
         * @param problems where to report a failure
         */
        private void check(G goal, Consumer<String> problems) {
            double value = extractor.applyAsDouble(goal);
            if (Double.isNaN(value)) {
                problems.accept("value for goal " + goal + " is NaN; expected a number within ["
                        + min + ", " + max + "]");
            } else if (value < min || value > max) {
                problems.accept("value " + value + " for goal " + goal + " is outside the allowed range ["
                        + min + ", " + max + "]");
            }
        }
    }

    /**
     * The actuator {@link CustomBuilder#done()} produces.
     *
     * <p>Every field is final and every unset option collapses to the interface default, so this
     * class is immutable after construction and safe to share. It holds no "last applied goal"
     * state of any kind, which is what keeps {@link Binding#atGoal} the pure measurement the engine needs
     * when it asks about states the machine is not in.
     *
     * <p>Nothing here throws at runtime. Where a user-supplied factory hands back {@code null}, or a
     * {@code null} goal arrives, the binding degrades to a do-nothing command rather than letting an
     * exception escape into {@code CommandScheduler.run()} and take the robot loop with it. Genuine
     * exceptions from inside a user's own lambda are deliberately <em>not</em> swallowed — hiding
     * those would trade a visible crash for an invisible one.
     *
     * @param <G> the goal type
     */
    private static final class Custom<G> implements Actuator<G> {

        /** Telemetry key. */
        private final String key;

        /** {@code MechanismView} kind string. */
        private final String kind;

        /** Unit label for {@link #measured()}. */
        private final String unit;

        /** Owned subsystems, unmodifiable. */
        private final Set<Subsystem> requirements;

        /** Pursue command factory. Never {@code null} — {@code done()} guarantees it. */
        private final Function<G, Command> pursue;

        /** Hold command factory, or {@code null} for "keep pursuing". */
        private final Function<G, Command> hold;

        /** Arrival test. Never {@code null} — {@code done()} guarantees it. */
        private final BiPredicate<G, Double> atGoal;

        /** Live measurement, or {@code null} for {@code NaN}. */
        private final DoubleSupplier measured;

        /** Signed error, or {@code null} for {@code NaN}. */
        private final ToDoubleFunction<G> error;

        /** Tolerance band, or {@code NaN}. */
        private final double tolerance;

        /** Explicit observability test, or {@code null} to use {@link #observableDefault}. */
        private final Predicate<G> observable;

        /** Observability when no explicit test was given. */
        private final boolean observableDefault;

        /** Label source, or {@code null} for {@code String.valueOf(goal)}. */
        private final Function<G, String> label;

        /** Note source, or {@code null} for {@code ""}. */
        private final Function<G, String> note;

        /** Homing test, or {@code null} for always-zeroed. */
        private final BooleanSupplier zeroed;

        /** Re-assertion period in loops; {@code 0} disables. */
        private final int reassertPeriodLoops;

        /** Release hook, or {@code null}. */
        private final Runnable onRelease;

        /** Build-time range checks; empty when none were declared. */
        private final List<RangeCheck<G>> ranges;

        /**
         * Copies a validated builder. Private because {@link CustomBuilder#done()} is the only
         * caller and it has already checked the two mandatory options.
         *
         * @param b the finished builder
         */
        private Custom(CustomBuilder<G> b) {
            this.key = b.key;
            this.kind = b.kind;
            this.unit = b.unit;
            this.requirements = b.requirements;
            this.pursue = b.pursue;
            this.hold = b.hold;
            this.atGoal = b.atGoal;
            this.measured = b.measured;
            this.error = b.error;
            this.tolerance = b.tolerance;
            this.observable = b.observable;
            this.observableDefault = b.observableDefault;
            this.label = b.label;
            this.note = b.note;
            this.zeroed = b.zeroed;
            this.reassertPeriodLoops = b.reassertPeriodLoops;
            this.onRelease = b.onRelease;
            this.ranges = List.copyOf(b.ranges);
        }

        /** {@inheritDoc} */
        @Override
        public String key() {
            return key;
        }

        /** {@inheritDoc} Whatever {@link CustomBuilder#kind(String)} declared, or {@code "custom"}. */
        @Override
        public String kind() {
            return kind;
        }

        /** {@inheritDoc} */
        @Override
        public String unit() {
            return unit;
        }

        /**
         * {@inheritDoc}
         *
         * <p>A fresh instance every call, provided the configured factory honours its contract. A
         * factory that returns {@code null} — or a {@code null} goal — degrades to
         * {@code Commands.none()} rather than letting a {@code NullPointerException} reach the
         * scheduler.
         */
        @Override
        public Command pursueCommand(G goal) {
            if (goal == null) {
                return Commands.none().withName(key + ".NoGoal");
            }
            Command command = pursue.apply(goal);
            return command == null ? Commands.none().withName(key + ".NoPursue") : command;
        }

        /**
         * {@inheritDoc}
         *
         * <p>{@code null} when no hold factory was configured, meaning "keep pursuing".
         */
        @Override
        public Command holdCommand(G goal) {
            if (hold == null || goal == null) {
                return null;
            }
            return hold.apply(goal);
        }

        /** {@inheritDoc} */
        @Override
        public Set<Subsystem> requirements() {
            return requirements;
        }

        /** {@inheritDoc} */
        @Override
        public int reassertPeriodLoops() {
            return reassertPeriodLoops;
        }

        /**
         * {@inheritDoc}
         *
         * <p>Delegates straight to the configured predicate. A {@code null} goal is never at goal,
         * which is the safe answer — reporting arrival for a goal that does not exist would let a
         * transition complete without anything having moved.
         */
        @Override
        public boolean atGoal(G goal, double secondsSinceApplied) {
            if (goal == null) {
                return false;
            }
            return atGoal.test(goal, secondsSinceApplied);
        }

        /** {@inheritDoc} {@code NaN} when no measurement source was configured. */
        @Override
        public double measured() {
            return measured == null ? Double.NaN : measured.getAsDouble();
        }

        /** {@inheritDoc} {@code NaN} when no error function was configured. */
        @Override
        public double error(G goal) {
            if (error == null || goal == null) {
                return Double.NaN;
            }
            return error.applyAsDouble(goal);
        }

        /** {@inheritDoc} {@code NaN} when no tolerance was declared. */
        @Override
        public double tolerance(G goal) {
            return tolerance;
        }

        /**
         * {@inheritDoc}
         *
         * <p>An explicit {@link CustomBuilder#observable(Predicate)} wins; otherwise this is
         * {@code true}, or {@code false} when {@link CustomBuilder#settle(double)} made arrival a
         * timer.
         */
        @Override
        public boolean observable(G goal) {
            if (observable == null || goal == null) {
                return observableDefault;
            }
            return observable.test(goal);
        }

        /** {@inheritDoc} {@code String.valueOf(goal)} when no label source was configured. */
        @Override
        public String label(G goal) {
            if (label == null) {
                return String.valueOf(goal);
            }
            String value = label.apply(goal);
            return value == null ? String.valueOf(goal) : value;
        }

        /** {@inheritDoc} {@code ""} when no note source was configured. */
        @Override
        public String note(G goal) {
            if (note == null || goal == null) {
                return "";
            }
            String value = note.apply(goal);
            return value == null ? "" : value;
        }

        /** {@inheritDoc} {@code true} when no homing test was configured. */
        @Override
        public boolean zeroed() {
            return zeroed == null || zeroed.getAsBoolean();
        }

        /**
         * {@inheritDoc}
         *
         * <p>Runs every declared {@link CustomBuilder#range} check. Problems are reported rather
         * than thrown so the builder can aggregate every bad goal across every binding into one
         * exception, instead of surfacing them one deploy cycle at a time.
         */
        @Override
        public void validate(G goal, Consumer<String> problems) {
            if (goal == null) {
                problems.accept("goal is null");
                return;
            }
            for (RangeCheck<G> range : ranges) {
                range.check(goal, problems);
            }
        }

        /** {@inheritDoc} Runs the configured release hook, if any. */
        @Override
        public void release() {
            if (onRelease != null) {
                onRelease.run();
            }
        }
    }
}
