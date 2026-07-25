package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.MechanismView;
import frc.lib.catalyst.mechanisms.TurretMechanism;
import frc.lib.catalyst.statemachine.goals.TurretGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;
import frc.lib.catalyst.util.AimingSolver;

import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Binds a {@link TurretMechanism} to the state machine as an {@link Actuator} over
 * {@link TurretGoal}.
 *
 * <p>The turret is the hardest of the nine Catalyst bindings to get right, and all of the
 * difficulty is in one place: <b>{@code TurretMechanism.atSetpoint()} lies</b>. It compares the
 * measured angle against {@code setpointDegrees}, which is whatever
 * {@link TurretMechanism#resolveTurretAngle} last handed the motor — and {@code resolveTurretAngle}
 * silently clamps a bearing it cannot reach to the nearest mechanical limit. A turret asked to
 * point at something directly behind a &plusmn;170&deg; machine therefore drives into its hard
 * stop, sits there, and reports arrival. A state machine that trusted {@code atSetpoint()} would
 * advance to "shoot" with the barrel pointed at the wrong half of the field.
 *
 * <p>This binding re-runs {@code resolveTurretAngle} itself and rejects arrival whenever the
 * resolved setpoint is not a legal {@code desired + 360&middot;k} representation of what was
 * actually asked for. That distinction matters: a turret configured {@code range(-200, 200)} and
 * asked for 350&deg; is resolved to &minus;10&deg;, which is <em>numerically</em> far from the
 * request but is the same physical bearing and is perfectly correct. Only a resolution that is not
 * congruent to the request modulo 360&deg; is a clamp. See {@link #isClamped(double, double)}.
 *
 * <p>Arrival for {@link TurretGoal.Solved} goes through {@link TurretMechanism#isOnTarget}, which
 * folds in {@code Solution.feasible()} — the only thing separating "aimed" from "holding still
 * because the solver gave up", since the mechanism's {@code track} command holds its last setpoint
 * on an infeasible solve and {@code atSetpoint()} calls that arrival too.
 *
 * <h2>Mechanical limits are snapshotted, not read live</h2>
 *
 * <p>{@code TurretMechanism} keeps its travel limits in a private {@code Config} whose fields are
 * package-private, and exposes no {@code getMinAngle()} / {@code getMaxAngle()}. The only public
 * path to them is {@link TurretMechanism#describe()}, which allocates a {@link java.util.Map} on
 * every call and so must never be touched from {@link #atGoal} — that runs at 50&nbsp;Hz for every
 * binding in the machine. The limits are therefore read exactly once, in the constructor, and
 * cached in {@code final} fields. They are configuration, not state, so a snapshot is not a
 * compromise; the constructor taking explicit limits exists for teams who would rather state them
 * than have them inferred.
 *
 * @since 1.2.0
 */
public final class TurretBinding implements Actuator<TurretGoal> {

    /**
     * How far a resolved setpoint may drift from an exact multiple of 360&deg; away from the
     * request before it counts as a clamp rather than floating-point noise.
     *
     * <p>Resolution arithmetic is {@code desired + 360.0 * k} for integer {@code k}, so a legal
     * representation differs from the request by a multiple of 360&deg; to within a few units in
     * the last place. A genuine clamp misses by the amount the request overshot the limit, which
     * on a real turret is degrees. There is a wide gap between the two and the threshold sits in
     * it, so a request that lands a millionth of a degree outside a soft limit is treated as
     * reachable rather than reported as a blocker nobody can act on.
     */
    private static final double WRAP_EPSILON_DEGREES = 1e-6;

    /** Widest turret representation offset {@code resolveTurretAngle} considers, in wraps. */
    private static final int MAX_WRAPS = 3;

    private final TurretMechanism mechanism;
    private final String key;
    private final DoubleSupplier robotHeadingDeg;
    private final Set<Subsystem> requirements;

    private final double minAngleDeg;
    private final double maxAngleDeg;
    private final boolean limitsKnown;

    /**
     * Build-time reachability verdict for each {@link TurretGoal.RobotRelative} goal, keyed by the
     * goal itself.
     *
     * <p>Only fixed-angle goals can be resolved ahead of time. The four aiming modes recompute
     * their bearing every loop from a supplier, so there is nothing about them to cache — and the
     * clamp test itself depends on the live turret angle, because which representation
     * {@code resolveTurretAngle} picks depends on where the turret currently is. What <em>is</em>
     * cacheable, and what this map holds, is the question "does any legal representation of this
     * angle exist inside the mechanical travel at all", which depends only on the request and the
     * limits. The value is the reachable representation nearest the middle of travel, or
     * {@link Double#NaN} when the angle is structurally unreachable.
     *
     * <p>A {@link ConcurrentHashMap} rather than a plain {@code HashMap} because {@link #validate}
     * runs on the build thread while the reads happen on the robot loop; the cost is nil at these
     * sizes and it removes a memory-visibility question that would otherwise have to be reasoned
     * about. Goals absent from the map — never validated, because the builder was bypassed — fall
     * back to computing the same answer inline, so a missing entry costs a few arithmetic
     * operations and never a wrong answer.
     */
    private final Map<TurretGoal, Double> reachableRepresentation = new ConcurrentHashMap<>();

    /**
     * Binds a turret with no fallback heading supplier and limits read from the mechanism.
     *
     * <p>Suitable when every {@link TurretGoal.FieldAngle} and {@link TurretGoal.Solved} goal
     * carries its own heading supplier, which the {@code TurretGoal} factories require anyway.
     *
     * @param mechanism the turret to drive; must not be {@code null}
     * @param key       stable, log-safe key for this binding
     */
    public TurretBinding(TurretMechanism mechanism, String key) {
        this(mechanism, key, null, Double.NaN, Double.NaN);
    }

    /**
     * Binds a turret with a fallback heading supplier and limits read from the mechanism.
     *
     * @param mechanism       the turret to drive; must not be {@code null}
     * @param key             stable, log-safe key for this binding
     * @param robotHeadingDeg robot heading in degrees, used by {@link TurretGoal.FieldAngle} and
     *                        {@link TurretGoal.Solved} goals that do not carry their own. May be
     *                        {@code null}; {@link #validate} then reports any goal that needed it.
     */
    public TurretBinding(TurretMechanism mechanism, String key, DoubleSupplier robotHeadingDeg) {
        this(mechanism, key, robotHeadingDeg, Double.NaN, Double.NaN);
    }

    /**
     * Binds a turret with a fallback heading supplier and explicit mechanical travel limits.
     *
     * <p>Pass the same numbers given to {@code TurretMechanism.Config.Builder.range(min, max)}.
     * When either limit is non-finite, or when {@code minAngleDeg >= maxAngleDeg}, the limits are
     * instead snapshotted from {@link TurretMechanism#describe()} — the mechanism's own report of
     * its configured range, and the only public route to it.
     *
     * @param mechanism       the turret to drive; must not be {@code null}
     * @param key             stable, log-safe key for this binding
     * @param robotHeadingDeg fallback robot heading supplier in degrees; may be {@code null}
     * @param minAngleDeg     lower mechanical travel limit in robot-relative degrees, or
     *                        {@link Double#NaN} to snapshot it from the mechanism
     * @param maxAngleDeg     upper mechanical travel limit in robot-relative degrees, or
     *                        {@link Double#NaN} to snapshot it from the mechanism
     * @throws IllegalArgumentException if {@code mechanism} or {@code key} is {@code null}
     */
    public TurretBinding(TurretMechanism mechanism, String key, DoubleSupplier robotHeadingDeg,
                         double minAngleDeg, double maxAngleDeg) {
        if (mechanism == null) {
            throw new IllegalArgumentException("TurretBinding requires a non-null TurretMechanism");
        }
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException("TurretBinding requires a non-blank key");
        }
        this.mechanism = mechanism;
        this.key = key;
        this.robotHeadingDeg = robotHeadingDeg;
        this.requirements = Set.of(mechanism);

        double lo = minAngleDeg;
        double hi = maxAngleDeg;
        if (!(Double.isFinite(lo) && Double.isFinite(hi) && lo < hi)) {
            // describe() is the only public accessor for the configured range. It builds a map, so
            // it is called here exactly once and never again for the life of the binding.
            MechanismView view = mechanism.describe();
            lo = view.min();
            hi = view.max();
        }
        this.minAngleDeg = lo;
        this.maxAngleDeg = hi;
        this.limitsKnown = Double.isFinite(lo) && Double.isFinite(hi) && lo < hi;
    }

    // ============================================================
    //                       IDENTITY
    // ============================================================

    /** {@inheritDoc} */
    @Override
    public String key() {
        return key;
    }

    /** {@inheritDoc} */
    @Override
    public String kind() {
        return "turret";
    }

    /** {@inheritDoc} */
    @Override
    public String unit() {
        return "deg";
    }

    /** {@inheritDoc} */
    @Override
    public Set<Subsystem> requirements() {
        return requirements;
    }

    // ============================================================
    //                       ACTUATION
    // ============================================================

    /**
     * {@inheritDoc}
     *
     * <p>Every branch calls a {@code TurretMechanism} factory, and each of those wraps a fresh
     * {@code runOnce}/{@code run} around the mechanism, so the fresh-instance contract holds
     * without any caching on this side.
     *
     * <p>This method never throws. A malformed goal — a null goal, or an aiming mode missing a
     * supplier that {@link #validate} would have reported at build time — falls back to
     * {@link TurretMechanism#holdAngle()}. That is the right failure: an exception here would
     * propagate out of {@code CommandScheduler.run()} and stop the entire robot loop, whereas a
     * turret that holds its last aim is merely unhelpful, and {@link #note} says so out loud.
     */
    @Override
    public Command pursueCommand(TurretGoal goal) {
        try {
            if (goal instanceof TurretGoal.RobotRelative r) {
                return mechanism.goToAngle(r.degrees());
            }
            if (goal instanceof TurretGoal.FieldAngle f) {
                DoubleSupplier heading = headingFor(f);
                if (f.fieldDegrees() != null && heading != null) {
                    return mechanism.aimAtFieldAngle(f.fieldDegrees(), heading);
                }
            } else if (goal instanceof TurretGoal.FieldPoint p) {
                if (p.robotPose() != null && p.target() != null) {
                    return mechanism.aimAtTarget(p.robotPose(), p.target());
                }
            } else if (goal instanceof TurretGoal.Solved s) {
                DoubleSupplier heading = headingFor(s);
                DoubleSupplier yawRate =
                        (s.yawRateDps() != null) ? s.yawRateDps() : TurretGoal.ZERO_YAW_RATE;
                if (s.solution() != null && heading != null) {
                    return mechanism.track(s.solution(), heading, yawRate);
                }
            } else if (goal instanceof TurretGoal.VisionServo v) {
                if (v.hasTarget() != null && v.errorDegrees() != null) {
                    return mechanism.aimWithVision(v.hasTarget(), v.errorDegrees());
                }
            }
        } catch (RuntimeException e) {
            // Fall through to the hold below rather than killing the scheduler.
        }
        return mechanism.holdAngle();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Only {@link TurretGoal.RobotRelative} gets a hold command. Its pursue command,
     * {@code goToAngle}, is a {@code runOnce} that latches a Motion Magic request and ends, so
     * once the turret has arrived the honest thing to run is {@link TurretMechanism#holdAngle()},
     * which keeps re-driving that same latched setpoint.
     *
     * <p>The four aiming modes return {@code null} deliberately. Their pursue commands never end
     * and must keep running after arrival — a turret that stopped tracking the moment it was first
     * on target would immediately fall behind a moving robot, and the state machine would flicker
     * between arrived and not. {@link TurretGoal.Hold} returns {@code null} for the same reason:
     * its pursue command already is {@code holdAngle()}.
     */
    @Override
    public Command holdCommand(TurretGoal goal) {
        if (goal instanceof TurretGoal.RobotRelative) {
            return mechanism.holdAngle();
        }
        return null;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Zero — no reassertion is needed. {@code goToAngle} is the only pursue command that ends,
     * and it ends having latched a Phoenix Motion Magic position request, which the motor keeps
     * servoing to indefinitely. That is exactly the "persistent effect" the {@link Actuator}
     * contract permits an ending command to have. The pneumatic binding needs reassertion because
     * its actuation can be silently refused for low pressure; nothing here is refusable.
     */
    @Override
    public int reassertPeriodLoops() {
        return 0;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Deliberately does nothing. Releasing a turret should not move it: the mechanism defaults
     * to brake mode precisely so the aim survives, and the last Motion Magic request stays latched
     * until something else commands the motor. Zeroing the output here would let a heavy turret
     * coast off target between states, and driving it to a safe angle would be a surprise the
     * state machine did not ask for — a state that wants the turret parked should say so with
     * {@link TurretGoal#forward()}.
     */
    @Override
    public void release() {
        // Intentionally empty; see javadoc.
    }

    // ============================================================
    //                        ARRIVAL
    // ============================================================

    /**
     * {@inheritDoc}
     *
     * <p>Pure in the sense the interface demands: it reads live sensors and the goal handed to it,
     * and holds no memory of what was last applied. That is what lets the engine ask this about
     * goals belonging to states the machine is not currently in.
     *
     * <p>The angle-valued goals are judged against the representation
     * {@link TurretMechanism#resolveTurretAngle} would choose for them <em>right now</em>, not
     * against the mechanism's stored setpoint. Consulting {@code setpointDegrees} through a bare
     * {@code atSetpoint()} would make the answer depend on which goal happened to be applied,
     * which is precisely the impurity the interface forbids; the extra conjunct below measures the
     * turret against this goal's own band instead. {@code atSetpoint()} is still required, so a
     * goal is never called arrived while the mechanism itself disagrees, but it is no longer the
     * whole test.
     *
     * <p>{@link TurretGoal.Hold} is unconditionally arrived — there is no target to reach, only a
     * setpoint to keep. {@link #observable} reports {@code false} for it so nobody reads that as a
     * measurement.
     *
     * <p>Never throws. A supplier that blows up — a solver dividing by a stale pose, a camera
     * client that lost its NetworkTables entry — reports "not arrived", which stalls the state
     * machine visibly instead of killing the command scheduler silently.
     */
    @Override
    public boolean atGoal(TurretGoal goal, double secondsSinceApplied) {
        if (goal == null) {
            return false;
        }
        try {
            if (goal instanceof TurretGoal.Hold) {
                return true;
            }
            if (goal instanceof TurretGoal.Solved s) {
                DoubleSupplier heading = headingFor(s);
                if (s.solution() == null || heading == null) {
                    return false;
                }
                return mechanism.isOnTarget(
                        s.solution().get(), heading.getAsDouble(), s.toleranceDegrees());
            }
            if (goal instanceof TurretGoal.VisionServo v) {
                if (v.hasTarget() == null || v.errorDegrees() == null) {
                    return false;
                }
                return v.hasTarget().getAsBoolean()
                        && Math.abs(v.errorDegrees().getAsDouble()) <= v.toleranceDegrees();
            }

            double desired = desiredRobotRelativeDeg(goal);
            if (!Double.isFinite(desired)) {
                return false;
            }
            double current = mechanism.getAngle();
            double resolved = TurretMechanism.resolveTurretAngle(
                    desired, current, minAngleDeg, maxAngleDeg);
            if (isClamped(desired, resolved)) {
                return false;
            }
            // Decide arrival purely from the live angle against this goal's own resolved bearing.
            // atSetpoint() compares against whatever bearing was most recently commanded, so folding
            // it in would make "is the turret at bearing A?" depend on whether A is the bearing
            // currently being pursued — a measurement must not. The clamp check above already covers
            // the unreachable-target case atSetpoint() was there to catch.
            return Math.abs(resolved - current) <= goal.toleranceDegrees();
        } catch (RuntimeException e) {
            return false;
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>True for every goal except {@link TurretGoal.Hold}. A turret has an encoder, so arrival
     * at an angle is genuinely measured rather than timed — including for
     * {@link TurretGoal.VisionServo}, whose error comes from a camera but is a real reading of
     * where the target is. {@code Hold} is the exception: {@link #atGoal} returns {@code true} for
     * it by definition rather than by observation, and flagging that keeps anyone reading a log
     * from mistaking a definition for a sensor.
     */
    @Override
    public boolean observable(TurretGoal goal) {
        return !(goal instanceof TurretGoal.Hold);
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code true}. {@code TurretMechanism} tracks a {@code hasBeenZeroed} flag and
     * surfaces it through its own health checks and logging, but exposes no accessor for it, so
     * this binding cannot honestly gate a state on homing. Reporting {@code true} makes the
     * turret non-blocking, which matches the common configuration: a turret with a fused CANcoder
     * knows its angle on boot and has no homing step at all. Teams running an unhomed turret
     * without an absolute encoder should gate on the mechanism's {@code NotZeroed} health check.
     */
    @Override
    public boolean zeroed() {
        return true;
    }

    // ============================================================
    //                       TELEMETRY
    // ============================================================

    /** {@inheritDoc} */
    @Override
    public double measured() {
        return mechanism.getAngle();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Signed degrees the turret still has to travel, positive meaning it must turn CCW. For
     * {@link TurretGoal.Solved} this is {@link TurretMechanism#aimErrorDeg}, which wraps into
     * &plusmn;180&deg; and accounts for the solver's field bearing; for the angle-valued goals it
     * is measured against the resolved representation, so a turret unwrapping the long way around
     * reports the distance it will actually cover rather than the short way it cannot take.
     */
    @Override
    public double error(TurretGoal goal) {
        if (goal == null) {
            return Double.NaN;
        }
        try {
            if (goal instanceof TurretGoal.Solved s) {
                DoubleSupplier heading = headingFor(s);
                if (s.solution() == null || heading == null) {
                    return Double.NaN;
                }
                return mechanism.aimErrorDeg(s.solution().get(), heading.getAsDouble());
            }
            if (goal instanceof TurretGoal.VisionServo v) {
                if (v.hasTarget() == null || v.errorDegrees() == null
                        || !v.hasTarget().getAsBoolean()) {
                    return Double.NaN;
                }
                return v.errorDegrees().getAsDouble();
            }
            double desired = desiredRobotRelativeDeg(goal);
            if (!Double.isFinite(desired)) {
                return Double.NaN;
            }
            double current = mechanism.getAngle();
            return TurretMechanism.resolveTurretAngle(desired, current, minAngleDeg, maxAngleDeg)
                    - current;
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>Read straight off the goal. Unlike {@code LinearMechanism} and
     * {@code RotationalMechanism}, {@code TurretMechanism} exposes no accessor for its configured
     * {@code toleranceDegrees}, so the goal is the only place a band can come from — which is why
     * {@link TurretGoal} carries one on every shape and normalises unsatisfiable values.
     */
    @Override
    public double tolerance(TurretGoal goal) {
        return (goal == null) ? Double.NaN : goal.toleranceDegrees();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Delegates to {@link TurretGoal#label()}. The goal type guarantees low cardinality: the
     * four supplier-bearing shapes carry an author-supplied label as a record component precisely
     * because their bearing exists only at runtime, and the two fixed shapes name a constant.
     */
    @Override
    public String label(TurretGoal goal) {
        return (goal == null) ? "None" : goal.label();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Interpolates live values, which is allowed here and forbidden in {@link #label}.
     */
    @Override
    public String detail(TurretGoal goal) {
        if (goal == null) {
            return "None";
        }
        try {
            double current = mechanism.getAngle();
            if (goal instanceof TurretGoal.Hold) {
                return String.format("Hold at %.1f deg (setpoint %.1f)",
                        current, mechanism.getSetpoint());
            }
            if (goal instanceof TurretGoal.Solved s) {
                DoubleSupplier heading = headingFor(s);
                if (s.solution() == null || heading == null) {
                    return goal.label() + " (unconfigured)";
                }
                AimingSolver.Solution solution = s.solution().get();
                if (solution == null) {
                    return String.format("%s: no solve, at %.1f deg", goal.label(), current);
                }
                return String.format("%s: bearing %.1f deg, err %.1f deg, %s",
                        goal.label(),
                        solution.turretFieldAngleDeg(),
                        mechanism.aimErrorDeg(solution, heading.getAsDouble()),
                        solution.feasible() ? "feasible" : "INFEASIBLE");
            }
            if (goal instanceof TurretGoal.VisionServo v) {
                if (v.hasTarget() == null || v.errorDegrees() == null) {
                    return goal.label() + " (unconfigured)";
                }
                if (!v.hasTarget().getAsBoolean()) {
                    return String.format("%s: no target, at %.1f deg", goal.label(), current);
                }
                return String.format("%s: err %.1f deg (tol %.1f)",
                        goal.label(), v.errorDegrees().getAsDouble(), v.toleranceDegrees());
            }
            double desired = desiredRobotRelativeDeg(goal);
            if (!Double.isFinite(desired)) {
                return goal.label() + " (unconfigured)";
            }
            double resolved = TurretMechanism.resolveTurretAngle(
                    desired, current, minAngleDeg, maxAngleDeg);
            return String.format("%s: want %.1f deg -> %.1f deg, at %.1f deg%s",
                    goal.label(), desired, resolved, current,
                    isClamped(desired, resolved) ? " [CLAMPED]" : "");
        } catch (RuntimeException e) {
            return goal.label();
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>Says the one thing a log cannot show by itself: <em>why</em> a turret that looks settled
     * is not arrived. The clamp message is the important one — it names the bearing that was asked
     * for and the limit the mechanism substituted, which turns "the turret is at its hard stop and
     * the sequence is stuck" into a one-line diagnosis.
     *
     * <p>Returns {@code ""} when the turret has nothing to complain about.
     */
    @Override
    public String note(TurretGoal goal) {
        if (goal == null) {
            return "";
        }
        try {
            if (!limitsKnown) {
                return "turret travel limits unknown; clamp detection disabled";
            }
            if (goal instanceof TurretGoal.Solved s) {
                DoubleSupplier heading = headingFor(s);
                if (s.solution() == null) {
                    return "no aiming solution supplier";
                }
                if (heading == null) {
                    return "no robot heading supplier";
                }
                AimingSolver.Solution solution = s.solution().get();
                if (solution == null) {
                    return "solver returned no solution; turret holding";
                }
                if (!solution.feasible()) {
                    return String.format(
                            "aiming solution infeasible at %.1f m; turret holding last setpoint",
                            solution.distanceMeters());
                }
                return unwrapNote();
            }
            if (goal instanceof TurretGoal.VisionServo v) {
                if (v.hasTarget() == null || v.errorDegrees() == null) {
                    return "vision servo goal is missing a supplier";
                }
                if (!v.hasTarget().getAsBoolean()) {
                    return "no vision target; turret holding last setpoint";
                }
                return unwrapNote();
            }
            if (goal instanceof TurretGoal.RobotRelative r && isStructurallyUnreachable(r)) {
                return String.format("target %.1f deg unreachable; clamped to %.1f deg",
                        r.degrees(),
                        TurretMechanism.resolveTurretAngle(
                                r.degrees(), mechanism.getAngle(), minAngleDeg, maxAngleDeg));
            }
            double desired = desiredRobotRelativeDeg(goal);
            if (!Double.isFinite(desired)) {
                return "goal is missing a supplier";
            }
            double resolved = TurretMechanism.resolveTurretAngle(
                    desired, mechanism.getAngle(), minAngleDeg, maxAngleDeg);
            if (isClamped(desired, resolved)) {
                return String.format("target %.1f deg unreachable; clamped to %.1f deg",
                        desired, resolved);
            }
            return unwrapNote();
        } catch (RuntimeException e) {
            return "";
        }
    }

    // ============================================================
    //                       VALIDATION
    // ============================================================

    /**
     * {@inheritDoc}
     *
     * <p>Catches on a laptop, at build, the three mistakes that otherwise cost a deploy cycle
     * each: a fixed angle that no representation can reach inside the mechanical travel, an
     * aiming mode missing one of the suppliers it needs to compute a bearing, and a heading-hungry
     * goal on a binding constructed without a fallback heading supplier.
     *
     * <p>It also populates {@link #reachableRepresentation}, so the runtime path never repeats the
     * reachability search for a fixed angle.
     */
    @Override
    public void validate(TurretGoal goal, Consumer<String> problems) {
        if (goal == null) {
            problems.accept(key + ": null goal");
            return;
        }
        if (!limitsKnown) {
            problems.accept(key + ": could not determine turret travel limits from "
                    + mechanism.getMechanismName()
                    + "; pass them explicitly to the TurretBinding constructor or a clamped,"
                    + " unreachable aim will be reported as arrival");
        }

        if (goal instanceof TurretGoal.RobotRelative r) {
            double representation =
                    nearestReachableRepresentation(r.degrees(), minAngleDeg, maxAngleDeg);
            reachableRepresentation.put(r, representation);
            if (!Double.isFinite(r.degrees())) {
                problems.accept(key + ": RobotRelative goal '" + r.label()
                        + "' has a non-finite angle");
            } else if (limitsKnown && Double.isNaN(representation)) {
                problems.accept(String.format(
                        "%s: RobotRelative goal '%s' asks for %.1f deg, which no +/-360 deg"
                                + " representation places inside the turret travel [%.1f, %.1f];"
                                + " the mechanism would clamp to a hard stop and never arrive",
                        key, r.label(), r.degrees(), minAngleDeg, maxAngleDeg));
            }
        } else if (goal instanceof TurretGoal.FieldAngle f) {
            if (f.fieldDegrees() == null) {
                problems.accept(key + ": FieldAngle goal '" + f.label()
                        + "' has no field-bearing supplier");
            }
            if (headingFor(f) == null) {
                problems.accept(key + ": FieldAngle goal '" + f.label()
                        + "' needs a robot heading supplier, and neither the goal nor this"
                        + " binding was given one");
            }
        } else if (goal instanceof TurretGoal.FieldPoint p) {
            if (p.robotPose() == null) {
                problems.accept(key + ": FieldPoint goal '" + p.label()
                        + "' has no robot pose supplier");
            }
            if (p.target() == null) {
                problems.accept(key + ": FieldPoint goal '" + p.label()
                        + "' has no field target");
            }
        } else if (goal instanceof TurretGoal.Solved s) {
            if (s.solution() == null) {
                problems.accept(key + ": Solved goal '" + s.label()
                        + "' has no AimingSolver.Solution supplier");
            }
            if (headingFor(s) == null) {
                problems.accept(key + ": Solved goal '" + s.label()
                        + "' needs a robot heading supplier, and neither the goal nor this"
                        + " binding was given one");
            }
        } else if (goal instanceof TurretGoal.VisionServo v) {
            if (v.hasTarget() == null) {
                problems.accept(key + ": VisionServo goal '" + v.label()
                        + "' has no hasTarget supplier, so a blind turret would report arrival");
            }
            if (v.errorDegrees() == null) {
                problems.accept(key + ": VisionServo goal '" + v.label()
                        + "' has no error supplier");
            }
        }
    }

    // ============================================================
    //                       INTERNALS
    // ============================================================

    /**
     * The heading supplier a goal should use: its own if it carries one, otherwise the binding's
     * fallback. Returns {@code null} when neither exists, which every caller treats as "cannot
     * answer" and {@link #validate} reports as a build error.
     */
    private DoubleSupplier headingFor(TurretGoal goal) {
        if (goal instanceof TurretGoal.FieldAngle f && f.headingDegrees() != null) {
            return f.headingDegrees();
        }
        if (goal instanceof TurretGoal.Solved s && s.headingDegrees() != null) {
            return s.headingDegrees();
        }
        return robotHeadingDeg;
    }

    /**
     * The robot-relative bearing, in degrees, that this goal is currently asking for — the number
     * the mechanism's own command body would hand to {@code resolveTurretAngle} this loop.
     *
     * <p>Defined only for the goals whose target is an angle. {@link TurretGoal.Solved} and
     * {@link TurretGoal.VisionServo} are excluded on purpose: the former is judged by
     * {@link TurretMechanism#isOnTarget}, which knows about feasibility, and the latter closes a
     * relative loop with no absolute bearing to compare against. Returns {@link Double#NaN} when a
     * required supplier is missing or hands back {@code null}.
     */
    private double desiredRobotRelativeDeg(TurretGoal goal) {
        if (goal instanceof TurretGoal.RobotRelative r) {
            return r.degrees();
        }
        if (goal instanceof TurretGoal.FieldAngle f) {
            DoubleSupplier heading = headingFor(f);
            if (f.fieldDegrees() == null || heading == null) {
                return Double.NaN;
            }
            return f.fieldDegrees().getAsDouble() - heading.getAsDouble();
        }
        if (goal instanceof TurretGoal.FieldPoint p) {
            if (p.robotPose() == null || p.target() == null) {
                return Double.NaN;
            }
            Pose2d pose = p.robotPose().get();
            if (pose == null) {
                return Double.NaN;
            }
            // Mirrors TurretMechanism.aimAtTarget exactly: bearing to the target in the field
            // frame, less the robot's own rotation.
            Translation2d toTarget = p.target().minus(pose.getTranslation());
            double fieldDeg = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
            return fieldDeg - pose.getRotation().getDegrees();
        }
        if (goal instanceof TurretGoal.Hold) {
            return mechanism.getSetpoint();
        }
        return Double.NaN;
    }

    /**
     * Did {@code resolveTurretAngle} clamp, as opposed to merely choosing a different wrap?
     *
     * <p>This is the whole reason the binding exists, and it is subtler than comparing the two
     * numbers. {@code resolveTurretAngle} is free to return any {@code desired + 360&middot;k}: on
     * a turret with overlap it routinely returns a value hundreds of degrees from the request that
     * points at exactly the same place. Such a resolution is correct and must not be reported as a
     * failure. A clamp, by contrast, produces a value that is <em>not</em> congruent to the request
     * modulo 360&deg;, because it came out of {@code MathUtil.clamp} against a limit rather than
     * out of the wrap search.
     *
     * <p>{@link Math#IEEEremainder} is what separates the two: it maps the difference into
     * &plusmn;180&deg; and returns essentially zero for any legal representation, so the test is a
     * single comparison against {@link #WRAP_EPSILON_DEGREES}.
     *
     * @param desiredDeg  the robot-relative bearing that was requested
     * @param resolvedDeg what {@code resolveTurretAngle} returned for it
     * @return {@code true} when the mechanism substituted a bearing that is not the requested one
     */
    private boolean isClamped(double desiredDeg, double resolvedDeg) {
        if (!limitsKnown || !Double.isFinite(desiredDeg) || !Double.isFinite(resolvedDeg)) {
            return false;
        }
        return Math.abs(Math.IEEEremainder(resolvedDeg - desiredDeg, 360.0))
                > WRAP_EPSILON_DEGREES;
    }

    /**
     * Whether a fixed-angle goal can never be reached from anywhere, using the cached build-time
     * verdict when {@link #validate} has seen this goal and recomputing it otherwise.
     */
    private boolean isStructurallyUnreachable(TurretGoal.RobotRelative goal) {
        Double cached = reachableRepresentation.get(goal);
        double representation = (cached != null)
                ? cached
                : nearestReachableRepresentation(goal.degrees(), minAngleDeg, maxAngleDeg);
        return Double.isNaN(representation);
    }

    /**
     * The reachable representation of {@code desiredDeg} nearest the middle of travel, or
     * {@link Double#NaN} when no representation lies inside the limits at all.
     *
     * <p>Unlike {@code resolveTurretAngle} this does not depend on where the turret currently is,
     * which is what makes it answerable at build time. It searches the same
     * {@code k} range the mechanism does, so the two agree about what is reachable. When the
     * limits are unknown the request is returned unchanged, so an uninferable range degrades to
     * "assume reachable" rather than to a false alarm on every goal.
     */
    private static double nearestReachableRepresentation(double desiredDeg,
                                                         double minDeg, double maxDeg) {
        if (!Double.isFinite(desiredDeg)) {
            return Double.NaN;
        }
        if (!Double.isFinite(minDeg) || !Double.isFinite(maxDeg) || minDeg >= maxDeg) {
            return desiredDeg;
        }
        double middle = 0.5 * (minDeg + maxDeg);
        double best = Double.NaN;
        double bestError = Double.POSITIVE_INFINITY;
        for (int k = -MAX_WRAPS; k <= MAX_WRAPS; k++) {
            double candidate = desiredDeg + 360.0 * k;
            if (candidate < minDeg || candidate > maxDeg) {
                continue;
            }
            double error = Math.abs(candidate - middle);
            if (error < bestError) {
                bestError = error;
                best = candidate;
            }
        }
        return best;
    }

    /**
     * Reports an in-progress unwrap, which explains a turret that is taking an implausibly long
     * route to a nearby bearing. Empty when the turret is going the short way.
     */
    private String unwrapNote() {
        return mechanism.isUnwrapping()
                ? "unwrapping: taking the long way around to stay inside the travel limits"
                : "";
    }
}
