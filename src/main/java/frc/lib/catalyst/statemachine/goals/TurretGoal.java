package frc.lib.catalyst.statemachine.goals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.catalyst.util.AimingSolver;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * What a {@link frc.lib.catalyst.mechanisms.TurretMechanism} should be pointing at.
 *
 * <p>A turret is the one Catalyst mechanism whose goal is usually not a number. Four of its six
 * goal shapes are <em>continuous aiming modes</em> — the setpoint is recomputed every loop from
 * odometry, a solver or a camera — and only {@link RobotRelative} and {@link Hold} name a fixed
 * angle. That is why this is a sealed hierarchy of records rather than an enum of presets: the
 * aiming modes have to carry their suppliers with them, and the state machine engine has to be
 * able to compare two goals for equality without accidentally rebuilding the pursue command at
 * 50 Hz.
 *
 * <p><b>The equality rule matters more here than anywhere else in the package.</b> The engine
 * calls {@link java.util.Objects#equals} on the wanted goal against the active goal every loop
 * and re-applies actuation whenever they differ. Records give value-based equality for the
 * {@code double} and {@code String} components, but a lambda has only identity equality — two
 * separately-written {@code () -> drive.getHeading().getDegrees()} lambdas are never equal.
 * Build each supplier-bearing goal <b>once</b>, store it in the state table, and hand the engine
 * that same instance forever. Calling a factory below inside a per-loop lambda would produce a
 * fresh unequal goal every iteration and the turret would restart its tracking command 50 times
 * a second, which shows up on the field as a turret that twitches and never converges.
 *
 * <p>Every public aiming command on {@code TurretMechanism} is reachable from here, and the
 * mapping is deliberately one-to-one so nothing is stranded:
 * <ul>
 *   <li>{@link RobotRelative} → {@code goToAngle(double)} (and {@code lockForward()}, which is
 *       just {@code goToAngle(0)}).</li>
 *   <li>{@link FieldAngle} → {@code aimAtFieldAngle(DoubleSupplier, DoubleSupplier)}.</li>
 *   <li>{@link FieldPoint} → {@code aimAtTarget(Supplier, Translation2d)}.</li>
 *   <li>{@link Solved} → {@code track(Supplier, DoubleSupplier, DoubleSupplier)}, which the
 *       two-argument {@code track} overload delegates to with a zero yaw rate. Carrying the yaw
 *       rate as a component therefore reaches both overloads through one goal shape.</li>
 *   <li>{@link VisionServo} → {@code aimWithVision(BooleanSupplier, DoubleSupplier)}.</li>
 *   <li>{@link Hold} → {@code holdAngle()}.</li>
 * </ul>
 *
 * <p>Arrival is deliberately <em>not</em> decided by {@code TurretMechanism.atSetpoint()} alone.
 * That method compares the measured angle against the setpoint the mechanism actually commanded,
 * and {@code resolveTurretAngle} silently clamps an unreachable bearing to the mechanical limits
 * — so a turret asked to point at something directly behind a ±170° machine will sit against its
 * hard stop reporting {@code atSetpoint() == true}. The binding that consumes these goals is
 * expected to re-run {@link frc.lib.catalyst.mechanisms.TurretMechanism#resolveTurretAngle} to
 * detect that clamp, and to use
 * {@link frc.lib.catalyst.mechanisms.TurretMechanism#isOnTarget} for {@link Solved}. The
 * tolerance carried by each goal below is the band that check uses.
 *
 * @since 1.2.0
 */
public sealed interface TurretGoal
        permits TurretGoal.RobotRelative,
                TurretGoal.FieldAngle,
                TurretGoal.FieldPoint,
                TurretGoal.Solved,
                TurretGoal.VisionServo,
                TurretGoal.Hold {

    /**
     * Aiming tolerance, in degrees, used when a goal is built without an explicit one or with a
     * value that could never be satisfied.
     *
     * <p>This mirrors the {@code toleranceDegrees} default of {@code TurretMechanism.Config},
     * which is the honest thing to do given that {@code TurretMechanism} exposes no accessor for
     * its configured tolerance — unlike {@code LinearMechanism} and {@code RotationalMechanism},
     * a binding cannot ask the turret what band it was tuned for, so the goal has to carry one.
     * Teams whose turret is configured tighter or looser than this should pass the tolerance
     * explicitly through the overloads that take one.
     */
    double DEFAULT_TOLERANCE_DEGREES = 1.0;

    /**
     * The yaw-rate supplier used by {@link #solved(Supplier, DoubleSupplier, String)} for robots
     * that are not spinning while they shoot.
     *
     * <p>It is a shared constant rather than a fresh {@code () -> 0.0} per call for the equality
     * reason described on the type: a fresh lambda would make two otherwise-identical goals
     * unequal, and identity-unequal goals are exactly what causes the engine to rebuild the
     * tracking command every loop.
     */
    DoubleSupplier ZERO_YAW_RATE = () -> 0.0;

    /**
     * Stable, low-cardinality label for this goal, suitable for the edge-detected string that
     * telemetry logs.
     *
     * <p>Declaring it on the interface is what keeps the binding free of an {@code instanceof}
     * chain just to name a goal. The four supplier-bearing shapes cannot derive a label — their
     * bearing only exists at runtime, and interpolating it would write a new string to the log
     * every loop forever — so they carry an author-supplied one as a record component and this
     * method is satisfied by the accessor the record already generates.
     *
     * @return a label that does not change while the goal is applied
     */
    String label();

    /**
     * Half-width of the band, in degrees, within which this goal counts as reached.
     *
     * @return a strictly positive, finite tolerance in degrees
     */
    double toleranceDegrees();

    /**
     * Point the turret at a fixed angle relative to the chassis.
     *
     * <p>The angle is handed to the mechanism untouched — it is deliberately <b>not</b> wrapped
     * into ±180°, because a turret configured with overlap ({@code range(-200, 200)}) has
     * reachable angles outside that window, and normalising here would quietly discard the
     * caller's choice of which way around to approach. Choosing the representation is
     * {@code resolveTurretAngle}'s job and it needs the raw request to do it.
     *
     * @param degrees          robot-relative bearing in degrees, CCW positive, 0 straight ahead
     * @param toleranceDegrees arrival band in degrees; see {@link #DEFAULT_TOLERANCE_DEGREES}
     */
    record RobotRelative(double degrees, double toleranceDegrees) implements TurretGoal {

        /**
         * Normalises the tolerance and leaves the commanded angle alone.
         *
         * <p>A tolerance of zero or a negative or non-finite one is replaced rather than
         * rejected: {@code Math.abs(error) <= 0} is unsatisfiable against a real encoder, so
         * such a goal would leave the state machine waiting forever with nothing in the log
         * explaining why. An out-of-range {@code degrees} is <em>not</em> checked here — the
         * mechanism's soft limits are not visible from this package, and the binding's
         * build-time {@code validate} reports that failure with the actual limits in the
         * message, which is far more useful than an exception thrown from a field initialiser.
         */
        public RobotRelative {
            toleranceDegrees = sanitizeTolerance(toleranceDegrees);
        }

        /** {@inheritDoc} */
        @Override
        public String label() {
            return degrees == 0.0 ? "Forward" : String.format("At %.1f", degrees);
        }
    }

    /**
     * Hold a field-relative bearing while the chassis rotates underneath the turret.
     *
     * <p>Maps to {@code aimAtFieldAngle}, which commands {@code fieldDegrees - headingDegrees}
     * every loop. Use this when something else already knows the bearing — an auto routine
     * aiming at a fixed field direction, or a solver whose output you have flattened to an
     * angle. If you have a live {@code AimingSolver.Solution}, prefer {@link Solved}: it also
     * feeds the solver's analytic bearing rate forward, and it knows when a solve is infeasible.
     *
     * @param fieldDegrees     field-relative bearing supplier, degrees, same convention as the
     *                         heading supplier
     * @param headingDegrees   live robot heading supplier, degrees
     * @param toleranceDegrees arrival band in degrees
     * @param label            stable name for this goal, e.g. {@code "AimSpeaker"}. Required
     *                         because the bearing is only known at runtime and a label that
     *                         interpolated it would be written to the log every loop.
     */
    record FieldAngle(DoubleSupplier fieldDegrees, DoubleSupplier headingDegrees,
                      double toleranceDegrees, String label) implements TurretGoal {

        /**
         * Normalises the tolerance and supplies a fallback label.
         *
         * <p>A null supplier is left to the binding's build-time {@code validate} rather than
         * throwing here, so that a misconfigured superstructure reports every one of its
         * problems in a single exception on a laptop instead of one per deploy cycle in the pit.
         */
        public FieldAngle {
            toleranceDegrees = sanitizeTolerance(toleranceDegrees);
            label = sanitizeLabel(label, "AimField");
        }
    }

    /**
     * Aim at a fixed point on the field, computing the bearing from the live robot pose.
     *
     * <p>Maps to {@code aimAtTarget}, which takes the vector from the robot to the target and
     * subtracts the pose's rotation. This ignores robot velocity entirely, so a shot taken while
     * translating will miss behind the target; that is the failure mode {@link Solved} exists to
     * prevent. Reach for this one when the robot is stationary at the moment of the shot, or
     * when the turret is carrying something that is placed rather than launched.
     *
     * @param robotPose        live pose supplier, in the same field frame as {@code target}
     * @param target           the field point to point at
     * @param toleranceDegrees arrival band in degrees
     * @param label            stable name for this goal, e.g. {@code "AimGoal"}
     */
    record FieldPoint(Supplier<Pose2d> robotPose, Translation2d target,
                      double toleranceDegrees, String label) implements TurretGoal {

        /** Normalises the tolerance and supplies a fallback label. */
        public FieldPoint {
            toleranceDegrees = sanitizeTolerance(toleranceDegrees);
            label = sanitizeLabel(label, "AimPoint");
        }
    }

    /**
     * Track an {@link AimingSolver.Solution} — the shoot-on-the-fly path.
     *
     * <p>Maps to the three-argument {@code track}, which leads a moving virtual goal using the
     * solver's analytic field-bearing rate converted into the robot frame by subtracting the
     * chassis yaw rate. Passing the yaw rate is what keeps the feedforward exact while the robot
     * spins; without it a turret on a rotating chassis lags its target by however far the robot
     * turns in one loop. The two-argument {@code track} overload is simply this one with a zero
     * yaw rate, so both are reachable through this single goal shape — use
     * {@link #solved(Supplier, DoubleSupplier, String)} for the stationary-heading case.
     *
     * <p>Arrival for this goal should be judged with {@code isOnTarget(solution, heading,
     * tolerance)} rather than {@code atSetpoint()}. {@code isOnTarget} folds in
     * {@code Solution.feasible()}, which is the only thing that distinguishes "aimed" from
     * "holding still because the solver gave up", and the mechanism holds its last setpoint on
     * an infeasible solve — a state that {@code atSetpoint()} happily calls arrival.
     *
     * @param solution         supplier of the latest solve; call the solver inside the lambda so
     *                         each loop sees a fresh result
     * @param headingDegrees   live robot heading supplier, degrees
     * @param yawRateDps       live chassis yaw rate supplier, degrees per second. With a swerve
     *                         drive this is
     *                         {@code () -> Math.toDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond)}.
     * @param toleranceDegrees arrival band in degrees
     * @param label            stable name for this goal, e.g. {@code "TrackGoal"}
     */
    record Solved(Supplier<AimingSolver.Solution> solution, DoubleSupplier headingDegrees,
                  DoubleSupplier yawRateDps, double toleranceDegrees, String label)
            implements TurretGoal {

        /**
         * Normalises the tolerance, supplies a fallback label, and substitutes the shared
         * {@link #ZERO_YAW_RATE} for a null yaw-rate supplier.
         *
         * <p>Defaulting the yaw rate is safe in a way that defaulting the other suppliers would
         * not be: a missing yaw rate costs only the feedforward correction, leaving a turret
         * that still converges through feedback, whereas a missing solution or heading supplier
         * is a structural mistake that build-time {@code validate} should name out loud.
         */
        public Solved {
            toleranceDegrees = sanitizeTolerance(toleranceDegrees);
            label = sanitizeLabel(label, "Track");
            if (yawRateDps == null) {
                yawRateDps = ZERO_YAW_RATE;
            }
        }
    }

    /**
     * Close the loop on a camera's horizontal error instead of on odometry.
     *
     * <p>Maps to {@code aimWithVision}, which nudges the setpoint by the reported error each
     * loop and holds the last setpoint whenever {@code hasTarget} is false. Because the error is
     * relative rather than absolute, this converges without any pose estimate at all, which is
     * what makes it the right fallback when odometry has drifted or the robot has been bumped.
     *
     * <p>The consequence to design around is that a turret servoing on a camera it cannot see
     * through reports an error of whatever the camera last said, or zero. Arrival must therefore
     * require {@code hasTarget} to be true as well as the error to be inside the band, or the
     * machine will call a blind turret aimed.
     *
     * @param hasTarget        true while the camera has a valid target
     * @param errorDegrees     horizontal angle to the target in degrees; positive should drive
     *                         the turret positive, and the mechanism's
     *                         {@code visionInverted} config flag flips it when it does not
     * @param toleranceDegrees arrival band in degrees
     * @param label            stable name for this goal, e.g. {@code "VisionAim"}
     */
    record VisionServo(BooleanSupplier hasTarget, DoubleSupplier errorDegrees,
                       double toleranceDegrees, String label) implements TurretGoal {

        /** Normalises the tolerance and supplies a fallback label. */
        public VisionServo {
            toleranceDegrees = sanitizeTolerance(toleranceDegrees);
            label = sanitizeLabel(label, "Vision");
        }
    }

    /**
     * Hold whatever setpoint the turret already has.
     *
     * <p>Maps to {@code holdAngle}, which keeps re-driving Motion Magic to the mechanism's
     * current {@code setpointDegrees} field. This is the goal for a state that cares about some
     * other mechanism and merely wants the turret to stop being commanded somewhere new — it
     * parks the aim without slewing it, so a turret that was tracking stays roughly where the
     * last solve left it rather than snapping back to zero. Use {@link #forward()} when you
     * actually want the turret returned to a known angle, for instance before climbing.
     */
    record Hold() implements TurretGoal {

        /** {@inheritDoc} */
        @Override
        public String label() {
            return "Hold";
        }

        /**
         * {@inheritDoc}
         *
         * <p>A hold has no target of its own, so the band reported here is the shared default.
         * It is what the binding compares the measured angle against to decide the turret has
         * settled on the setpoint it inherited.
         */
        @Override
        public double toleranceDegrees() {
            return DEFAULT_TOLERANCE_DEGREES;
        }
    }

    // ============================================================
    //                        FACTORIES
    // ============================================================

    /**
     * Point the turret straight ahead, at robot-relative 0°.
     *
     * <p>This is the goal to give a stow or climb state. A turret left aimed off to one side is
     * outside the frame perimeter for scoring purposes on most designs, and it is the part most
     * likely to be hit by another robot.
     *
     * @return a goal pointing the bore along the chassis +X axis
     */
    static TurretGoal forward() {
        return new RobotRelative(0.0, DEFAULT_TOLERANCE_DEGREES);
    }

    /**
     * Point the turret at a fixed robot-relative angle using the default tolerance.
     *
     * @param degrees robot-relative bearing in degrees, CCW positive
     * @return a fixed-angle goal
     */
    static TurretGoal robotRelative(double degrees) {
        return new RobotRelative(degrees, DEFAULT_TOLERANCE_DEGREES);
    }

    /**
     * Point the turret at a fixed robot-relative angle with an explicit tolerance.
     *
     * <p>Widen the tolerance for a goal that only has to get the turret out of the way, and
     * tighten it for one the shot depends on. A single band for both is the usual reason a
     * sequence either fires wide or waits on a turret that is already close enough.
     *
     * @param degrees          robot-relative bearing in degrees, CCW positive
     * @param toleranceDegrees arrival band in degrees; non-positive or non-finite values fall
     *                         back to {@link #DEFAULT_TOLERANCE_DEGREES}
     * @return a fixed-angle goal
     */
    static TurretGoal robotRelative(double degrees, double toleranceDegrees) {
        return new RobotRelative(degrees, toleranceDegrees);
    }

    /**
     * Hold a field-relative bearing while the chassis rotates, using the default tolerance.
     *
     * @param fieldDegrees   field-relative bearing supplier, degrees
     * @param headingDegrees live robot heading supplier, degrees, same convention
     * @param label          stable low-cardinality name for this goal
     * @return a field-bearing aiming goal
     */
    static TurretGoal fieldAngle(DoubleSupplier fieldDegrees, DoubleSupplier headingDegrees,
                                 String label) {
        return new FieldAngle(fieldDegrees, headingDegrees, DEFAULT_TOLERANCE_DEGREES, label);
    }

    /**
     * Hold a field-relative bearing while the chassis rotates, with an explicit tolerance.
     *
     * @param fieldDegrees     field-relative bearing supplier, degrees
     * @param headingDegrees   live robot heading supplier, degrees, same convention
     * @param toleranceDegrees arrival band in degrees
     * @param label            stable low-cardinality name for this goal
     * @return a field-bearing aiming goal
     */
    static TurretGoal fieldAngle(DoubleSupplier fieldDegrees, DoubleSupplier headingDegrees,
                                 double toleranceDegrees, String label) {
        return new FieldAngle(fieldDegrees, headingDegrees, toleranceDegrees, label);
    }

    /**
     * Aim at a fixed field point from the live robot pose, using the default tolerance.
     *
     * @param robotPose live pose supplier, in the same field frame as {@code target}
     * @param target    the field point to aim at
     * @param label     stable low-cardinality name for this goal
     * @return a field-point aiming goal
     */
    static TurretGoal at(Supplier<Pose2d> robotPose, Translation2d target, String label) {
        return new FieldPoint(robotPose, target, DEFAULT_TOLERANCE_DEGREES, label);
    }

    /**
     * Aim at a fixed field point from the live robot pose, with an explicit tolerance.
     *
     * <p>Remember that the angle subtended by the target shrinks with distance, so a band that
     * is generous point-blank is far too loose from across the field. If the shot has to work at
     * range, either tighten this or move to {@link #solved(Supplier, DoubleSupplier, String)}
     * and let the solver reason about distance for you.
     *
     * @param robotPose        live pose supplier, in the same field frame as {@code target}
     * @param target           the field point to aim at
     * @param toleranceDegrees arrival band in degrees
     * @param label            stable low-cardinality name for this goal
     * @return a field-point aiming goal
     */
    static TurretGoal at(Supplier<Pose2d> robotPose, Translation2d target,
                         double toleranceDegrees, String label) {
        return new FieldPoint(robotPose, target, toleranceDegrees, label);
    }

    /**
     * Track an aiming solution on a chassis that is not rotating meaningfully while it shoots.
     *
     * <p>Equivalent to the two-argument {@code TurretMechanism.track} overload: the yaw rate is
     * the shared {@link #ZERO_YAW_RATE}. If the robot spins while tracking, use
     * {@link #solved(Supplier, DoubleSupplier, DoubleSupplier, double, String)} instead — the
     * missing term is exactly the turret's lag behind a rotating chassis.
     *
     * @param solution       supplier of the latest solve
     * @param headingDegrees live robot heading supplier, degrees
     * @param label          stable low-cardinality name for this goal
     * @return a shoot-on-the-fly tracking goal
     */
    static TurretGoal solved(Supplier<AimingSolver.Solution> solution, DoubleSupplier headingDegrees,
                             String label) {
        return new Solved(solution, headingDegrees, ZERO_YAW_RATE, DEFAULT_TOLERANCE_DEGREES, label);
    }

    /**
     * Track an aiming solution on a chassis that may translate and rotate at once.
     *
     * @param solution         supplier of the latest solve
     * @param headingDegrees   live robot heading supplier, degrees
     * @param yawRateDps       live chassis yaw rate supplier, degrees per second
     * @param toleranceDegrees arrival band in degrees
     * @param label            stable low-cardinality name for this goal
     * @return a shoot-on-the-fly tracking goal
     */
    static TurretGoal solved(Supplier<AimingSolver.Solution> solution, DoubleSupplier headingDegrees,
                             DoubleSupplier yawRateDps, double toleranceDegrees, String label) {
        return new Solved(solution, headingDegrees, yawRateDps, toleranceDegrees, label);
    }

    /**
     * Servo the turret on a camera's horizontal error, using the default tolerance.
     *
     * @param hasTarget    true while the camera has a valid target
     * @param errorDegrees horizontal angle to the target, degrees
     * @param label        stable low-cardinality name for this goal
     * @return a vision-servo goal
     */
    static TurretGoal vision(BooleanSupplier hasTarget, DoubleSupplier errorDegrees, String label) {
        return new VisionServo(hasTarget, errorDegrees, DEFAULT_TOLERANCE_DEGREES, label);
    }

    /**
     * Servo the turret on a camera's horizontal error, with an explicit tolerance.
     *
     * <p>A vision servo converges by repeatedly correcting, so its tolerance should be at least
     * as wide as the camera's own noise floor. Asking for a band tighter than the measurement
     * repeats gives a turret that hunts around the target and a state that never reports arrival.
     *
     * @param hasTarget        true while the camera has a valid target
     * @param errorDegrees     horizontal angle to the target, degrees
     * @param toleranceDegrees arrival band in degrees
     * @param label            stable low-cardinality name for this goal
     * @return a vision-servo goal
     */
    static TurretGoal vision(BooleanSupplier hasTarget, DoubleSupplier errorDegrees,
                             double toleranceDegrees, String label) {
        return new VisionServo(hasTarget, errorDegrees, toleranceDegrees, label);
    }

    /**
     * Hold the turret's existing setpoint without slewing it anywhere new.
     *
     * @return the hold goal
     */
    static TurretGoal hold() {
        return new Hold();
    }

    // ============================================================
    //                       NORMALISATION
    // ============================================================

    /**
     * Replaces a tolerance that could never be satisfied with {@link #DEFAULT_TOLERANCE_DEGREES}.
     *
     * <p>Zero, negative and non-finite tolerances are all normalised rather than rejected. The
     * house rule is that this file never throws for something build-time {@code validate} can
     * report better with the mechanism's real configuration in hand; the one thing worth fixing
     * here is a value that would make the state machine wait forever with a plausible-looking
     * goal in the log.
     */
    private static double sanitizeTolerance(double toleranceDegrees) {
        if (Double.isNaN(toleranceDegrees) || Double.isInfinite(toleranceDegrees)
                || toleranceDegrees <= 0.0) {
            return DEFAULT_TOLERANCE_DEGREES;
        }
        return toleranceDegrees;
    }

    /**
     * Substitutes a mode-appropriate fallback for a missing or blank label.
     *
     * <p>Labels reach telemetry as edge-detected strings, and a null one would either print as
     * {@code "null"} or throw somewhere far away from the mistake. A blank label is treated the
     * same way, because an empty string in a log column is indistinguishable from a binding that
     * has not been applied yet.
     */
    private static String sanitizeLabel(String label, String fallback) {
        return (label == null || label.isBlank()) ? fallback : label;
    }
}
