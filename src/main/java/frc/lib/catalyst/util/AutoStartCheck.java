package frc.lib.catalyst.util;

import frc.lib.catalyst.logging.CatalystLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Is the robot where the selected auto expects it to be?
 *
 * <h2>The failure this exists for</h2>
 *
 * <p>Every auto is written for a starting pose, and nothing checks that the robot is at it. The
 * robot is set down by hand, the drive team looks at the tape on the carpet, the match starts, and
 * the path runs from wherever the robot actually was - into the wall, or into a partner. With four
 * cameras and a working pose estimate the robot <em>knows</em> where it is during the disabled period
 * before the match, so the check is a subtraction; what was missing was anywhere to see the answer.
 *
 * <p>This publishes the answer while the robot is disabled, as numbers a dashboard can put where the
 * drive team is already looking, and as a warning if the robot is out of tolerance.
 *
 * <h2>What it does not do</h2>
 *
 * <p>It does not move the robot, reset its pose, or block the auto. A tolerance that is too tight
 * would otherwise stop a match, and a check that resets the pose would hide the exact thing it exists
 * to reveal. It reports; the humans decide.
 *
 * <p>The expected pose is whatever the supplier says. {@link AutoSelector#selectedStartingPose()}
 * answers for PathPlanner autos; a custom auto can answer with its own. An empty answer means the
 * check stands down and says so, rather than comparing against a pose nobody chose.
 *
 * <h2>Keys</h2>
 *
 * <pre>
 *   /Catalyst/Auto/StartCheck/Available       bool     both poses known
 *   /Catalyst/Auto/StartCheck/Ready           bool     within tolerance
 *   /Catalyst/Auto/StartCheck/DistanceMeters  double
 *   /Catalyst/Auto/StartCheck/HeadingErrorDeg double   signed, wrapped to ±180
 *   /Catalyst/Auto/StartCheck/Expected        double[] [x, y, theta rad]
 *   /Catalyst/Auto/StartCheck/Current         double[] [x, y, theta rad]
 *   /Catalyst/Auto/StartCheck/Detail          string
 * </pre>
 *
 * @since 2.0.0
 */
public final class AutoStartCheck {

    /** Default position tolerance. Wider than a well-placed robot, narrower than a wrong one. */
    public static final double DEFAULT_TOLERANCE_METERS = 0.30;
    /** Default heading tolerance. */
    public static final double DEFAULT_TOLERANCE_DEGREES = 10.0;

    private static final String KEY = "Auto/StartCheck/";
    private static final String ALERT_SUBSYSTEM = "Auto";
    // Stable text on purpose. AlertManager de-duplicates and clears by exact message, so a message
    // carrying the live distance could never be cleared. The numbers are on their own keys.
    private static final String ALERT_TEXT = "Robot is not at the selected auto's starting pose";

    /** One evaluation. */
    public record Result(boolean available, boolean ready, double distanceMeters,
                         double headingErrorDegrees, String detail) {}

    private final Supplier<Pose2d> currentPose;
    private final Supplier<Optional<Pose2d>> expectedStart;
    private double toleranceMeters = DEFAULT_TOLERANCE_METERS;
    private double toleranceDegrees = DEFAULT_TOLERANCE_DEGREES;
    private boolean alerting = false;

    /**
     * @param currentPose   the robot's pose estimate, usually the drivetrain's
     * @param expectedStart the selected auto's starting pose, or empty if none is selected or it has
     *                      no known start
     */
    public AutoStartCheck(Supplier<Pose2d> currentPose, Supplier<Optional<Pose2d>> expectedStart) {
        this.currentPose = currentPose;
        this.expectedStart = expectedStart;
    }

    /** How far off is still "ready". Defaults are 0.30 m and 10 degrees. */
    public AutoStartCheck tolerances(double meters, double degrees) {
        this.toleranceMeters = meters;
        this.toleranceDegrees = degrees;
        return this;
    }

    /**
     * Compare, publish, and raise or clear the warning. Call from a disabled periodic.
     *
     * <p>Safe to call while enabled too - it only reports - but during a match the number is history.
     */
    public Result update() {
        Optional<Pose2d> expected;
        Pose2d current;
        try {
            expected = expectedStart.get();
            current = currentPose.get();
        } catch (RuntimeException e) {
            expected = Optional.empty();
            current = null;
        }

        Result r;
        if (expected == null || expected.isEmpty() || current == null) {
            r = new Result(false, false, Double.NaN, Double.NaN, "no auto start pose to check against");
        } else {
            r = evaluate(current, expected.get(), toleranceMeters, toleranceDegrees);
            CatalystLog.log(KEY + "Expected", new double[] {
                    expected.get().getX(), expected.get().getY(),
                    expected.get().getRotation().getRadians()});
            CatalystLog.log(KEY + "Current", new double[] {
                    current.getX(), current.getY(), current.getRotation().getRadians()});
        }

        CatalystLog.log(KEY + "Available", r.available());
        CatalystLog.log(KEY + "Ready", r.ready());
        CatalystLog.log(KEY + "DistanceMeters", r.distanceMeters());
        CatalystLog.log(KEY + "HeadingErrorDeg", r.headingErrorDegrees());
        CatalystLog.log(KEY + "Detail", r.detail());

        boolean shouldAlert = r.available() && !r.ready();
        if (shouldAlert && !alerting) {
            AlertManager.getInstance().warning(ALERT_SUBSYSTEM, ALERT_TEXT);
        } else if (!shouldAlert && alerting) {
            AlertManager.getInstance().clearWarning(ALERT_SUBSYSTEM, ALERT_TEXT);
        }
        alerting = shouldAlert;
        return r;
    }

    /** The comparison itself, with no publishing. Pure, so it can be tested as arithmetic. */
    public static Result evaluate(Pose2d current, Pose2d expected,
                                  double toleranceMeters, double toleranceDegrees) {
        double distance = current.getTranslation().getDistance(expected.getTranslation());
        double heading = wrapDegrees(current.getRotation().getDegrees()
                - expected.getRotation().getDegrees());
        boolean ready = distance <= toleranceMeters && Math.abs(heading) <= toleranceDegrees;
        String detail = ready
                ? String.format("at start: %.2f m, %.0f deg off", distance, heading)
                : String.format("%.2f m and %.0f deg from the auto's start", distance, heading);
        return new Result(true, ready, distance, heading, detail);
    }

    /** Wrap to (-180, 180]. */
    static double wrapDegrees(double degrees) {
        double d = degrees % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d <= -180.0) d += 360.0;
        return d;
    }

    /** Convenience for the common case of an expected pose that is always known. */
    public static AutoStartCheck fixed(Supplier<Pose2d> currentPose, Pose2d start) {
        return new AutoStartCheck(currentPose, () -> Optional.of(start));
    }

    /** Convenience for a heading-only start, e.g. a robot placed against a wall at a known angle. */
    public static Rotation2d headingOnly(double degrees) {
        return Rotation2d.fromDegrees(degrees);
    }
}
