package frc.lib.catalyst.physics.model;

import java.util.Locale;
import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Closed-form ballistics for a launched game piece — mainly so a shot gate can stop guessing at time
 * of flight.
 *
 * <p>{@link frc.lib.catalyst.physics.prediction.LaunchState#missRadiusMeters(double)} needs to know
 * how long the piece is in the air, because velocity error at release keeps compounding for the whole
 * flight. Without a model the caller has to supply a constant, and a constant is wrong at every
 * distance but one. This turns exit speed, hood angle, and distance into a real number.
 *
 * <pre>{@code
 * ProjectileModel shot = ProjectileModel.noDrag();
 *
 * double flight = shot.timeOfFlightSeconds(distance, exitSpeed, hoodAngle).orElse(1.0);
 * boolean take  = launch.fitsTarget(flight, GOAL_RADIUS);
 * }</pre>
 *
 * <h2>What is modelled, and what is not</h2>
 * Two models, both solved exactly rather than integrated:
 *
 * <ul>
 *   <li>{@link #noDrag()} — the textbook parabola. Accurate for a dense piece over a short flight, and
 *       the right default.</li>
 *   <li>{@link #withLinearDrag(double)} — drag proportional to speed, {@code v̇ = -k·v}. Still exact,
 *       and it captures the leading effect of air on a light piece.</li>
 * </ul>
 *
 * <p><b>Quadratic drag and spin are deliberately absent.</b> Real drag on a light game piece goes as
 * {@code v²}, and backspin produces lift that a ballistic model has no term for. Neither has a closed
 * form, and fitting them needs coefficients a team can only get by measuring — at which point the
 * measurement itself is the better model. That is exactly what {@code AimingSolver}'s and
 * {@code AimingSolverVector}'s interpolating tables already are, and they remain the way to
 * <em>aim</em>. Use this to reason about timing and clearance, not to replace a characterisation
 * sweep.
 *
 * <p>All angles are elevations above horizontal, all distances are metres, all times are seconds.
 * Pure math, no state, no hardware.
 *
 * @since 1.6.0
 */
public final class ProjectileModel {

    /**
     * Below this drag coefficient the drag solution is evaluated as if there were none.
     *
     * <p>Not a physical choice, a numerical one. The drag height solution contains
     * {@code (vy + g/k)·(1 - e^-kt)/k - g·t/k}, and as {@code k} shrinks both terms grow like
     * {@code 1/k} while their difference stays around a metre. At {@code k = 1e-7} that is two
     * numbers near {@code 1e8} differing by 4, and double precision has nothing left to give — the
     * answer comes back wrong by centimetres. A drag coefficient of {@code 1e-3} is a time constant of
     * a thousand seconds, which is no drag by any physical standard, so switching to the exact
     * parabola below this point costs nothing and removes the failure.
     */
    private static final double NEGLIGIBLE_DRAG = 1e-3;

    private final double gravity;
    /** Linear drag coefficient in 1/s. Zero means no drag. */
    private final double dragPerSecond;

    private ProjectileModel(double gravity, double dragPerSecond) {
        this.gravity = gravity;
        this.dragPerSecond = dragPerSecond;
    }

    /** The textbook parabola: gravity only. */
    public static ProjectileModel noDrag() {
        return new ProjectileModel(RobotModel.GRAVITY, 0.0);
    }

    /**
     * Drag proportional to speed, {@code v̇ = -k·v}.
     *
     * @param dragPerSecond {@code k}, in 1/s. Roughly the reciprocal of how long the piece takes to
     *                      lose most of its speed to air; {@code 0.1} is a mild effect over a
     *                      one-second flight, {@code 0.5} is pronounced. Must be non-negative.
     */
    public static ProjectileModel withLinearDrag(double dragPerSecond) {
        if (!(dragPerSecond >= 0) || Double.isInfinite(dragPerSecond)) {
            throw new IllegalArgumentException("dragPerSecond must be finite and >= 0 (got "
                    + dragPerSecond + ")");
        }
        return new ProjectileModel(RobotModel.GRAVITY, dragPerSecond);
    }

    /**
     * True when this model includes drag large enough to be worth solving for. A coefficient below
     * {@link #NEGLIGIBLE_DRAG} reports false and takes the exact no-drag path, because at that point
     * the drag solution is numerically worse than the parabola it is approximating.
     */
    public boolean hasDrag() {
        return dragPerSecond >= NEGLIGIBLE_DRAG;
    }

    /**
     * How long the piece takes to cover {@code horizontalDistance}, in seconds.
     *
     * <p>Empty when the shot cannot get there at all: a non-positive exit speed, a hood at or past
     * vertical (no horizontal travel), or — with drag — a shot whose horizontal reach asymptotes short
     * of the target no matter how long you wait.
     *
     * @param horizontalDistanceMeters ground distance to the target
     * @param exitSpeedMps             speed of the piece as it leaves the shooter
     * @param launchAngle              hood elevation above horizontal
     */
    public OptionalDouble timeOfFlightSeconds(double horizontalDistanceMeters, double exitSpeedMps,
                                              Rotation2d launchAngle) {
        if (horizontalDistanceMeters <= 0) return OptionalDouble.of(0.0);
        if (!(exitSpeedMps > 0)) return OptionalDouble.empty();

        double horizontalSpeed = exitSpeedMps * launchAngle.getCos();
        if (horizontalSpeed <= 1e-9) return OptionalDouble.empty();

        if (!hasDrag()) {
            return OptionalDouble.of(horizontalDistanceMeters / horizontalSpeed);
        }
        // x(t) = (vx/k)(1 - e^-kt) asymptotes at vx/k, so anything beyond that is unreachable.
        double reach = horizontalSpeed / dragPerSecond;
        if (horizontalDistanceMeters >= reach) return OptionalDouble.empty();
        return OptionalDouble.of(-Math.log(1.0 - horizontalDistanceMeters / reach) / dragPerSecond);
    }

    /**
     * Height of the piece, relative to the launch point, when it has travelled
     * {@code horizontalDistanceMeters}. Empty when it never gets that far.
     *
     * <p>Useful for checking clearance — whether the shot passes over an obstacle, or arrives
     * descending rather than climbing.
     */
    public OptionalDouble heightAtDistance(double horizontalDistanceMeters, double exitSpeedMps,
                                           Rotation2d launchAngle) {
        OptionalDouble flight = timeOfFlightSeconds(horizontalDistanceMeters, exitSpeedMps, launchAngle);
        if (flight.isEmpty()) return OptionalDouble.empty();
        return OptionalDouble.of(heightAtTime(flight.getAsDouble(), exitSpeedMps, launchAngle));
    }

    /** Height above the launch point at time {@code t}, in metres. */
    public double heightAtTime(double timeSeconds, double exitSpeedMps, Rotation2d launchAngle) {
        double t = Math.max(0.0, timeSeconds);
        double verticalSpeed = exitSpeedMps * launchAngle.getSin();
        if (!hasDrag()) {
            return verticalSpeed * t - 0.5 * gravity * t * t;
        }
        double k = dragPerSecond;
        return (verticalSpeed + gravity / k) * (1.0 - Math.exp(-k * t)) / k - gravity * t / k;
    }

    /**
     * The two hood angles that land a shot of {@code exitSpeedMps} on a target
     * {@code horizontalDistanceMeters} away and {@code heightDeltaMeters} above the launch point.
     *
     * <p>Ballistics gives two answers — a flat, fast arc and a lobbed one. Both hit; the flat one is
     * less sensitive to speed error and the lobbed one drops in more steeply. Empty when the target is
     * out of range for that speed.
     *
     * <p><b>No-drag only.</b> With drag there is no closed form for the angle, so this throws rather
     * than quietly returning the drag-free answer and letting it be mistaken for the real one.
     *
     * @throws UnsupportedOperationException if this model has drag
     */
    public Optional<LaunchAngles> launchAnglesFor(double horizontalDistanceMeters,
                                                  double heightDeltaMeters, double exitSpeedMps) {
        if (hasDrag()) {
            throw new UnsupportedOperationException("launchAnglesFor has no closed form with drag - "
                    + "use ProjectileModel.noDrag() for the angle, or characterise the shooter");
        }
        if (!(exitSpeedMps > 0) || horizontalDistanceMeters <= 0) return Optional.empty();

        double d = horizontalDistanceMeters;
        double v2 = exitSpeedMps * exitSpeedMps;
        // tan(theta) = (v^2 +/- sqrt(v^4 - g(g d^2 + 2 h v^2))) / (g d)
        double discriminant = v2 * v2 - gravity * (gravity * d * d + 2.0 * heightDeltaMeters * v2);
        if (discriminant < 0) return Optional.empty();

        double root = Math.sqrt(discriminant);
        double flat = Math.atan((v2 - root) / (gravity * d));
        double lobbed = Math.atan((v2 + root) / (gravity * d));
        return Optional.of(new LaunchAngles(new Rotation2d(flat), new Rotation2d(lobbed)));
    }

    /**
     * The farthest this speed can reach a target {@code heightDeltaMeters} above the launch point, in
     * metres. Empty for a shot that cannot clear that height at all.
     *
     * <p><b>No-drag only</b>, for the same reason as {@link #launchAnglesFor}.
     *
     * @throws UnsupportedOperationException if this model has drag
     */
    public OptionalDouble maxRangeMeters(double heightDeltaMeters, double exitSpeedMps) {
        if (hasDrag()) {
            throw new UnsupportedOperationException("maxRangeMeters has no closed form with drag");
        }
        if (!(exitSpeedMps > 0)) return OptionalDouble.empty();
        double v2 = exitSpeedMps * exitSpeedMps;
        // Range is maximised where the two launch angles coincide, i.e. the discriminant is zero.
        double inner = v2 * v2 - 2.0 * gravity * heightDeltaMeters * v2;
        if (inner < 0) return OptionalDouble.empty();
        return OptionalDouble.of(Math.sqrt(inner) / gravity);
    }

    /** Highest point of the arc above the launch point, in metres. */
    public double apexHeightMeters(double exitSpeedMps, Rotation2d launchAngle) {
        double verticalSpeed = exitSpeedMps * launchAngle.getSin();
        if (verticalSpeed <= 0) return 0.0;
        if (!hasDrag()) {
            return verticalSpeed * verticalSpeed / (2.0 * gravity);
        }
        // Apex is where vertical velocity reaches zero: t = ln(1 + k·vy/g) / k.
        double k = dragPerSecond;
        double apexTime = Math.log(1.0 + k * verticalSpeed / gravity) / k;
        return heightAtTime(apexTime, exitSpeedMps, launchAngle);
    }

    /** One line naming the model in use. */
    public String describe() {
        return hasDrag()
                ? String.format(Locale.ROOT, "ProjectileModel[linear drag k=%.3f /s]", dragPerSecond)
                : "ProjectileModel[no drag]";
    }

    /**
     * The two hood angles that reach the same target.
     *
     * @param flat   the lower, faster arc — less sensitive to exit-speed error
     * @param lobbed the higher arc — drops in more steeply, better over an obstacle
     */
    public record LaunchAngles(Rotation2d flat, Rotation2d lobbed) {

        /** True when the two solutions have collapsed together — the target is at maximum range. */
        public boolean atMaxRange() {
            return Math.abs(flat.getRadians() - lobbed.getRadians()) < 1e-6;
        }
    }
}
