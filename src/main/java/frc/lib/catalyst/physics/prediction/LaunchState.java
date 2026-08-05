package frc.lib.catalyst.physics.prediction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.lib.catalyst.physics.LocalizationQuality;

/**
 * Where the robot will be at the instant the game piece actually leaves it.
 *
 * <p>Shoot-on-the-fly solvers take a pose and a field-relative velocity. Handed the values from
 * <em>now</em>, they solve for a shot that will not be taken for another hundred milliseconds or so —
 * the feeder has to index, the flywheel has to recover, the piece has to travel up the shooter. On a
 * robot moving at 3 m/s, 120 ms of that is 36 cm of error before the piece has even left.
 *
 * <p>A {@code LaunchState} is the same two values, propagated to the moment of release. It drops
 * straight into the existing solver — no new overload, no new API:
 *
 * <pre>{@code
 * LaunchState launch = physics.predictLaunchState(feeder.releaseDelaySeconds());
 *
 * AimingSolverVector.TargetState shot =
 *     aimingSolver.calculate(launch.pose(), launch.fieldVelocity());
 *
 * if (launch.fitsTarget(SHOT_FLIGHT_SECONDS, GOAL_RADIUS_METERS)) { fire(); }
 * }</pre>
 *
 * @param pose                 predicted field-relative pose at release
 * @param fieldVelocity        predicted field-relative chassis velocity at release
 * @param timestampSeconds     the predicted instant of release, on the FPGA clock
 * @param releaseDelaySeconds  how far ahead this was predicted
 * @param quality              confidence and uncertainty at release, already degraded for the horizon
 * @since 1.5.0
 */
public record LaunchState(
        Pose2d pose,
        ChassisSpeeds fieldVelocity,
        double timestampSeconds,
        double releaseDelaySeconds,
        LocalizationQuality quality) {

    /**
     * Roughly how far off the shot is likely to land, in metres — a 1-sigma radius.
     *
     * <p>Two errors compound. The robot may not be where the prediction says at release
     * ({@code translationStdDev}), and it may not be travelling at the predicted velocity, which
     * mis-cancels the motion compensation for the whole flight ({@code velocityStdDev * timeOfFlight}).
     *
     * <p>This accounts only for uncertainty in the <em>robot's</em> state. Shooter repeatability,
     * piece-to-piece variation, and spin are the shooter's own error budget and are not modelled here
     * — so treat the result as a floor on the miss radius, not the whole of it.
     *
     * @param timeOfFlightSeconds how long the piece is in the air, from your shot solution
     */
    public double missRadiusMeters(double timeOfFlightSeconds) {
        double flight = Math.max(0.0, timeOfFlightSeconds);
        return quality.translationStdDevMeters()
                + quality.velocityStdDevMetersPerSecond() * flight;
    }

    /**
     * Whether this shot is worth taking: the predicted miss radius fits inside the target.
     *
     * @param timeOfFlightSeconds how long the piece is in the air
     * @param targetRadiusMeters  how much room there is to miss by
     */
    public boolean fitsTarget(double timeOfFlightSeconds, double targetRadiusMeters) {
        return missRadiusMeters(timeOfFlightSeconds) <= targetRadiusMeters;
    }

    /** Ground speed at release, in metres per second. */
    public double speedMetersPerSecond() {
        return Math.hypot(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    }
}
