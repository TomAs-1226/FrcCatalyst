package frc.lib.catalyst.physics;

import java.util.Locale;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.lib.catalyst.physics.diagnostics.CollisionEvent;

/**
 * What Physics Core currently thinks is happening to the robot, physically.
 *
 * <p>Where {@link PhysicalRobotState} answers "where is it and how fast", this answers "and is
 * anything going wrong". It is the read-out for a dashboard, a driver alert, or a guard on a state
 * transition:
 *
 * <pre>{@code
 * transition(STOWED, EXTENDED)
 *     .guard(() -> physics.analyze().tractionUsage() < 0.5, "traction");
 * }</pre>
 *
 * <p>Physics Core produces this and stops. It does not slow the robot down, block a transition, or
 * reroute a path on its own — the state machine, {@code BehaviorEngine}, and {@code Autopilot} keep
 * owning those decisions. That separation is deliberate: an advisory layer that quietly intervenes
 * is one nobody can debug.
 *
 * @param slipFactor        mean wheel slip across modules, 0 to 1
 * @param peakSlip          worst single module's slip score, 0 to 1
 * @param worstModule       index of the worst module, or {@code -1} when nothing is slipping
 * @param tractionUsage     fraction of available grip in use; above 1 the drivetrain is not the cause
 * @param tippingUsage      fraction of the tipping margin in use; above 1 the robot is going over
 * @param disturbanceMpsSq  acceleration the wheels cannot account for, m/s^2
 * @param disturbanceFraction the same as a fraction of the traction limit — {@code 1.0} means an
 *                          acceleration the drivetrain could not have produced at all
 * @param disturbanceDirection field-relative direction that disturbance is pushing
 * @param lastCollision     the most recent detected impact, if there has been one
 * @since 1.5.0
 */
public record PhysicsAnalysis(
        double slipFactor,
        double peakSlip,
        int worstModule,
        double tractionUsage,
        double tippingUsage,
        double disturbanceMpsSq,
        double disturbanceFraction,
        Rotation2d disturbanceDirection,
        Optional<CollisionEvent> lastCollision) {

    /** Nothing measured yet — what {@link PhysicsCore#analyze()} returns before the first update. */
    public static PhysicsAnalysis nominal() {
        return new PhysicsAnalysis(0.0, 0.0, -1, 0.0, 0.0, 0.0, 0.0, Rotation2d.kZero, Optional.empty());
    }

    /** True when any module is slipping enough to be worth reacting to (peak above 0.5). */
    public boolean isSlipping() {
        return peakSlip > 0.5;
    }

    /** True when the robot is using more than 90% of its tipping margin. */
    public boolean isNearTipping() {
        return tippingUsage > 0.9;
    }

    /**
     * True when the robot is doing something the drivetrain cannot explain — being pushed, having
     * hit something, or spinning its wheels hard enough to break the model.
     *
     * <p>Checks the unexplained acceleration directly, not just the traction usage. Uniform slip, where
     * every wheel breaks loose together, leaves no per-module residual <em>and</em> pulls the fused
     * acceleration <em>down</em> toward the truth once the estimator starts distrusting the wheels — so
     * a robot sliding across the carpet can show a perfectly modest traction usage while the wheels and
     * the IMU are shouting at each other. That case has to come from the disturbance figure or it is
     * missed entirely.
     */
    public boolean isDisturbed() {
        return tractionUsage > 1.0 || isSlipping() || disturbanceFraction > 0.5;
    }

    /** One line describing the dominant physical concern, or {@code "nominal"} when there is none. */
    public String describe() {
        if (isNearTipping()) {
            return String.format(Locale.ROOT, "tipping margin %.0f%% used", tippingUsage * 100.0);
        }
        if (isSlipping()) {
            return worstModule >= 0
                    ? String.format(Locale.ROOT, "module %d slipping (%.0f%%)", worstModule, peakSlip * 100.0)
                    : String.format(Locale.ROOT, "wheel slip %.0f%%", peakSlip * 100.0);
        }
        if (disturbanceMpsSq > 1.0) {
            return String.format(Locale.ROOT, "external push %.1f m/s^2 from %.0f deg",
                    disturbanceMpsSq, disturbanceDirection.getDegrees());
        }
        return "nominal";
    }
}
