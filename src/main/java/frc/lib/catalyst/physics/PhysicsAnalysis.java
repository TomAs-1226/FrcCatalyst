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
        Rotation2d disturbanceDirection,
        Optional<CollisionEvent> lastCollision) {

    /** Nothing measured yet — what {@link PhysicsCore#analyze()} returns before the first update. */
    public static PhysicsAnalysis nominal() {
        return new PhysicsAnalysis(0.0, 0.0, -1, 0.0, 0.0, 0.0, Rotation2d.kZero, Optional.empty());
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
     */
    public boolean isDisturbed() {
        return tractionUsage > 1.0 || isSlipping();
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
