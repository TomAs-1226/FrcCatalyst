package frc.lib.catalyst.physics.diagnostics;

import java.util.Locale;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A recorded impact: when it happened, how hard, and from which direction.
 *
 * <p>Emitted by {@link CollisionDetector} when the robot experiences an acceleration the drivetrain
 * cannot account for. The useful consumers are downstream of Physics Core — an auto routine that
 * should request a replan after being knocked off a path, a log review asking why cycle four went
 * wrong, a driver alert.
 *
 * @param timestampSeconds  when the impact was detected, on the FPGA clock
 * @param magnitudeMpsSq    peak unexplained acceleration during the impact, m/s^2
 * @param direction         field-relative direction the impulse pushed the robot
 * @param peakForceNewtons  the same peak expressed as a force, {@code m * a}
 * @since 1.5.0
 */
public record CollisionEvent(
        double timestampSeconds,
        double magnitudeMpsSq,
        Rotation2d direction,
        double peakForceNewtons) {

    /** One line for a log or a driver alert. */
    public String describe() {
        return String.format(Locale.ROOT, "impact %.1f m/s^2 (%.0f N) from %.0f deg at t=%.2f",
                magnitudeMpsSq, peakForceNewtons, direction.getDegrees(), timestampSeconds);
    }
}
