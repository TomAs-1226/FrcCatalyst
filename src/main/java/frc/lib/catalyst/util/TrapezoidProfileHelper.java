package frc.lib.catalyst.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Helper for creating WPILib trapezoidal motion profiles.
 * Trapezoidal profiles generate smooth motion by limiting both velocity
 * and acceleration, creating a trapezoidal velocity curve over time.
 *
 * <p>Use this as an alternative to CTRE Motion Magic when you want
 * WPILib-native control, or for mechanisms not using TalonFX.
 *
 * <p>This helper wraps {@link ProfiledPIDController} with convenient
 * factory methods for common FRC mechanism types.
 *
 * <p>Example usage:
 * <pre>{@code
 * // Create a trapezoidal profile for an elevator
 * ProfiledPIDController elevatorPID = TrapezoidProfileHelper.createLinear(
 *     50, 0, 0.5,   // PID gains
 *     2.0, 4.0      // max velocity (m/s), max accel (m/s^2)
 * );
 *
 * // In periodic:
 * double voltage = elevatorPID.calculate(currentPosition, targetPosition);
 * voltage += gravityFF; // add gravity compensation
 * motor.setVoltage(voltage);
 *
 * // Check if at goal
 * if (elevatorPID.atGoal()) { ... }
 * }</pre>
 */
public final class TrapezoidProfileHelper {

    private TrapezoidProfileHelper() {}

    /**
     * Create a ProfiledPIDController for a linear mechanism (elevator, linear slide).
     * Units are in meters and meters/sec.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param maxVelocity max velocity in meters/sec
     * @param maxAcceleration max acceleration in meters/sec^2
     * @return configured ProfiledPIDController
     */
    public static ProfiledPIDController createLinear(
            double kP, double kI, double kD,
            double maxVelocity, double maxAcceleration) {
        return new ProfiledPIDController(kP, kI, kD,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }

    /**
     * Create a ProfiledPIDController for a rotational mechanism (arm, wrist).
     * Units are in rotations and rotations/sec.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param maxVelocityRPS max angular velocity in rotations/sec
     * @param maxAccelerationRPSS max angular acceleration in rotations/sec^2
     * @return configured ProfiledPIDController
     */
    public static ProfiledPIDController createRotational(
            double kP, double kI, double kD,
            double maxVelocityRPS, double maxAccelerationRPSS) {
        return new ProfiledPIDController(kP, kI, kD,
                new TrapezoidProfile.Constraints(maxVelocityRPS, maxAccelerationRPSS));
    }

    /**
     * Create a ProfiledPIDController for a rotational mechanism with continuous input.
     * Used for turrets or other mechanisms that can wrap around (0-360).
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param maxVelocityRPS max angular velocity in rotations/sec
     * @param maxAccelerationRPSS max angular acceleration in rotations/sec^2
     * @return configured ProfiledPIDController with continuous input [-0.5, 0.5] rotations
     */
    public static ProfiledPIDController createContinuousRotational(
            double kP, double kI, double kD,
            double maxVelocityRPS, double maxAccelerationRPSS) {
        ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD,
                new TrapezoidProfile.Constraints(maxVelocityRPS, maxAccelerationRPSS));
        controller.enableContinuousInput(-0.5, 0.5); // rotations
        return controller;
    }

    /**
     * Create a standalone TrapezoidProfile (no PID) for generating
     * motion setpoints. Useful when you want to use Phoenix 6 PID
     * but generate the profile yourself.
     *
     * @param maxVelocity max velocity
     * @param maxAcceleration max acceleration
     * @return TrapezoidProfile.Constraints
     */
    public static TrapezoidProfile.Constraints createConstraints(
            double maxVelocity, double maxAcceleration) {
        return new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    }

    /**
     * Create a TrapezoidProfile for generating motion setpoints.
     * In WPILib 2026, call {@code profile.calculate(dt, goal, current)} each cycle
     * to step the profile forward.
     *
     * @param maxVelocity max velocity
     * @param maxAcceleration max acceleration
     * @return configured TrapezoidProfile
     */
    public static TrapezoidProfile createProfile(
            double maxVelocity, double maxAcceleration) {
        return new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }
}
