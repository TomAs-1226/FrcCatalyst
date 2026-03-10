package frc.lib.catalyst.util;

/**
 * Stores feedforward gains for different mechanism types.
 * These gains are used by CTRE's closed-loop control (Slot0 config)
 * and can also be used with WPILib feedforward classes.
 *
 * <p>Feedforward terms:
 * <ul>
 *   <li><b>kS</b> — Static friction voltage. The minimum voltage to overcome friction and start moving.</li>
 *   <li><b>kV</b> — Velocity gain. Volts per unit of velocity (rot/s for rotational, m/s for linear).</li>
 *   <li><b>kA</b> — Acceleration gain. Volts per unit of acceleration.</li>
 *   <li><b>kG</b> — Gravity gain. Volts to hold against gravity (constant for elevator, cosine-modulated for arm).</li>
 * </ul>
 *
 * <p>Example usage:
 * <pre>{@code
 * // From SysId characterization results:
 * FeedforwardGains elevatorFF = FeedforwardGains.elevator(0.12, 2.5, 0.1, 0.35);
 * FeedforwardGains armFF = FeedforwardGains.arm(0.15, 1.8, 0.05, 0.5);
 * FeedforwardGains flywheelFF = FeedforwardGains.simple(0.12, 0.11, 0.01);
 *
 * // Use in mechanism config:
 * LinearMechanism.Config.builder()
 *     .feedforward(elevatorFF.kS, elevatorFF.kV, elevatorFF.kA)
 *     .gravityGain(elevatorFF.kG)
 *     // ...
 *
 * // Calculate voltage manually:
 * double holdVoltage = elevatorFF.calculateElevator();
 * double armHoldVoltage = armFF.calculateArm(Math.toRadians(45)); // at 45 degrees
 * double flywheelVoltage = flywheelFF.calculateSimple(50.0); // at 50 RPS
 * }</pre>
 */
public class FeedforwardGains {

    public final double kS;
    public final double kV;
    public final double kA;
    public final double kG;

    private FeedforwardGains(double kS, double kV, double kA, double kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    /**
     * Create gains for a simple velocity-controlled mechanism (flywheel, conveyor).
     * No gravity compensation needed.
     */
    public static FeedforwardGains simple(double kS, double kV) {
        return new FeedforwardGains(kS, kV, 0, 0);
    }

    /** Simple FF with acceleration term. */
    public static FeedforwardGains simple(double kS, double kV, double kA) {
        return new FeedforwardGains(kS, kV, kA, 0);
    }

    /**
     * Create gains for an elevator (constant gravity load).
     * kG is the voltage needed to hold the mechanism stationary against gravity.
     */
    public static FeedforwardGains elevator(double kS, double kV, double kA, double kG) {
        return new FeedforwardGains(kS, kV, kA, kG);
    }

    /**
     * Create gains for an arm (gravity varies with angle via cosine).
     * kG is the voltage to hold the arm horizontal (worst case).
     */
    public static FeedforwardGains arm(double kS, double kV, double kA, double kG) {
        return new FeedforwardGains(kS, kV, kA, kG);
    }

    // --- Manual voltage calculations ---

    /**
     * Calculate feedforward voltage for a simple mechanism (flywheel, conveyor).
     * @param velocity target velocity (same units as kV was characterized with)
     * @return feedforward voltage
     */
    public double calculateSimple(double velocity) {
        return kS * Math.signum(velocity) + kV * velocity;
    }

    /**
     * Calculate feedforward voltage for a simple mechanism with acceleration.
     * @param velocity target velocity
     * @param acceleration target acceleration
     * @return feedforward voltage
     */
    public double calculateSimple(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
    }

    /**
     * Calculate feedforward voltage for an elevator.
     * Gravity is constant regardless of position.
     * @param velocity target velocity (m/s or rot/s)
     * @return feedforward voltage
     */
    public double calculateElevator(double velocity) {
        return kG + kS * Math.signum(velocity) + kV * velocity;
    }

    /**
     * Calculate feedforward voltage for an elevator at a given velocity and acceleration.
     */
    public double calculateElevator(double velocity, double acceleration) {
        return kG + kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
    }

    /**
     * Calculate the static holding voltage for an elevator (velocity = 0).
     */
    public double calculateElevator() {
        return kG;
    }

    /**
     * Calculate feedforward voltage for an arm at a given angle.
     * Gravity torque varies with cosine of the angle from horizontal.
     * @param angleRadians angle from horizontal in radians
     * @param velocity target angular velocity
     * @return feedforward voltage
     */
    public double calculateArm(double angleRadians, double velocity) {
        return kG * Math.cos(angleRadians) + kS * Math.signum(velocity) + kV * velocity;
    }

    /**
     * Calculate feedforward voltage for an arm with acceleration.
     */
    public double calculateArm(double angleRadians, double velocity, double acceleration) {
        return kG * Math.cos(angleRadians) + kS * Math.signum(velocity)
                + kV * velocity + kA * acceleration;
    }

    /**
     * Calculate the static holding voltage for an arm at a given angle.
     * @param angleRadians angle from horizontal in radians
     */
    public double calculateArm(double angleRadians) {
        return kG * Math.cos(angleRadians);
    }

    @Override
    public String toString() {
        return String.format("FF(kS=%.4f, kV=%.4f, kA=%.4f, kG=%.4f)", kS, kV, kA, kG);
    }
}
