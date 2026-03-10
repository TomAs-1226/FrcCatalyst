package frc.lib.catalyst.hardware;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Enum of supported FRC motors with their specifications.
 * Used for accurate simulation and physics calculations.
 *
 * <p>Example usage:
 * <pre>{@code
 * LinearMechanism.Config.builder()
 *     .motorType(MotorType.KRAKEN_X60)
 *     .motorCount(2)
 *     // ...
 * }</pre>
 */
public enum MotorType {
    KRAKEN_X60(7.09, 6000, 366, "Kraken X60"),
    KRAKEN_X60_FOC(7.09, 5800, 366, "Kraken X60 FOC"),
    FALCON_500(4.69, 6380, 257, "Falcon 500"),
    FALCON_500_FOC(4.69, 6080, 257, "Falcon 500 FOC");

    /** Stall torque in Nm. */
    public final double stallTorqueNm;
    /** Free speed in RPM. */
    public final double freeSpeedRPM;
    /** Stall current in amps. */
    public final double stallCurrentAmps;
    /** Human-readable name. */
    public final String displayName;

    MotorType(double stallTorqueNm, double freeSpeedRPM, double stallCurrentAmps, String displayName) {
        this.stallTorqueNm = stallTorqueNm;
        this.freeSpeedRPM = freeSpeedRPM;
        this.stallCurrentAmps = stallCurrentAmps;
        this.displayName = displayName;
    }

    /** Get the free speed in rotations per second. */
    public double freeSpeedRPS() {
        return freeSpeedRPM / 60.0;
    }

    /** Get the free speed in radians per second. */
    public double freeSpeedRadPerSec() {
        return freeSpeedRPM * 2.0 * Math.PI / 60.0;
    }

    /**
     * Get the WPILib DCMotor model for simulation.
     * @param numMotors number of motors ganged together
     */
    public DCMotor getDCMotor(int numMotors) {
        switch (this) {
            case KRAKEN_X60:
            case KRAKEN_X60_FOC:
                return DCMotor.getKrakenX60(numMotors);
            case FALCON_500:
            case FALCON_500_FOC:
                return DCMotor.getFalcon500(numMotors);
            default:
                return DCMotor.getKrakenX60(numMotors);
        }
    }

    /**
     * Calculate the maximum mechanism speed given a gear ratio.
     * @param gearRatio motor rotations per mechanism rotation
     * @return max mechanism speed in RPM
     */
    public double maxMechanismRPM(double gearRatio) {
        return freeSpeedRPM / gearRatio;
    }

    /**
     * Calculate the maximum mechanism torque given a gear ratio and motor count.
     * @param gearRatio motor rotations per mechanism rotation
     * @param motorCount number of motors
     * @return max torque in Nm
     */
    public double maxMechanismTorque(double gearRatio, int motorCount) {
        return stallTorqueNm * gearRatio * motorCount;
    }

    /**
     * Estimate the voltage needed to hold a load against gravity.
     * @param loadTorqueNm the gravity torque at the mechanism in Nm
     * @param gearRatio motor rotations per mechanism rotation
     * @return estimated holding voltage (0-12V)
     */
    public double holdingVoltage(double loadTorqueNm, double gearRatio) {
        double motorTorque = loadTorqueNm / gearRatio;
        return (motorTorque / stallTorqueNm) * 12.0;
    }
}
