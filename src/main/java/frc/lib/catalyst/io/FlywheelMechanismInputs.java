package frc.lib.catalyst.io;

import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.LogTable;

/**
 * Per-loop input snapshot for a {@code FlywheelMechanism}.
 *
 * <p>Captures velocity, setpoint, and at-speed state for one or two motors.
 * Field names match existing v0.2 telemetry keys.
 */
public class FlywheelMechanismInputs implements CatalystInputs {

    /** Velocity of the primary motor in rotations per second. */
    public double primaryVelocityRPS = 0.0;

    /** Velocity of the secondary motor in rotations per second; {@code 0} when not configured. */
    public double secondaryVelocityRPS = 0.0;

    /** Stator current of the primary motor in amps. */
    public double primaryStatorCurrentAmps = 0.0;

    /** Stator current of the secondary motor in amps; {@code 0} when not configured. */
    public double secondaryStatorCurrentAmps = 0.0;

    /** Applied voltage on the primary motor. */
    public double primaryAppliedVolts = 0.0;

    /** Applied voltage on the secondary motor; {@code 0} when not configured. */
    public double secondaryAppliedVolts = 0.0;

    /** Primary motor temperature in Celsius. */
    public double primaryTemperatureC = 0.0;

    /** Secondary motor temperature in Celsius; {@code 0} when not configured. */
    public double secondaryTemperatureC = 0.0;

    /** Last commanded setpoint for the primary motor (rotations per second). */
    public double primarySetpointRPS = 0.0;

    /** Last commanded setpoint for the secondary motor (rotations per second). */
    public double secondarySetpointRPS = 0.0;

    /** True when both motors are within the configured velocity tolerance of their setpoints. */
    public boolean atSpeed = false;

    @Override
    public void toLog(LogTable table) {
        table.put("PrimaryVelocityRPS", primaryVelocityRPS);
        table.put("SecondaryVelocityRPS", secondaryVelocityRPS);
        table.put("PrimaryStatorCurrentAmps", primaryStatorCurrentAmps);
        table.put("SecondaryStatorCurrentAmps", secondaryStatorCurrentAmps);
        table.put("PrimaryAppliedVolts", primaryAppliedVolts);
        table.put("SecondaryAppliedVolts", secondaryAppliedVolts);
        table.put("PrimaryTemperatureC", primaryTemperatureC);
        table.put("SecondaryTemperatureC", secondaryTemperatureC);
        table.put("PrimarySetpointRPS", primarySetpointRPS);
        table.put("SecondarySetpointRPS", secondarySetpointRPS);
        table.put("AtSpeed", atSpeed);
    }

    @Override
    public void fromLog(LogTable table) {
        primaryVelocityRPS = table.get("PrimaryVelocityRPS", primaryVelocityRPS);
        secondaryVelocityRPS = table.get("SecondaryVelocityRPS", secondaryVelocityRPS);
        primaryStatorCurrentAmps = table.get("PrimaryStatorCurrentAmps", primaryStatorCurrentAmps);
        secondaryStatorCurrentAmps = table.get("SecondaryStatorCurrentAmps", secondaryStatorCurrentAmps);
        primaryAppliedVolts = table.get("PrimaryAppliedVolts", primaryAppliedVolts);
        secondaryAppliedVolts = table.get("SecondaryAppliedVolts", secondaryAppliedVolts);
        primaryTemperatureC = table.get("PrimaryTemperatureC", primaryTemperatureC);
        secondaryTemperatureC = table.get("SecondaryTemperatureC", secondaryTemperatureC);
        primarySetpointRPS = table.get("PrimarySetpointRPS", primarySetpointRPS);
        secondarySetpointRPS = table.get("SecondarySetpointRPS", secondarySetpointRPS);
        atSpeed = table.get("AtSpeed", atSpeed);
    }
}
