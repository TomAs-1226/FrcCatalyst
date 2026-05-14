package frc.lib.catalyst.io;

import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.LogTable;

/**
 * Per-loop input snapshot for a {@code DifferentialWristMechanism}.
 *
 * <p>A diffy wrist couples two motors through a bevel-gear differential so that
 * the sum of motor rotations drives one axis (pitch) and the difference drives
 * a second axis (roll). This snapshot exposes both per-motor signals and the
 * resolved pitch / roll state.
 */
public class DifferentialWristMechanismInputs implements CatalystInputs {

    /** Resolved pitch axis position in degrees. */
    public double pitchDegrees = 0.0;

    /** Resolved roll axis position in degrees. */
    public double rollDegrees = 0.0;

    /** Resolved pitch axis velocity in degrees per second. */
    public double pitchVelocityDPS = 0.0;

    /** Resolved roll axis velocity in degrees per second. */
    public double rollVelocityDPS = 0.0;

    /** Last commanded pitch setpoint in degrees. */
    public double pitchSetpointDegrees = 0.0;

    /** Last commanded roll setpoint in degrees. */
    public double rollSetpointDegrees = 0.0;

    /** True when both axes are within their configured tolerance. */
    public boolean atSetpoint = false;

    /** Stator current on the left motor in amps. */
    public double leftStatorCurrentAmps = 0.0;

    /** Stator current on the right motor in amps. */
    public double rightStatorCurrentAmps = 0.0;

    /** Applied voltage on the left motor. */
    public double leftAppliedVolts = 0.0;

    /** Applied voltage on the right motor. */
    public double rightAppliedVolts = 0.0;

    /** Left motor temperature in degrees Celsius. */
    public double leftTemperatureC = 0.0;

    /** Right motor temperature in degrees Celsius. */
    public double rightTemperatureC = 0.0;

    /** True when both motors have been seeded to a known position. */
    public boolean hasBeenZeroed = false;

    @Override
    public void toLog(LogTable table) {
        table.put("PitchDegrees", pitchDegrees);
        table.put("RollDegrees", rollDegrees);
        table.put("PitchVelocityDPS", pitchVelocityDPS);
        table.put("RollVelocityDPS", rollVelocityDPS);
        table.put("PitchSetpointDegrees", pitchSetpointDegrees);
        table.put("RollSetpointDegrees", rollSetpointDegrees);
        table.put("AtSetpoint", atSetpoint);
        table.put("LeftStatorCurrentAmps", leftStatorCurrentAmps);
        table.put("RightStatorCurrentAmps", rightStatorCurrentAmps);
        table.put("LeftAppliedVolts", leftAppliedVolts);
        table.put("RightAppliedVolts", rightAppliedVolts);
        table.put("LeftTemperatureC", leftTemperatureC);
        table.put("RightTemperatureC", rightTemperatureC);
        table.put("HasBeenZeroed", hasBeenZeroed);
    }

    @Override
    public void fromLog(LogTable table) {
        pitchDegrees = table.get("PitchDegrees", pitchDegrees);
        rollDegrees = table.get("RollDegrees", rollDegrees);
        pitchVelocityDPS = table.get("PitchVelocityDPS", pitchVelocityDPS);
        rollVelocityDPS = table.get("RollVelocityDPS", rollVelocityDPS);
        pitchSetpointDegrees = table.get("PitchSetpointDegrees", pitchSetpointDegrees);
        rollSetpointDegrees = table.get("RollSetpointDegrees", rollSetpointDegrees);
        atSetpoint = table.get("AtSetpoint", atSetpoint);
        leftStatorCurrentAmps = table.get("LeftStatorCurrentAmps", leftStatorCurrentAmps);
        rightStatorCurrentAmps = table.get("RightStatorCurrentAmps", rightStatorCurrentAmps);
        leftAppliedVolts = table.get("LeftAppliedVolts", leftAppliedVolts);
        rightAppliedVolts = table.get("RightAppliedVolts", rightAppliedVolts);
        leftTemperatureC = table.get("LeftTemperatureC", leftTemperatureC);
        rightTemperatureC = table.get("RightTemperatureC", rightTemperatureC);
        hasBeenZeroed = table.get("HasBeenZeroed", hasBeenZeroed);
    }
}
