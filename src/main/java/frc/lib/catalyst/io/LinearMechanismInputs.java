package frc.lib.catalyst.io;

import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.LogTable;

/**
 * Per-loop input snapshot for a {@code LinearMechanism}.
 *
 * <p>Holds everything an elevator/linear-slide mechanism observes from its
 * hardware in a single loop iteration. Field names match the
 * {@code /Catalyst/&lt;name&gt;/...} keys that have been published since
 * v0.2, so existing dashboards and tuning workflows do not change.
 *
 * <p>This is a plain mutable POJO — the {@code *IO} layer (or, in v0.3,
 * the mechanism itself) writes to the fields each loop; the mechanism
 * reads from them.
 */
public class LinearMechanismInputs implements CatalystInputs {

    /** Mechanism position in meters (after gear ratio + drum/spool/stage conversion). */
    public double positionMeters = 0.0;

    /** Mechanism velocity in meters per second. */
    public double velocityMPS = 0.0;

    /** Stator current draw on the leader motor in amps. */
    public double statorCurrentAmps = 0.0;

    /** Supply current draw on the leader motor in amps. */
    public double supplyCurrentAmps = 0.0;

    /** Motor output voltage in volts (leader). */
    public double appliedVolts = 0.0;

    /** Leader motor temperature in degrees Celsius. */
    public double temperatureC = 0.0;

    /** Stator current draws of all follower motors, in the order they were configured. */
    public double[] followerStatorCurrentAmps = new double[0];

    /** Temperatures of all follower motors, in the order they were configured. */
    public double[] followerTemperatureC = new double[0];

    /** Last commanded setpoint in meters. */
    public double setpointMeters = 0.0;

    /** True when {@code |position - setpoint| &lt;= positionTolerance}. */
    public boolean atSetpoint = false;

    /** True when the encoder has been seeded (manually or by an auto-zero event). */
    public boolean hasBeenZeroed = false;

    /** State of the forward (top) limit switch, or {@code false} when not configured. */
    public boolean forwardLimitPressed = false;

    /** State of the reverse (bottom) limit switch, or {@code false} when not configured. */
    public boolean reverseLimitPressed = false;

    @Override
    public void toLog(LogTable table) {
        table.put("PositionMeters", positionMeters);
        table.put("VelocityMPS", velocityMPS);
        table.put("StatorCurrentAmps", statorCurrentAmps);
        table.put("SupplyCurrentAmps", supplyCurrentAmps);
        table.put("AppliedVolts", appliedVolts);
        table.put("TemperatureC", temperatureC);
        table.put("FollowerStatorCurrentAmps", followerStatorCurrentAmps);
        table.put("FollowerTemperatureC", followerTemperatureC);
        table.put("SetpointMeters", setpointMeters);
        table.put("AtSetpoint", atSetpoint);
        table.put("HasBeenZeroed", hasBeenZeroed);
        table.put("ForwardLimitPressed", forwardLimitPressed);
        table.put("ReverseLimitPressed", reverseLimitPressed);
    }

    @Override
    public void fromLog(LogTable table) {
        positionMeters = table.get("PositionMeters", positionMeters);
        velocityMPS = table.get("VelocityMPS", velocityMPS);
        statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
        supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
        appliedVolts = table.get("AppliedVolts", appliedVolts);
        temperatureC = table.get("TemperatureC", temperatureC);
        followerStatorCurrentAmps = table.get("FollowerStatorCurrentAmps", followerStatorCurrentAmps);
        followerTemperatureC = table.get("FollowerTemperatureC", followerTemperatureC);
        setpointMeters = table.get("SetpointMeters", setpointMeters);
        atSetpoint = table.get("AtSetpoint", atSetpoint);
        hasBeenZeroed = table.get("HasBeenZeroed", hasBeenZeroed);
        forwardLimitPressed = table.get("ForwardLimitPressed", forwardLimitPressed);
        reverseLimitPressed = table.get("ReverseLimitPressed", reverseLimitPressed);
    }
}
