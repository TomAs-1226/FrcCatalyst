package frc.lib.catalyst.io;

import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.LogTable;

/**
 * Per-loop input snapshot for a {@code RotationalMechanism}.
 *
 * <p>Captures angular position, velocity, motor health, hard-stop state,
 * and the current setpoint. Field names mirror existing
 * {@code /Catalyst/&lt;name&gt;/...} NT keys so v0.2 dashboards keep working.
 */
public class RotationalMechanismInputs implements CatalystInputs {

    /** Mechanism angle in degrees. */
    public double angleDegrees = 0.0;

    /** Mechanism angular velocity in degrees per second. */
    public double angularVelocityDPS = 0.0;

    /** Stator current on the leader motor in amps. */
    public double statorCurrentAmps = 0.0;

    /** Supply current on the leader motor in amps. */
    public double supplyCurrentAmps = 0.0;

    /** Applied output voltage on the leader. */
    public double appliedVolts = 0.0;

    /** Leader motor temperature in degrees Celsius. */
    public double temperatureC = 0.0;

    /** Stator currents of follower motors, in configuration order. */
    public double[] followerStatorCurrentAmps = new double[0];

    /** Temperatures of follower motors, in configuration order. */
    public double[] followerTemperatureC = new double[0];

    /** Last commanded setpoint in degrees. */
    public double setpointDegrees = 0.0;

    /** True when the mechanism is within the configured angular tolerance of the setpoint. */
    public boolean atSetpoint = false;

    /** True when the encoder has been seeded by hard stop or manual reset. */
    public boolean hasBeenZeroed = false;

    /** True when the hard-stop switch is currently pressed. */
    public boolean hardStopPressed = false;

    @Override
    public void toLog(LogTable table) {
        table.put("AngleDegrees", angleDegrees);
        table.put("AngularVelocityDPS", angularVelocityDPS);
        table.put("StatorCurrentAmps", statorCurrentAmps);
        table.put("SupplyCurrentAmps", supplyCurrentAmps);
        table.put("AppliedVolts", appliedVolts);
        table.put("TemperatureC", temperatureC);
        table.put("FollowerStatorCurrentAmps", followerStatorCurrentAmps);
        table.put("FollowerTemperatureC", followerTemperatureC);
        table.put("SetpointDegrees", setpointDegrees);
        table.put("AtSetpoint", atSetpoint);
        table.put("HasBeenZeroed", hasBeenZeroed);
        table.put("HardStopPressed", hardStopPressed);
    }

    @Override
    public void fromLog(LogTable table) {
        angleDegrees = table.get("AngleDegrees", angleDegrees);
        angularVelocityDPS = table.get("AngularVelocityDPS", angularVelocityDPS);
        statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
        supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
        appliedVolts = table.get("AppliedVolts", appliedVolts);
        temperatureC = table.get("TemperatureC", temperatureC);
        followerStatorCurrentAmps = table.get("FollowerStatorCurrentAmps", followerStatorCurrentAmps);
        followerTemperatureC = table.get("FollowerTemperatureC", followerTemperatureC);
        setpointDegrees = table.get("SetpointDegrees", setpointDegrees);
        atSetpoint = table.get("AtSetpoint", atSetpoint);
        hasBeenZeroed = table.get("HasBeenZeroed", hasBeenZeroed);
        hardStopPressed = table.get("HardStopPressed", hardStopPressed);
    }
}
