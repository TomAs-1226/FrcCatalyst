package frc.lib.catalyst.io;

import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.LogTable;

/**
 * Per-loop input snapshot for a {@code WinchMechanism}.
 *
 * <p>Captures cable extension (computed from spool rotations), motor health,
 * and the current commanded velocity. Designed for climbing winches and
 * dual-motor coupled spools.
 */
public class WinchMechanismInputs implements CatalystInputs {

    /** Estimated extension of the winch cable in meters. */
    public double extensionMeters = 0.0;

    /** Current cable speed in meters per second. */
    public double velocityMPS = 0.0;

    /** Stator current on the leader motor in amps. */
    public double statorCurrentAmps = 0.0;

    /** Supply current on the leader motor in amps. */
    public double supplyCurrentAmps = 0.0;

    /** Applied voltage on the leader. */
    public double appliedVolts = 0.0;

    /** Leader motor temperature in Celsius. */
    public double temperatureC = 0.0;

    /** Stator currents of follower motors, in configuration order. */
    public double[] followerStatorCurrentAmps = new double[0];

    /** Temperatures of follower motors, in configuration order. */
    public double[] followerTemperatureC = new double[0];

    /** True when the mechanism has been seeded with a known extension. */
    public boolean hasBeenZeroed = false;

    @Override
    public void toLog(LogTable table) {
        table.put("ExtensionMeters", extensionMeters);
        table.put("VelocityMPS", velocityMPS);
        table.put("StatorCurrentAmps", statorCurrentAmps);
        table.put("SupplyCurrentAmps", supplyCurrentAmps);
        table.put("AppliedVolts", appliedVolts);
        table.put("TemperatureC", temperatureC);
        table.put("FollowerStatorCurrentAmps", followerStatorCurrentAmps);
        table.put("FollowerTemperatureC", followerTemperatureC);
        table.put("HasBeenZeroed", hasBeenZeroed);
    }

    @Override
    public void fromLog(LogTable table) {
        extensionMeters = table.get("ExtensionMeters", extensionMeters);
        velocityMPS = table.get("VelocityMPS", velocityMPS);
        statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
        supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
        appliedVolts = table.get("AppliedVolts", appliedVolts);
        temperatureC = table.get("TemperatureC", temperatureC);
        followerStatorCurrentAmps = table.get("FollowerStatorCurrentAmps", followerStatorCurrentAmps);
        followerTemperatureC = table.get("FollowerTemperatureC", followerTemperatureC);
        hasBeenZeroed = table.get("HasBeenZeroed", hasBeenZeroed);
    }
}
