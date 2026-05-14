package frc.lib.catalyst.io;

import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.LogTable;

/**
 * Per-loop input snapshot for a {@code ClawMechanism}.
 *
 * <p>Captures motor health, the current grip-state machine, and piece-detection
 * signals (beam break and stall-based). Designed for motor-driven open/close
 * grippers — pneumatic claws should use a {@code PneumaticMechanism} instead.
 */
public class ClawMechanismInputs implements CatalystInputs {

    /** Current commanded duty cycle in [-1, 1], estimated from applied voltage. */
    public double dutyCycle = 0.0;

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

    /**
     * Current claw grip state, encoded as a string for ease of dashboarding.
     * One of {@code "OPEN"}, {@code "CLOSING"}, {@code "HOLDING"}, {@code "OPENING"}, {@code "IDLE"}.
     */
    public String gripState = "IDLE";

    /** True when the mechanism is currently confident it is holding a piece. */
    public boolean hasPiece = false;

    /** Raw beam-break sensor state — {@code true} when the beam is broken (piece present). */
    public boolean beamBreakBroken = false;

    /** True when the close-current stall detector has tripped this cycle. */
    public boolean stallDetected = false;

    @Override
    public void toLog(LogTable table) {
        table.put("DutyCycle", dutyCycle);
        table.put("StatorCurrentAmps", statorCurrentAmps);
        table.put("SupplyCurrentAmps", supplyCurrentAmps);
        table.put("AppliedVolts", appliedVolts);
        table.put("TemperatureC", temperatureC);
        table.put("FollowerStatorCurrentAmps", followerStatorCurrentAmps);
        table.put("FollowerTemperatureC", followerTemperatureC);
        table.put("GripState", gripState);
        table.put("HasPiece", hasPiece);
        table.put("BeamBreakBroken", beamBreakBroken);
        table.put("StallDetected", stallDetected);
    }

    @Override
    public void fromLog(LogTable table) {
        dutyCycle = table.get("DutyCycle", dutyCycle);
        statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
        supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
        appliedVolts = table.get("AppliedVolts", appliedVolts);
        temperatureC = table.get("TemperatureC", temperatureC);
        followerStatorCurrentAmps = table.get("FollowerStatorCurrentAmps", followerStatorCurrentAmps);
        followerTemperatureC = table.get("FollowerTemperatureC", followerTemperatureC);
        gripState = table.get("GripState", gripState);
        hasPiece = table.get("HasPiece", hasPiece);
        beamBreakBroken = table.get("BeamBreakBroken", beamBreakBroken);
        stallDetected = table.get("StallDetected", stallDetected);
    }
}
