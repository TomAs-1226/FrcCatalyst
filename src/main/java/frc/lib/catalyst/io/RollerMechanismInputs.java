package frc.lib.catalyst.io;

import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.LogTable;

/**
 * Per-loop input snapshot for a {@code RollerMechanism}.
 *
 * <p>Captures roller velocity, current draw, beam-break and stall state,
 * and a duty-cycle estimate. Used for both live telemetry and replay.
 */
public class RollerMechanismInputs implements CatalystInputs {

    /** Current commanded duty cycle (-1 to 1) as estimated from applied voltage. */
    public double dutyCycle = 0.0;

    /** Stator current draw on the motor in amps. */
    public double statorCurrentAmps = 0.0;

    /** Supply current draw on the motor in amps. */
    public double supplyCurrentAmps = 0.0;

    /** Applied output voltage. */
    public double appliedVolts = 0.0;

    /** Motor temperature in degrees Celsius. */
    public double temperatureC = 0.0;

    /** Cumulative motor rotations (useful for stall recovery diagnostics). */
    public double motorRotations = 0.0;

    /** True when the mechanism currently thinks it is holding a game piece. */
    public boolean hasPiece = false;

    /**
     * Raw beam-break sensor state, or {@code false} when no beam break is
     * configured. {@code true} when the beam is broken (piece in the path).
     */
    public boolean beamBreakBroken = false;

    /** True when the stall-detection state machine is currently tripped. */
    public boolean stalled = false;

    @Override
    public void toLog(LogTable table) {
        table.put("DutyCycle", dutyCycle);
        table.put("StatorCurrentAmps", statorCurrentAmps);
        table.put("SupplyCurrentAmps", supplyCurrentAmps);
        table.put("AppliedVolts", appliedVolts);
        table.put("TemperatureC", temperatureC);
        table.put("MotorRotations", motorRotations);
        table.put("HasPiece", hasPiece);
        table.put("BeamBreakBroken", beamBreakBroken);
        table.put("Stalled", stalled);
    }

    @Override
    public void fromLog(LogTable table) {
        dutyCycle = table.get("DutyCycle", dutyCycle);
        statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
        supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
        appliedVolts = table.get("AppliedVolts", appliedVolts);
        temperatureC = table.get("TemperatureC", temperatureC);
        motorRotations = table.get("MotorRotations", motorRotations);
        hasPiece = table.get("HasPiece", hasPiece);
        beamBreakBroken = table.get("BeamBreakBroken", beamBreakBroken);
        stalled = table.get("Stalled", stalled);
    }
}
