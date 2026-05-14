package frc.lib.catalyst.io;

import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.LogTable;

/**
 * Per-loop input snapshot for a {@code PneumaticMechanism}.
 *
 * <p>Captures the commanded state, optional compressor pressure, and the
 * timestamp of the most recent state change. Useful for diagnosing pneumatic
 * cycle counts and pressure dips during competition.
 */
public class PneumaticMechanismInputs implements CatalystInputs {

    /**
     * Commanded state, encoded as a string for dashboarding.
     * One of {@code "FORWARD"}, {@code "REVERSE"}, {@code "OFF"}.
     * Single solenoids report {@code "FORWARD"} (extended) or {@code "OFF"} (retracted).
     */
    public String state = "OFF";

    /** Raw boolean — true when commanded forward / extended, false otherwise. */
    public boolean forwardCommanded = false;

    /** Raw boolean — true when commanded reverse on a double solenoid. */
    public boolean reverseCommanded = false;

    /**
     * Last analog compressor pressure reading in psi, or {@code -1} if no
     * pressure sensor was configured. REVPH only.
     */
    public double pressurePSI = -1.0;

    /** FPGA timestamp (seconds) of the most recent state transition. */
    public double lastTransitionTimestamp = 0.0;

    /** Cumulative count of FORWARD ↔ REVERSE transitions since construction (useful for cycle-life tracking). */
    public long transitionCount = 0L;

    @Override
    public void toLog(LogTable table) {
        table.put("State", state);
        table.put("ForwardCommanded", forwardCommanded);
        table.put("ReverseCommanded", reverseCommanded);
        table.put("PressurePSI", pressurePSI);
        table.put("LastTransitionTimestamp", lastTransitionTimestamp);
        table.put("TransitionCount", transitionCount);
    }

    @Override
    public void fromLog(LogTable table) {
        state = table.get("State", state);
        forwardCommanded = table.get("ForwardCommanded", forwardCommanded);
        reverseCommanded = table.get("ReverseCommanded", reverseCommanded);
        pressurePSI = table.get("PressurePSI", pressurePSI);
        lastTransitionTimestamp = table.get("LastTransitionTimestamp", lastTransitionTimestamp);
        transitionCount = table.get("TransitionCount", transitionCount);
    }
}
