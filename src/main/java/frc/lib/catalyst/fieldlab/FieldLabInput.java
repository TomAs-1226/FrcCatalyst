package frc.lib.catalyst.fieldlab;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * The two controls a person uses to answer the campaign: a number, and "go on".
 *
 * <h2>The wire path, and why it is this one</h2>
 *
 * <p>Both live under {@code /Catalyst/Tuning/}, the table Catalyst Console and Catalyst Tab already
 * know how to write:
 *
 * <ul>
 *   <li>{@code /Catalyst/Tuning/FieldLab/Value} — the number being typed.
 *   <li>{@code /Catalyst/Tuning/FieldLab/Advance} — change it to any other number to commit.
 * </ul>
 *
 * <p>Reusing the tunables path means no dashboard has to be modified to run a campaign, and — the
 * part that matters — it does not give a dashboard any new authority. Catalyst publishes no command
 * triggers over NetworkTables, on purpose: the diagnostic routines are Driver Station Utility op
 * modes and a dashboard only shows their results. A tape-measure reading is data of exactly the kind
 * tunables already carry. "Advance" looks more like a command than a gain does, but it cannot start,
 * stop or steer anything; it can only let a Utility op mode that a person already selected and
 * enabled move to its next step.
 *
 * <h2>Why not TunableNumber</h2>
 *
 * <p>{@link frc.lib.catalyst.util.TunableNumber} writes the same keys and would be the obvious
 * choice, but it is gated on a global {@code tuningEnabled} flag that teams are told to switch off
 * for competition — and a campaign whose operator prompts silently stop working because someone
 * called {@code disableTuning()} in the robot constructor would be very hard to diagnose at a field.
 * These read NetworkTables directly for that reason, while keeping the same keys so the dashboards
 * see no difference.
 *
 * <h2>Advance is edge-triggered, not level-triggered</h2>
 *
 * <p>{@link #advanced()} reports a change since the last call, not a value. A level would mean the
 * operator had to set it and clear it for every step, and a step that forgot to clear would run the
 * whole campaign through in one loop — twenty procedures in twenty milliseconds, which is a real
 * failure mode and not a hypothetical one. A change is also what a dashboard's number field naturally
 * produces: type anything different and press enter.
 *
 * @since 2.0.0
 */
public final class FieldLabInput {

    /** The table tunables live in, so these keys arrive where the dashboards already look. */
    private static final String TABLE = "Catalyst/Tuning";

    private final NetworkTableEntry value;
    private final NetworkTableEntry advance;
    private final NetworkTableEntry prompt;
    private final NetworkTableEntry unit;

    private double lastAdvance;

    /** Wire up the input entries and seed them. */
    public FieldLabInput() {
        NetworkTable tuning = NetworkTableInstance.getDefault().getTable(TABLE);
        this.value = tuning.getEntry("FieldLab/Value");
        this.advance = tuning.getEntry("FieldLab/Advance");
        this.value.setDouble(0.0);
        this.advance.setDouble(0.0);
        this.lastAdvance = 0.0;

        // The prompt is read-only status, so it belongs beside the rest of the campaign's telemetry
        // rather than among the writable tunables.
        NetworkTable lab = NetworkTableInstance.getDefault().getTable("Catalyst").getSubTable("FieldLab");
        this.prompt = lab.getEntry("Prompt");
        this.unit = lab.getEntry("PromptUnit");
        this.prompt.setString("");
        this.unit.setString("");
    }

    /**
     * Whether the operator has committed since this was last asked.
     *
     * <p>Edge-triggered and self-clearing: calling it twice in one loop reports the change once.
     */
    public boolean advanced() {
        double now = advance.getDouble(lastAdvance);
        if (now != lastAdvance) {
            lastAdvance = now;
            return true;
        }
        return false;
    }

    /** The number currently typed in. */
    public double value() {
        return value.getDouble(0.0);
    }

    /** Show an instruction, with the unit the answer should be in. */
    public void ask(String instruction, String units) {
        prompt.setString(instruction == null ? "" : instruction);
        unit.setString(units == null ? "" : units);
    }

    /** Show an instruction that wants no number. */
    public void ask(String instruction) {
        ask(instruction, "");
    }

    /** Stop asking. */
    public void clear() {
        ask("", "");
    }

    /**
     * Forget any pending edge, so a step does not inherit an operator's earlier press.
     *
     * <p>Called as each waiting step begins. Without it, an operator who double-tapped on the
     * previous step would skip the next one — which reads on the log as a measurement of zero.
     */
    public void armed() {
        lastAdvance = advance.getDouble(lastAdvance);
    }
}
