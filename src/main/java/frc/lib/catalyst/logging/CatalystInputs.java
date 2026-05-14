package frc.lib.catalyst.logging;

/**
 * Contract for input snapshots produced by hardware IO layers.
 *
 * <p>An "inputs" object is a plain Java object that holds the most recent
 * sensor readings for a mechanism — position, velocity, current, temperature,
 * faults, sensor states, etc. The IO layer fills it in every loop; the
 * mechanism reads from it.
 *
 * <p>Implementations must:
 * <ul>
 *   <li>Have public mutable fields (or accessors) for each measurement, so the
 *       IO layer can write them directly.</li>
 *   <li>Implement {@link #toLog(LogTable)} to serialize all fields into a
 *       {@link LogTable}. This is what gets published and recorded.</li>
 *   <li>Implement {@link #fromLog(LogTable)} to populate fields from a
 *       {@link LogTable}. This is what replay tooling calls.</li>
 * </ul>
 *
 * <p>The shape of this interface intentionally mirrors AdvantageKit's
 * {@code LoggableInputs} so an in-house Catalyst inputs class can be bridged
 * to AdvantageKit in user code with no Catalyst-side dependency on it.
 */
public interface CatalystInputs {

    /**
     * Serialize every field into {@code table}.
     * Called once per loop after the IO layer has updated the inputs.
     */
    void toLog(LogTable table);

    /**
     * Populate every field from {@code table}.
     * Called by replay tooling to inject recorded inputs back into the mechanism.
     */
    void fromLog(LogTable table);
}
