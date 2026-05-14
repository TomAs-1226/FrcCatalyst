package frc.lib.catalyst.io;

/**
 * Hardware contract for a roller mechanism (intake, conveyor, indexer).
 *
 * <p>Catalyst v0.3 ships the interface and {@link RollerMechanismInputs}.
 * Default Phoenix 6 implementation lands in v0.4.
 */
public interface RollerMechanismIO {

    /** Fill {@code inputs} with the latest sensor state. */
    void updateInputs(RollerMechanismInputs inputs);

    /** Command an open-loop duty cycle in [-1, 1]. */
    void setPercent(double percent);

    /** Command an open-loop voltage. */
    void setVoltage(double volts);

    /** Neutral / coast / brake — the mechanism's current configuration determines which. */
    void stop();
}
