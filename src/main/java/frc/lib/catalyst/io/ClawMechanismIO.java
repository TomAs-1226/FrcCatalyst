package frc.lib.catalyst.io;

/**
 * Hardware contract for a motor-driven claw / gripper.
 *
 * <p>Catalyst v0.3 ships the interface and {@link ClawMechanismInputs}.
 * Default Phoenix 6 implementation lands in v0.4.
 */
public interface ClawMechanismIO {

    /** Fill {@code inputs} with the latest sensor state. */
    void updateInputs(ClawMechanismInputs inputs);

    /** Command an open-loop duty cycle in [-1, 1]. Positive = close, negative = open by convention. */
    void setPercent(double percent);

    /** Command an open-loop voltage. Positive = close, negative = open by convention. */
    void setVoltage(double volts);

    /** Neutral / coast / brake — the mechanism's current configuration determines which. */
    void stop();
}
