package frc.lib.catalyst.io;

/**
 * Hardware contract for a winch / climbing mechanism.
 *
 * <p>Catalyst v0.3 ships the interface and {@link WinchMechanismInputs}.
 * Default Phoenix 6 / DCMotorSim implementation lands in v0.4.
 */
public interface WinchMechanismIO {

    /** Fill {@code inputs} with the latest sensor state. */
    void updateInputs(WinchMechanismInputs inputs);

    /** Command an open-loop duty cycle in [-1, 1]. Positive = extend. */
    void setPercent(double percent);

    /** Command an open-loop voltage. */
    void setVoltage(double volts);

    /** Seed the winch's extension reading to {@code meters} on the next loop. */
    void setExtension(double meters);

    /** Neutral / coast / brake — the mechanism's current configuration determines which. */
    void stop();
}
