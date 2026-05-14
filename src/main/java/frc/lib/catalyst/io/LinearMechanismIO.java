package frc.lib.catalyst.io;

/**
 * Hardware contract for a linear (elevator / slide / telescoping) mechanism.
 *
 * <p>An implementation owns one leader motor plus any followers, the
 * conversion between motor rotations and meters, and any wired limit
 * switches. The mechanism layer issues control commands through this
 * interface and pulls sensor state through {@link #updateInputs}.
 *
 * <p>Catalyst v0.3 ships the interface and {@link LinearMechanismInputs}.
 * Default Phoenix 6 / ElevatorSim implementations land in v0.4. Teams
 * that want IO swapping today can implement this interface directly
 * against any motor controller; the rest of Catalyst is agnostic.
 */
public interface LinearMechanismIO {

    /**
     * Fill {@code inputs} with the latest sensor state.
     * Called once per loop, before the mechanism makes any decisions.
     */
    void updateInputs(LinearMechanismInputs inputs);

    /**
     * Command a Motion-Magic-style smoothed position target.
     * @param meters absolute mechanism position
     */
    void setMotionMagicPosition(double meters);

    /**
     * Command a Motion-Magic-style position target with an additional
     * feedforward voltage.
     * @param meters         absolute mechanism position
     * @param feedforwardVolts arbitrary feedforward voltage added to the closed loop output
     */
    void setMotionMagicPosition(double meters, double feedforwardVolts);

    /**
     * Open-loop voltage command. Use for jogging and manual override.
     * @param volts target output voltage (clamped by the implementation to battery range)
     */
    void setVoltage(double volts);

    /**
     * Seed the encoder so the mechanism's current physical position reads as
     * {@code meters} on the next call to {@link #updateInputs}.
     */
    void setEncoderPosition(double meters);

    /** Neutral / coast / brake — the mechanism's current configuration determines which. */
    void stop();
}
