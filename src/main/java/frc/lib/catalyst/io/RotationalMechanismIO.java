package frc.lib.catalyst.io;

/**
 * Hardware contract for a rotational mechanism (arm, wrist, turret, hood).
 *
 * <p>An implementation owns one or more motors driving a single rotational
 * axis. The mechanism layer hands position commands to this interface and
 * pulls sensor state through {@link #updateInputs}.
 *
 * <p>Catalyst v0.3 ships the interface and {@link RotationalMechanismInputs}.
 * Default Phoenix 6 / SingleJointedArmSim implementations land in v0.4.
 */
public interface RotationalMechanismIO {

    /**
     * Fill {@code inputs} with the latest sensor state.
     * Called once per loop, before the mechanism makes any decisions.
     */
    void updateInputs(RotationalMechanismInputs inputs);

    /**
     * Command a Motion-Magic-style smoothed angular target in degrees.
     */
    void setMotionMagicPosition(double degrees);

    /**
     * Command a Motion-Magic-style angular target with an additional
     * feedforward voltage.
     */
    void setMotionMagicPosition(double degrees, double feedforwardVolts);

    /** Open-loop voltage command. */
    void setVoltage(double volts);

    /** Seed the encoder so the mechanism reads as {@code degrees} on the next loop. */
    void setEncoderPosition(double degrees);

    /** Neutral / coast / brake — the mechanism's current configuration determines which. */
    void stop();
}
