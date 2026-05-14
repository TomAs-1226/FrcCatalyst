package frc.lib.catalyst.io;

/**
 * Hardware contract for a flywheel or shooter wheel.
 *
 * <p>Supports a primary wheel and an optional secondary wheel running at an
 * independent velocity (e.g., differential spin for shot arc).
 *
 * <p>Catalyst v0.3 ships the interface and {@link FlywheelMechanismInputs}.
 * Default Phoenix 6 / FlywheelSim implementation lands in v0.4.
 */
public interface FlywheelMechanismIO {

    /** Fill {@code inputs} with the latest sensor state. */
    void updateInputs(FlywheelMechanismInputs inputs);

    /**
     * Command a closed-loop velocity target in rotations per second.
     * For dual-motor flywheels, both motors run at this velocity.
     */
    void setVelocity(double rps);

    /**
     * Command independent velocities for the primary and secondary motors.
     * Implementations that only own one motor should ignore {@code secondaryRps}.
     */
    void setVelocity(double primaryRps, double secondaryRps);

    /** Open-loop voltage command applied to both motors. */
    void setVoltage(double volts);

    /** Coast both wheels. */
    void stop();
}
