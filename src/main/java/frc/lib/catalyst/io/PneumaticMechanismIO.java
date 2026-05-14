package frc.lib.catalyst.io;

/**
 * Hardware contract for a pneumatic actuator (single or double solenoid).
 *
 * <p>Catalyst v0.3 ships the interface and {@link PneumaticMechanismInputs}.
 * Default Single/DoubleSolenoid implementation lands in v0.4.
 */
public interface PneumaticMechanismIO {

    /** Fill {@code inputs} with the latest commanded state and pressure. */
    void updateInputs(PneumaticMechanismInputs inputs);

    /** Drive the actuator to its forward / extended position. */
    void setForward();

    /**
     * Drive the actuator to its reverse / retracted position.
     * Single-solenoid implementations should treat this as {@link #setOff()}.
     */
    void setReverse();

    /**
     * Stop driving the solenoid. For double solenoids this is the explicit
     * "off" state; for single solenoids it retracts the piston.
     */
    void setOff();
}
