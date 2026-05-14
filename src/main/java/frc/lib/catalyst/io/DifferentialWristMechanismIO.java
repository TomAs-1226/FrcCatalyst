package frc.lib.catalyst.io;

/**
 * Hardware contract for a two-motor differential wrist.
 *
 * <p>Commands are expressed in per-motor mechanism rotations (after gearing).
 * Pitch and roll resolution is the mechanism's responsibility; the IO layer
 * just deals with the two physical motors.
 *
 * <p>Catalyst v0.3 ships the interface and {@link DifferentialWristMechanismInputs}.
 * Default Phoenix 6 implementation lands in v0.4.
 */
public interface DifferentialWristMechanismIO {

    /** Fill {@code inputs} with the latest sensor state. */
    void updateInputs(DifferentialWristMechanismInputs inputs);

    /** Command Motion Magic position targets for the left and right motors, in mechanism rotations. */
    void setMotionMagicPositions(double leftRotations, double rightRotations);

    /** Command open-loop voltages for the left and right motors. */
    void setVoltages(double leftVolts, double rightVolts);

    /** Seed the encoder positions for both motors, in mechanism rotations. */
    void setEncoderPositions(double leftRotations, double rightRotations);

    /** Stop both motors. */
    void stop();
}
