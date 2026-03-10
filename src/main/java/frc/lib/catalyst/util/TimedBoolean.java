package frc.lib.catalyst.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * A boolean that must be true for a specified duration before it triggers.
 * Useful for debouncing noisy sensor inputs, detecting sustained conditions,
 * or requiring a button to be held.
 *
 * <p>Example usage:
 * <pre>{@code
 * // Game piece detection: current must be above threshold for 0.2 seconds
 * TimedBoolean hasPiece = new TimedBoolean(0.2);
 *
 * // In periodic:
 * boolean pieceDetected = hasPiece.update(motor.getStatorCurrent() > 25);
 *
 * // Rising edge detection: true only on the cycle it first triggers
 * TimedBoolean risingEdge = new TimedBoolean(0.0); // instant
 * if (risingEdge.risingEdge(sensor.get())) {
 *     // This runs once when sensor first becomes true
 * }
 * }</pre>
 */
public class TimedBoolean {

    private final double requiredDuration;
    private final Timer timer = new Timer();
    private boolean lastInput = false;
    private boolean output = false;
    private boolean previousOutput = false;

    /**
     * Create a timed boolean.
     * @param requiredDurationSeconds how long the input must be true to trigger
     */
    public TimedBoolean(double requiredDurationSeconds) {
        this.requiredDuration = requiredDurationSeconds;
    }

    /**
     * Update with the current input value.
     * @param input the raw boolean value
     * @return true if input has been true for the required duration
     */
    public boolean update(boolean input) {
        previousOutput = output;

        if (input && !lastInput) {
            timer.restart();
        }

        if (!input) {
            output = false;
        } else if (timer.get() >= requiredDuration) {
            output = true;
        }

        lastInput = input;
        return output;
    }

    /** Get the current output without updating. */
    public boolean get() {
        return output;
    }

    /** Check if the output just became true (rising edge). */
    public boolean risingEdge(boolean input) {
        update(input);
        return output && !previousOutput;
    }

    /** Check if the output just became false (falling edge). */
    public boolean fallingEdge(boolean input) {
        update(input);
        return !output && previousOutput;
    }

    /** Reset the internal state. */
    public void reset() {
        lastInput = false;
        output = false;
        previousOutput = false;
    }
}
