package frc.lib.catalyst.util;

/**
 * A simple moving average filter for smoothing noisy sensor data.
 *
 * <p>Example usage:
 * <pre>{@code
 * MovingAverage distanceFilter = new MovingAverage(5); // 5-sample window
 *
 * // In periodic:
 * double smoothedDistance = distanceFilter.calculate(rawSensorReading);
 * }</pre>
 */
public class MovingAverage {

    private final double[] buffer;
    private int index = 0;
    private int count = 0;
    private double sum = 0;

    /**
     * Create a moving average filter.
     * @param windowSize number of samples to average over
     */
    public MovingAverage(int windowSize) {
        if (windowSize < 1) throw new IllegalArgumentException("Window size must be >= 1");
        this.buffer = new double[windowSize];
    }

    /**
     * Add a new sample and return the current average.
     * @param value new sample value
     * @return the moving average
     */
    public double calculate(double value) {
        if (count >= buffer.length) {
            sum -= buffer[index];
        } else {
            count++;
        }

        buffer[index] = value;
        sum += value;
        index = (index + 1) % buffer.length;

        return sum / count;
    }

    /** Get the current average without adding a new sample. */
    public double get() {
        if (count == 0) return 0;
        return sum / count;
    }

    /** Reset the filter, clearing all samples. */
    public void reset() {
        index = 0;
        count = 0;
        sum = 0;
    }

    /** Check if the filter has been fully filled at least once. */
    public boolean isFull() {
        return count >= buffer.length;
    }

    /** Get the number of samples currently in the buffer. */
    public int getCount() {
        return count;
    }
}
