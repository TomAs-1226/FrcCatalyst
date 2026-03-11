package frc.lib.catalyst.util;

import java.util.Arrays;

/**
 * Advanced signal processing utilities for sensor data conditioning.
 * Provides various filter types commonly needed in FRC for noise reduction
 * and signal smoothing without the lag of simple moving averages.
 *
 * <p>Filters available:
 * <ul>
 *   <li>{@link ExponentialMovingAverage} - Low-pass filter with configurable time constant</li>
 *   <li>{@link MedianFilter} - Removes impulse noise (spikes) while preserving edges</li>
 *   <li>{@link LowPassFilter} - First-order IIR low-pass filter with cutoff frequency</li>
 *   <li>{@link CompositeFilter} - Median + low-pass chain for robust filtering</li>
 *   <li>{@link RateOfChange} - Calculates derivative of a signal with smoothing</li>
 * </ul>
 */
public final class SignalProcessor {

    private SignalProcessor() {}

    /**
     * Exponential Moving Average (EMA) filter.
     * Responds faster to changes than a simple moving average while still
     * smoothing noise. The alpha parameter controls the balance:
     * higher alpha = faster response but less smoothing.
     *
     * <p>Formula: output = alpha * input + (1 - alpha) * previousOutput
     */
    public static class ExponentialMovingAverage {
        private final double alpha;
        private double value;
        private boolean initialized;

        /**
         * @param alpha smoothing factor (0, 1]. 0.1 = heavy smoothing, 0.9 = light smoothing.
         */
        public ExponentialMovingAverage(double alpha) {
            this.alpha = Math.max(0.001, Math.min(1.0, alpha));
            this.initialized = false;
        }

        /**
         * Create from time constant and sample period.
         * @param timeConstantSeconds how quickly the filter responds (smaller = faster)
         * @param samplePeriodSeconds how often calculate() is called (typically 0.02)
         */
        public static ExponentialMovingAverage fromTimeConstant(double timeConstantSeconds,
                                                                 double samplePeriodSeconds) {
            double alpha = 1.0 - Math.exp(-samplePeriodSeconds / timeConstantSeconds);
            return new ExponentialMovingAverage(alpha);
        }

        /** Process a new sample and return the filtered value. */
        public double calculate(double input) {
            if (!initialized) {
                value = input;
                initialized = true;
            } else {
                value = alpha * input + (1.0 - alpha) * value;
            }
            return value;
        }

        /** Get the current filtered value without adding a sample. */
        public double get() {
            return value;
        }

        /** Reset the filter. */
        public void reset() {
            initialized = false;
            value = 0;
        }

        /** Reset to a specific value. */
        public void reset(double initialValue) {
            value = initialValue;
            initialized = true;
        }
    }

    /**
     * Median filter — removes impulse noise (spikes) while preserving edges.
     * Unlike averaging filters, median filters do not blur sharp transitions.
     * Excellent for cleaning up noisy sensor data that occasionally produces outliers.
     */
    public static class MedianFilter {
        private final double[] buffer;
        private final double[] sorted;
        private int index;
        private boolean full;

        /**
         * @param windowSize number of samples in the filter window (odd numbers recommended)
         */
        public MedianFilter(int windowSize) {
            this.buffer = new double[windowSize];
            this.sorted = new double[windowSize];
            this.index = 0;
            this.full = false;
        }

        /** Process a new sample and return the median. */
        public double calculate(double input) {
            buffer[index] = input;
            index = (index + 1) % buffer.length;
            if (index == 0) full = true;

            int count = full ? buffer.length : index;
            System.arraycopy(buffer, 0, sorted, 0, count);
            Arrays.sort(sorted, 0, count);
            return sorted[count / 2];
        }

        /** Reset the filter. */
        public void reset() {
            index = 0;
            full = false;
        }
    }

    /**
     * First-order IIR low-pass filter with cutoff frequency specification.
     * More intuitive than raw alpha values — specify the cutoff frequency
     * and the filter automatically computes the correct coefficient.
     */
    public static class LowPassFilter {
        private final ExponentialMovingAverage ema;

        /**
         * @param cutoffFrequencyHz frequencies above this are attenuated
         * @param samplePeriodSeconds how often calculate() is called (typically 0.02)
         */
        public LowPassFilter(double cutoffFrequencyHz, double samplePeriodSeconds) {
            double rc = 1.0 / (2.0 * Math.PI * cutoffFrequencyHz);
            double alpha = samplePeriodSeconds / (samplePeriodSeconds + rc);
            this.ema = new ExponentialMovingAverage(alpha);
        }

        /** Process a new sample. */
        public double calculate(double input) {
            return ema.calculate(input);
        }

        /** Get current value. */
        public double get() {
            return ema.get();
        }

        /** Reset the filter. */
        public void reset() {
            ema.reset();
        }
    }

    /**
     * Rate of change calculator with built-in smoothing.
     * Computes the derivative of a signal, useful for detecting acceleration,
     * jerk, or velocity from position data.
     */
    public static class RateOfChange {
        private final ExponentialMovingAverage smoother;
        private double previousValue;
        private double previousTime;
        private boolean initialized;

        /**
         * @param smoothingAlpha EMA alpha for the derivative (0.1 = smooth, 0.9 = responsive)
         */
        public RateOfChange(double smoothingAlpha) {
            this.smoother = new ExponentialMovingAverage(smoothingAlpha);
            this.initialized = false;
        }

        /**
         * Calculate rate of change given current value and timestamp.
         * @param value current signal value
         * @param timestampSeconds current timestamp (use Timer.getFPGATimestamp())
         * @return smoothed rate of change (units per second)
         */
        public double calculate(double value, double timestampSeconds) {
            if (!initialized) {
                previousValue = value;
                previousTime = timestampSeconds;
                initialized = true;
                return 0;
            }

            double dt = timestampSeconds - previousTime;
            if (dt <= 0) return smoother.get();

            double rawRate = (value - previousValue) / dt;
            previousValue = value;
            previousTime = timestampSeconds;

            return smoother.calculate(rawRate);
        }

        /** Get the current smoothed rate. */
        public double get() {
            return smoother.get();
        }

        /** Reset the calculator. */
        public void reset() {
            initialized = false;
            smoother.reset();
        }
    }

    /**
     * Combined filter chain: median filter followed by low-pass filter.
     * Removes spikes first (median), then smooths the result (low-pass).
     * This is the recommended approach for noisy FRC sensors.
     */
    public static class CompositeFilter {
        private final MedianFilter median;
        private final LowPassFilter lowPass;

        /**
         * @param medianWindowSize median filter window (3-7 recommended)
         * @param cutoffFrequencyHz low-pass cutoff frequency
         * @param samplePeriodSeconds sample period (typically 0.02)
         */
        public CompositeFilter(int medianWindowSize, double cutoffFrequencyHz,
                                double samplePeriodSeconds) {
            this.median = new MedianFilter(medianWindowSize);
            this.lowPass = new LowPassFilter(cutoffFrequencyHz, samplePeriodSeconds);
        }

        /** Process a sample through both filters. */
        public double calculate(double input) {
            return lowPass.calculate(median.calculate(input));
        }

        /** Get current value. */
        public double get() {
            return lowPass.get();
        }

        /** Reset both filters. */
        public void reset() {
            median.reset();
            lowPass.reset();
        }
    }
}
