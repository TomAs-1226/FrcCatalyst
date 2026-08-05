package frc.lib.catalyst.physics.diagnostics;

import java.util.Locale;

import frc.lib.catalyst.util.SignalProcessor;

/**
 * Watches the gap between what a model predicted and what actually happened, and tells noise from a
 * real problem.
 *
 * <p>Threshold alarms on a raw error are the usual approach and they are bad at this. Set the
 * threshold tight and every bump trips it; set it loose and a genuine 10% calibration error never
 * does. The distinction that matters is not size, it is <em>shape</em>:
 *
 * <ul>
 *   <li>A residual that swings both ways and averages to zero is <b>noise</b>. The model is right and
 *       the sensor is imperfect. Nothing to do.</li>
 *   <li>A residual that sits consistently to one side is <b>bias</b>. The model is wrong — a wheel
 *       radius that is off, a gear ratio typo, an encoder scaled by the wrong constant, a mechanism
 *       fighting friction it did not used to.</li>
 * </ul>
 *
 * <p>So this tracks both: the long-run mean and spread (Welford, exact and single-pass) and a recent
 * mean that can move. A bias is called when the recent mean is both large enough to matter and large
 * relative to how much the signal naturally scatters — which is a standard-error test, not a guess.
 *
 * <pre>{@code
 * ModelResidualMonitor wheelSpeed = new ModelResidualMonitor("FL wheel speed", 0.05);   // m/s
 *
 * wheelSpeed.record(measuredSpeed - predictedSpeed);
 *
 * if (wheelSpeed.isBiased()) {
 *     alerts.warning("Drive", wheelSpeed.describe());   // e.g. "FL wheel speed: +0.14 m/s bias"
 * }
 * }</pre>
 *
 * <p>Pure arithmetic — no hardware, no HAL, no allocation per sample.
 *
 * @since 1.6.0
 */
public final class ModelResidualMonitor {

    private final String name;
    private final double tolerance;
    private final double significance;
    private final int minimumSamples;
    private final SignalProcessor.ExponentialMovingAverage recentMean;

    private long count = 0;
    private double mean = 0.0;
    /** Sum of squared deviations from the running mean — Welford's M2. */
    private double sumSquaredDeviations = 0.0;
    private double lastValue = 0.0;

    /**
     * @param name      what this residual describes, used in {@link #describe()}
     * @param tolerance how large a persistent offset has to be before it is worth reporting, in the
     *                  residual's own units. Below this, a bias is real but too small to act on
     */
    public ModelResidualMonitor(String name, double tolerance) {
        this(name, tolerance, 3.0, 30, 0.05);
    }

    /**
     * @param name           what this residual describes
     * @param tolerance      the smallest offset worth reporting, in the residual's units
     * @param significance   how many standard errors the recent mean must exceed. 3.0 is the usual
     *                       "unlikely to be chance" bar
     * @param minimumSamples samples required before any bias call is made
     * @param smoothing      EMA factor for the recent mean, in {@code (0, 1]}
     */
    public ModelResidualMonitor(String name, double tolerance, double significance,
                                int minimumSamples, double smoothing) {
        if (!(tolerance > 0)) {
            throw new IllegalArgumentException("tolerance must be > 0 for '" + name + "' (got "
                    + tolerance + ")");
        }
        if (minimumSamples < 2) {
            throw new IllegalArgumentException("minimumSamples must be >= 2 (got " + minimumSamples + ")");
        }
        this.name = name;
        this.tolerance = tolerance;
        this.significance = significance;
        this.minimumSamples = minimumSamples;
        this.recentMean = new SignalProcessor.ExponentialMovingAverage(smoothing);
    }

    /**
     * Fold in one residual — {@code measured - predicted}, with the sign kept, because the direction
     * of a bias is most of the diagnostic value.
     *
     * @return the recent mean after this sample
     */
    public double record(double residual) {
        if (!Double.isFinite(residual)) return recentMean.get();
        lastValue = residual;
        count++;
        double delta = residual - mean;
        mean += delta / count;
        sumSquaredDeviations += delta * (residual - mean);
        return recentMean.calculate(residual);
    }

    /** The residual's long-run mean. Near zero means the model is unbiased. */
    public double mean() {
        return mean;
    }

    /** The recent mean, which moves when the model starts being wrong. */
    public double recentMean() {
        return recentMean.get();
    }

    /** Spread of the residual about its mean — how noisy this signal naturally is. */
    public double standardDeviation() {
        return count < 2 ? 0.0 : Math.sqrt(sumSquaredDeviations / (count - 1));
    }

    /**
     * Uncertainty of the mean itself: {@code σ/√n}. This is what a bias has to beat — a noisy signal
     * needs a larger offset before you can say the offset is real.
     */
    public double standardError() {
        return count < 2 ? Double.POSITIVE_INFINITY : standardDeviation() / Math.sqrt(count);
    }

    /** The most recent residual. */
    public double lastValue() {
        return lastValue;
    }

    /** How many residuals have been recorded. */
    public long sampleCount() {
        return count;
    }

    /** What this residual describes. */
    public String name() {
        return name;
    }

    /**
     * True when the residual is consistently to one side by more than the tolerance, and by more than
     * chance would explain. Needs enough samples first, so it never fires on a handful of readings.
     */
    public boolean isBiased() {
        if (count < minimumSamples) return false;
        double bias = Math.abs(recentMean.get());
        return bias > tolerance && bias > significance * standardError();
    }

    /**
     * The size of the bias relative to the tolerance: {@code 0} clean, {@code 1} right at the limit,
     * above {@code 1} past it. The number to rank several residuals against each other, which is how
     * {@link FaultIsolator} uses it.
     */
    public double normalizedBias() {
        return Math.abs(recentMean.get()) / tolerance;
    }

    /** Direction of the bias: {@code +1}, {@code -1}, or {@code 0} when unbiased. */
    public double biasDirection() {
        if (!isBiased()) return 0.0;
        return Math.signum(recentMean.get());
    }

    /** Clear all history. */
    public void reset() {
        count = 0;
        mean = 0.0;
        sumSquaredDeviations = 0.0;
        lastValue = 0.0;
        recentMean.reset();
    }

    /** One line, naming the bias and its direction, or saying the residual looks like noise. */
    public String describe() {
        if (count < minimumSamples) {
            return String.format(Locale.ROOT, "%s: only %d samples, no verdict yet", name, count);
        }
        if (!isBiased()) {
            return String.format(Locale.ROOT, "%s: unbiased (mean %+.4f, noise %.4f)",
                    name, recentMean.get(), standardDeviation());
        }
        return String.format(Locale.ROOT, "%s: %+.4f bias (%.1fx tolerance, noise %.4f)",
                name, recentMean.get(), normalizedBias(), standardDeviation());
    }
}
