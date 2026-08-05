package frc.lib.catalyst.physics.estimation;

import java.util.Arrays;

/**
 * Fits a linear model to data as it arrives, one sample at a time, without storing any of it.
 *
 * <p>The workhorse behind Catalyst's online parameter identification. Given a model of the form
 * {@code y = θ₁x₁ + θ₂x₂ + … } — which is what a feedforward equation and a battery's voltage droop
 * both are — this recovers {@code θ} from a stream of {@code (x, y)} pairs in constant time and
 * constant memory. A batch least-squares fit over a log would give the same answer; this gives it
 * while the robot is still driving.
 *
 * <p>The update is the standard recursive least-squares step with a forgetting factor:
 *
 * <pre>
 * k = P·x / (λ + xᵀ·P·x)          gain
 * θ = θ + k·(y - xᵀ·θ)            correct by the prediction error
 * P = (P - k·xᵀ·P) / λ            shrink the covariance
 * </pre>
 *
 * <p>{@code λ} is how fast old data stops counting. At {@code 1.0} every sample counts forever, which
 * is right for a constant like battery resistance over one match. Below {@code 1.0} the fit tracks a
 * parameter that genuinely drifts, at the cost of never fully settling — {@code 0.999} has a memory of
 * roughly a thousand samples, or twenty seconds at 50&nbsp;Hz.
 *
 * <p>{@code P} starts large, which says "I have no idea yet" and lets the first samples move the
 * estimate a long way. It shrinks as evidence accumulates, and {@link #isConverged()} reports when it
 * has shrunk enough to be worth reading.
 *
 * <p>Pure arithmetic on {@code double[]} — no hardware, no allocation per update beyond the working
 * vectors, and fully deterministic, so the tests drive it with synthetic data and check it recovers
 * known coefficients.
 *
 * @since 1.6.0
 */
public final class RecursiveLeastSquares {

    private final int parameterCount;
    private final double forgettingFactor;
    private final double convergenceThreshold;
    private final double[] theta;
    private final double[][] covariance;
    private final double initialCovariance;

    private long sampleCount = 0;
    private double lastResidual = 0.0;

    /**
     * @param parameterCount       how many coefficients the model has
     * @param forgettingFactor     {@code λ} in {@code (0, 1]}; {@code 1.0} never forgets
     * @param initialCovariance    starting diagonal of {@code P}; large means "no prior belief".
     *                             {@code 1000} is a reasonable default for well-scaled regressors
     * @param convergenceThreshold mean per-parameter variance below which {@link #isConverged()} is
     *                             true — see {@link #meanParameterVariance()}
     */
    public RecursiveLeastSquares(int parameterCount, double forgettingFactor,
                                 double initialCovariance, double convergenceThreshold) {
        if (parameterCount < 1) {
            throw new IllegalArgumentException("parameterCount must be >= 1 (got " + parameterCount + ")");
        }
        if (!(forgettingFactor > 0) || forgettingFactor > 1) {
            throw new IllegalArgumentException("forgettingFactor must be in (0, 1] (got "
                    + forgettingFactor + ")");
        }
        if (!(initialCovariance > 0)) {
            throw new IllegalArgumentException("initialCovariance must be > 0 (got "
                    + initialCovariance + ")");
        }
        this.parameterCount = parameterCount;
        this.forgettingFactor = forgettingFactor;
        this.initialCovariance = initialCovariance;
        this.convergenceThreshold = convergenceThreshold;
        this.theta = new double[parameterCount];
        this.covariance = new double[parameterCount][parameterCount];
        resetCovariance();
    }

    /** A never-forgetting fit with sensible defaults, for a parameter that is genuinely constant. */
    public static RecursiveLeastSquares forConstant(int parameterCount) {
        return new RecursiveLeastSquares(parameterCount, 1.0, 1000.0, 0.01);
    }

    /** A fit that tracks slow drift, forgetting over roughly a thousand samples. */
    public static RecursiveLeastSquares forDrifting(int parameterCount) {
        return new RecursiveLeastSquares(parameterCount, 0.999, 1000.0, 0.05);
    }

    /**
     * Fold in one observation.
     *
     * <p>A sample whose denominator would be non-positive — which only happens if the covariance has
     * been driven numerically negative — is skipped rather than allowed to corrupt the fit.
     *
     * @param regressors the {@code x} vector; length must equal the parameter count
     * @param measured   the observed {@code y}
     * @return the prediction error before this update, in the units of {@code y}
     */
    public double add(double[] regressors, double measured) {
        if (regressors == null || regressors.length != parameterCount) {
            throw new IllegalArgumentException("expected " + parameterCount + " regressors, got "
                    + (regressors == null ? "null" : regressors.length));
        }

        double[] covarianceTimesX = new double[parameterCount];
        for (int i = 0; i < parameterCount; i++) {
            double sum = 0.0;
            for (int j = 0; j < parameterCount; j++) sum += covariance[i][j] * regressors[j];
            covarianceTimesX[i] = sum;
        }

        double denominator = forgettingFactor;
        for (int i = 0; i < parameterCount; i++) denominator += regressors[i] * covarianceTimesX[i];
        if (!(denominator > 1e-12)) return lastResidual;

        double predicted = predict(regressors);
        double residual = measured - predicted;
        lastResidual = residual;

        for (int i = 0; i < parameterCount; i++) {
            theta[i] += covarianceTimesX[i] / denominator * residual;
        }

        // P = (P - (P·x)(P·x)ᵀ / denominator) / λ, symmetric by construction.
        for (int i = 0; i < parameterCount; i++) {
            for (int j = 0; j < parameterCount; j++) {
                covariance[i][j] = (covariance[i][j]
                        - covarianceTimesX[i] * covarianceTimesX[j] / denominator) / forgettingFactor;
            }
        }
        sampleCount++;
        return residual;
    }

    /** What the current fit predicts for a given regressor vector. */
    public double predict(double[] regressors) {
        double sum = 0.0;
        for (int i = 0; i < parameterCount; i++) sum += theta[i] * regressors[i];
        return sum;
    }

    /** A copy of the current coefficient estimates. */
    public double[] parameters() {
        return theta.clone();
    }

    /** One coefficient by index. */
    public double parameter(int index) {
        return theta[index];
    }

    /**
     * Rough 1-sigma uncertainty of one coefficient, from the diagonal of {@code P}. Comparable
     * between coefficients of the same fit; not a calibrated confidence interval, because that would
     * need the measurement noise variance, which nobody measures on a robot.
     */
    public double uncertainty(int index) {
        return Math.sqrt(Math.max(0.0, covariance[index][index]));
    }

    /** Sum of the covariance diagonal — how much total uncertainty is left in the fit. */
    public double covarianceTrace() {
        double trace = 0.0;
        for (int i = 0; i < parameterCount; i++) trace += covariance[i][i];
        return trace;
    }

    /**
     * Mean remaining variance per coefficient — the trace divided by the parameter count.
     *
     * <p>This, rather than the raw trace, is what convergence is judged on. The trace grows with the
     * number of coefficients simply because there are more of them to be uncertain about, so a fixed
     * trace threshold quietly demands three times as much evidence from a three-parameter fit as from
     * a one-parameter fit. Dividing through makes the threshold mean the same thing everywhere.
     */
    public double meanParameterVariance() {
        return covarianceTrace() / parameterCount;
    }

    /**
     * Whether the fit has seen enough varied data to be worth reading. Requires both a minimum sample
     * count and a mean parameter variance below the configured threshold — a robot that sat still for
     * a thousand loops has plenty of samples and no information, and this rejects that case, because
     * an unexcited direction keeps its initial variance no matter how many samples arrive.
     */
    public boolean isConverged() {
        return sampleCount >= 50 && meanParameterVariance() < convergenceThreshold;
    }

    /** The prediction error on the most recent sample. */
    public double lastResidual() {
        return lastResidual;
    }

    /** How many samples have been folded in. */
    public long sampleCount() {
        return sampleCount;
    }

    /** How many coefficients this fit has. */
    public int parameterCount() {
        return parameterCount;
    }

    /** Discard everything and start over with no prior belief. */
    public void reset() {
        Arrays.fill(theta, 0.0);
        resetCovariance();
        sampleCount = 0;
        lastResidual = 0.0;
    }

    /**
     * Seed the fit with a starting guess — the values already in your constants file. The covariance
     * still starts wide, so evidence overrides the guess quickly; this only saves the first few
     * samples from swinging wildly.
     */
    public void seed(double... initial) {
        if (initial.length != parameterCount) {
            throw new IllegalArgumentException("expected " + parameterCount + " initial values, got "
                    + initial.length);
        }
        System.arraycopy(initial, 0, theta, 0, parameterCount);
    }

    private void resetCovariance() {
        for (double[] row : covariance) Arrays.fill(row, 0.0);
        for (int i = 0; i < parameterCount; i++) covariance[i][i] = initialCovariance;
    }
}
