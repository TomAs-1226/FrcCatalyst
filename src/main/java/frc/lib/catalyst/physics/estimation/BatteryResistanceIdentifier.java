package frc.lib.catalyst.physics.estimation;

import java.util.Locale;
import java.util.Optional;

/**
 * Measures this battery's internal resistance from how the bus voltage sags under load.
 *
 * <p>Every brownout prediction rests on one number: {@code R}, the battery's internal resistance.
 * Teams pick it once — 0.02&nbsp;Ω is the usual guess — and then use the same figure for a fresh
 * battery, a warm one, and the four-year-old one at the bottom of the cart. Those genuinely differ,
 * often by a factor of two, and the prediction is only ever as good as the number.
 *
 * <p>The battery is a voltage source behind a resistance:
 *
 * <pre>
 * V_bus = V_open − I·R
 * </pre>
 *
 * <p>which is linear in {@code (1, −I)}, so {@link RecursiveLeastSquares} recovers both the
 * open-circuit voltage and the resistance from voltage and current samples the robot is already
 * logging. No characterisation routine, no special test — just drive it.
 *
 * <pre>{@code
 * BatteryResistanceIdentifier battery = new BatteryResistanceIdentifier();
 *
 * // in periodic:
 * battery.addSample(RobotController.getBatteryVoltage(), pdh.getTotalCurrent());
 *
 * // feed the measured value into brownout prediction, once it has settled:
 * battery.recommendation().ifPresent(r -> System.out.println(r.describe()));
 * }</pre>
 *
 * <h2>When to believe it</h2>
 * Resistance is only identifiable if the current <em>varies</em>. A robot sitting at a steady 20&nbsp;A
 * gives one equation for two unknowns forever, and no number of samples fixes that. The covariance
 * catches this, so {@link #recommendation()} stays empty until the load has actually moved around —
 * which, on a robot that drives and runs mechanisms, happens within seconds.
 *
 * <p>Like the rest of Catalyst's parameter identification, this only ever reports. Nothing reads the
 * measured resistance back into {@code BrownoutMonitor} on its own; you look at it and decide.
 *
 * <p>No hardware and no HAL — both inputs are arguments.
 *
 * @since 1.6.0
 */
public final class BatteryResistanceIdentifier {

    private final RecursiveLeastSquares fit;
    private final double minimumCurrentSpread;

    private double minCurrentSeen = Double.POSITIVE_INFINITY;
    private double maxCurrentSeen = Double.NEGATIVE_INFINITY;

    /** An identifier with defaults suited to a match: slow forgetting, 5&nbsp;A of required spread. */
    public BatteryResistanceIdentifier() {
        this(0.9995, 5.0);
    }

    /**
     * @param forgettingFactor     how fast old samples decay, in {@code (0, 1]}. Slightly below 1 lets
     *                             the fit follow a battery warming up over a match
     * @param minimumCurrentSpread how many amps the load must have varied by before the fit is
     *                             trusted — without variation, resistance is unidentifiable
     */
    public BatteryResistanceIdentifier(double forgettingFactor, double minimumCurrentSpread) {
        this.fit = new RecursiveLeastSquares(2, forgettingFactor, 100.0, 0.05);
        this.minimumCurrentSpread = minimumCurrentSpread;
        // Seed with a typical FRC battery so early numbers are plausible rather than zero.
        fit.seed(12.6, 0.020);
    }

    /**
     * Fold in one loop's measurement.
     *
     * @param busVoltage       measured bus voltage, from {@code RobotController.getBatteryVoltage()}
     * @param totalCurrentAmps total robot current, from the PDH/PDP
     */
    public void addSample(double busVoltage, double totalCurrentAmps) {
        if (!Double.isFinite(busVoltage) || !Double.isFinite(totalCurrentAmps)) return;
        minCurrentSeen = Math.min(minCurrentSeen, totalCurrentAmps);
        maxCurrentSeen = Math.max(maxCurrentSeen, totalCurrentAmps);
        // V = Voc·1 + R·(−I)
        fit.add(new double[]{1.0, -totalCurrentAmps}, busVoltage);
    }

    /** Fitted open-circuit voltage, in volts — what the battery reads with nothing drawing. */
    public double openCircuitVolts() {
        return fit.parameter(0);
    }

    /** Fitted internal resistance, in ohms. */
    public double resistanceOhms() {
        return fit.parameter(1);
    }

    /** How far the current has varied across all samples, in amps. */
    public double currentSpreadAmps() {
        return maxCurrentSeen > minCurrentSeen ? maxCurrentSeen - minCurrentSeen : 0.0;
    }

    /** How many samples have been folded in. */
    public long sampleCount() {
        return fit.sampleCount();
    }

    /**
     * Whether the fit has settled <em>and</em> the load has varied enough for the answer to mean
     * anything.
     */
    public boolean isConverged() {
        return fit.isConverged() && currentSpreadAmps() >= minimumCurrentSpread;
    }

    /**
     * Predicted bus voltage at a given total draw, using the fitted parameters. Handy for sanity
     * checking the fit against what you actually see.
     */
    public double predictedVoltageAt(double totalCurrentAmps) {
        return openCircuitVolts() - resistanceOhms() * totalCurrentAmps;
    }

    /** The measured battery parameters — empty until the fit has converged on varied data. */
    public Optional<Recommendation> recommendation() {
        if (!isConverged()) return Optional.empty();
        return Optional.of(new Recommendation(openCircuitVolts(), resistanceOhms(),
                fit.uncertainty(1), fit.sampleCount(), currentSpreadAmps()));
    }

    /** Throw the fit away — after a battery swap, for instance. */
    public void reset() {
        fit.reset();
        fit.seed(12.6, 0.020);
        minCurrentSeen = Double.POSITIVE_INFINITY;
        maxCurrentSeen = Double.NEGATIVE_INFINITY;
    }

    /** The underlying fit, for callers that want the raw covariance. */
    public RecursiveLeastSquares fit() {
        return fit;
    }

    /**
     * What this battery actually measures. Reported only; nothing applies it automatically.
     *
     * @param openCircuitVolts    fitted no-load voltage
     * @param resistanceOhms      fitted internal resistance
     * @param resistanceUncertainty rough 1-sigma spread on the resistance
     * @param sampleCount         how many samples the fit is based on
     * @param currentSpreadAmps   how much the load varied, which is what made the fit possible
     */
    public record Recommendation(
            double openCircuitVolts,
            double resistanceOhms,
            double resistanceUncertainty,
            long sampleCount,
            double currentSpreadAmps) {

        /** A line for the console or a log. */
        public String describe() {
            return String.format(Locale.ROOT,
                    "battery measured after %d samples over %.0f A of load variation: "
                            + "R=%.4f ohm (+/-%.4f), open-circuit %.2f V",
                    sampleCount, currentSpreadAmps, resistanceOhms, resistanceUncertainty, openCircuitVolts);
        }

        /**
         * Whether this battery is meaningfully worse than the assumption in use. A resistance well
         * above what {@code BrownoutMonitor} was configured with means brownouts arrive earlier than
         * predicted — usually a tired battery rather than a modelling error.
         *
         * @param assumedOhms the value currently configured
         */
        public boolean isWorseThan(double assumedOhms) {
            return resistanceOhms > assumedOhms * 1.25;
        }
    }
}
