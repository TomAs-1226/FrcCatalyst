package frc.lib.catalyst.physics.model;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Turns "how accurately do I need to measure this?" into a number, by propagating measurement error
 * through to the limits Physics Core actually uses.
 *
 * <p>Every input to {@link RobotModel} is a measurement, and every measurement is wrong by some
 * amount. What matters is not how wrong, but <em>how much it matters</em> — and the answer is very
 * different for each one, in ways that are not obvious:
 *
 * <ul>
 *   <li><b>Mass does not affect the traction limit at all.</b> A heavier robot needs more force to
 *       accelerate and gets proportionally more grip to do it with, and the two cancel exactly. Weigh
 *       the robot to the nearest kilogram and move on.</li>
 *   <li><b>Centre-of-mass height is the one to get right.</b> The tipping limit is inversely
 *       proportional to it, so a 20% error is a 20% error in the limit — and it is also the hardest
 *       thing on this list to measure, which is an unfortunate combination.</li>
 *   <li><b>Coefficient of friction maps one-for-one onto the traction limit</b>, and one-for-one
 *       (inverted) onto stopping distance.</li>
 * </ul>
 *
 * <p>Give it your uncertainties and it reports the resulting range:
 *
 * <pre>{@code
 * ModelUncertainty uncertainty = ModelUncertainty.builder(model)
 *     .massKg(2.0)                       // weighed on a bathroom scale
 *     .centerOfMassHeightMeters(0.04)    // estimated by eye, so be honest about it
 *     .coefficientOfFriction(0.15)       // measured with a tilt test
 *     .footprintMeters(0.005)            // tape measure
 *     .build();
 *
 * System.out.println(uncertainty.describe());
 * // traction limit   9.81 m/s^2  +/- 1.47  (15.0%)   <- driven by friction
 * // tipping limit   15.60 m/s^2  +/- 2.86  (18.3%)   <- driven by CoM height
 * // ...
 * }</pre>
 *
 * <h2>How the propagation works</h2>
 * First-order, in quadrature: each output's sensitivity to each input is the analytic partial
 * derivative, and independent errors add as {@code sqrt(sum of squares)} rather than by simple
 * addition. That is the standard treatment and it is right when the errors are independent, which
 * they broadly are here — a tape measure being off does not make a bathroom scale wrong.
 *
 * <p>It is deliberately <b>not</b> a statistical guarantee. It is a sensitivity analysis: it tells you
 * which measurement to spend an afternoon improving, and which one to stop worrying about. If you
 * treat the resulting range as a hard bound you will be disappointed; if you treat it as "this is the
 * number I should double-check", it will point you at the right one every time.
 *
 * <p>Pure arithmetic, no state, no hardware.
 *
 * @since 1.7.0
 */
public final class ModelUncertainty {

    private final RobotModel model;
    private final double massSigma;
    private final double comHeightSigma;
    private final double frictionSigma;
    private final double footprintSigma;

    private ModelUncertainty(Builder builder) {
        this.model = builder.model;
        this.massSigma = builder.massSigma;
        this.comHeightSigma = builder.comHeightSigma;
        this.frictionSigma = builder.frictionSigma;
        this.footprintSigma = builder.footprintSigma;
    }

    /** The model these uncertainties describe. */
    public RobotModel model() {
        return model;
    }

    /**
     * Uncertainty in the traction limit, in m/s^2.
     *
     * <p>{@code a = mu*g}, so this depends on the friction measurement and <b>nothing else</b>. Mass,
     * footprint, and centre of mass do not enter it. Teams routinely re-weigh the robot hoping to
     * sharpen this figure; it cannot help.
     */
    public double tractionLimitSigma() {
        return frictionSigma * RobotModel.GRAVITY;
    }

    /**
     * Uncertainty in the tipping limit, in m/s^2.
     *
     * <p>{@code a = g*w/h}, so relative errors in the half-width and the centre-of-mass height combine
     * in quadrature. In practice {@code h} dominates: a footprint is measurable to a few millimetres,
     * a centre-of-mass height usually is not.
     */
    public double tippingLimitSigma() {
        DrivetrainModel drivetrain = new DrivetrainModel(model);
        double limit = drivetrain.maxTippingAccelerationMpsSq();
        double relativeWidth = footprintSigma / 2.0 / model.tippingHalfWidthMeters();
        double relativeHeight = comHeightSigma / model.centerOfMassHeightMeters();
        return limit * Math.hypot(relativeWidth, relativeHeight);
    }

    /** Uncertainty in whichever limit actually binds — the one a constraint should be built from. */
    public double safeAccelerationSigma() {
        DrivetrainModel drivetrain = new DrivetrainModel(model);
        return drivetrain.isTippingLimited() ? tippingLimitSigma() : tractionLimitSigma();
    }

    /**
     * Uncertainty in the stopping distance from a given speed, in metres.
     *
     * <p>{@code d = v^2 / 2a}, so the relative error in the distance equals the relative error in the
     * acceleration limit. Assumes the speed itself is known — if the velocity estimate is also
     * uncertain, that contributes separately and twice as hard, since it appears squared.
     */
    public double stoppingDistanceSigma(double speedMetersPerSecond) {
        DrivetrainModel drivetrain = new DrivetrainModel(model);
        double limit = drivetrain.maxSafeAccelerationMpsSq();
        if (limit <= 0) return Double.POSITIVE_INFINITY;
        double distance = drivetrain.stoppingDistanceMeters(speedMetersPerSecond);
        return distance * safeAccelerationSigma() / limit;
    }

    /**
     * Uncertainty in the maximum traction force, in newtons.
     *
     * <p>{@code F = mu*m*g}. This is the one figure where mass genuinely matters, because it does not
     * cancel — which is why the force is uncertain even though the acceleration is not.
     */
    public double tractionForceSigma() {
        double relativeFriction = frictionSigma / model.coefficientOfFriction();
        double relativeMass = massSigma / model.massKg();
        return new DrivetrainModel(model).maxTractionForceNewtons()
                * Math.hypot(relativeFriction, relativeMass);
    }

    /**
     * Which measurement is limiting the accuracy of the binding acceleration limit, and what fraction
     * of the total variance it accounts for. The answer to "what should I go and measure better?".
     */
    public DominantSource dominantSource() {
        DrivetrainModel drivetrain = new DrivetrainModel(model);
        if (!drivetrain.isTippingLimited()) {
            return new DominantSource("coefficient of friction", 1.0,
                    "the traction limit is mu*g and depends on nothing else");
        }
        double limit = drivetrain.maxTippingAccelerationMpsSq();
        double fromWidth = limit * footprintSigma / 2.0 / model.tippingHalfWidthMeters();
        double fromHeight = limit * comHeightSigma / model.centerOfMassHeightMeters();
        double total = fromWidth * fromWidth + fromHeight * fromHeight;
        if (total <= 0) {
            return new DominantSource("nothing", 0.0, "every input was given as exact");
        }
        return fromHeight >= fromWidth
                ? new DominantSource("centre-of-mass height", fromHeight * fromHeight / total,
                        "the tipping limit is inversely proportional to it")
                : new DominantSource("footprint", fromWidth * fromWidth / total,
                        "the tipping limit is proportional to the half-width");
    }

    /** A multi-line report: every derived limit, its uncertainty, and what is driving it. */
    public String describe() {
        DrivetrainModel drivetrain = new DrivetrainModel(model);
        List<String> lines = new ArrayList<>();
        lines.add(model.describe());
        lines.add(line("traction limit", drivetrain.maxTractionAccelerationMpsSq(),
                tractionLimitSigma(), "m/s^2"));
        lines.add(line("tipping limit", drivetrain.maxTippingAccelerationMpsSq(),
                tippingLimitSigma(), "m/s^2"));
        lines.add(line("binding limit", drivetrain.maxSafeAccelerationMpsSq(),
                safeAccelerationSigma(), "m/s^2"));
        lines.add(line("traction force", drivetrain.maxTractionForceNewtons(),
                tractionForceSigma(), "N"));
        lines.add(line("stopping distance from 4 m/s", drivetrain.stoppingDistanceMeters(4.0),
                stoppingDistanceSigma(4.0), "m"));
        DominantSource dominant = dominantSource();
        lines.add(String.format(Locale.ROOT, "  limited by: %s (%.0f%% of the variance) - %s",
                dominant.measurement(), dominant.varianceShare() * 100, dominant.why()));
        return String.join("\n", lines);
    }

    private static String line(String name, double value, double sigma, String units) {
        double percent = value == 0 ? 0 : Math.abs(sigma / value) * 100;
        return String.format(Locale.ROOT, "  %-28s %8.2f %-6s +/- %6.2f  (%4.1f%%)",
                name, value, units, sigma, percent);
    }

    /**
     * The measurement whose error dominates a derived limit.
     *
     * @param measurement   what to go and measure better
     * @param varianceShare how much of the total variance it accounts for, 0 to 1
     * @param why           the reason, in one clause
     */
    public record DominantSource(String measurement, double varianceShare, String why) {}

    /**
     * Start describing the uncertainty in a model. Every uncertainty defaults to a typical
     * pit-measurement value, so a team that supplies none still gets a realistic picture rather than a
     * misleading zero.
     */
    public static Builder builder(RobotModel model) {
        return new Builder(model);
    }

    /** Builder for {@link ModelUncertainty}. */
    public static final class Builder {
        private final RobotModel model;
        private double massSigma = 1.0;
        private double comHeightSigma = 0.04;
        private double frictionSigma = 0.15;
        private double footprintSigma = 0.005;

        Builder(RobotModel model) {
            if (model == null) throw new IllegalArgumentException("model must not be null");
            this.model = model;
        }

        /**
         * 1-sigma uncertainty in the robot's mass, in kilograms. Defaults to 1.0 — a bathroom scale.
         * Barely matters: it affects the traction <em>force</em> and nothing else.
         */
        public Builder massKg(double massSigma) {
            this.massSigma = massSigma;
            return this;
        }

        /**
         * 1-sigma uncertainty in the centre-of-mass height, in metres. Defaults to 0.04, which is
         * about what an educated guess is worth. <b>This is usually the one that matters</b> — the
         * tipping limit is inversely proportional to it. A tilt test gets it to about 0.01.
         */
        public Builder centerOfMassHeightMeters(double comHeightSigma) {
            this.comHeightSigma = comHeightSigma;
            return this;
        }

        /**
         * 1-sigma uncertainty in the coefficient of friction. Defaults to 0.15, which is roughly the
         * spread between "new tread on clean carpet" and "worn tread on a dusty field" — a wide band,
         * and an honest one if you have not measured it. A tilt test gets it to about 0.05.
         */
        public Builder coefficientOfFriction(double frictionSigma) {
            this.frictionSigma = frictionSigma;
            return this;
        }

        /**
         * 1-sigma uncertainty in the footprint dimensions, in metres. Defaults to 0.005. Almost never
         * the limiting factor, because a tape measure is better than every other tool on this list.
         */
        public Builder footprintMeters(double footprintSigma) {
            this.footprintSigma = footprintSigma;
            return this;
        }

        /** Validate and build. */
        public ModelUncertainty build() {
            if (massSigma < 0 || comHeightSigma < 0 || frictionSigma < 0 || footprintSigma < 0) {
                throw new IllegalStateException("uncertainties must be >= 0 - they are magnitudes, "
                        + "not offsets");
            }
            return new ModelUncertainty(this);
        }
    }
}
