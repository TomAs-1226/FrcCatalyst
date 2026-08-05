package frc.lib.catalyst.physics.estimation;

import java.util.Locale;
import java.util.function.DoubleSupplier;

/**
 * Recovers a mechanism's feedforward gains from how it actually behaves, while it is running.
 *
 * <p>{@code kS}, {@code kV}, and {@code kA} come from a SysId sweep in the pit, and then the robot
 * gets driven for six weeks: the tread wears, a bearing tightens, the gearbox beds in, someone swaps a
 * motor. The gains in the constants file describe the robot as it was that afternoon. This watches the
 * voltage-versus-motion relationship in normal operation and tells you what the gains look like now.
 *
 * <p>The model is the standard one, fitted by {@link RecursiveLeastSquares}:
 *
 * <pre>
 * V = kS·sign(v) + kV·v + kA·a   [+ kG·(1 or cos θ)]
 * </pre>
 *
 * <h2>It never changes anything</h2>
 * This class has no way to write a gain. It produces a {@link Recommendation} you read, review, and
 * type into your constants file if you agree with it. That is deliberate and it is the RFC's rule:
 * a value learned mid-match, applied mid-match, with nobody watching, is how a robot develops a
 * behaviour nobody can reproduce in the pit.
 *
 * <pre>{@code
 * FeedforwardIdentifier elevatorFf = FeedforwardIdentifier.builder("Elevator")
 *     .withElevatorGravity()
 *     .seed(0.15, 2.4, 0.06, 0.35)    // your current kS, kV, kA, kG — just a starting point
 *     .build();
 *
 * // in periodic, while enabled:
 * elevatorFf.addSample(motor.getAppliedVoltage(), mech.getVelocity(), mech.getAcceleration());
 *
 * // in the pit, when you want to know:
 * elevatorFf.recommendation().ifPresent(r -> System.out.println(r.describe()));
 * }</pre>
 *
 * <h2>When to believe it</h2>
 * A fit needs <em>varied</em> data. A mechanism that only ever runs at one speed cannot separate
 * {@code kS} from {@code kV} — both explain the same observation — and no amount of sitting there
 * collecting samples fixes that. The covariance tracks exactly this, so {@link #recommendation()}
 * stays empty until the fit has genuinely pinned the gains down rather than merely seen a lot of data.
 *
 * <p>Samples below the velocity deadband are dropped: near zero, {@code sign(v)} is noise, and feeding
 * it in poisons {@code kS}.
 *
 * <p>No hardware and no HAL — every input is an argument.
 *
 * @since 1.6.0
 */
public final class FeedforwardIdentifier {

    /** Which gravity term, if any, the mechanism needs. */
    public enum GravityType {
        /** No gravity term — a flywheel, a roller, a drivetrain. */
        NONE,
        /** A constant term — an elevator, where gravity pulls the same at every height. */
        ELEVATOR,
        /** A cosine term — an arm, where gravity's torque falls off as it approaches vertical. */
        ARM
    }

    private final String name;
    private final GravityType gravityType;
    private final double velocityDeadband;
    private final DoubleSupplier armAngleRadians;
    private final RecursiveLeastSquares fit;

    private long rejectedSamples = 0;

    private FeedforwardIdentifier(Builder builder) {
        this.name = builder.name;
        this.gravityType = builder.gravityType;
        this.velocityDeadband = builder.velocityDeadband;
        this.armAngleRadians = builder.armAngleRadians;
        int parameters = gravityType == GravityType.NONE ? 3 : 4;
        this.fit = new RecursiveLeastSquares(parameters, builder.forgettingFactor, 100.0, builder.convergenceThreshold);
        if (builder.seed != null) fit.seed(builder.seed);
    }

    /**
     * Fold in one loop's measurement.
     *
     * @param appliedVolts    voltage the motor actually applied, from the controller
     * @param velocity        mechanism velocity, in whatever units the gains are expressed in
     * @param acceleration    mechanism acceleration, in the matching units
     * @return true if the sample was used; false if it was below the velocity deadband
     */
    public boolean addSample(double appliedVolts, double velocity, double acceleration) {
        if (Math.abs(velocity) < velocityDeadband) {
            rejectedSamples++;
            return false;
        }
        fit.add(regressors(velocity, acceleration), appliedVolts);
        return true;
    }

    /** Static friction voltage, {@code kS}. */
    public double kS() {
        return fit.parameter(0);
    }

    /** Velocity gain, {@code kV}, in volts per unit of velocity. */
    public double kV() {
        return fit.parameter(1);
    }

    /** Acceleration gain, {@code kA}, in volts per unit of acceleration. */
    public double kA() {
        return fit.parameter(2);
    }

    /** Gravity gain, {@code kG}. Zero when this mechanism was configured without a gravity term. */
    public double kG() {
        return gravityType == GravityType.NONE ? 0.0 : fit.parameter(3);
    }

    /** The mechanism this identifier is watching. */
    public String name() {
        return name;
    }

    /** How many samples have been used. */
    public long sampleCount() {
        return fit.sampleCount();
    }

    /** How many samples were dropped for being below the velocity deadband. */
    public long rejectedSampleCount() {
        return rejectedSamples;
    }

    /** Voltage prediction error on the most recent accepted sample, in volts. */
    public double lastResidualVolts() {
        return fit.lastResidual();
    }

    /** Whether the fit has enough varied data to be worth reading. */
    public boolean isConverged() {
        return fit.isConverged();
    }

    /**
     * The gains this mechanism currently looks like it has — <b>empty until the fit has converged</b>,
     * so there is no way to read a half-formed estimate by accident.
     */
    public java.util.Optional<Recommendation> recommendation() {
        if (!isConverged()) return java.util.Optional.empty();
        return java.util.Optional.of(new Recommendation(name, kS(), kV(), kA(), kG(), gravityType,
                fit.sampleCount(), fit.uncertainty(0), fit.uncertainty(1), fit.uncertainty(2)));
    }

    /** Throw the fit away and start again — after changing hardware, for instance. */
    public void reset() {
        fit.reset();
        rejectedSamples = 0;
    }

    /** The underlying fit, for callers that want the raw covariance. */
    public RecursiveLeastSquares fit() {
        return fit;
    }

    private double[] regressors(double velocity, double acceleration) {
        double signV = Math.signum(velocity);
        return switch (gravityType) {
            case NONE -> new double[]{signV, velocity, acceleration};
            case ELEVATOR -> new double[]{signV, velocity, acceleration, 1.0};
            case ARM -> new double[]{signV, velocity, acceleration,
                    Math.cos(armAngleRadians == null ? 0.0 : armAngleRadians.getAsDouble())};
        };
    }

    /**
     * What the robot's own behaviour says its gains are. Produced only once the fit has converged, and
     * only ever <em>reported</em> — nothing in Catalyst applies these.
     *
     * @param mechanism     which mechanism this describes
     * @param kS            fitted static friction voltage
     * @param kV            fitted velocity gain
     * @param kA            fitted acceleration gain
     * @param kG            fitted gravity gain, or 0 when the mechanism has no gravity term
     * @param gravityType   which gravity model was fitted
     * @param sampleCount   how many samples the fit is based on
     * @param kSUncertainty rough 1-sigma spread on {@code kS}
     * @param kVUncertainty rough 1-sigma spread on {@code kV}
     * @param kAUncertainty rough 1-sigma spread on {@code kA}
     */
    public record Recommendation(
            String mechanism,
            double kS, double kV, double kA, double kG,
            GravityType gravityType,
            long sampleCount,
            double kSUncertainty, double kVUncertainty, double kAUncertainty) {

        /** A copy-paste-ready summary for the console or a log. */
        public String describe() {
            String base = String.format(Locale.ROOT,
                    "%s measured gains after %d samples: kS=%.4f (+/-%.4f), kV=%.4f (+/-%.4f), kA=%.4f (+/-%.4f)",
                    mechanism, sampleCount, kS, kSUncertainty, kV, kVUncertainty, kA, kAUncertainty);
            return gravityType == GravityType.NONE
                    ? base
                    : base + String.format(Locale.ROOT, ", kG=%.4f", kG);
        }

        /**
         * How far these differ from the gains you are running, as a fraction. A large number on
         * {@code kV} usually means something mechanical changed; a large number on {@code kS} usually
         * means friction did.
         */
        public double relativeChangeFrom(double currentKs, double currentKv, double currentKa) {
            double sum = relative(kS, currentKs) + relative(kV, currentKv) + relative(kA, currentKa);
            return sum / 3.0;
        }

        private static double relative(double measured, double current) {
            double scale = Math.max(1e-6, Math.abs(current));
            return Math.abs(measured - current) / scale;
        }
    }

    /** Start configuring an identifier for a named mechanism. */
    public static Builder builder(String name) {
        return new Builder(name);
    }

    /** Builder for {@link FeedforwardIdentifier}. */
    public static final class Builder {
        private final String name;
        private GravityType gravityType = GravityType.NONE;
        private DoubleSupplier armAngleRadians;
        private double velocityDeadband = 0.05;
        private double forgettingFactor = 1.0;
        private double convergenceThreshold = 0.02;
        private double[] seed;

        Builder(String name) {
            this.name = name;
        }

        /** Fit a constant gravity term — an elevator, or anything that fights gravity equally throughout. */
        public Builder withElevatorGravity() {
            this.gravityType = GravityType.ELEVATOR;
            return this;
        }

        /**
         * Fit a cosine gravity term — an arm, where the holding torque falls off toward vertical.
         *
         * @param angleRadians live joint angle, measured so that {@code 0} is horizontal
         */
        public Builder withArmGravity(DoubleSupplier angleRadians) {
            this.gravityType = GravityType.ARM;
            this.armAngleRadians = angleRadians;
            return this;
        }

        /**
         * Velocity below which samples are dropped, in the mechanism's velocity units. Defaults to
         * 0.05. Near zero the {@code sign(v)} regressor is noise and would corrupt {@code kS}.
         */
        public Builder velocityDeadband(double velocityDeadband) {
            this.velocityDeadband = velocityDeadband;
            return this;
        }

        /**
         * Let old data decay so the fit tracks a mechanism that genuinely changes over a match.
         * Defaults to {@code 1.0} (never forget), which suits gains that should be constant.
         */
        public Builder forgettingFactor(double forgettingFactor) {
            this.forgettingFactor = forgettingFactor;
            return this;
        }

        /** Covariance trace below which the fit counts as converged. Defaults to 0.02. */
        public Builder convergenceThreshold(double convergenceThreshold) {
            this.convergenceThreshold = convergenceThreshold;
            return this;
        }

        /**
         * Start from the gains you are already running, so early estimates are sane. Purely a starting
         * point — evidence overrides it within a few samples.
         *
         * @param values {@code kS, kV, kA} for a mechanism with no gravity term; {@code kS, kV, kA, kG}
         *               for one with
         */
        public Builder seed(double... values) {
            this.seed = values.clone();
            return this;
        }

        /** Validate and build. */
        public FeedforwardIdentifier build() {
            if (name == null || name.isBlank()) {
                throw new IllegalStateException("a mechanism name is required");
            }
            if (gravityType == GravityType.ARM && armAngleRadians == null) {
                throw new IllegalStateException("withArmGravity needs a live angle supplier for '"
                        + name + "' - the gravity term is cos(angle)");
            }
            if (velocityDeadband < 0) {
                throw new IllegalStateException("velocityDeadband must be >= 0 (got "
                        + velocityDeadband + ")");
            }
            int expected = gravityType == GravityType.NONE ? 3 : 4;
            if (seed != null && seed.length != expected) {
                throw new IllegalStateException("seed needs " + expected + " values for this gravity "
                        + "type (got " + seed.length + ")");
            }
            return new FeedforwardIdentifier(this);
        }
    }
}
