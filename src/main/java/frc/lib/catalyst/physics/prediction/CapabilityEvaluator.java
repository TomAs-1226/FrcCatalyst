package frc.lib.catalyst.physics.prediction;

import java.util.Locale;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Translation2d;

import frc.lib.catalyst.physics.PhysicalRobotState;
import frc.lib.catalyst.physics.model.DrivetrainModel;
import frc.lib.catalyst.physics.model.StabilityModel;

/**
 * Asks whether an action is worth attempting <em>before</em> it is scheduled, and what it will cost.
 *
 * <p>{@code BehaviorEngine} already picks a fallback when an action fails. This lets it pick one
 * before the action fails, which is a much better place to make the decision — a cycle abandoned at
 * second 1.4 is a cycle wasted, and the information needed to skip it was available at second 0.
 *
 * <pre>{@code
 * Action scoreHigh = Action.named("Score high")
 *     .when(() -> evaluator.evaluateDriveTo(physics.state(), scoringSpot, 55).isReliable())
 *     .run(superstructure::scoreHigh)
 *     .build();
 *
 * BehaviorEngine.sequence("Cycle").attempt(scoreHigh).orElse(scoreLow).build();
 * }</pre>
 *
 * <h2>Where the numbers come from</h2>
 * Nothing here is a heuristic dressed up as a prediction. Each figure has a derivation:
 *
 * <ul>
 *   <li><b>Time</b> — an exact trapezoidal profile from the robot's current speed to a stop at the
 *       target, under the acceleration and speed limits in force. Triangular when the move is too
 *       short to reach cruising speed; infeasible when the robot is already going too fast to stop in
 *       the distance available, which is a real answer worth having.</li>
 *   <li><b>Position error</b> — the state estimate's own uncertainty, compounded over the predicted
 *       duration. The same arithmetic {@link LaunchState#missRadiusMeters(double)} uses.</li>
 *   <li><b>Tip margin</b> — {@link StabilityModel} evaluated at the acceleration the move actually
 *       requires, in the direction it requires it, using the live centre of mass.</li>
 *   <li><b>Minimum voltage</b> — {@link PowerPredictor} given the action's expected draw on top of
 *       what the robot is already pulling.</li>
 * </ul>
 *
 * <p>Anything not configured is simply not evaluated: with no {@link PowerPredictor} the voltage
 * figure is absent rather than invented, and the verdict is made on what is actually known.
 *
 * <p>Pure math over the models you hand it — no hardware, no HAL.
 *
 * @since 1.6.0
 */
public final class CapabilityEvaluator {

    private final DrivetrainModel drivetrain;
    private final StabilityModel stability;
    private final PowerPredictor power;
    private final double maxSpeedMps;
    private final double usableFraction;

    private CapabilityEvaluator(Builder builder) {
        this.drivetrain = builder.drivetrain;
        this.stability = builder.stability;
        this.power = builder.power;
        this.maxSpeedMps = builder.maxSpeedMps;
        this.usableFraction = builder.usableFraction;
    }

    /**
     * Evaluate driving from where the robot is to a field position and stopping there.
     *
     * @param state          the robot's current physical state
     * @param target         where to end up, field-relative
     * @param expectedAmps   what the action will draw while it runs; pass {@code 0} if you have no
     *                       estimate and the voltage figure will simply reflect the present draw
     */
    public Capability evaluateDriveTo(PhysicalRobotState state, Translation2d target, double expectedAmps) {
        Translation2d toTarget = target.minus(state.pose().getTranslation());
        double distance = toTarget.getNorm();
        if (distance < 1e-6) {
            return new Capability(true, OptionalDouble.of(0.0),
                    state.quality().translationStdDevMeters(),
                    marginAt(Translation2d.kZero, state), voltageAt(expectedAmps), 0.0,
                    Risk.LOW, "already there");
        }

        Translation2d direction = toTarget.div(distance);
        // Speed along the line to the target. A robot travelling sideways arrives with none of its
        // current speed helping, and the projection is what says so.
        double approachSpeed = state.fieldVelocity().vxMetersPerSecond * direction.getX()
                + state.fieldVelocity().vyMetersPerSecond * direction.getY();
        double acceleration = accelerationLimit(direction, state);

        OptionalDouble seconds = timeToTravel(distance, Math.max(0.0, approachSpeed),
                maxSpeedMps, acceleration);

        double tipMargin = marginAt(direction.times(acceleration), state);
        OptionalDouble volts = voltageAt(expectedAmps);
        double positionError = seconds.isPresent()
                ? state.quality().translationStdDevMeters()
                        + state.quality().velocityStdDevMetersPerSecond() * seconds.getAsDouble()
                : Double.POSITIVE_INFINITY;

        return verdict(seconds, positionError, tipMargin, volts, acceleration,
                approachSpeed, distance, acceleration);
    }

    /**
     * Evaluate an action that is not a drive — raising an elevator, spinning up a shooter — where you
     * already know roughly how long it takes and what it draws.
     *
     * @param state        the robot's current physical state
     * @param seconds      how long the action takes
     * @param expectedAmps what it draws while it runs
     */
    public Capability evaluateAction(PhysicalRobotState state, double seconds, double expectedAmps) {
        double positionError = state.quality().translationStdDevMeters()
                + state.quality().velocityStdDevMetersPerSecond() * Math.max(0.0, seconds);
        double tipMargin = marginAt(state.fieldAcceleration(), state);
        return verdict(OptionalDouble.of(Math.max(0.0, seconds)), positionError, tipMargin,
                voltageAt(expectedAmps), 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Exact duration of a trapezoidal move from {@code initialSpeed} to a stop over {@code distance},
     * limited by {@code maxSpeed} and {@code maxAcceleration}. Empty when the robot is already
     * travelling too fast to stop within the distance — it would overshoot, so the move as specified
     * is not possible.
     *
     * <p>Exposed because it is generally useful and worth being able to test on its own.
     */
    public static OptionalDouble timeToTravel(double distance, double initialSpeed,
                                              double maxSpeed, double maxAcceleration) {
        if (distance <= 0) return OptionalDouble.of(0.0);
        if (!(maxAcceleration > 0) || !(maxSpeed > 0)) return OptionalDouble.empty();

        double v0 = Math.max(0.0, initialSpeed);
        double vmax = Math.max(maxSpeed, v0);       // already faster than the cap: coast, do not speed up
        double stoppingDistance = v0 * v0 / (2.0 * maxAcceleration);
        if (stoppingDistance > distance) return OptionalDouble.empty();

        // Triangular: accelerate to a peak, then straight back down.
        double peak = Math.sqrt((v0 * v0 + 2.0 * maxAcceleration * distance) / 2.0);
        if (peak <= vmax) {
            return OptionalDouble.of((peak - v0) / maxAcceleration + peak / maxAcceleration);
        }

        // Trapezoidal: accelerate to the cap, cruise, then decelerate.
        double accelDistance = (vmax * vmax - v0 * v0) / (2.0 * maxAcceleration);
        double decelDistance = vmax * vmax / (2.0 * maxAcceleration);
        double cruiseDistance = distance - accelDistance - decelDistance;
        return OptionalDouble.of((vmax - v0) / maxAcceleration
                + cruiseDistance / vmax
                + vmax / maxAcceleration);
    }

    /** The acceleration limit in force for a move in {@code direction}, in m/s^2. */
    private double accelerationLimit(Translation2d direction, PhysicalRobotState state) {
        double limit = drivetrain.maxSafeAccelerationMpsSq(usableFraction);
        if (stability != null) {
            Translation2d robotDirection =
                    StabilityModel.toRobotFrame(direction, state.pose().getRotation());
            limit = Math.min(limit, stability.maxAccelerationMpsSq(robotDirection) * usableFraction);
        }
        return limit;
    }

    /** Tip margin at a field-relative acceleration, or {@link Double#NaN} when not configured. */
    private double marginAt(Translation2d fieldAccel, PhysicalRobotState state) {
        if (stability == null) return Double.NaN;
        return stability.tipMarginMeters(fieldAccel, state.pose().getRotation());
    }

    private OptionalDouble voltageAt(double expectedAmps) {
        return power == null ? OptionalDouble.empty()
                : OptionalDouble.of(power.predictedVoltage(expectedAmps));
    }

    private Capability verdict(OptionalDouble seconds, double positionError, double tipMargin,
                               OptionalDouble volts, double requiredAccel,
                               double approachSpeed, double distance, double accelerationLimit) {
        if (seconds.isEmpty()) {
            String why = distance > 0
                    ? String.format(Locale.ROOT,
                            "cannot stop in %.2f m from %.1f m/s at %.1f m/s^2",
                            distance, approachSpeed, accelerationLimit)
                    : "no feasible motion profile";
            return new Capability(false, seconds, positionError, tipMargin, volts, requiredAccel,
                    Risk.INFEASIBLE, why);
        }

        boolean tipping = !Double.isNaN(tipMargin) && tipMargin <= 0;
        boolean brownout = volts.isPresent() && power != null
                && volts.getAsDouble() < power.minimumVoltage();
        if (tipping) {
            return new Capability(false, seconds, positionError, tipMargin, volts, requiredAccel,
                    Risk.INFEASIBLE, "would tip: no margin at the required acceleration");
        }
        if (brownout) {
            return new Capability(false, seconds, positionError, tipMargin, volts, requiredAccel,
                    Risk.INFEASIBLE, String.format(Locale.ROOT,
                            "would brown out: %.1f V predicted, floor is %.1f V",
                            volts.getAsDouble(), power.minimumVoltage()));
        }

        Risk risk = Risk.LOW;
        String limiting = "nominal";
        if (!Double.isNaN(tipMargin) && tipMargin < 0.05) {
            risk = Risk.HIGH;
            limiting = String.format(Locale.ROOT, "tip margin only %.0f mm", tipMargin * 1000);
        } else if (volts.isPresent() && power != null
                && volts.getAsDouble() < power.minimumVoltage() + 0.5) {
            risk = Risk.HIGH;
            limiting = String.format(Locale.ROOT, "voltage close to the floor (%.1f V)", volts.getAsDouble());
        } else if (positionError > 0.15) {
            risk = Risk.MODERATE;
            limiting = String.format(Locale.ROOT, "position uncertain to %.0f cm on arrival",
                    positionError * 100);
        } else if (!Double.isNaN(tipMargin) && tipMargin < 0.12) {
            risk = Risk.MODERATE;
            limiting = String.format(Locale.ROOT, "tip margin %.0f mm", tipMargin * 1000);
        }
        return new Capability(true, seconds, positionError, tipMargin, volts, requiredAccel, risk, limiting);
    }

    /** How much could go wrong with an action. */
    public enum Risk {
        /** Comfortable margins all round. */
        LOW,
        /** Doable, but something is tight. Worth preferring an alternative if there is one. */
        MODERATE,
        /** Doable on paper only. A margin is nearly gone. */
        HIGH,
        /** Not possible as specified. */
        INFEASIBLE
    }

    /**
     * What an action would cost and whether it is worth attempting.
     *
     * @param feasible                 whether it can be done at all
     * @param seconds                  predicted duration; empty when infeasible
     * @param positionErrorMeters      1-sigma position uncertainty on arrival
     * @param tipMarginMeters          stability margin at the required acceleration;
     *                                 {@link Double#NaN} when no stability model was configured
     * @param minimumVolts             predicted bus voltage while it runs; empty when no power
     *                                 predictor was configured
     * @param requiredAccelerationMpsSq acceleration the move demands
     * @param risk                     overall assessment
     * @param limitingFactor           short text naming whatever is tightest
     */
    public record Capability(
            boolean feasible,
            OptionalDouble seconds,
            double positionErrorMeters,
            double tipMarginMeters,
            OptionalDouble minimumVolts,
            double requiredAccelerationMpsSq,
            Risk risk,
            String limitingFactor) {

        /** Feasible and nothing is tight — the usual gate for "just do it". */
        public boolean isReliable() {
            return feasible && risk == Risk.LOW;
        }

        /** Feasible, though something is tight. The gate for "do it if there is no better option". */
        public boolean isAcceptable() {
            return feasible && risk != Risk.HIGH;
        }

        /** A multi-line report in the shape the RFC asked for. */
        public String describe() {
            StringBuilder text = new StringBuilder();
            text.append("Feasible: ").append(feasible ? "yes" : "no").append('\n');
            if (seconds.isPresent()) {
                text.append(String.format(Locale.ROOT, "Predicted completion: %.2f s%n", seconds.getAsDouble()));
            }
            if (minimumVolts.isPresent()) {
                text.append(String.format(Locale.ROOT, "Predicted minimum voltage: %.1f V%n",
                        minimumVolts.getAsDouble()));
            }
            if (Double.isFinite(positionErrorMeters)) {
                text.append(String.format(Locale.ROOT, "Predicted position error: %.1f cm%n",
                        positionErrorMeters * 100));
            }
            if (!Double.isNaN(tipMarginMeters)) {
                text.append(String.format(Locale.ROOT, "Tip margin: %.1f cm%n", tipMarginMeters * 100));
            }
            text.append("Risk: ").append(risk.name().toLowerCase(Locale.ROOT));
            if (!"nominal".equals(limitingFactor)) text.append(" (").append(limitingFactor).append(')');
            return text.toString();
        }
    }

    /** Start building an evaluator. A drivetrain model is required; everything else sharpens it. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link CapabilityEvaluator}. */
    public static final class Builder {
        private DrivetrainModel drivetrain;
        private StabilityModel stability;
        private PowerPredictor power;
        private double maxSpeedMps = 4.5;
        private double usableFraction = 0.8;

        /** Traction and tipping limits from the static model. Required. */
        public Builder drivetrain(DrivetrainModel drivetrain) {
            this.drivetrain = drivetrain;
            return this;
        }

        /**
         * The live stability model. Without it, tip margin is not evaluated and the acceleration limit
         * falls back to the static one — which is optimistic for a robot with its elevator up.
         */
        public Builder stability(StabilityModel stability) {
            this.stability = stability;
            return this;
        }

        /** The power predictor. Without it, voltage is not evaluated. */
        public Builder power(PowerPredictor power) {
            this.power = power;
            return this;
        }

        /** The drivetrain's maximum translational speed, in m/s. Defaults to 4.5. */
        public Builder maxSpeedMps(double maxSpeedMps) {
            this.maxSpeedMps = maxSpeedMps;
            return this;
        }

        /**
         * How much of the physical acceleration limit to plan against, in {@code (0, 1]}. Defaults to
         * 0.8 — planning at 100% of the limit means every prediction assumes a perfect controller.
         */
        public Builder usableTractionFraction(double usableFraction) {
            this.usableFraction = usableFraction;
            return this;
        }

        /** Validate and build. */
        public CapabilityEvaluator build() {
            if (drivetrain == null) {
                throw new IllegalStateException("a DrivetrainModel is required - it supplies the "
                        + "acceleration limit every prediction is built on");
            }
            if (!(maxSpeedMps > 0)) {
                throw new IllegalStateException("maxSpeedMps must be > 0 (got " + maxSpeedMps + ")");
            }
            if (!(usableFraction > 0) || usableFraction > 1) {
                throw new IllegalStateException("usableTractionFraction must be in (0, 1] (got "
                        + usableFraction + ")");
            }
            return new CapabilityEvaluator(this);
        }
    }
}
