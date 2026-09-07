package frc.lib.catalyst.autonomy;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Everything the autonomy layer is allowed to know about the robot this loop, sampled once.
 *
 * <h2>Why a snapshot</h2>
 *
 * <p>Before this, each decision-maker asked the world its own questions at its own moment:
 * {@code Strategist} read the clock in its scorer, {@code GoalDirector} read the coordinator in its
 * monitor, and a robot applying a physics speed limit re-read {@code speedScale()} three times in
 * one loop. Nothing guaranteed they agreed. A snapshot makes them agree by construction, and makes
 * the whole decision reproducible: given this record, the answer is a pure function.
 *
 * <p>It is also cheaper. One sample per loop replaces N live reads, several of which cross into
 * NetworkTables or the HAL.
 *
 * <h2>Honest validity</h2>
 *
 * <p>Every facet carries its own {@code valid} flag and <b>no facet invents a number</b>. This is
 * the lesson from {@code BrownoutMonitor}, which defaulted its current source to zero and so
 * silently reported the present voltage as a prediction for a season. A robot with no power
 * measurement has {@code power().valid() == false}, not {@code headroomAmps() == 0}, and a consumer
 * that ignores the flag is making a choice rather than being misled.
 *
 * <p>Immutable, allocation-light, and free of any WPILib type that the released 2027 alpha-6 does
 * not carry. Nothing here reads hardware - see {@link SituationSource}.
 *
 * @since 2.1.0
 */
public record Situation(
        double timestampSeconds,
        Localization localization,
        Motion motion,
        Traction traction,
        Power power,
        Match match) {

    /**
     * Where the robot thinks it is, and how much that is worth.
     *
     * @param pose                  field-relative best estimate
     * @param confidence            0..1, from the estimator
     * @param level                 the confidence band, for a switch rather than a threshold
     * @param secondsSinceAbsoluteFix how long since a vision fix corrected the estimate
     * @param valid                 false when there is no estimator at all
     */
    public record Localization(Pose2d pose, double confidence, Level level,
                               double secondsSinceAbsoluteFix, boolean valid) {

        /** Mirrors the estimator's own bands so autonomy never re-derives a threshold. */
        public enum Level { HIGH, MODERATE, LOW, LOST }

        /** Good enough to commit to a precise move. */
        public boolean trustworthy() {
            return valid && (level == Level.HIGH || level == Level.MODERATE);
        }

        static Localization unknown() {
            return new Localization(Pose2d.kZero, 0.0, Level.LOST, Double.POSITIVE_INFINITY, false);
        }
    }

    /**
     * How the robot is moving, field-relative.
     *
     * @param fieldVelocity  field-relative velocity
     * @param speedMps       its magnitude, precomputed because every consumer wants it
     * @param accelMpsSq     magnitude of field acceleration
     * @param valid          false when there is no estimator
     */
    public record Motion(ChassisVelocities fieldVelocity, double speedMps, double accelMpsSq,
                         boolean valid) {

        /** Close enough to stationary that heading-of-travel is meaningless. */
        public boolean atRest() {
            return !valid || speedMps < 0.05;
        }

        static Motion unknown() {
            return new Motion(new ChassisVelocities(), 0.0, 0.0, false);
        }
    }

    /**
     * How much of the available grip and stability the robot is already spending.
     *
     * @param slipFactor    0 = gripping, 1 = fully slipping
     * @param tractionUsage fraction of the friction budget in use
     * @param tippingUsage  fraction of the tipping margin in use
     * @param valid         false when there is no physics model
     */
    public record Traction(double slipFactor, double tractionUsage, double tippingUsage,
                           boolean valid) {

        /** Wheels are not delivering what is being asked of them. */
        public boolean slipping() {
            return valid && slipFactor > 0.2;
        }

        /** Close enough to a tip that a new demand should not be added. */
        public boolean nearTipping() {
            return valid && tippingUsage > 0.8;
        }

        static Traction unknown() {
            return new Traction(0.0, 0.0, 0.0, false);
        }
    }

    /**
     * What the electrical system can still give.
     *
     * <p>{@code valid} is false unless something is genuinely measuring current. Without a power
     * distribution hub on the bus there is no total-current measurement, and a headroom computed
     * from voltage alone is a battery-sag figure rather than a budget - see
     * {@code PowerPredictor.headroomAmps()}, which will happily report 245 A on a robot whose main
     * breaker is 120 A.
     *
     * @param busVolts     measured bus voltage; meaningful even when the rest is not
     * @param headroomAmps amps that may still be drawn, when known
     * @param drawAmps     amps flowing now, when known
     * @param bindingLimit which limit is holding: the breaker, the battery, or neither
     * @param valid        false when nothing is measuring current
     */
    public record Power(double busVolts, double headroomAmps, double drawAmps,
                        String bindingLimit, boolean valid) {

        /** Past the budget: something should be shed. Never true on an unmeasured robot. */
        public boolean overBudget() {
            return valid && headroomAmps < 0;
        }

        static Power unmeasured(double busVolts) {
            return new Power(busVolts, 0.0, 0.0, "unmeasured", false);
        }
    }

    /**
     * The match, as far as the driver station has said.
     *
     * @param enabled         the robot is enabled
     * @param autonomous      in the autonomous period
     * @param secondsRemaining time left in the period, or NaN when the DS has not said
     * @param valid           false when no driver station has ever connected
     */
    public record Match(boolean enabled, boolean autonomous, double secondsRemaining, boolean valid) {

        /** Time is short enough that a long plan should not be started. */
        public boolean endgame(double thresholdSeconds) {
            return valid && !Double.isNaN(secondsRemaining) && secondsRemaining <= thresholdSeconds;
        }

        static Match unknown() {
            return new Match(false, false, Double.NaN, false);
        }
    }

    /**
     * A situation that knows nothing, for a robot with no physics core, a test that does not care,
     * and the first loop before anything has been sampled.
     *
     * <p>Every facet is invalid, so a consumer written against the flags degrades to "the driver has
     * control" rather than acting on zeros.
     */
    public static Situation blind() {
        return blind(0.0);
    }

    /** {@link #blind()} stamped with a time, for a test that cares about ordering. */
    public static Situation blind(double timestampSeconds) {
        return new Situation(timestampSeconds, Localization.unknown(), Motion.unknown(),
                Traction.unknown(), Power.unmeasured(0.0), Match.unknown());
    }

    /** True when nothing in this snapshot can be relied on. */
    public boolean blindfolded() {
        return !localization.valid() && !motion.valid() && !traction.valid() && !power.valid();
    }
}
