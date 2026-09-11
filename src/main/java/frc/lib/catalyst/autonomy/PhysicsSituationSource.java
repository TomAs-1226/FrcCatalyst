package frc.lib.catalyst.autonomy;

import frc.lib.catalyst.physics.PhysicalRobotState;
import frc.lib.catalyst.physics.PhysicsAnalysis;
import frc.lib.catalyst.physics.PhysicsCore;
import frc.lib.catalyst.physics.prediction.PowerPredictor;
import frc.lib.catalyst.util.RobotState;

/**
 * Builds a {@link Situation} out of the pieces a robot actually has.
 *
 * <p>Every input is optional. A robot with a physics core and no power measurement gets a snapshot
 * whose power facet is marked invalid rather than zeroed; a robot with neither gets
 * {@link Situation#blind()}. Nothing here invents a number, and nothing here throws: each facet is
 * built inside its own guard, so one broken source costs that facet and not the loop.
 *
 * <pre>{@code
 * SituationSource situations = PhysicsSituationSource.builder()
 *         .physics(physics)
 *         .power(powerPredictor)      // optional; omit and power is marked unmeasured
 *         .build()
 *         .cachedPerLoop();
 * }</pre>
 *
 * <h2>Power without a power module</h2>
 *
 * <p>The power facet needs a current measurement, not a {@code PowerDistribution}. Bus voltage comes
 * from the robot controller and is reported either way; {@link PowerPredictor} takes a
 * {@code DoubleSupplier} for current, so a robot with nothing on CAN to measure total draw can sum
 * its Talon FX supply currents instead. With no current source at all the facet is
 * {@code Power.unmeasured(volts)} - valid is false, and every core that reads power declines rather
 * than acting on a zero.
 *
 * @since 2.1.0
 */
public final class PhysicsSituationSource implements SituationSource {

    private final PhysicsCore physics;
    private final PowerPredictor power;

    private PhysicsSituationSource(Builder b) {
        this.physics = b.physics;
        this.power = b.power;
    }

    @Override
    public Situation sample(double nowSeconds) {
        return new Situation(nowSeconds, localization(), motion(), traction(), power(), match());
    }

    // ------------------------------------------------------------------ facets

    private Situation.Localization localization() {
        if (physics == null) {
            return Situation.Localization.unknown();
        }
        try {
            PhysicalRobotState s = physics.state();
            var q = s.quality();
            return new Situation.Localization(
                    s.pose(),
                    q.confidence(),
                    switch (q.level()) {
                        case HIGH -> Situation.Localization.Level.HIGH;
                        case MODERATE -> Situation.Localization.Level.MODERATE;
                        case LOW -> Situation.Localization.Level.LOW;
                        case LOST -> Situation.Localization.Level.LOST;
                    },
                    q.secondsSinceAbsoluteFix(),
                    true);
        } catch (Throwable t) {
            return Situation.Localization.unknown();
        }
    }

    private Situation.Motion motion() {
        if (physics == null) {
            return Situation.Motion.unknown();
        }
        try {
            PhysicalRobotState s = physics.state();
            return new Situation.Motion(
                    s.fieldVelocity(), s.speedMetersPerSecond(), s.accelerationMetersPerSecSq(), true);
        } catch (Throwable t) {
            return Situation.Motion.unknown();
        }
    }

    private Situation.Traction traction() {
        if (physics == null) {
            return Situation.Traction.unknown();
        }
        try {
            PhysicsAnalysis a = physics.analyze();
            return new Situation.Traction(a.slipFactor(), a.tractionUsage(), a.tippingUsage(), true);
        } catch (Throwable t) {
            return Situation.Traction.unknown();
        }
    }

    private Situation.Power power() {
        double volts;
        try {
            volts = RobotState.batteryVoltage();
        } catch (Throwable t) {
            volts = 0.0;
        }
        if (power == null) {
            return Situation.Power.unmeasured(volts);
        }
        try {
            // Only a predictor that has been given a breaker budget is answering the question an
            // allocator asks. Without one its headroom is a battery-sag figure - true, and roughly
            // twice what the main breaker allows - so the facet stays invalid rather than handing a
            // consumer a number that reads like a budget and is not one.
            boolean budgeted = power.breakerBudgetAmps().isPresent();
            return new Situation.Power(
                    volts,
                    budgeted ? power.headroomAmps() : 0.0,
                    power.presentCurrentAmps(),
                    budgeted ? power.bindingLimit() : "no breaker budget set",
                    budgeted);
        } catch (Throwable t) {
            return Situation.Power.unmeasured(volts);
        }
    }

    private Situation.Match match() {
        try {
            return new Situation.Match(
                    RobotState.isEnabled(),
                    RobotState.isAutonomous(),
                    RobotState.matchTimeRemaining(),
                    RobotState.isDsAttached());
        } catch (Throwable t) {
            return Situation.Match.unknown();
        }
    }

    // ------------------------------------------------------------------ builder

    public static Builder builder() {
        return new Builder();
    }

    /** Everything is optional; what is missing is reported missing rather than faked. */
    public static final class Builder {
        private PhysicsCore physics;
        private PowerPredictor power;

        /** Source of pose, motion and traction. Without it those three facets are invalid. */
        public Builder physics(PhysicsCore physics) {
            this.physics = physics;
            return this;
        }

        /**
         * Source of the power facet. Give it a predictor built with
         * {@code breakerBudgetAmps(...)}, or the facet stays invalid on purpose.
         */
        public Builder power(PowerPredictor power) {
            this.power = power;
            return this;
        }

        public PhysicsSituationSource build() {
            return new PhysicsSituationSource(this);
        }
    }
}
