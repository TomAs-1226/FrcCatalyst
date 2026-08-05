package frc.lib.catalyst.physics.constraints;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.catalyst.physics.LocalizationQuality;
import frc.lib.catalyst.physics.PhysicsCore;
import frc.lib.catalyst.physics.model.StabilityModel;
import frc.lib.catalyst.physics.prediction.PowerPredictor;

/**
 * Turns everything Physics Core knows into two numbers a drivetrain can actually use — and hands them
 * to you rather than applying them.
 *
 * <h2>Opt-in, and visibly so</h2>
 * This is Phase 3 of the Physics Core RFC: bounded intervention. It is the first part of the package
 * that could change how the robot drives, and so it is built to make that impossible by accident.
 * There is no {@code install()}, no periodic hook, nothing that reaches into {@code SwerveSubsystem}.
 * It computes limits; you decide what to do with them, on a line you wrote:
 *
 * <pre>{@code
 * PhysicsConstraints limits = PhysicsConstraints.builder()
 *     .physics(physics)
 *     .stability(stabilityModel)
 *     .power(powerPredictor)
 *     .build();
 *
 * // in periodic - your code, your call:
 * drive.setSpeedMultiplier(limits.speedScale());
 * }</pre>
 *
 * <p>A team that never writes that line gets exactly the robot they had before. That is the point: an
 * advisory layer that quietly reaches into the drivetrain is one that turns "why did it slow down
 * there?" into an afternoon of debugging.
 *
 * <h2>What limits it applies</h2>
 * Four, each of which reduces the limit independently, and the tightest wins:
 *
 * <ul>
 *   <li><b>Traction and tipping</b> — from {@link StabilityModel}, using the live centre of mass, so
 *       the limit tightens on its own when the elevator goes up.</li>
 *   <li><b>Confidence</b> — a robot that does not know where it is has no business moving at full
 *       speed toward a scoring position. Scales with {@link LocalizationQuality}.</li>
 *   <li><b>Slip</b> — wheels already breaking traction will not deliver more acceleration; asking for
 *       it only makes the odometry worse.</li>
 *   <li><b>Power</b> — an accelerating drivetrain is the largest load on the robot, and there is no
 *       point commanding an acceleration the battery cannot supply.</li>
 * </ul>
 *
 * <p>{@link #explain()} names which one is binding, so a slowdown is always attributable.
 *
 * @since 1.6.0
 */
public final class PhysicsConstraints {

    private final PhysicsCore physics;
    private final StabilityModel stability;
    private final PowerPredictor power;
    private final double maxSpeedMps;
    private final double usableFraction;
    private final double minimumSpeedScale;
    private final double accelerationCurrentPerMpsSq;
    private final boolean confidenceScalingEnabled;

    private PhysicsConstraints(Builder builder) {
        this.physics = builder.physics;
        this.stability = builder.stability;
        this.power = builder.power;
        this.maxSpeedMps = builder.maxSpeedMps;
        this.usableFraction = builder.usableFraction;
        this.minimumSpeedScale = builder.minimumSpeedScale;
        this.accelerationCurrentPerMpsSq = builder.accelerationCurrentPerMpsSq;
        this.confidenceScalingEnabled = builder.confidenceScalingEnabled;
    }

    /**
     * The hardest the robot should accelerate right now, in m/s^2 — the tightest of the traction,
     * tipping, and power limits, with the configured margin applied.
     */
    public double maxAccelerationMpsSq() {
        double limit = physics.drivetrainModel().maxSafeAccelerationMpsSq(usableFraction);

        if (stability != null) {
            limit = Math.min(limit, stability.worstCaseAccelerationMpsSq() * usableFraction);
        }
        if (power != null && accelerationCurrentPerMpsSq > 0) {
            double affordable = Math.max(0.0, power.headroomAmps()) / accelerationCurrentPerMpsSq;
            limit = Math.min(limit, affordable);
        }
        return Math.max(0.0, limit);
    }

    /**
     * The hardest the robot should accelerate in a specific direction, in m/s^2. Tighter than
     * {@link #maxAccelerationMpsSq()} when the robot is more stable one way than another — a tall
     * robot mid-turn can take more lateral acceleration one way than the other.
     *
     * @param fieldDirection direction of travel, field-relative; magnitude ignored
     */
    public double maxAccelerationMpsSq(Translation2d fieldDirection) {
        double limit = maxAccelerationMpsSq();
        if (stability != null) {
            Translation2d robotDirection = StabilityModel.toRobotFrame(
                    fieldDirection, physics.state().pose().getRotation());
            limit = Math.min(limit, stability.maxAccelerationMpsSq(robotDirection) * usableFraction);
        }
        return Math.max(0.0, limit);
    }

    /**
     * A multiplier from {@code 0} to {@code 1} for the drivetrain's commanded speed, reflecting how
     * much the robot currently deserves to be trusted with. Feeds
     * {@code SwerveSubsystem.setSpeedMultiplier(...)} directly.
     *
     * <p>Never returns exactly zero unless the estimate is genuinely lost: a robot frozen mid-match
     * because a camera blinked is worse than a robot moving cautiously. The floor is
     * {@link Builder#minimumSpeedScale(double)}.
     */
    public double speedScale() {
        double scale = 1.0;

        if (confidenceScalingEnabled) {
            scale = Math.min(scale, switch (physics.state().quality().level()) {
                case HIGH -> 1.0;
                case MODERATE -> 0.75;
                case LOW -> 0.45;
                case LOST -> minimumSpeedScale;
            });
        }
        // Wheels that are already slipping will not deliver more, so asking for it only degrades the
        // odometry further.
        double slip = physics.analyze().slipFactor();
        if (slip > 0.2) scale = Math.min(scale, 1.0 - 0.5 * Math.min(1.0, slip));

        if (power != null && power.headroomAmps() < 0) scale = Math.min(scale, 0.6);

        return Math.max(minimumSpeedScale, Math.min(1.0, scale));
    }

    /** The speed cap in m/s: {@link #speedScale()} applied to the configured maximum. */
    public double maxSpeedMps() {
        return maxSpeedMps * speedScale();
    }

    // ===========================================
    //   Conditions — plain predicates, then Triggers
    // ===========================================
    //
    // Each condition comes in two forms. The predicate is pure and can be called anywhere, including
    // a unit test; the Trigger wraps it for robot code. Constructing a WPILib Trigger reaches into
    // the command scheduler, so keeping the logic in the predicate is what lets all of this be tested
    // with no HAL.

    /** True while the physics says the robot should be taking it easy. */
    public boolean isCautionAdvised() {
        return speedScale() < 0.95;
    }

    /** True while the state estimate is not worth acting on at all. */
    public boolean isLocalizationLost() {
        return physics.state().quality().level() == LocalizationQuality.Level.LOST;
    }

    /** True while the robot is close enough to tipping to be worth reacting to. */
    public boolean isTipRiskHigh() {
        return stability != null
                && stability.tipMarginMeters(physics.state().fieldAcceleration(),
                        physics.state().pose().getRotation()) < 0.05;
    }

    /**
     * A trigger that fires while the physics says the robot should be taking it easy — low confidence,
     * slipping, tipping close, or out of electrical headroom. Composes with the WPILib bindings you
     * already use.
     *
     * <pre>{@code
     * limits.cautionAdvised().whileTrue(Commands.run(() -> drive.setSpeedMultiplier(0.5)));
     * }</pre>
     */
    public Trigger cautionAdvised() {
        return new Trigger(this::isCautionAdvised);
    }

    /** A trigger that fires while the state estimate is not worth acting on at all. */
    public Trigger localizationLost() {
        return new Trigger(this::isLocalizationLost);
    }

    /** A trigger that fires while the robot is close enough to tipping to be worth reacting to. */
    public Trigger tipRiskHigh() {
        return new Trigger(this::isTipRiskHigh);
    }

    /**
     * Which limit is currently binding, and why. Every slowdown this class recommends should be
     * attributable to a sentence, and this is that sentence.
     */
    public String explain() {
        List<String> reasons = new ArrayList<>(3);
        LocalizationQuality quality = physics.state().quality();

        if (confidenceScalingEnabled && quality.level() != LocalizationQuality.Level.HIGH) {
            reasons.add(String.format(Locale.ROOT, "confidence %s (%s)", quality.level(), quality.reason()));
        }
        double slip = physics.analyze().slipFactor();
        if (slip > 0.2) reasons.add(String.format(Locale.ROOT, "wheel slip %.0f%%", slip * 100));
        if (power != null && power.headroomAmps() < 0) {
            reasons.add(String.format(Locale.ROOT, "over the current budget by %.0f A", -power.headroomAmps()));
        }
        if (stability != null) {
            double margin = stability.tipMarginMeters(physics.state().fieldAcceleration(),
                    physics.state().pose().getRotation());
            if (margin < 0.12) reasons.add(String.format(Locale.ROOT, "tip margin %.0f mm", margin * 1000));
        }

        if (reasons.isEmpty()) return "no limit binding - full speed and acceleration available";
        return String.format(Locale.ROOT, "speed scaled to %.0f%%, accel capped at %.1f m/s^2: %s",
                speedScale() * 100, maxAccelerationMpsSq(), String.join("; ", reasons));
    }

    /** Start building. A {@link PhysicsCore} is required; the other models sharpen the limits. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link PhysicsConstraints}. */
    public static final class Builder {
        private PhysicsCore physics;
        private StabilityModel stability;
        private PowerPredictor power;
        private double maxSpeedMps = 4.5;
        private double usableFraction = 0.85;
        private double minimumSpeedScale = 0.25;
        private double accelerationCurrentPerMpsSq = 0.0;
        private boolean confidenceScalingEnabled = true;

        /** The running Physics Core these limits read from. Required. */
        public Builder physics(PhysicsCore physics) {
            this.physics = physics;
            return this;
        }

        /**
         * The live stability model. Without it the tipping limit stays static, which is optimistic for
         * a robot that extends.
         */
        public Builder stability(StabilityModel stability) {
            this.stability = stability;
            return this;
        }

        /** The power predictor. Without it, electrical headroom does not constrain acceleration. */
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
         * How much of the physical limit to actually use, in {@code (0, 1]}. Defaults to 0.85 — the
         * models are good, not perfect, and the margin is where that honesty lives.
         */
        public Builder usableFraction(double usableFraction) {
            this.usableFraction = usableFraction;
            return this;
        }

        /**
         * The lowest {@link #speedScale()} will ever return, in {@code [0, 1]}. Defaults to 0.25.
         * Setting this to zero means a lost estimate stops the robot dead, which is a real choice but
         * rarely the right one in a match.
         */
        public Builder minimumSpeedScale(double minimumSpeedScale) {
            this.minimumSpeedScale = minimumSpeedScale;
            return this;
        }

        /**
         * How many amps the drivetrain draws per m/s^2 of commanded acceleration. Set it and the
         * acceleration limit respects the electrical budget too; leave it at zero and power only
         * affects the speed scale. Measure it once from a log: peak drive current divided by peak
         * acceleration.
         */
        public Builder accelerationCurrentPerMpsSq(double accelerationCurrentPerMpsSq) {
            this.accelerationCurrentPerMpsSq = accelerationCurrentPerMpsSq;
            return this;
        }

        /** Turn confidence-based speed scaling off, leaving only the physical limits. On by default. */
        public Builder withoutConfidenceScaling() {
            this.confidenceScalingEnabled = false;
            return this;
        }

        /** Validate and build. */
        public PhysicsConstraints build() {
            if (physics == null) {
                throw new IllegalStateException("a PhysicsCore is required - these limits are derived "
                        + "from its state and analysis");
            }
            if (!(maxSpeedMps > 0)) {
                throw new IllegalStateException("maxSpeedMps must be > 0 (got " + maxSpeedMps + ")");
            }
            if (!(usableFraction > 0) || usableFraction > 1) {
                throw new IllegalStateException("usableFraction must be in (0, 1] (got "
                        + usableFraction + ")");
            }
            if (minimumSpeedScale < 0 || minimumSpeedScale > 1) {
                throw new IllegalStateException("minimumSpeedScale must be in [0, 1] (got "
                        + minimumSpeedScale + ")");
            }
            if (accelerationCurrentPerMpsSq < 0) {
                throw new IllegalStateException("accelerationCurrentPerMpsSq must be >= 0 (got "
                        + accelerationCurrentPerMpsSq + ")");
            }
            return new PhysicsConstraints(this);
        }
    }
}
