package frc.lib.catalyst.physics.constraints;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.command3.Trigger;

import frc.lib.catalyst.physics.LocalizationQuality;
import frc.lib.catalyst.physics.PhysicalRobotState;
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
 * The acceleration limit ({@link #maxAccelerationMpsSq()}) takes the tightest of traction, tipping
 * and power. The speed scale ({@link #speedScale()}) takes the tightest of two physical limits:
 *
 * <ul>
 *   <li><b>Slip</b> — wheels already breaking traction will not deliver more; asking for it only
 *       makes the odometry worse. Engages above 20% module slip and lets go below 10%.</li>
 *   <li><b>Power</b> — an accelerating drivetrain is the largest load on the robot, and there is no
 *       point commanding an acceleration the battery cannot supply.</li>
 * </ul>
 *
 * <h2>What the speed scale does not do</h2>
 * It does not slow the robot because the <em>pose</em> is uncertain. Localisation confidence says how
 * far to trust {@code Pose2d}; it says nothing about whether the robot can safely go fast, and a
 * driver steering by eye does not use the pose at all. Through 2.0.0-rc.1 it was folded in, in
 * bands — 75%, 45%, 25% of top speed — and on a real robot (Catalyst X1, 2026-09-19) that stepped
 * the driver's top speed down as often as 30 times a minute: whenever the camera had been off the tags
 * for 1.8 s, and whenever the driver pushed the stick hard enough for the wheels and the IMU to
 * disagree about the acceleration. Each step decelerated the robot hard and each recovery
 * re-accelerated it, which drives like a gearbox shifting. Confidence is now its own number,
 * {@link #confidenceScale()}, for code that drives <em>on the pose</em> — an auto-align, a path
 * follower — to apply if it chooses.
 *
 * <h2>Ease in, hold, ease out</h2>
 * A limit on a driver's speed must never be a step. {@link #speedScale()} moves toward the limit at no
 * more than {@link Builder#easing(double, double, double) a set rate}, holds the reduction for a
 * moment after the cause clears, then gives the speed back more slowly still. A limit that flickers —
 * one slip reading at the start of an acceleration — costs a few percent for a moment instead of a
 * lurch, and a real one still arrives within a fraction of a second. {@link #targetSpeedScale()} is the
 * instantaneous limit, for logging.
 *
 * <p>{@link #explain()} names which limit is binding, so a slowdown is always attributable.
 *
 * @since 1.6.0
 */
public final class PhysicsConstraints {

    /** Module slip above which the slip limit engages. */
    private static final double SLIP_ENGAGE = 0.2;
    /**
     * Module slip below which an engaged slip limit lets go. The gap between the two is the hysteresis:
     * a slip factor hovering at the threshold holds one decision instead of toggling it every loop.
     */
    private static final double SLIP_RELEASE = 0.1;
    /** Speed scale while over the electrical budget. */
    private static final double OVER_BUDGET_SCALE = 0.6;
    /** Confidence from which {@link #confidenceScale()} is 1: the edge of {@code HIGH}. */
    private static final double CONFIDENT = 0.75;
    /** Confidence at which {@link #confidenceScale()} reaches its floor: the edge of {@code LOST}. */
    private static final double UNCONFIDENT = 0.15;
    /**
     * A gap between Physics Core samples longer than this is not eased across. Nobody was driving
     * through it — the robot was disabled, or the loop stalled — so the scale starts again at the limit.
     */
    private static final double MAX_EASING_GAP_SECONDS = 0.5;

    private final PhysicsCore physics;
    private final StabilityModel stability;
    private final PowerPredictor power;
    private final double maxSpeedMps;
    private final double usableFraction;
    private final double minimumSpeedScale;
    private final double accelerationCurrentPerMpsSq;
    private final boolean confidenceScalingEnabled;
    private final Easing speedEasing;
    private final Easing confidenceEasing;

    /** The Physics Core state the cached values below were computed from. */
    private PhysicalRobotState evaluated = null;
    private boolean slipLimiting = false;
    private double target = 1.0;
    private String targetReason = null;
    /** The last reason that held the speed down, kept so a scale still easing back names its cause. */
    private String lastReason = null;

    private PhysicsConstraints(Builder builder) {
        this.physics = builder.physics;
        this.stability = builder.stability;
        this.power = builder.power;
        this.maxSpeedMps = builder.maxSpeedMps;
        this.usableFraction = builder.usableFraction;
        this.minimumSpeedScale = builder.minimumSpeedScale;
        this.accelerationCurrentPerMpsSq = builder.accelerationCurrentPerMpsSq;
        this.confidenceScalingEnabled = builder.confidenceScalingEnabled;
        this.speedEasing =
                new Easing(builder.easeInPerSecond, builder.holdSeconds, builder.easeOutPerSecond);
        this.confidenceEasing =
                new Easing(builder.easeInPerSecond, builder.holdSeconds, builder.easeOutPerSecond);
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
     * A multiplier from {@code 0} to {@code 1} for the drivetrain's commanded speed, from the physical
     * limits only — slip and electrical headroom. Feeds {@code SwerveSubsystem.setSpeedMultiplier(...)}
     * directly, which is why it is eased: it moves toward {@link #targetSpeedScale()} at no more than the
     * configured rate, holds a reduction briefly after its cause clears, then recovers more slowly.
     * See the class notes.
     *
     * <p>Localisation confidence is not part of it; see {@link #confidenceScale()}.
     *
     * <p>Advances once per Physics Core sample, however many times it is called in a loop, so reading
     * it from several places costs nothing and changes nothing. Never below
     * {@link Builder#minimumSpeedScale(double)}.
     */
    public double speedScale() {
        refresh();
        return speedEasing.value();
    }

    /**
     * The instantaneous physical limit that {@link #speedScale()} eases toward. For logging next to
     * the applied scale; not for the drivetrain, where its steps are exactly what the easing is there
     * to remove.
     */
    public double targetSpeedScale() {
        refresh();
        return target;
    }

    /**
     * A multiplier from localisation confidence, for motion driven <em>by the pose</em>: an auto-align,
     * a path follower, a drive-to-point. A robot unsure where it is should approach a scoring position
     * cautiously, and a controller steering on that pose is the thing that needs to slow down.
     *
     * <p>Continuous rather than banded — 1 at {@link LocalizationQuality.Level#HIGH} confidence,
     * falling linearly to {@link Builder#minimumSpeedScale(double)} at the edge of
     * {@link LocalizationQuality.Level#LOST} — and eased the same way as {@link #speedScale()}.
     *
     * <p>Never apply it to a driver's sticks. A driver steers by eye, not by the pose, and on X1 on
     * 2026-09-19 its predecessor inside {@code speedScale()} fired whenever a tag had been out of view
     * for 1.8 s and whenever an acceleration made the wheels and the IMU disagree:
     *
     * <pre>{@code
     * double autoScale = Math.min(limits.speedScale(), limits.confidenceScale());
     * }</pre>
     *
     * <p>Always 1 when built with {@link Builder#withoutConfidenceScaling()}.
     */
    public double confidenceScale() {
        refresh();
        return confidenceScalingEnabled ? confidenceEasing.value() : 1.0;
    }

    /** The speed cap in m/s: {@link #speedScale()} applied to the configured maximum. */
    public double maxSpeedMps() {
        return maxSpeedMps * speedScale();
    }

    /**
     * Recompute the limits and advance the easing, once per Physics Core sample. Keyed to the sample
     * rather than to calls, so {@link #explain()}, {@link #isCautionAdvised()} and the robot's
     * own read all see the same value and none of them moves it on.
     */
    private void refresh() {
        PhysicalRobotState state = physics.state();
        if (state == evaluated) return;
        evaluated = state;
        double now = state.timestampSeconds();

        double limit = 1.0;
        String why = null;
        // Wheels that are already slipping will not deliver more, so asking for it only degrades the
        // odometry further. Hysteresis, so a slip factor sitting on the threshold holds one decision.
        double slip = physics.analyze().slipFactor();
        slipLimiting = slipLimiting ? slip > SLIP_RELEASE : slip > SLIP_ENGAGE;
        if (slipLimiting) {
            limit = 1.0 - 0.5 * Math.min(1.0, slip);
            why = String.format(Locale.ROOT, "wheel slip %.0f%%", slip * 100);
        }
        if (power != null && power.headroomAmps() < 0 && OVER_BUDGET_SCALE < limit) {
            limit = OVER_BUDGET_SCALE;
            why = String.format(Locale.ROOT, "over the current budget by %.0f A", -power.headroomAmps());
        }
        target = clampScale(limit);
        targetReason = why;
        if (why != null) lastReason = why;
        speedEasing.step(target, now);

        double confidence = state.quality().confidence();
        double fraction = (confidence - UNCONFIDENT) / (CONFIDENT - UNCONFIDENT);
        confidenceEasing.step(clampScale(minimumSpeedScale + (1.0 - minimumSpeedScale) * fraction), now);
    }

    private double clampScale(double scale) {
        return Math.max(minimumSpeedScale, Math.min(1.0, scale));
    }

    /**
     * Ease in, hold, ease out: the only shape an intervention on a driver's speed is allowed to have.
     *
     * <p>Falls toward a lower target at no more than {@code fallPerSecond}. Once the target rises
     * again, holds where it is until the target has stayed above it for {@code holdSeconds}, then rises
     * at no more than {@code risePerSecond}. The first sample, a clock that went backwards (a reset)
     * and a gap longer than {@link #MAX_EASING_GAP_SECONDS} all start again at the target, because
     * nobody was driving through them.
     */
    static final class Easing {
        private final double fallPerSecond;
        private final double holdSeconds;
        private final double risePerSecond;
        private double value = 1.0;
        private double lastTime = Double.NaN;
        /** When the target last asked for this much reduction or more; NaN when it never has. */
        private double lastAskedAt = Double.NaN;

        Easing(double fallPerSecond, double holdSeconds, double risePerSecond) {
            this.fallPerSecond = fallPerSecond;
            this.holdSeconds = holdSeconds;
            this.risePerSecond = risePerSecond;
        }

        double step(double target, double now) {
            if (now == lastTime) return value;
            if (Double.isNaN(lastTime) || !(now > lastTime) || now - lastTime > MAX_EASING_GAP_SECONDS) {
                value = target;
                lastTime = now;
                lastAskedAt = target < 1.0 ? now : Double.NaN;
                return value;
            }
            double dt = now - lastTime;
            lastTime = now;
            if (target <= value) {
                value = Math.max(target, value - fallPerSecond * dt);
                if (target < 1.0) lastAskedAt = now;
            } else if (Double.isNaN(lastAskedAt) || now - lastAskedAt >= holdSeconds) {
                value = Math.min(target, value + risePerSecond * dt);
            }
            return value;
        }

        double value() {
            return value;
        }
    }

    // ===========================================
    //   Conditions — plain predicates, then Triggers
    // ===========================================
    //
    // Each condition comes in two forms. The predicate is pure and can be called anywhere, including
    // a unit test; the Trigger wraps it for robot code. Constructing a WPILib Trigger reaches into
    // the command scheduler, so keeping the logic in the predicate is what lets all of this be tested
    // with no HAL.

    /**
     * True while the physical limits are holding the speed down — slip or electrical headroom. Low
     * localisation confidence alone does not count; see {@link #isLocalizationLost()}.
     */
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
     * A trigger that fires while the physical limits are holding the speed down — slipping or out of
     * electrical headroom. Composes with the WPILib bindings you already use, for a rumble or a light;
     * to slow the robot, apply {@link #speedScale()}, which is eased, rather than a fixed multiplier
     * switched on and off by this.
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
        refresh();
        List<String> reasons = new ArrayList<>(3);
        if (targetReason != null) reasons.add(targetReason);
        if (stability != null) {
            double margin = stability.tipMarginMeters(physics.state().fieldAcceleration(),
                    physics.state().pose().getRotation());
            if (margin < 0.12) reasons.add(String.format(Locale.ROOT, "tip margin %.0f mm", margin * 1000));
        }

        double scale = speedScale();
        String text;
        if (!reasons.isEmpty()) {
            text = String.format(Locale.ROOT,
                    "speed scaled to %.0f%% (limit %.0f%%), accel capped at %.1f m/s^2: %s", scale * 100, target * 100, maxAccelerationMpsSq(), String.join("; ", reasons));
        } else if (scale < 1.0) {
            text = String.format(Locale.ROOT, "speed easing back to full, now %.0f%%, after %s",
                    scale * 100, lastReason == null ? "a limit" : lastReason);
        } else {
            text = "no limit binding - full speed and acceleration available";
        }

        // Reported, never folded into the speed scale: see confidenceScale().
        LocalizationQuality quality = physics.state().quality();
        if (confidenceScalingEnabled && quality.level() != LocalizationQuality.Level.HIGH) {
            text += String.format(Locale.ROOT, "; localisation confidence %s (%s): confidenceScale %.0f%% "
                    + "for pose-driven motion, not applied to the speed scale",
                    quality.level(), quality.reason(), confidenceScale() * 100);
        }
        return text;
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
        private double easeInPerSecond = 1.0;
        private double holdSeconds = 0.5;
        private double easeOutPerSecond = 0.5;

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
         * The lowest {@link PhysicsConstraints#speedScale()} and
         * {@link PhysicsConstraints#confidenceScale()} will ever return, in {@code [0, 1]}. Defaults to
         * 0.25. Setting this to zero means a lost estimate stops pose-driven motion dead, which is a
         * real choice but rarely the right one in a match.
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

        /**
         * Pin {@link PhysicsConstraints#confidenceScale()} at 1 and leave confidence out of
         * {@link PhysicsConstraints#explain()}.
         *
         * <p>It used to be how a team kept confidence out of {@link PhysicsConstraints#speedScale()}.
         * Confidence is no longer in the speed scale at all, so a robot that called this for that
         * reason can stop.
         */
        public Builder withoutConfidenceScaling() {
            this.confidenceScalingEnabled = false;
            return this;
        }

        /**
         * How the speed scale moves: it falls toward a tighter limit at no more than
         * {@code inPerSecond}, holds for {@code holdSeconds} after the limit has eased, then recovers
         * at no more than {@code outPerSecond}. Defaults to 1.0 per second in, 0.5 s held, 0.5 per
         * second out — a limit that takes the scale to 60% arrives in 0.4 s and is fully given
         * back 1.3 s after its cause clears.
         *
         * <p>Recovering slower than it falls is deliberate: speed given back too eagerly re-provokes
         * whatever took it away, and a scale that bounces between the two is the lurch this exists to
         * prevent.
         *
         * @param inPerSecond  largest fall in the scale per second, {@code > 0}
         * @param holdSeconds  how long a reduction is held once its cause clears, {@code >= 0}
         * @param outPerSecond largest rise in the scale per second, {@code > 0}
         */
        public Builder easing(double inPerSecond, double holdSeconds, double outPerSecond) {
            this.easeInPerSecond = inPerSecond;
            this.holdSeconds = holdSeconds;
            this.easeOutPerSecond = outPerSecond;
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
            if (!(easeInPerSecond > 0) || !(easeOutPerSecond > 0) || !(holdSeconds >= 0)) {
                throw new IllegalStateException("easing needs rates > 0 and a hold >= 0 (got in "
                        + easeInPerSecond + "/s, hold " + holdSeconds + " s, out " + easeOutPerSecond
                        + "/s) - a zero rate would freeze the scale where it is");
            }
            return new PhysicsConstraints(this);
        }
    }
}
