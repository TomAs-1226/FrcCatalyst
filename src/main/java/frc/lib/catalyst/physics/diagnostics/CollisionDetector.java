package frc.lib.catalyst.physics.diagnostics;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.lib.catalyst.physics.estimation.DisturbanceEstimator;

/**
 * Decides when the unexplained acceleration on the robot has become an impact.
 *
 * <p>{@link DisturbanceEstimator} produces a continuous residual — the acceleration the wheels cannot
 * account for. This turns that into discrete events, which is what the rest of the robot can act on:
 * an auto routine cannot subscribe to "0.42 of traction limit", but it can react to "you were just
 * hit".
 *
 * <p>Three rules keep it from firing on noise:
 *
 * <ul>
 *   <li><b>Threshold</b> — the residual has to exceed a fraction of the traction limit. Defaults to
 *       0.7: an acceleration the drivetrain could not have produced even at full grip.</li>
 *   <li><b>Persistence</b> — it has to stay there for a couple of loops. A single frame over
 *       threshold is far more likely to be a carpet seam or a noisy accelerometer sample than
 *       another robot.</li>
 *   <li><b>Refractory period</b> — after firing, it stays quiet briefly. One collision rings the
 *       chassis for several loops and should be reported once, not six times.</li>
 * </ul>
 *
 * <p>While an impact is in progress the detector tracks the peak, so the event it finally emits
 * describes the worst of the hit rather than whichever loop happened to cross the threshold first.
 *
 * <p>The clock is a parameter, so this is fully unit testable with no HAL and no robot.
 *
 * @since 1.5.0
 */
public final class CollisionDetector {

    private final DisturbanceEstimator disturbance;
    private final double thresholdFraction;
    private final int requiredLoops;
    private final double refractorySeconds;

    private int consecutiveLoops = 0;
    private double peakMagnitude = 0.0;
    private Rotation2d peakDirection = Rotation2d.kZero;
    private CollisionEvent lastEvent = null;
    private double lastEventTimestamp = Double.NEGATIVE_INFINITY;

    private CollisionDetector(Builder builder) {
        this.disturbance = builder.disturbance;
        this.thresholdFraction = builder.thresholdFraction;
        this.requiredLoops = builder.requiredLoops;
        this.refractorySeconds = builder.refractorySeconds;
    }

    /**
     * Evaluate the current disturbance. Call once per loop, after updating the
     * {@link DisturbanceEstimator}.
     *
     * @param timestampSeconds current FPGA-clock time
     * @return the event, on the loop it fires; empty otherwise
     */
    public Optional<CollisionEvent> update(double timestampSeconds) {
        // Threshold on the part of the residual that slip cannot explain, not on the raw magnitude.
        // Hard acceleration that breaks traction produces just as large a residual as being hit, and
        // reporting that as a collision means the detector fires every time the robot launches off
        // the line - which it did, until the simulation validation caught it.
        double normalized = disturbance.impactEvidence();
        boolean overThreshold = normalized >= thresholdFraction;
        boolean inRefractory = timestampSeconds - lastEventTimestamp < refractorySeconds;

        if (!overThreshold) {
            consecutiveLoops = 0;
            peakMagnitude = 0.0;
            return Optional.empty();
        }

        // Track the peak of this impact even while suppressed, so a report that fires after the
        // refractory period describes the real worst case rather than its tail.
        if (disturbance.magnitudeMpsSq() > peakMagnitude) {
            peakMagnitude = disturbance.magnitudeMpsSq();
            peakDirection = disturbance.direction();
        }
        consecutiveLoops++;

        if (consecutiveLoops < requiredLoops || inRefractory) return Optional.empty();

        CollisionEvent event = new CollisionEvent(
                timestampSeconds,
                peakMagnitude,
                peakDirection,
                peakMagnitude * disturbance.drivetrain().robot().massKg());
        lastEvent = event;
        lastEventTimestamp = timestampSeconds;
        consecutiveLoops = 0;
        peakMagnitude = 0.0;
        return Optional.of(event);
    }

    /** True while the residual is over threshold — an impact is happening right now. */
    public boolean isImpactInProgress() {
        return consecutiveLoops > 0;
    }

    /** The most recent collision, or empty if none has been detected since the last {@link #reset()}. */
    public Optional<CollisionEvent> lastEvent() {
        return Optional.ofNullable(lastEvent);
    }

    /**
     * Seconds since the last collision, or {@link Double#POSITIVE_INFINITY} if there has not been one.
     * The usual form for "have we been hit recently?".
     */
    public double secondsSinceLastEvent(double timestampSeconds) {
        if (lastEvent == null) return Double.POSITIVE_INFINITY;
        return timestampSeconds - lastEventTimestamp;
    }

    /** Clear the detector's history and any impact in progress. */
    public void reset() {
        consecutiveLoops = 0;
        peakMagnitude = 0.0;
        peakDirection = Rotation2d.kZero;
        lastEvent = null;
        lastEventTimestamp = Double.NEGATIVE_INFINITY;
    }

    /** Start building a detector. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link CollisionDetector}. */
    public static final class Builder {
        private DisturbanceEstimator disturbance;
        private double thresholdFraction = 0.7;
        private int requiredLoops = 2;
        private double refractorySeconds = 0.5;

        /** The disturbance estimator to watch. Required. */
        public Builder disturbance(DisturbanceEstimator disturbance) {
            this.disturbance = disturbance;
            return this;
        }

        /**
         * Fraction of the traction limit the <em>unexplained-by-slip</em> acceleration must exceed.
         * Defaults to 0.7. Lower it to catch gentler contact at the cost of firing on rough carpet.
         *
         * @see frc.lib.catalyst.physics.estimation.DisturbanceEstimator#impactEvidence()
         */
        public Builder threshold(double thresholdFraction) {
            this.thresholdFraction = thresholdFraction;
            return this;
        }

        /** Consecutive loops over threshold before firing. Defaults to 2 — about 40 ms at 50 Hz. */
        public Builder requiredLoops(int requiredLoops) {
            this.requiredLoops = requiredLoops;
            return this;
        }

        /** Quiet period after an event, in seconds. Defaults to 0.5. */
        public Builder refractoryPeriod(double refractorySeconds) {
            this.refractorySeconds = refractorySeconds;
            return this;
        }

        /** Validate and build. */
        public CollisionDetector build() {
            if (disturbance == null) {
                throw new IllegalStateException("a DisturbanceEstimator is required - collisions are "
                        + "detected from the acceleration it cannot explain");
            }
            if (!(thresholdFraction > 0)) {
                throw new IllegalStateException("threshold must be > 0 (got " + thresholdFraction + ")");
            }
            if (requiredLoops < 1) {
                throw new IllegalStateException("requiredLoops must be >= 1 (got " + requiredLoops + ")");
            }
            if (refractorySeconds < 0) {
                throw new IllegalStateException("refractoryPeriod must be >= 0 (got " + refractorySeconds + ")");
            }
            return new CollisionDetector(this);
        }
    }
}
