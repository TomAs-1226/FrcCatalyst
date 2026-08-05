package frc.lib.catalyst.physics;

import java.util.Locale;

/**
 * How much you should believe the current state estimate, and why.
 *
 * <p>Most FRC code treats {@code Pose2d} as unquestioned truth. It usually is — right up until a
 * wheel slips, a bumper takes a hit, or the last AprilTag went out of view six seconds ago. This
 * record is the honest answer to "how good is that pose right now?", carried alongside every
 * {@link PhysicalRobotState} so a caller can act on it:
 *
 * <pre>{@code
 * if (physics.state().quality().level() == LocalizationQuality.Level.LOST) {
 *     // stop trusting the pose — fall back to a driver-relative or sensor-relative behaviour
 * }
 * }</pre>
 *
 * <p>{@link #confidence()} is the single number for quick decisions; the standard deviations are
 * the ones to use for real math (gating a shot, sizing a search window). {@link #reason()} is short,
 * stable text meant to be shown to a driver or logged — "vision stale 4.1 s", not a sentence.
 *
 * @param confidence                      0.0 (worthless) to 1.0 (as good as this robot gets)
 * @param translationStdDevMeters         1-sigma position uncertainty, in metres
 * @param rotationStdDevRadians           1-sigma heading uncertainty, in radians
 * @param velocityStdDevMetersPerSecond   1-sigma velocity uncertainty, in metres per second
 * @param secondsSinceAbsoluteFix         age of the last accepted absolute observation (a vision
 *                                        pose), in seconds; large when vision has been unavailable
 * @param reason                          short, stable text describing the dominant limiting factor
 * @since 1.5.0
 */
public record LocalizationQuality(
        double confidence,
        double translationStdDevMeters,
        double rotationStdDevRadians,
        double velocityStdDevMetersPerSecond,
        double secondsSinceAbsoluteFix,
        String reason) {

    /** Coarse confidence bands, for code that wants a switch rather than a threshold. */
    public enum Level {
        /** Trust the estimate for precise work — scoring alignment, a long shot. */
        HIGH,
        /** Usable, but leave margin — slow down for fine alignment. */
        MODERATE,
        /** Directionally right at best. Seek an absolute correction before committing. */
        LOW,
        /** Do not act on the pose. */
        LOST
    }

    /** Compact constructor: clamps confidence to {@code [0, 1]} and never stores a null reason. */
    public LocalizationQuality {
        confidence = Math.max(0.0, Math.min(1.0, confidence));
        reason = reason == null ? "" : reason;
    }

    /** The confidence band this estimate falls into. */
    public Level level() {
        if (confidence >= 0.75) return Level.HIGH;
        if (confidence >= 0.45) return Level.MODERATE;
        if (confidence >= 0.15) return Level.LOW;
        return Level.LOST;
    }

    /** True when {@link #level()} is {@link Level#HIGH} — the usual gate for precision actions. */
    public boolean isTrustworthy() {
        return level() == Level.HIGH;
    }

    /**
     * The quality of a state estimate that has not run yet: zero confidence, effectively infinite
     * uncertainty. Returned by {@link PhysicalRobotState#unknown()} before the first update.
     */
    public static LocalizationQuality unknown() {
        return new LocalizationQuality(0.0, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, "no estimate yet");
    }

    /** One line suitable for a dashboard string or an alert. */
    public String describe() {
        return String.format(Locale.ROOT, "%s (%.0f%%, +/-%.2f m) - %s",
                level(), confidence * 100.0, translationStdDevMeters, reason);
    }
}
