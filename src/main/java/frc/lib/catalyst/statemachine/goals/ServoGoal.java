package frc.lib.catalyst.statemachine.goals;

/**
 * A goal for a {@link frc.lib.catalyst.mechanisms.ServoMechanism} in a state machine: an angle to
 * hold, or a named preset.
 *
 * <p>A servo is open-loop — there is no encoder to prove it arrived — so a state carrying a servo
 * goal counts the servo "arrived" once a short {@code settleSeconds} window has elapsed. That is the
 * honest way to gate a transition on a mechanism you cannot actually sense; the binding marks itself
 * {@code observable == false} so a log reader knows the arrival was a timer, not a measurement.
 *
 * <p>Value-equal (it is a {@code record}), so the engine's per-loop goal comparison is cheap. A
 * {@code preset} goal carries a non-null preset name and its {@code degrees} is resolved from the
 * mechanism's named positions at build time.
 *
 * @param degrees       target angle in degrees (resolved from {@code preset} at build if that is set)
 * @param settleSeconds seconds to wait after commanding before the goal counts as reached
 * @param preset        a named-position key to resolve, or {@code null} for a raw angle
 * @since 1.3.0
 */
public record ServoGoal(double degrees, double settleSeconds, String preset) {

    /** Default settle time — long enough for a typical hobby servo to complete a modest move. */
    public static final double DEFAULT_SETTLE_SECONDS = 0.35;

    /** Clamps a negative settle time to zero so a bad value can never mean "never arrives". */
    public ServoGoal {
        if (settleSeconds < 0 || Double.isNaN(settleSeconds)) settleSeconds = DEFAULT_SETTLE_SECONDS;
    }

    /** Drive to {@code degrees}, arriving after the default settle time. */
    public static ServoGoal degrees(double degrees) {
        return new ServoGoal(degrees, DEFAULT_SETTLE_SECONDS, null);
    }

    /** Drive to {@code degrees}, arriving after {@code settleSeconds}. */
    public static ServoGoal degrees(double degrees, double settleSeconds) {
        return new ServoGoal(degrees, settleSeconds, null);
    }

    /** Drive to a named preset (resolved from the mechanism at build time). */
    public static ServoGoal preset(String name) {
        return new ServoGoal(Double.NaN, DEFAULT_SETTLE_SECONDS, name);
    }

    /** Drive to a named preset with an explicit settle time. */
    public static ServoGoal preset(String name, double settleSeconds) {
        return new ServoGoal(Double.NaN, settleSeconds, name);
    }
}
