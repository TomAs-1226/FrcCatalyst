package frc.lib.catalyst.statemachine;

/**
 * What the machine does when a hop blows its deadline in strict mode.
 *
 * <p>The default is deliberately the least dramatic option. Honest arrival detection turns
 * robots that "worked" only because nothing was checking into robots that fault — so the default
 * behaviour on a blown deadline is to keep holding whatever position the mechanisms reached, mark
 * the state unconfirmed, and shout in the log. Nothing is cut, and the robot behaves roughly as it
 * did before; the difference is that the log now says so.
 *
 * @since 1.2.0
 */
public enum FaultPolicy {

    /**
     * Leave every binding pursuing its current goal and report loudly. Motors keep holding.
     * This is the default, and the right choice for an elevator that would otherwise fall.
     */
    HOLD_AND_REPORT,

    /**
     * Release every binding, so each mechanism's {@code GoalRunner} idles and the subsystem is
     * free for a driver command. Appropriate when a stuck mechanism is better left limp.
     */
    RELEASE_ALL,

    /**
     * Immediately request a declared recovery state. Never the default, because unexpected
     * motion after a fault is startling and can chain — a fault <em>during</em> recovery
     * escalates to {@link #HOLD_AND_REPORT} rather than recursing.
     */
    RECOVER_TO
}
