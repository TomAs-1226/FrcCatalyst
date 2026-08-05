package frc.lib.catalyst.physics;

/**
 * How much computation Physics Core is allowed to spend per loop.
 *
 * <p>The roboRIO has a 20 ms budget shared with everything else on the robot, and a coprocessor has
 * considerably more. Rather than hard-coding behaviour to a controller name — which ages badly the
 * moment the control system changes — Physics Core is told what it may spend, and enables the
 * services that fit.
 *
 * <p>Every profile is safe on a roboRIO. {@link #BALANCED}, the default, is what Phase 1 was designed
 * and measured against; {@link #MINIMAL} exists for a robot whose loop is already tight.
 *
 * @since 1.5.0
 */
public enum PhysicsProfile {

    /**
     * State history and confidence only. No slip scoring, no residual monitoring, no collision
     * detection. Costs almost nothing — a fused velocity and a quality number.
     */
    MINIMAL(false, false),

    /**
     * The default, and what the shadow-mode benchmark targets. Adds per-module slip scoring,
     * disturbance residuals, collision detection, and short-horizon prediction. Measured at well
     * under a millisecond per loop on a roboRIO with a four-module swerve.
     */
    BALANCED(true, true),

    /**
     * Everything in {@link #BALANCED}. Reserved for the richer nonlinear estimation and online
     * parameter identification of a later phase; selecting it today behaves identically to
     * {@code BALANCED} and does not silently enable anything unvalidated.
     */
    ADVANCED(true, true),

    /**
     * Everything in {@link #BALANCED}. Reserved for the fixed-lag smoothing, delayed-measurement
     * replay, and short-window optimisation that need a coprocessor. Behaves identically to
     * {@code BALANCED} today.
     */
    SYSTEMCORE(true, true);

    private final boolean slipEnabled;
    private final boolean diagnosticsEnabled;

    PhysicsProfile(boolean slipEnabled, boolean diagnosticsEnabled) {
        this.slipEnabled = slipEnabled;
        this.diagnosticsEnabled = diagnosticsEnabled;
    }

    /** Whether per-module slip scoring runs under this profile. */
    public boolean slipEnabled() {
        return slipEnabled;
    }

    /** Whether disturbance residuals and collision detection run under this profile. */
    public boolean diagnosticsEnabled() {
        return diagnosticsEnabled;
    }
}
