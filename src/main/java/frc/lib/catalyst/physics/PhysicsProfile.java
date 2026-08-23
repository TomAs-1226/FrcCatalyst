package frc.lib.catalyst.physics;

/**
 * How much computation Physics Core is allowed to spend per loop.
 *
 * <p>The loop has a 20 ms budget shared with everything else on the robot. Rather than hard-coding
 * behaviour to a controller name — which ages badly the moment the control system changes, as it
 * just did — Physics Core is told what it may spend and enables the services that fit.
 *
 * <p>Every profile is safe on any supported controller. {@link #BALANCED}, the default, is what
 * Phase 1 was designed and measured against; {@link #MINIMAL} exists for a robot whose loop is
 * already tight.
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
     * Everything in {@link #BALANCED}. Reserved for fixed-lag smoothing, delayed-measurement replay
     * and short-window optimisation. <b>Behaves identically to {@code BALANCED} today, deliberately.</b>
     *
     * <p>Both things this was waiting for have now arrived, and it is worth recording precisely what
     * they are so the next person does not have to rediscover them:
     *
     * <ul>
     *   <li><b>The compute.</b> Systemcore is a Compute Module 5 — four Cortex-A76 cores on a
     *       {@code PREEMPT_RT} kernel, with the robot program started at real-time priority. This
     *       profile was written for "if a coprocessor arrives". It has.</li>
     *   <li><b>The solver.</b> WPILib 2027 ships Java bindings for Sleipnir:
     *       {@code org.wpilib.math.autodiff} ({@code Variable}, {@code Gradient}, {@code Hessian},
     *       {@code Jacobian}) and {@code org.wpilib.math.optimization} ({@code Problem},
     *       {@code OCP}, {@code Constraints}). Moving-horizon estimation and fixed-lag smoothing no
     *       longer need a hand-rolled optimiser.</li>
     * </ul>
     *
     * <p><b>So why is this still a no-op?</b> Because Physics Core has never run on a robot. Its
     * existing estimator is simulation-validated and nothing more, and the roadmap gates the entire
     * physics track on carpet validation for a good reason: a simulation assumes constant friction,
     * rigid contact and uniform slip, and a real field is messier than all three. Layering an
     * unvalidated nonlinear estimator on top of an unvalidated linear one would produce a number
     * that is harder to check and no more trustworthy.
     *
     * <p>The order is: run {@code BALANCED} in shadow mode on carpet, confirm sensor disagreement
     * sits near zero and the identifiers converge, and only then decide what this profile should
     * switch on. Selecting it today is a statement of intent that costs nothing and changes nothing.
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
