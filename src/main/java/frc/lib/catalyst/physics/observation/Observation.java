package frc.lib.catalyst.physics.observation;

/**
 * One piece of evidence about the robot's physical state, stamped with when it was actually true.
 *
 * <p>Observations are how outside information reaches {@link frc.lib.catalyst.physics.PhysicsCore}:
 * a vision pose, a velocity from an optical-flow sensor, a range to a known wall. They are pushed in
 * through {@code physics.observe(...)} as they arrive, at whatever rate they arrive, which is the
 * point — a 12 Hz camera and a 200 Hz gyro do not have to agree on a schedule.
 *
 * <p>Every observation carries two things Physics Core cannot recover on its own: the instant it
 * describes ({@link #timestampSeconds()}, <b>not</b> the instant it was handed over), and how
 * trustworthy it is ({@link #standardDeviation()}). An observation with an honest standard deviation
 * can be down-weighted; one without has to be either believed completely or thrown away.
 *
 * @since 1.5.0
 */
public interface Observation {

    /**
     * The FPGA-clock instant this observation describes. For a camera this is the capture time, which
     * is typically tens of milliseconds before the pose arrived in robot code.
     */
    double timestampSeconds();

    /**
     * 1-sigma uncertainty of this measurement, in the observation's own units — metres for a pose or
     * a range, metres per second for a velocity. Smaller means "believe me more".
     */
    double standardDeviation();

    /** Short, stable label for logging and diagnostics, e.g. {@code "limelight-front"}. */
    String source();
}
