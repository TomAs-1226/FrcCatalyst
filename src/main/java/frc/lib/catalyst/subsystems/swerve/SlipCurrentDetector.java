package frc.lib.catalyst.subsystems.swerve;

/**
 * Finds the drive current at which a swerve robot's wheels break traction, the way CTRE describes measuring Phoenix's
 * slip current: the robot's front bumper against a wall, the modules straight, the drive voltage ramped slowly.
 * Held by the wall, the wheels cannot turn while they grip, so the current climbs with the voltage and the wheel
 * speed stays near zero; the moment they slip, they spin up. The current just before that is the slip current,
 * and Phoenix limits the drive's stator current to it ({@code kSlipCurrent}), so the wheels never spin on the
 * carpet - which a light robot on a copied 120 A does, and which odometry reads as motion. The pure half of
 * {@link SlipCurrentCalibration}; ported from team 5805's Catalyst X1.
 *
 * <p>Plain doubles, one call a loop: {@link #step} takes what the drive motors measure and returns the voltage to
 * apply next. It stops - asking for 0 V - when a wheel slips, when a wheel turns before the current says the robot
 * is held (it is rolling free: not against a wall), at its voltage cap, or after its timeout.
 */
final class SlipCurrentDetector {

    /** The default ramp, volts per second: slow enough that the current follows the voltage, not the rotor's inertia. */
    static final double RAMP_V_PER_S = 0.5;
    /** The default cap. A light robot slips far below this; one that does not is not held by a wall. */
    static final double MAX_VOLTS = 6.0;

    private final double rampVoltsPerSecond;
    private final double maxVolts;
    private final double timeoutSeconds;
    /** A wheel surface this fast while the robot is held is slipping, m/s. */
    static final double SLIP_MPS = 0.3;
    /**
     * Below this stator current a turning wheel means the robot is rolling, not slipping, A: rolling takes a few
     * amps, and no swerve wheel on carpet breaks traction on so little.
     */
    static final double HELD_AMPS = 15.0;

    enum Phase { RAMPING, SLIPPED, STOPPED }

    SlipCurrentDetector() {
        this(RAMP_V_PER_S, MAX_VOLTS);
    }

    /** @param rampVoltsPerSecond clamped to 0.1-2; @param maxVolts clamped to 1-8 */
    SlipCurrentDetector(double rampVoltsPerSecond, double maxVolts) {
        this.rampVoltsPerSecond = Double.isFinite(rampVoltsPerSecond) ? Math.max(0.1, Math.min(2.0, rampVoltsPerSecond)) : RAMP_V_PER_S;
        this.maxVolts = Double.isFinite(maxVolts) ? Math.max(1.0, Math.min(8.0, maxVolts)) : MAX_VOLTS;
        // Time to reach the cap at the ramp, and half as long again.
        this.timeoutSeconds = 1.5 * this.maxVolts / this.rampVoltsPerSecond;
    }

    private Phase phase = Phase.RAMPING;
    private double t0 = Double.NaN;
    private double volts;
    private double[] wheelPeak;
    private double slipAmps = Double.NaN;
    private int slipWheel = -1;
    private double peakAmps;
    private String reason = "";

    /**
     * One loop.
     *
     * @param now        seconds
     * @param wheelMps   each drive wheel's surface speed, m/s (any sign)
     * @param statorAmps each drive motor's stator current, A (any sign)
     * @return the drive voltage to apply until the next call
     */
    double step(double now, double[] wheelMps, double[] statorAmps) {
        if (phase != Phase.RAMPING) {
            return 0.0;
        }
        if (Double.isNaN(t0)) {
            t0 = now;
            wheelPeak = new double[wheelMps.length];
        }
        for (int i = 0; i < wheelMps.length; i++) {
            double amps = Math.abs(statorAmps[i]);
            if (!Double.isFinite(wheelMps[i]) || !Double.isFinite(amps)) {
                return stop("a drive motor reported no reading");
            }
            // Held, a wheel's current only climbs with the voltage; once it slips it falls. So its peak so far is
            // the current it broke loose at, however the slip began within a loop.
            wheelPeak[i] = Math.max(wheelPeak[i], amps);
            peakAmps = Math.max(peakAmps, amps);
        }
        for (int i = 0; i < wheelMps.length; i++) {
            if (Math.abs(wheelMps[i]) > SLIP_MPS) {
                double before = wheelPeak[i];
                if (before < HELD_AMPS) {
                    return stop(String.format("a wheel turned at %.0f A: the robot is rolling, not held by a wall", before));
                }
                slipAmps = before;
                slipWheel = i;
                phase = Phase.SLIPPED;
                volts = 0.0;
                return 0.0;
            }
        }
        if (now - t0 > timeoutSeconds) {
            return stop(String.format("timed out at %.1f V", volts));
        }
        volts = Math.min(maxVolts, rampVoltsPerSecond * (now - t0));
        if (volts >= maxVolts) {
            return stop(String.format("no slip by %.0f V (%.0f A): is the robot against the wall?", maxVolts, peakAmps));
        }
        return volts;
    }

    private double stop(String why) {
        phase = Phase.STOPPED;
        reason = why;
        volts = 0.0;
        return 0.0;
    }

    Phase phase() {
        return phase;
    }

    boolean done() {
        return phase != Phase.RAMPING;
    }

    /** The stator current the first wheel to slip carried just before it did, A; NaN until one has. */
    double slipAmps() {
        return slipAmps;
    }

    /** Which wheel slipped first, in the drivetrain's module order; -1 until one has. */
    int slipWheel() {
        return slipWheel;
    }

    /** The most current any drive motor has carried so far, A. */
    double peakAmps() {
        return peakAmps;
    }

    /** The voltage being applied. */
    double volts() {
        return volts;
    }

    /** Why it stopped without a result; empty otherwise. */
    String reason() {
        return reason;
    }

    /**
     * The limit to set: the measured slip current, rounded down to 5 A, so every wheel stays under the first one
     * to slip.
     */
    double recommendedAmps() {
        return Double.isNaN(slipAmps) ? Double.NaN : Math.floor(slipAmps / 5.0) * 5.0;
    }
}
