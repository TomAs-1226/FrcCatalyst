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
 * <p>A wheel that carries little of the robot's weight spins in place on a few amps while the others still hold.
 * That is not the robot rolling, and its current is not a loaded wheel's slip current, so such a wheel is noted as
 * light and the ramp goes on to the first wheel that breaks loose under load. Team 5805's X1 measured this against
 * a wall on 2026-09-19: its front right spun on 2 A, its front left and back left on 12 A, and its back right held
 * past 25 A. Only every wheel turning on a few amps is the robot rolling - a rolling robot turns them all with the
 * floor, its loaded ones at the robot's own speed.
 *
 * <p>Each current is read through the median of three loops, and a wheel's break current is its highest over the
 * last {@link #WINDOW_LOOPS} loops, so neither a one-loop spike nor a transient at the start passes for a slip.
 *
 * <p>Plain doubles, one call a loop: {@link #step} takes what the drive motors measure and returns the voltage to
 * apply next. It stops - asking for 0 V - when a wheel slips under load, when every wheel turns on a few amps (the
 * robot is rolling free: not against a wall), at its voltage cap, or after its timeout.
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
     * Below this stator current a turning wheel is not gripping under load, A: rolling takes a few amps, and no
     * swerve wheel that carries its share of the robot breaks traction on carpet on so little.
     */
    static final double HELD_AMPS = 15.0;
    /** How far back a wheel's break current is looked for, in loops: 0.3 s at 50 Hz. */
    static final int WINDOW_LOOPS = 15;

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
    private double[][] lastThree;
    private double[][] window;
    private double[] amps;
    private double[] breakAmps;
    private int loops;
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
        int n = wheelMps.length;
        if (Double.isNaN(t0)) {
            t0 = now;
            lastThree = new double[n][3];
            window = new double[n][WINDOW_LOOPS];
            amps = new double[n];
            breakAmps = new double[n];
            java.util.Arrays.fill(breakAmps, Double.NaN);
        }
        for (int i = 0; i < n; i++) {
            double a = Math.abs(statorAmps[i]);
            if (!Double.isFinite(wheelMps[i]) || !Double.isFinite(a)) {
                return stop("a drive motor reported no reading");
            }
            lastThree[i][loops % 3] = a;
            amps[i] = median(lastThree[i]);
            window[i][loops % WINDOW_LOOPS] = amps[i];
            peakAmps = Math.max(peakAmps, amps[i]);
        }
        loops++;

        // Held, a wheel's current only climbs with the voltage; once it slips it falls. So its highest in the
        // window is the current it broke loose at, however the slip began within a loop.
        for (int i = 0; i < n; i++) {
            if (Double.isNaN(breakAmps[i]) && Math.abs(wheelMps[i]) > SLIP_MPS) {
                breakAmps[i] = java.util.Arrays.stream(window[i]).max().orElse(0.0);
            }
        }
        if (lightWheels().length == n) {
            return stop(String.format(
                    "every wheel turned on under %.0f A: the robot is rolling, not held by a wall", HELD_AMPS));
        }
        int first = -1;
        for (int i = 0; i < n; i++) {
            if (breakAmps[i] >= HELD_AMPS && (first < 0 || breakAmps[i] < breakAmps[first])) {
                first = i;
            }
        }
        if (first >= 0) {
            slipAmps = breakAmps[first];
            slipWheel = first;
            phase = Phase.SLIPPED;
            volts = 0.0;
            return 0.0;
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

    private static double median(double[] three) {
        return Math.max(Math.min(three[0], three[1]), Math.min(Math.max(three[0], three[1]), three[2]));
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

    /** Each wheel's current when it first turned faster than {@link #SLIP_MPS}, A; NaN for one that has not. */
    double[] breakAmps() {
        return breakAmps == null ? new double[0] : breakAmps.clone();
    }

    /** The wheels that turned on under {@link #HELD_AMPS}: light ones while the others held, in module order. */
    int[] lightWheels() {
        if (breakAmps == null) {
            return new int[0];
        }
        // NaN compares false: a wheel that has not turned is not light.
        return java.util.stream.IntStream.range(0, breakAmps.length).filter(i -> breakAmps[i] < HELD_AMPS).toArray();
    }

    /** Each drive motor's current this loop, through the median of three, A. */
    double[] wheelAmps() {
        return amps == null ? new double[0] : amps.clone();
    }

    /** The stator current the first wheel to slip under load carried just before it did, A; NaN until one has. */
    double slipAmps() {
        return slipAmps;
    }

    /** Which wheel slipped first under load, in the drivetrain's module order; -1 until one has. */
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
     * The limit to set: the measured slip current, rounded down to 5 A, so every loaded wheel stays under the
     * first one to slip.
     */
    double recommendedAmps() {
        return Double.isNaN(slipAmps) ? Double.NaN : Math.floor(slipAmps / 5.0) * 5.0;
    }
}
