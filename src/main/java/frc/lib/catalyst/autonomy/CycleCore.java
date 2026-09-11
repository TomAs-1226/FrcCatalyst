package frc.lib.catalyst.autonomy;

/**
 * Which phase of a repeating cycle a co-pilot should be in, and whether it may run it.
 *
 * <p>{@code Autopilot} chose between two phases by reading one boolean, every loop, with no memory:
 * a game-piece sensor that chattered across its threshold flipped acquire and score against each
 * other as fast as the loop ran, and an action whose precondition could never be met was scheduled
 * anyway and repeated forever, so the driver lost the drivetrain until they noticed and let go.
 *
 * <p>This adds the three things that were missing and nothing else:
 *
 * <ul>
 *   <li><b>Dwell.</b> A phase change must be asked for continuously for {@code dwellSeconds} before
 *       it happens. A sensor that chatters no longer thrashes the cycle.
 *   <li><b>Stall.</b> A phase whose action cannot start is reported as stalled rather than run.
 *   <li><b>Handback.</b> After {@code stallHandbackSeconds} of stalling, the answer becomes "give
 *       the robot back to the driver", so an impossible cycle is a pause and not a lockout.
 * </ul>
 *
 * <p>All state is here and all inputs are arguments, so a whole match of cycling can be exercised in
 * a unit test by calling {@link #decide} with a made-up clock. Nothing here touches hardware, the
 * scheduler, or the phases themselves - it only says which one and whether.
 *
 * @since 2.1.0
 */
public final class CycleCore {

    /** What the caller should do with the phase this returns. */
    public enum Act {
        /** Run the phase's action. */
        RUN,
        /** The phase cannot start; hold position and keep checking. */
        HOLD,
        /** It has been stalled long enough; release the mechanisms to the driver. */
        HAND_BACK
    }

    /**
     * @param phase  the phase index to act on
     * @param act    what to do with it
     * @param reason a short human-readable explanation, published as telemetry
     */
    public record Decision(int phase, Act act, String reason) {}

    private final int phaseCount;
    private final double dwellSeconds;
    private final double stallHandbackSeconds;

    private int phase;
    /** When the currently-requested-but-not-yet-adopted phase was first asked for. */
    private int pending = -1;
    private double pendingSince = Double.NaN;
    private double stalledSince = Double.NaN;

    /**
     * @param phaseCount           how many phases the cycle has; must be at least one
     * @param dwellSeconds         how long a change must be asked for before it is made; 0 disables
     * @param stallHandbackSeconds how long to stall before handing back; 0 never hands back
     */
    public CycleCore(int phaseCount, double dwellSeconds, double stallHandbackSeconds) {
        if (phaseCount < 1) {
            throw new IllegalArgumentException("a cycle needs at least one phase, got " + phaseCount);
        }
        this.phaseCount = phaseCount;
        this.dwellSeconds = Math.max(0.0, dwellSeconds);
        this.stallHandbackSeconds = Math.max(0.0, stallHandbackSeconds);
    }

    /**
     * Decide this loop.
     *
     * @param nowSeconds       monotonic seconds
     * @param desired          the phase the caller's own logic wants, e.g. "score, we are holding"
     * @param desiredCanStart  whether that phase's action can start, already evaluated and guarded
     */
    public Decision decide(double nowSeconds, int desired, boolean desiredCanStart) {
        int want = Math.floorMod(desired, phaseCount);

        // Dwell. A change has to be asked for continuously; anything else resets the timer, so a
        // sensor that chatters never accumulates enough to switch.
        if (want != phase) {
            if (want != pending) {
                pending = want;
                pendingSince = nowSeconds;
            }
            if (nowSeconds - pendingSince >= dwellSeconds) {
                phase = want;
                pending = -1;
                pendingSince = Double.NaN;
                stalledSince = Double.NaN;
            }
        } else {
            pending = -1;
            pendingSince = Double.NaN;
        }

        // Whether the phase we settled on can actually run. Note this is the *adopted* phase, which
        // during a dwell is still the old one - and the caller evaluated canStart for the desired
        // phase. They agree except in the dwell window, where holding the old phase is right anyway.
        boolean canRun = phase == want ? desiredCanStart : true;

        if (canRun) {
            stalledSince = Double.NaN;
            return new Decision(phase, Act.RUN, "running");
        }

        if (Double.isNaN(stalledSince)) {
            stalledSince = nowSeconds;
        }
        double stalledFor = nowSeconds - stalledSince;
        if (stallHandbackSeconds > 0 && stalledFor >= stallHandbackSeconds) {
            return new Decision(phase, Act.HAND_BACK,
                    String.format("stalled %.1fs, handing back to the driver", stalledFor));
        }
        return new Decision(phase, Act.HOLD, "cannot start");
    }

    /** The phase currently adopted. */
    public int phase() {
        return phase;
    }

    /** How long the current phase has been unable to start, or 0 when it is not stalled. */
    public double stalledSeconds(double nowSeconds) {
        return Double.isNaN(stalledSince) ? 0.0 : nowSeconds - stalledSince;
    }

    /** Back to the first phase with no history, as if the cycle had just been engaged. */
    public void reset() {
        phase = 0;
        pending = -1;
        pendingSince = Double.NaN;
        stalledSince = Double.NaN;
    }
}
