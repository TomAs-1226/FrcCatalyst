package frc.lib.catalyst.util;

import java.util.ArrayDeque;

/**
 * May the robot shoot from where its cameras say it is? A shot gate built on camera evidence that has
 * been checked against the wheels, with a short, bounded carry on the wheels alone after the tags drop.
 *
 * <h2>The failure this exists for</h2>
 *
 * <p>A shot that aims from the fused pose aims from whatever the estimator last believed. After a hit,
 * a wheel slip or a spell without tags that belief can be tens of centimetres off, and the shot still
 * looks ready: the flywheel is at speed, the heading is on its target, the hood is set. The pose is the
 * one input nothing checks. This gate checks it, and answers with a reason when it says no.
 *
 * <h2>What qualifies a shot</h2>
 *
 * <ol>
 *   <li><b>Camera motion that the wheels agree with.</b> At least three camera fixes spanning at least
 *       {@value #MIN_WINDOW_SECONDS} s, inside a {@value #WINDOW_SECONDS} s window, each landing where
 *       the measured wheel velocity integrated from the first fix says it should. One fix can be a
 *       lucky frame; three that move the way the robot moved are the robot. A stationary camera on a
 *       robot whose wheels spin, or a camera moving the wrong way, fails.
 *   <li><b>Capture-time motion.</b> A camera frame is paired with the wheel velocity at the instant
 *       it was captured, interpolated between the drive samples that bracket it, never with the
 *       newest one. A frame waits in a queue until a later drive sample arrives. Pairing a frame
 *       100 ms old with the current velocity is what made moving shots stutter through every change
 *       of direction.
 *   <li><b>Fresh.</b> The newest fix no older than {@value #FRESH_FIX_SECONDS} s, and the drive state
 *       no older than {@value #MAX_DRIVE_AGE_SECONDS} s.
 *   <li><b>Or a bounded carry.</b> Once a window has qualified, the shot may carry on the wheels alone
 *       for at most {@link Config#carrySeconds} and {@link Config#carryMeters} of <em>path length</em>
 *       (driving out and back does not reset it). Repeating an old capture, or a stream of weaker
 *       frames, never extends either budget.
 * </ol>
 *
 * <h2>Limits that scale with range</h2>
 *
 * <p>A shot needs an aim error, not a length. Two degrees of bearing is 10 cm at 2.9 m and 14-18 cm at
 * 4-5 m, where the shot tables barely move. So the agreement limit is {@link Config#aimBudgetRad} times
 * the distance to the target, held between 10 and 20 cm; a frame modelled worse than 5 % of the range
 * (10-20 cm) cannot refresh the evidence; a fix more than 5 % of the range (15-25 cm) from the
 * estimator's pose is rejected outright. Without a target ({@link #setTarget} never called) every
 * limit stays at its floor. Fixed at 10 cm, team 5805's robot refused every shot past 3.2 m in a
 * match (CAMIS 2026, Q63): one tag past 2.6 m is modelled worse than 10 cm, and a 10 cm wobble against
 * the wheels voided the rest.
 *
 * <h2>Cameras that disagree with each other</h2>
 *
 * <p>Two cameras reporting the same capture time are one sample, and if they are further apart than
 * the agreement limit one of them is wrong: everything up to that capture is revoked, in whichever
 * order the frames arrive. A weaker frame that contradicts measured motion revokes the evidence too; a
 * weaker frame that agrees is ignored without changing anything.
 *
 * <h2>Disturbances</h2>
 *
 * <p>{@link #invalidateBefore(double)} drops every piece of evidence up to an instant: call it at an
 * impact, a slip, or a pose reset. {@link #invalidateCarryBefore(double)} only forbids the blind carry,
 * for a motion model that has become uncertain while the cameras are still good.
 *
 * <h2>Using it</h2>
 *
 * <pre>{@code
 * VisionShotGate gate = new VisionShotGate();
 * gate.setTarget(goal.getX(), goal.getY());
 *
 * // Every drive sample, on the robot clock, with the measured field-relative velocity:
 * gate.recordMotion(state.timestamp, fieldVx, fieldVy);
 * // Every accepted camera pose, at its capture time, with its disagreement from the estimator's pose
 * // at that time and its modelled position sigma:
 * gate.recordCamera(capture, pose.getX(), pose.getY(), disagreementMeters, sigmaMeters);
 *
 * // In the shot's gate, alongside the flywheel, hood and heading checks:
 * String blocker = gate.scoreBlocker(now, driveStateAge, speed, yawRate, lastFaultTime);
 * boolean poseReady = blocker.isEmpty();
 * }</pre>
 *
 * <p>Pure: no NetworkTables, no clock, no hardware. Everything arrives as an argument, on one clock.
 * It qualifies the pose for a shot; it never moves the robot and never feeds the estimator.
 *
 * <p>Ported from team 5805's competition robot, where it gated every scoring shot at CAMIS 2026. The
 * range-scaled limits were added after that event and have run in simulation only.
 *
 * @since 2.0.0
 */
public final class VisionShotGate {

    /** Fixes the trajectory check looks back over, s. */
    public static final double WINDOW_SECONDS = 0.40;
    /** The least span of fixes that counts as camera motion, s. */
    public static final double MIN_WINDOW_SECONDS = 0.20;
    /** A fix this old or younger is a fresh camera shot, s. */
    public static final double FRESH_FIX_SECONDS = 0.12;
    /** The oldest drive state a shot is judged on, s. */
    public static final double MAX_DRIVE_AGE_SECONDS = 0.035;
    /** The longest gap between two drive samples that still counts as continuous motion, s. */
    public static final double MAX_MOTION_GAP_SECONDS = 0.05;
    /** Two consecutive fixes further apart than this start a new window, s. */
    private static final double MAX_FIX_GAP_SECONDS = 0.15;
    private static final double EPS = 1e-9;

    /** Tunable limits. The defaults are the ones the gate ran with on a robot. */
    public static final class Config {
        double fieldLengthMeters = CatalystMath.FIELD_LENGTH;
        double fieldWidthMeters = CatalystMath.FIELD_WIDTH;
        double aimBudgetRad = 0.035;
        double carrySeconds = 1.5;
        double carryMeters = 1.0;

        /** The field a fix must land on, m. Default the 2026 field, as {@link CatalystMath} has it. */
        public Config field(double lengthMeters, double widthMeters) {
            if (!(lengthMeters > 0) || !(widthMeters > 0) || !Double.isFinite(lengthMeters + widthMeters)) {
                throw new IllegalArgumentException("field size must be positive and finite");
            }
            fieldLengthMeters = lengthMeters;
            fieldWidthMeters = widthMeters;
            return this;
        }

        /** The aim error the agreement limit allows, rad. Default 0.035 (2 degrees), accepted 0.005-0.1. */
        public Config aimBudgetRad(double rad) {
            if (!(rad >= 0.005 && rad <= 0.1)) {
                throw new IllegalArgumentException("aimBudgetRad must be in [0.005, 0.1]");
            }
            aimBudgetRad = rad;
            return this;
        }

        /**
         * How long and how far a qualified shot may carry on the wheels alone. Defaults 1.5 s and 1.0 m;
         * zero forbids the carry.
         */
        public Config carry(double seconds, double meters) {
            if (!(seconds >= 0) || !(meters >= 0) || !Double.isFinite(seconds + meters)) {
                throw new IllegalArgumentException("carry budgets must be finite and not negative");
            }
            carrySeconds = seconds;
            carryMeters = meters;
            return this;
        }

        public double carrySeconds() {
            return carrySeconds;
        }

        public double carryMeters() {
            return carryMeters;
        }
    }

    private record Fix(double time, double x, double y, double vx, double vy) {}
    private record Motion(double time, double vx, double vy) {}
    private record Capture(double time, double x, double y, double disagreement, double sigma) {}
    /** A frame that agreed with an equal-time fix: evidence against a contradiction, never a fix. */
    private record Peer(double time, double x, double y) {}
    private record Delta(double x, double y, double travel) {}

    private static final String MISMATCH = "camera/wheel motion mismatch";

    private final Config config;
    private final ArrayDeque<Fix> fixes = new ArrayDeque<>();
    private final ArrayDeque<Motion> motion = new ArrayDeque<>();
    private final ArrayDeque<Capture> pending = new ArrayDeque<>();
    private final ArrayDeque<Peer> peers = new ArrayDeque<>();
    private String captureStatus = "no camera capture";
    private Fix carryAnchor;
    private double carryWindowStart = Double.NaN;
    private double invalidBefore = Double.NEGATIVE_INFINITY;
    private double carryInvalidBefore = Double.NEGATIVE_INFINITY;
    private double targetX = Double.NaN;
    private double targetY = Double.NaN;

    public VisionShotGate() {
        this(new Config());
    }

    public VisionShotGate(Config config) {
        this.config = config;
    }

    public Config config() {
        return config;
    }

    /** The point being shot at, field metres. The limits scale with the distance to it. */
    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    // ---- limits ---------------------------------------------------------------------------------

    private double targetDistance(double x, double y) {
        return finite(targetX, targetY, x, y) ? Math.hypot(x - targetX, y - targetY) : Double.NaN;
    }

    /** Camera fixes and wheel motion must agree this well at {@code range} metres, m: 10 to 20 cm. */
    public double agreementMeters(double range) {
        return Double.isFinite(range) ? Math.clamp(config.aimBudgetRad * range, 0.10, 0.20) : 0.10;
    }

    /** A frame modelled worse than this cannot refresh the evidence, m: 10 to 20 cm. */
    public static double weakSigmaMeters(double range) {
        return Double.isFinite(range) ? Math.clamp(0.05 * range, 0.10, 0.20) : 0.10;
    }

    /** A fix this far from the estimator's pose is rejected outright, m: 15 to 25 cm. */
    public static double maxDisagreementMeters(double range) {
        return Double.isFinite(range) ? Math.clamp(0.05 * range, 0.15, 0.25) : 0.15;
    }

    // ---- disturbances ---------------------------------------------------------------------------

    /** Drop every fix, queued frame and carry up to {@code epoch}: an impact, a slip, a pose reset. */
    public void invalidateBefore(double epoch) {
        if (!Double.isFinite(epoch) || epoch <= invalidBefore) return;
        invalidBefore = epoch;
        fixes.removeIf(f -> f.time() <= epoch);
        pending.removeIf(c -> c.time() <= epoch);
        peers.removeIf(p -> p.time() <= epoch);
        if (carryAnchor != null && carryWindowStart <= epoch) carryAnchor = null;
    }

    /** Forbid the blind carry for windows that began by {@code epoch}, keeping the camera evidence. */
    public void invalidateCarryBefore(double epoch) {
        if (!Double.isFinite(epoch)) return;
        carryInvalidBefore = Math.max(carryInvalidBefore, epoch);
        carryAnchor = null;
        peers.removeIf(p -> p.time() <= epoch);
    }

    private void clearEvidence() {
        fixes.clear();
        pending.clear();
        peers.clear();
        carryAnchor = null;
    }

    // ---- inputs ---------------------------------------------------------------------------------

    /**
     * One drive sample: the measured field-relative velocity, m/s, on the same clock as the camera
     * capture times. A non-finite sample, a gap longer than {@value #MAX_MOTION_GAP_SECONDS} s or a
     * clock that runs backwards clears every piece of evidence.
     */
    public void recordMotion(double time, double vx, double vy) {
        if (!finite(time, vx, vy)) {
            motion.clear();
            clearEvidence();
            captureStatus = "invalid drive motion";
            return;
        }
        if (!motion.isEmpty()) {
            double dt = time - motion.getLast().time();
            if (dt == 0) return;
            if (dt < 0 || dt > MAX_MOTION_GAP_SECONDS + EPS) {
                motion.clear();
                clearEvidence();
                captureStatus = "drive motion gap or backwards clock";
                if (dt < 0) return;
            }
        }
        motion.addLast(new Motion(time, vx, vy));
        while (motion.size() > 256 || time - motion.getFirst().time() > 2.0) motion.removeFirst();
        consumePending();
    }

    /**
     * One accepted camera pose, queued until drive motion brackets its capture time.
     *
     * @param capture      when the frame was captured, s, on the drive samples' clock
     * @param x            field x, m
     * @param y            field y, m
     * @param disagreement how far it lies from the estimator's pose at that time, m
     * @param sigma        its modelled position standard deviation, m
     */
    public void recordCamera(double capture, double x, double y, double disagreement, double sigma) {
        if (!qualifies(capture, x, y, disagreement, sigma)) {
            clearEvidence();
            captureStatus = "camera quality/agreement rejected";
            return;
        }
        if (capture <= invalidBefore) return;
        if (simultaneousConflict(capture, x, y)) return;
        Motion velocity = velocityAt(capture);
        if (ignoreWeakerFrame(capture, x, y, sigma,
                velocity == null ? Double.NaN : velocity.vx(),
                velocity == null ? Double.NaN : velocity.vy())) return;
        if (!fixes.isEmpty() && capture <= fixes.getLast().time()
                || !pending.isEmpty() && capture <= pending.getLast().time()) return;
        pending.addLast(new Capture(capture, x, y, disagreement, sigma));
        while (pending.size() > 8) {
            pending.removeFirst();
            fixes.clear();
            carryAnchor = null;
        }
        consumePending();
    }

    /**
     * One camera pose with the field velocity at its capture time already known. Prefer
     * {@link #recordCamera}, which interpolates the velocity from the drive samples itself.
     */
    public void record(double capture, double x, double y, double disagreement, double sigma,
                       double vx, double vy) {
        if (!finite(vx, vy) || !qualifies(capture, x, y, disagreement, sigma)) {
            clearEvidence();
            return;
        }
        if (capture <= invalidBefore) return;
        if (simultaneousConflict(capture, x, y)) return;
        if (ignoreWeakerFrame(capture, x, y, sigma, vx, vy)) return;
        if (carryAnchor != null && capture > carryAnchor.time()) {
            Delta d = integrate(carryAnchor.time(), capture);
            if (d == null) {
                carryAnchor = null;
            } else if (Math.hypot(x - carryAnchor.x() - d.x(), y - carryAnchor.y() - d.y())
                    > agreementMeters(targetDistance(x, y))) {
                carryAnchor = null;
                fixes.clear();
            }
        }
        if (!fixes.isEmpty() && capture <= fixes.getLast().time()) return;
        if (!fixes.isEmpty() && capture - fixes.getLast().time() > MAX_FIX_GAP_SECONDS) fixes.clear();
        fixes.addLast(new Fix(capture, x, y, vx, vy));
        while (fixes.size() > 32 || capture - fixes.getFirst().time() > WINDOW_SECONDS) fixes.removeFirst();
        String trajectory = trajectoryBlocker(invalidBefore);
        if (trajectory.equals(MISMATCH)) {
            // Revoke the carry but keep the contradictory window. Clearing it would let a continuing
            // velocity bias look acceptable in repeated short, newly anchored windows.
            carryAnchor = null;
            captureStatus = trajectory;
        } else if (trajectory.isEmpty()) {
            carryAnchor = fixes.getLast();
            carryWindowStart = fixes.getFirst().time();
        }
    }

    private boolean qualifies(double capture, double x, double y, double disagreement, double sigma) {
        return finite(capture, x, y, disagreement, sigma) && capture >= 0 && disagreement >= 0
                && disagreement <= maxDisagreementMeters(targetDistance(x, y)) && sigma > 0
                && x >= 0 && x <= config.fieldLengthMeters && y >= 0 && y <= config.fieldWidthMeters;
    }

    private void consumePending() {
        while (!pending.isEmpty() && !motion.isEmpty()) {
            Capture c = pending.getFirst();
            if (c.time() > motion.getLast().time()) {
                captureStatus = "waiting for capture-time motion";
                return;
            }
            pending.removeFirst();
            Motion velocity = velocityAt(c.time());
            if (velocity == null) {
                fixes.clear();
                carryAnchor = null;
                captureStatus = "capture-time motion missing or gapped";
                continue;
            }
            record(c.time(), c.x(), c.y(), c.disagreement(), c.sigma(), velocity.vx(), velocity.vy());
            if (!fixes.isEmpty() && fixes.getLast().time() == c.time()
                    && !trajectoryBlocker(invalidBefore).equals(MISMATCH)) {
                captureStatus = "capture-time motion aligned";
            }
        }
    }

    /** The velocity at {@code time}, interpolated between the drive samples that bracket it. */
    private Motion velocityAt(double time) {
        Motion before = null;
        for (Motion frame : motion) {
            if (Math.abs(time - frame.time()) < EPS) return frame;
            if (frame.time() > time) {
                if (before == null || frame.time() - before.time() > MAX_MOTION_GAP_SECONDS + EPS) return null;
                double fraction = (time - before.time()) / (frame.time() - before.time());
                return new Motion(time, before.vx() + fraction * (frame.vx() - before.vx()),
                        before.vy() + fraction * (frame.vy() - before.vy()));
            }
            before = frame;
        }
        return null;
    }

    /** Equal-time cameras are one sample, but a contradictory peer revokes it. */
    private boolean simultaneousConflict(double capture, double x, double y) {
        boolean conflict = false;
        double agree = agreementMeters(targetDistance(x, y));
        for (Fix f : fixes) if (capture == f.time() && Math.hypot(x - f.x(), y - f.y()) > agree) conflict = true;
        for (Capture c : pending) if (capture == c.time() && Math.hypot(x - c.x(), y - c.y()) > agree) conflict = true;
        for (Peer p : peers) if (capture == p.time() && Math.hypot(x - p.x(), y - p.y()) > agree) conflict = true;
        if (conflict) {
            invalidateBefore(capture);
            carryAnchor = null;
            captureStatus = "simultaneous camera disagreement";
        } else if (velocityAt(capture) != null) {
            // Only bracketed frames are remembered. A peer supplies no fix, timestamp or motion sample;
            // it exists only so a later contradictory frame at the same time is caught.
            peers.removeIf(p -> capture - p.time() > WINDOW_SECONDS);
            boolean duplicate = false;
            for (Peer p : peers) if (p.time() == capture && p.x() == x && p.y() == y) duplicate = true;
            if (!duplicate) peers.addLast(new Peer(capture, x, y));
            while (peers.size() > 128) peers.removeFirst();
        }
        return conflict;
    }

    /** A less precise frame cannot refresh the evidence, but is not by itself a fault. */
    private boolean ignoreWeakerFrame(double capture, double x, double y, double sigma, double vx, double vy) {
        double range = targetDistance(x, y);
        if (sigma <= weakSigmaMeters(range)) return false;
        if (!fixes.isEmpty()) {
            Fix last = fixes.getLast();
            double dt = capture - last.time();
            Delta delta = dt >= 0 ? integrate(last.time(), capture) : null;
            double expectedX = Double.NaN;
            double expectedY = Double.NaN;
            if (delta != null) {
                expectedX = last.x() + delta.x();
                expectedY = last.y() + delta.y();
            } else if (dt >= 0 && dt <= MAX_FIX_GAP_SECONDS && finite(vx, vy)) {
                expectedX = last.x() + (last.vx() + vx) * 0.5 * dt;
                expectedY = last.y() + (last.vy() + vy) * 0.5 * dt;
            }
            if (Double.isFinite(expectedX) && Math.hypot(x - expectedX, y - expectedY) > agreementMeters(range)) {
                clearEvidence();
                invalidateBefore(capture);
                captureStatus = "weaker camera contradicts measured motion";
                return true;
            }
        }
        captureStatus = "weaker camera frame ignored; prior budgets unchanged";
        return true;
    }

    /** Integrate measured field velocity over complete observed intervals, never a commanded one. */
    private Delta integrate(double from, double to) {
        if (to < from) return null;
        Motion previous = velocityAt(from);
        Motion end = velocityAt(to);
        if (previous == null || end == null) return null;
        double x = 0;
        double y = 0;
        double travel = 0;
        for (Motion frame : motion) {
            if (frame.time() <= from || frame.time() >= to) continue;
            double dt = frame.time() - previous.time();
            if (dt > MAX_MOTION_GAP_SECONDS + EPS) return null;
            x += (previous.vx() + frame.vx()) * 0.5 * dt;
            y += (previous.vy() + frame.vy()) * 0.5 * dt;
            travel += (Math.hypot(previous.vx(), previous.vy()) + Math.hypot(frame.vx(), frame.vy())) * 0.5 * dt;
            previous = frame;
        }
        double dt = to - previous.time();
        if (dt > MAX_MOTION_GAP_SECONDS + EPS) return null;
        return new Delta(x + (previous.vx() + end.vx()) * 0.5 * dt,
                y + (previous.vy() + end.vy()) * 0.5 * dt,
                travel + (Math.hypot(previous.vx(), previous.vy()) + Math.hypot(end.vx(), end.vy())) * 0.5 * dt);
    }

    // ---- answers --------------------------------------------------------------------------------

    /** What the last camera input did, for a dashboard. */
    public String captureStatus() {
        return captureStatus;
    }

    /** The capture time of the newest accepted fix, or NaN. Drive samples never advance it. */
    public double captureSeconds() {
        return fixes.isEmpty() ? Double.NaN : fixes.getLast().time();
    }

    /** Wheel path length since the carry anchor, m, or NaN without one. Driving back does not undo it. */
    public double carryTravelMeters() {
        if (carryAnchor == null || motion.isEmpty()) return Double.NaN;
        Delta d = integrate(carryAnchor.time(), motion.getLast().time());
        return d == null ? Double.NaN : d.travel();
    }

    /** Seconds since the carry anchor's capture, or NaN without one. */
    public double carryAgeSeconds(double now) {
        return carryAnchor == null ? Double.NaN : now - carryAnchor.time();
    }

    /** Whether a shot now would be on the carry rather than a fresh fix. */
    public boolean usingCarry(double now) {
        return !fixes.isEmpty() && now - fixes.getLast().time() > FRESH_FIX_SECONDS;
    }

    /**
     * Is the camera position fresh enough to aim from? The ordinary aim, rate, speed, hood and
     * flywheel gates still apply.
     *
     * @param now             s, on the drive samples' clock
     * @param driveStateAge   age of the drive state the caller is using, s
     * @param speed           robot speed, m/s
     * @param faultEpoch      the last disturbance, s, or NaN for none
     */
    public boolean canAim(double now, double driveStateAge, double speed, double faultEpoch) {
        if (fixes.isEmpty() || !finite(now, driveStateAge, speed) || driveStateAge < 0
                || driveStateAge > MAX_DRIVE_AGE_SECONDS || speed < 0) return false;
        Fix last = fixes.getLast();
        double age = now - last.time();
        double epoch = Math.max(invalidBefore, Double.isFinite(faultEpoch) ? faultEpoch : Double.NEGATIVE_INFINITY);
        return age >= 0 && age <= FRESH_FIX_SECONDS && last.time() > epoch
                || carryBlocker(now, driveStateAge, speed, epoch).isEmpty();
    }

    /** {@link #scoreBlocker} is empty: the camera evidence qualifies a shot, moving or parked. */
    public boolean canScore(double now, double driveStateAge, double speed, double yawRate, double faultEpoch) {
        return scoreBlocker(now, driveStateAge, speed, yawRate, faultEpoch).isEmpty();
    }

    /**
     * Why the shot may not go, or an empty string if it may.
     *
     * @param yawRate the gyro's rate, rad/s; NaN blocks, since the carry never invents one
     */
    public String scoreBlocker(double now, double driveStateAge, double speed, double yawRate, double faultEpoch) {
        if (!Double.isFinite(yawRate)) return "gyro rate unavailable";
        double epoch = Math.max(invalidBefore, Double.isFinite(faultEpoch) ? faultEpoch : Double.NEGATIVE_INFINITY);
        if (canAim(now, driveStateAge, speed, epoch) && !fixes.isEmpty()
                && now - fixes.getLast().time() <= FRESH_FIX_SECONDS) {
            String fresh = trajectoryBlocker(epoch);
            if (fresh.isEmpty()) return "";
            if (fresh.equals(MISMATCH)) {
                carryAnchor = null;
                return fresh;
            }
        }
        return carryBlocker(now, driveStateAge, speed, epoch);
    }

    private String carryBlocker(double now, double driveStateAge, double speed, double faultEpoch) {
        if (carryAnchor == null) return "camera motion not yet qualified";
        if (carryWindowStart <= Math.max(faultEpoch, carryInvalidBefore)) return "new post-fault camera motion required";
        double age = now - carryAnchor.time();
        if (!finite(now, driveStateAge, speed) || speed < 0 || driveStateAge < 0
                || driveStateAge > MAX_DRIVE_AGE_SECONDS || motion.isEmpty()) return "fresh drive motion unavailable";
        double motionAge = now - motion.getLast().time();
        if (motionAge < 0 || motionAge > MAX_DRIVE_AGE_SECONDS) return "drive motion history stale";
        if (age < 0 || age > config.carrySeconds) return "carry time budget exhausted";
        double travel = carryTravelMeters();
        if (!Double.isFinite(travel) || travel > config.carryMeters) return "carry travel budget exhausted";
        return "";
    }

    private String trajectoryBlocker(double faultEpoch) {
        Fix first = null;
        Fix previous = null;
        double expectedX = 0;
        double expectedY = 0;
        int count = 0;
        for (Fix fix : fixes) {
            if (Double.isFinite(faultEpoch) && fix.time() <= faultEpoch) continue;
            if (first == null) {
                first = fix;
                expectedX = fix.x();
                expectedY = fix.y();
            } else {
                double dt = fix.time() - previous.time();
                expectedX += (previous.vx() + fix.vx()) * 0.5 * dt;
                expectedY += (previous.vy() + fix.vy()) * 0.5 * dt;
                if (Math.hypot(fix.x() - expectedX, fix.y() - expectedY)
                        > agreementMeters(targetDistance(fix.x(), fix.y()))) return MISMATCH;
            }
            previous = fix;
            count++;
        }
        return count >= 3 && previous.time() - first.time() >= MIN_WINDOW_SECONDS - EPS
                ? "" : "collecting post-fault camera motion";
    }

    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
}
