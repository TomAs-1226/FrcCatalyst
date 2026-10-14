package frc.lib.catalyst.physics.estimation;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.lib.catalyst.physics.PhysicsSample;

/**
 * Opt-in, bounded shadow odometry for wheel/IMU disagreement. Never writes native drivetrain pose.
 * Clean motion follows native pose; suspected slip integrates horizontal, gravity-removed IMU
 * acceleration briefly, then holds translation and reports uncertainty. A steady wheelspin cannot
 * erase the fault. Zero acceleration alone is NOT evidence of being stationary.
 *
 * <p>The caller must supply fresh, time-aligned measurements at the chassis center. The thresholds
 * are conservative engineering limits, not a probability model or proof that a robot is stuck.
 * A trusted post-fault fix or explicit externally validated pose reset repairs the position anchor.
 */
public final class WheelMotionObserver {
    public enum Mode { UNINITIALIZED, ROLLING, INERTIAL, UNCERTAIN }

    public record State(Pose2d pose, Translation2d fieldVelocity, Mode mode,
            boolean requiresAbsoluteFix, boolean accelerationAvailable, double timestampSeconds,
            double faultTimestampSeconds, double disagreementMps, double rejectedWheelTravelMeters,
            String reason) {
        /** Safe to project a path or declare arrival; separate from driver control/DS enable. */
        public boolean usable() { return mode == Mode.ROLLING && !requiresAbsoluteFix; }
        public static State unknown() {
            return new State(Pose2d.ZERO, Translation2d.ZERO, Mode.UNINITIALIZED, false, false,
                    Double.NaN, Double.NaN, 0, 0, "no motion sample");
        }
    }

    private static final double MAX_GAP_S = .10;
    private static final double CANDIDATE_ERROR_MPS = .20;
    private static final double CONFIRMED_ERROR_MPS = .35;
    private static final double CONFIRM_S = .06;
    private static final double INERTIAL_BUDGET_S = .60;
    private static final double FIX_MAX_AGE_S = .25;
    private static final double FIX_AGREEMENT_M = .25;
    private State state = State.unknown();
    private double candidateSince = Double.NaN;
    private double severeSince = Double.NaN;
    private Pose2d lastNativePose;
    private Translation2d lastWheels = Translation2d.ZERO;
    private Translation2d inertialReferenceVelocity = Translation2d.ZERO;
    private double referenceTime = Double.NaN;
    private double lastVelocityFix = Double.NEGATIVE_INFINITY;
    private double anchorTimestamp = Double.NaN;

    public State state() { return state; }

    public State update(PhysicsSample sample, double differentialSlip) {
        double t = sample.timestampSeconds();
        Pose2d nativePose = sample.pose();
        ChassisVelocities speeds = sample.robotRelativeSpeeds();
        if (!Double.isFinite(t) || !finite(nativePose.getX(), nativePose.getY(),
                nativePose.getRotation().getRadians(), speeds.vx, speeds.vy, speeds.omega,
                sample.yawRateRadPerSec())) return invalidate(t, "invalid motion sample");
        Translation2d wheels = new Translation2d(speeds.vx, speeds.vy).rotateBy(nativePose.getRotation());
        Translation2d a = sample.robotRelativeAcceleration();
        boolean available = a != null && finite(a.getX(), a.getY());
        if (a != null && !available) return invalidate(t, "invalid accelerometer sample");
        if (state.mode() == Mode.UNINITIALIZED || lastNativePose == null) {
            lastNativePose = nativePose;
            lastWheels = wheels;
            inertialReferenceVelocity = wheels;
            referenceTime = t;
            state = new State(nativePose, wheels, Mode.ROLLING, false, available, t,
                    Double.NaN, 0, 0, "native wheels; no prior acceleration interval");
            return state;
        }
        // A deliberate pose reset is newer than a frame still in the native cache. Ignore
        // that bounded pre-reset frame; do not let it undo the new position anchor.
        if (Double.isFinite(anchorTimestamp) && t < anchorTimestamp
                && anchorTimestamp - t <= .035) return state;
        double dt = t - state.timestampSeconds();
        // Repeated frames do not integrate twice or manufacture a fresh measurement.
        if (dt == 0) return state;
        if (!(dt > 0 && dt <= MAX_GAP_S + 1e-9)) {
            if (dt > 0) {
                // Fresh measurements after a gap remain measurements, not a velocity anchor.
                // Independent pose+velocity evidence must compare with THIS frame, not old wheels.
                lastNativePose = nativePose;
                lastWheels = wheels;
            }
            // The unobserved interval starts at the last valid frame. Truly independent
            // measurements captured at its end can repair it now; wheel data alone cannot.
            return invalidateFrom(t, "motion sampling gap", dt > 0 ? state.timestampSeconds() : t);
        }
        Translation2d previous = state.fieldVelocity();
        // Keep a short independent reference instead of anchoring it to wheels each loop:
        // otherwise a slow wheelspin ramp leaks into the reference one small increment at a time.
        if (!state.accelerationAvailable() && state.mode() == Mode.ROLLING) {
            inertialReferenceVelocity = previous;
            referenceTime = state.timestampSeconds();
        }
        Translation2d predicted = available
                ? inertialReferenceVelocity.plus(a.rotateBy(nativePose.getRotation()).times(dt)) : previous;
        double error = wheels.minus(predicted).getNorm();
        boolean suspect = available && error > CANDIDATE_ERROR_MPS;
        boolean moduleSlip = Double.isFinite(differentialSlip) && differentialSlip >= .5;
        if (suspect || moduleSlip) {
            if (!Double.isFinite(candidateSince)) candidateSince = state.timestampSeconds();
            if (error >= CONFIRMED_ERROR_MPS || moduleSlip) {
                if (!Double.isFinite(severeSince)) severeSince = state.timestampSeconds();
            } else severeSince = Double.NaN;
        } else if (!state.requiresAbsoluteFix() && available) {
            candidateSince = Double.NaN;
            severeSince = Double.NaN;
        }
        boolean confirmed = Double.isFinite(severeSince) && t - severeSince >= CONFIRM_S - 1e-9;
        boolean tainted = state.requiresAbsoluteFix() || confirmed;
        double fault = state.faultTimestampSeconds();
        if (confirmed && !Double.isFinite(fault)) fault = candidateSince;
        boolean inertial = Double.isFinite(candidateSince) || tainted;
        Mode mode;
        Translation2d velocity;
        Pose2d pose;
        double rejected = state.rejectedWheelTravelMeters();
        if (inertial) {
            // Missing acceleration cannot be replaced with zero; prolonged integration is unknown.
            boolean withinBudget = available && Double.isFinite(candidateSince)
                    && t - candidateSince <= INERTIAL_BUDGET_S + 1e-9
                    && state.mode() != Mode.UNCERTAIN;
            mode = withinBudget ? Mode.INERTIAL : Mode.UNCERTAIN;
            velocity = withinBudget ? predicted : Translation2d.ZERO;
            Translation2d delta = withinBudget ? previous.plus(velocity).times(dt / 2)
                    : Translation2d.ZERO;
            pose = new Pose2d(state.pose().getTranslation().plus(delta), nativePose.getRotation());
            if (lastNativePose != null) rejected += nativePose.getTranslation()
                    .minus(lastNativePose.getTranslation()).minus(delta).getNorm();
            if (!withinBudget) {
                tainted = true;
                if (!Double.isFinite(fault)) fault = state.timestampSeconds();
            }
        } else {
            mode = Mode.ROLLING;
            velocity = wheels;
            pose = nativePose;
        }
        inertialReferenceVelocity = predicted;
        // Refresh a quiet reference, but do not erase a slowly accumulating disagreement.
        if (mode == Mode.ROLLING && (!available || t - referenceTime >= .25 && error < .05)) {
            inertialReferenceVelocity = wheels;
            referenceTime = t;
        }
        lastNativePose = nativePose;
        lastWheels = wheels;
        state = new State(pose, velocity, mode, tainted, available, t, fault, error, rejected,
                mode == Mode.ROLLING ? (available ? "wheel and inertial increments agree"
                        : "wheels only; independent acceleration unavailable")
                : mode == Mode.INERTIAL ? "wheel/IMU disagreement; bounded inertial bridge"
                : "motion unknown; waiting for an independent agreeing fix");
        return state;
    }

    /** Invalid native/IMU data does not get a new wheel-derived position or velocity. */
    public State invalidate(double timestamp, String reason) {
        return invalidateFrom(timestamp, reason, timestamp);
    }

    private State invalidateFrom(double timestamp, String reason, double uncertainSince) {
        double t = Double.isFinite(timestamp) ? Math.max(timestamp, state.timestampSeconds())
                : state.timestampSeconds();
        if (!Double.isFinite(t)) t = Double.isFinite(timestamp) ? timestamp : 0;
        double fault = Double.isFinite(state.faultTimestampSeconds())
                ? state.faultTimestampSeconds() : (Double.isFinite(uncertainSince) ? uncertainSince : t);
        state = new State(state.pose(), Translation2d.ZERO, Mode.UNCERTAIN, true, false, t,
                fault, state.disagreementMps(), state.rejectedWheelTravelMeters(), reason);
        return state;
    }

    /**
     * Only a recent post-fault observation agreeing with BOTH the guarded pose and native pose,
     * and consistent current wheel velocity, can clear the fault. Does not write native pose.
     * Larger repairs belong to the caller's validated native pose-reset path and {@link #reset}.
     */
    public boolean acceptAbsoluteFix(Pose2d observation, double timestamp) {
        // UNCERTAIN's zero velocity is an unknown placeholder, never independent evidence
        // that stopped wheels represent a stationary chassis. A fresh velocity observation
        // first renews the INERTIAL reference; a position-only reset cannot do that.
        if (!state.requiresAbsoluteFix() || state.mode() == Mode.UNCERTAIN
                || lastNativePose == null || observation == null
                || !finite(observation.getX(), observation.getY(), timestamp)
                || !(timestamp > state.faultTimestampSeconds())
                || timestamp > state.timestampSeconds()
                || state.timestampSeconds() - timestamp > FIX_MAX_AGE_S
                || lastWheels.minus(state.fieldVelocity()).getNorm() > CANDIDATE_ERROR_MPS) return false;
        // Compare at current time, with only a bounded short forward projection of the capture.
        Translation2d current = observation.getTranslation().plus(
                state.fieldVelocity().times(state.timestampSeconds() - timestamp));
        if (current.getDistance(state.pose().getTranslation()) > FIX_AGREEMENT_M
                || current.getDistance(lastNativePose.getTranslation()) > FIX_AGREEMENT_M) return false;
        state = new State(lastNativePose, lastWheels, Mode.ROLLING, false,
                state.accelerationAvailable(), state.timestampSeconds(), Double.NaN, 0,
                state.rejectedWheelTravelMeters(), "post-fault independent fix agrees");
        candidateSince = severeSince = Double.NaN;
        inertialReferenceVelocity = lastWheels;
        referenceTime = state.timestampSeconds();
        return true;
    }

    /**
     * Independent ground velocity can renew the inertial reference, but never repairs position or
     * certifies a disagreeing wheel velocity. Caller must identify an independent physical source.
     */
    public boolean acceptVelocity(Translation2d fieldVelocity, double timestamp, double standardDeviation) {
        if (state.mode() == Mode.UNINITIALIZED || fieldVelocity == null
                || !finite(fieldVelocity.getX(), fieldVelocity.getY(), timestamp, standardDeviation)
                || !(standardDeviation > 0 && standardDeviation <= .20)
                || timestamp <= lastVelocityFix || timestamp > state.timestampSeconds()
                || state.timestampSeconds() - timestamp > FIX_MAX_AGE_S
                || state.requiresAbsoluteFix() && !(timestamp > state.faultTimestampSeconds())) return false;
        lastVelocityFix = timestamp;
        inertialReferenceVelocity = fieldVelocity;
        referenceTime = state.timestampSeconds();
        // Only this fresh independent source can renew an exhausted inertial bridge.
        if (!state.usable()) {
            candidateSince = state.timestampSeconds();
            severeSince = Double.NaN;
            state = new State(state.pose(), fieldVelocity, Mode.INERTIAL,
                    state.requiresAbsoluteFix(), state.accelerationAvailable(), state.timestampSeconds(),
                    state.faultTimestampSeconds(), lastWheels.minus(fieldVelocity).getNorm(),
                    state.rejectedWheelTravelMeters(), "independent velocity; position still needs agreement");
        }
        return true;
    }

    /** Rebase only after the caller has deliberately validated and reset native pose. */
    public void resetTo(Pose2d pose, double timestamp) {
        if (pose == null || !finite(pose.getX(), pose.getY(), pose.getRotation().getRadians(), timestamp))
            throw new IllegalArgumentException("finite validated pose and timestamp required");
        // A position observation does not prove velocity. In particular, never seed an
        // inertial reference with wheel speed already known to be wrong during uniform spin.
        boolean faulted = state.requiresAbsoluteFix() || !state.usable() && lastNativePose != null;
        Translation2d velocity = lastNativePose == null ? Translation2d.ZERO
                : (faulted ? state.fieldVelocity() : lastWheels).rotateBy(
                        pose.getRotation().minus(lastNativePose.getRotation()));
        double oldFault = Double.isFinite(state.faultTimestampSeconds())
                ? state.faultTimestampSeconds() : state.timestampSeconds();
        double rejected = state.rejectedWheelTravelMeters();
        boolean acceleration = state.accelerationAvailable();
        Mode oldMode = state.mode();
        lastNativePose = pose;
        inertialReferenceVelocity = velocity;
        referenceTime = anchorTimestamp = timestamp;
        if (!faulted) candidateSince = severeSince = Double.NaN;
        state = new State(pose, velocity, faulted ? oldMode : Mode.ROLLING, faulted,
                acceleration, timestamp, faulted ? oldFault : Double.NaN, state.disagreementMps(),
                rejected, faulted ? "position repaired; motion still untrusted"
                : "externally validated native pose reset");
    }

    /** Known, stationary authored start. The caller, not a camera pose, validates standstill. */
    public void seedAtRest(Pose2d pose, double timestamp) {
        reset();
        resetTo(pose, timestamp);
    }

    /** Clear every estimate; the caller must deliberately establish a new anchor. */
    public void reset() {
        state = State.unknown();
        candidateSince = severeSince = Double.NaN;
        lastNativePose = null;
        lastWheels = inertialReferenceVelocity = Translation2d.ZERO;
        referenceTime = Double.NaN;
        lastVelocityFix = Double.NEGATIVE_INFINITY;
        anchorTimestamp = Double.NaN;
    }

    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
}
