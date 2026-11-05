package frc.lib.catalyst.util;

import org.wpilib.math.geometry.Pose2d;

import java.util.Optional;

/**
 * Where an autonomous routine should start from: the pose the cameras placed, or the routine's own
 * starting mark.
 *
 * <h2>The two ways to get it wrong</h2>
 *
 * <p>Reset to the mark every time, and a robot set down 20 cm off its mark drives the whole routine
 * 20 cm off, though its cameras could see exactly where it was. Never reset, and a robot whose cameras
 * saw nothing starts from wherever its odometry last was, or from the origin.
 *
 * <p>The less obvious case is a camera fix that looks good but is not. With MegaTag2 and most
 * single-camera solves the heading is the gyro's, and the camera's position is worked out from it. A
 * robot switched on where it was set down - sideways, say - has a heading a quarter turn out, and a
 * fix made with it lands somewhere else on the field. So a fix only stands when the gyro heading is
 * within {@link #DEFAULT_HEADING_TOLERANCE_DEGREES} of the heading the routine starts at.
 *
 * <h2>The rule</h2>
 *
 * <ul>
 *   <li>A recent vision fix and a heading that agrees with the routine's: keep the pose.
 *   <li>A recent fix with a heading that does not agree: reset to the mark.
 *   <li>No recent fix, but a pose the robot trusts (it was placed earlier and has not been disturbed):
 *       keep it.
 *   <li>No recent fix and no trusted pose: reset to the mark.
 *   <li>Wherever a reset is called for but the routine names no start, or resets are switched off,
 *       nothing changes, and the decision says so.
 * </ul>
 *
 * <p>Pure: it decides and the caller applies {@link Decision#pose()}, then logs
 * {@link Decision#detail()}. {@link AutoStartCheck} remains the disabled-period report of how far the
 * robot is from its mark. Ported from team 5805's auto start, which ran it through two events; a rule
 * of theirs that always resets in one start zone stays theirs, as {@code poseTrusted = false}.
 *
 * @since 2.0.0
 */
public final class AutoStartPose {

    /** How far the gyro heading may be from the routine's start heading for a vision fix to stand. */
    public static final double DEFAULT_HEADING_TOLERANCE_DEGREES = 20.0;

    /** Where the pose came from. */
    public enum Source {
        /** The current pose stands: a vision fix, or a pose the robot already trusted. */
        CURRENT,
        /** Reset to the routine's starting pose. */
        START_POSE,
        /** A reset was called for, but there is no start pose or resets are off. */
        NOTHING
    }

    /**
     * @param source what to do
     * @param pose   the pose to reset to, present only for {@link Source#START_POSE}
     * @param detail one line for the log
     */
    public record Decision(Source source, Optional<Pose2d> pose, String detail) {}

    private AutoStartPose() {}

    /**
     * Decide.
     *
     * @param current          the robot's pose now
     * @param start            the routine's starting pose, or empty if it names none
     * @param visionFixRecent  a camera fix was accepted within the last second or so: a camera is
     *                         looking at a tag right now
     * @param poseTrusted      the pose was placed (vision, or a deliberate reset) and nothing has
     *                         disturbed it since. Pass false for a routine that should always start
     *                         from its mark
     * @param resetAllowed     a team switch: false never resets
     */
    public static Decision decide(Pose2d current, Optional<Pose2d> start, boolean visionFixRecent,
                                  boolean poseTrusted, boolean resetAllowed) {
        return decide(current, start, visionFixRecent, poseTrusted, resetAllowed,
                DEFAULT_HEADING_TOLERANCE_DEGREES);
    }

    /** {@link #decide(Pose2d, Optional, boolean, boolean, boolean)} with its own heading tolerance. */
    public static Decision decide(Pose2d current, Optional<Pose2d> start, boolean visionFixRecent,
                                  boolean poseTrusted, boolean resetAllowed, double headingToleranceDegrees) {
        Optional<Pose2d> mark = start == null ? Optional.empty() : start;
        boolean headingAgrees = mark.isEmpty() || current == null
                || Math.abs(mark.get().getRotation().minus(current.getRotation()).getDegrees())
                        <= headingToleranceDegrees;
        if (current != null && visionFixRecent && headingAgrees) {
            return new Decision(Source.CURRENT, Optional.empty(), "VISION");
        }
        if (current != null && !visionFixRecent && poseTrusted) {
            return new Decision(Source.CURRENT, Optional.empty(), "CURRENT (trusted, no recent fix)");
        }
        if (mark.isEmpty()) {
            return new Decision(Source.NOTHING, Optional.empty(), "NOTHING (routine names no start)");
        }
        if (!resetAllowed) {
            return new Decision(Source.NOTHING, Optional.empty(), "NOTHING (start reset switched off)");
        }
        return new Decision(Source.START_POSE, mark, visionFixRecent
                ? "START POSE (the heading was not the routine's)" : "START POSE");
    }
}
