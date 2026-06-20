package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Alliance coordinate flipping. Author your field coordinates once in the
 * <b>blue-origin</b> frame (goals, scoring poses, auto waypoints), then call
 * {@link #apply} at runtime to get the right coordinate for the current
 * alliance. No more duplicate red/blue constants.
 *
 * <p>Set the field once (defaults to the 2026 REBUILT field) and the symmetry
 * your game uses:
 * <ul>
 *   <li>{@link Symmetry#ROTATIONAL} — the field is symmetric under a 180°
 *       rotation about its center (most modern FRC fields). Flipping mirrors
 *       both X and Y and adds 180° to a heading.</li>
 *   <li>{@link Symmetry#MIRRORED} — the field mirrors across the vertical
 *       center line. Flipping mirrors X only and reflects a heading.</li>
 * </ul>
 *
 * <pre>{@code
 * // once, at startup (REBUILT is the default, so this is optional):
 * AllianceFlipUtil.configure(16.54, 8.21, AllianceFlipUtil.Symmetry.ROTATIONAL);
 *
 * // author in blue coordinates, use anywhere:
 * Translation2d hub = AllianceFlipUtil.apply(Field.BLUE_HUB);
 * Pose2d start      = AllianceFlipUtil.apply(Field.BLUE_START);
 * }</pre>
 *
 * <p>{@link #apply} flips only when the robot is on the red alliance (via
 * {@link RobotState#isRed()}). The raw {@code flip(...)} methods always flip,
 * regardless of alliance — handy for tests and mirroring.
 */
public final class AllianceFlipUtil {

    /** How the field is symmetric between the two alliances. */
    public enum Symmetry { ROTATIONAL, MIRRORED }

    private static volatile double fieldLength = 16.54; // metres (REBUILT 2026)
    private static volatile double fieldWidth = 8.21;
    private static volatile Symmetry symmetry = Symmetry.ROTATIONAL;

    private AllianceFlipUtil() {}

    /** Configure the field footprint (metres) and its symmetry. */
    public static void configure(double lengthMeters, double widthMeters, Symmetry sym) {
        fieldLength = lengthMeters;
        fieldWidth = widthMeters;
        symmetry = sym;
    }

    /** Set just the field length (metres). */
    public static void setFieldLength(double lengthMeters) {
        fieldLength = lengthMeters;
    }

    public static double fieldLength() {
        return fieldLength;
    }

    public static double fieldWidth() {
        return fieldWidth;
    }

    /** True when the robot is on the red alliance, so blue-frame coords need flipping. */
    public static boolean shouldFlip() {
        return RobotState.isRed();
    }

    // --- raw flips (always flip) -------------------------------------------

    public static double flipX(double x) {
        return fieldLength - x;
    }

    public static double flipY(double y) {
        return fieldWidth - y;
    }

    /** Flip a translation across the field per the configured symmetry. */
    public static Translation2d flip(Translation2d blue) {
        return symmetry == Symmetry.ROTATIONAL
                ? new Translation2d(fieldLength - blue.getX(), fieldWidth - blue.getY())
                : new Translation2d(fieldLength - blue.getX(), blue.getY());
    }

    /** Flip a heading per the configured symmetry. */
    public static Rotation2d flip(Rotation2d blue) {
        return symmetry == Symmetry.ROTATIONAL
                ? blue.rotateBy(new Rotation2d(Math.PI))
                : new Rotation2d(Math.PI - blue.getRadians());
    }

    /** Flip a full pose (translation + heading). */
    public static Pose2d flip(Pose2d blue) {
        return new Pose2d(flip(blue.getTranslation()), flip(blue.getRotation()));
    }

    // --- alliance-aware apply (flip only when red) -------------------------

    public static double applyX(double blueX) {
        return shouldFlip() ? flipX(blueX) : blueX;
    }

    public static Translation2d apply(Translation2d blue) {
        return shouldFlip() ? flip(blue) : blue;
    }

    public static Rotation2d apply(Rotation2d blue) {
        return shouldFlip() ? flip(blue) : blue;
    }

    public static Pose2d apply(Pose2d blue) {
        return shouldFlip() ? flip(blue) : blue;
    }
}
