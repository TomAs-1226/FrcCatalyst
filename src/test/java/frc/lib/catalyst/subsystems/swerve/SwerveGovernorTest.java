package frc.lib.catalyst.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

import org.junit.jupiter.api.Test;
import org.wpilib.math.controller.PIDController;

/**
 * The speed multiplier is the one thing that stands between the driver's sticks and the wheels:
 * slow mode goes through it, and so does Physics Core's slip and tipping scale. It applied to two
 * of the eight drive commands. A driver who engaged slow mode and then held the heading-lock button
 * got full speed; a robot whose slip detector had cut it to 60% went back to 100% for as long as
 * the point-at button was held.
 *
 * <p>And the heading loop's output went to the drivetrain unbounded, which is a rotation rate with
 * no relation to the drivetrain: at kP = 5 an error of half a turn asks for 15.7 rad/s.
 *
 * <p>Both are checked here without a HAL, on a subsystem allocated without its constructor - the
 * same trick {@link SwerveSimYieldTest} uses.
 */
class SwerveGovernorTest {

    private static final double MAX_SPEED = 4.0;      // m/s
    private static final double MAX_ANGULAR = 10.0;   // rad/s

    @Test
    void everyDriverFacingCommandScalesTranslationByTheMultiplier() throws Exception {
        // The source is the contract here: a command that multiplies by maxSpeedMPS without the
        // multiplier is a command that ignores slow mode. Six of them did.
        String source = java.nio.file.Files.readString(java.nio.file.Path.of(
                "src/main/java/frc/lib/catalyst/subsystems/swerve/SwerveSubsystem.java"));
        int unscaled = 0;
        for (String line : source.split("\n")) {
            String t = line.trim();
            if (t.startsWith("//") || t.startsWith("*")) {
                continue;
            }
            // A stick scaled to a speed, without the governor on the same line.
            if ((t.contains("* maxSpeedMPS;") || t.contains("* maxAngularRate;"))
                    && (t.contains("getAsDouble()") || t.contains("applyDeadband"))) {
                unscaled++;
            }
        }
        assertEquals(0, unscaled,
                "a drive command scales a stick to a speed without the speed multiplier");
    }

    @Test
    void theHeadingLoopOutputIsClampedToWhatTheStickCouldAsk() throws Exception {
        SwerveSubsystem drive = allocate();
        set(drive, "maxAngularRate", MAX_ANGULAR);
        set(drive, "speedMultiplier", 1.0);
        set(drive, "headingPID", pid());

        // Half a turn of error, kP = 5: the raw loop asks for about 15.7 rad/s.
        double rate = headingRate(drive, 0.0, Math.PI, true);
        assertEquals(MAX_ANGULAR, Math.abs(rate), 1e-9,
                "clamped to the drivetrain's own max rate, not the loop's arithmetic");
    }

    @Test
    void theClampFollowsTheMultiplierSoSlowModeSlowsTheTurnToo() throws Exception {
        SwerveSubsystem drive = allocate();
        set(drive, "maxAngularRate", MAX_ANGULAR);
        set(drive, "speedMultiplier", 0.35);
        set(drive, "headingPID", pid());

        double rate = headingRate(drive, 0.0, Math.PI, true);
        assertEquals(MAX_ANGULAR * 0.35, Math.abs(rate), 1e-9);
    }

    @Test
    void anAutonomousMoveKeepsItsOwnSpeedWhateverTheTeleopFlagSays() throws Exception {
        SwerveSubsystem drive = allocate();
        set(drive, "maxAngularRate", MAX_ANGULAR);
        set(drive, "speedMultiplier", 0.35);
        set(drive, "headingPID", pid());

        double rate = headingRate(drive, 0.0, Math.PI, false);
        assertEquals(MAX_ANGULAR, Math.abs(rate), 1e-9, "still bounded, but not by a teleop flag");
    }

    @Test
    void aHeadingAlreadyReachedCommandsNothingRatherThanATwitch() throws Exception {
        SwerveSubsystem drive = allocate();
        set(drive, "maxAngularRate", MAX_ANGULAR);
        set(drive, "speedMultiplier", 1.0);
        set(drive, "headingPID", pid());

        double halfADegree = Math.toRadians(0.5);
        assertEquals(0.0, headingRate(drive, halfADegree, 0.0, true), 1e-12,
                "inside the loop's tolerance the output is zero");
    }

    @Test
    void aSmallErrorStillTurnsTheRightWay() throws Exception {
        SwerveSubsystem drive = allocate();
        set(drive, "maxAngularRate", MAX_ANGULAR);
        set(drive, "speedMultiplier", 1.0);
        set(drive, "headingPID", pid());

        // 10 degrees of error, kP = 5 -> 0.87 rad/s, well inside the clamp, sign toward the target.
        double rate = headingRate(drive, 0.0, Math.toRadians(10), true);
        assertTrue(rate > 0 && rate < MAX_ANGULAR, "turns toward the target, unclamped: " + rate);
        assertEquals(5.0 * Math.toRadians(10), rate, 1e-9);
    }

    @Test
    void theMultiplierIsClampedToItsDocumentedRange() throws Exception {
        SwerveSubsystem drive = allocate();
        set(drive, "speedMultiplier", 1.0);
        drive.setSpeedMultiplier(2.5);
        assertEquals(1.0, drive.getSpeedMultiplier(), 1e-12, "a multiplier cannot be a turbo above 1");
        drive.setSpeedMultiplier(-1.0);
        assertEquals(0.0, drive.getSpeedMultiplier(), 1e-12, "nor negative, which would invert the sticks");
    }

    // ------------------------------------------------------------------ harness

    private static PIDController pid() {
        PIDController p = new PIDController(5.0, 0, 0);
        p.enableContinuousInput(-Math.PI, Math.PI);
        p.setTolerance(Math.toRadians(1.5));
        return p;
    }

    private static double headingRate(SwerveSubsystem drive, double current, double target, boolean governed)
            throws Exception {
        Method m = SwerveSubsystem.class.getDeclaredMethod(
                "headingRate", double.class, double.class, boolean.class);
        m.setAccessible(true);
        return (double) m.invoke(drive, current, target, governed);
    }

    private static void set(SwerveSubsystem drive, String name, Object value) throws Exception {
        Field f = SwerveSubsystem.class.getDeclaredField(name);
        f.setAccessible(true);
        f.set(drive, value);
    }

    /** A subsystem without its constructor, which would want CAN hardware. */
    private static SwerveSubsystem allocate() throws Exception {
        Field unsafeField = sun.misc.Unsafe.class.getDeclaredField("theUnsafe");
        unsafeField.setAccessible(true);
        sun.misc.Unsafe unsafe = (sun.misc.Unsafe) unsafeField.get(null);
        return (SwerveSubsystem) unsafe.allocateInstance(SwerveSubsystem.class);
    }
}
