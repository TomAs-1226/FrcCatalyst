package frc.lib.catalyst.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import frc.lib.catalyst.subsystems.swerve.SwerveSetpointGenerator.Priority;

/**
 * The generator's rotation priority: when translation and rotation together ask a module for more than
 * it has, the translation alone gives way, so a robot aiming itself keeps its turn. Ported from the
 * Catalyst X1's drive shaper, where it kept Phoenix's desaturation from ever engaging in a 5 m/s pass
 * (it had, 25-35% of the time) on a model of the robot. The default stays what it was.
 */
class SwerveSetpointGeneratorTest {

    /** The X1's modules: a 28 x 26 in frame, wheels 0.391 m from the centre. */
    private static final double[][] X1_XY = {
        {0.289, 0.2635}, {0.289, -0.2635}, {-0.289, 0.2635}, {-0.289, -0.2635}};
    private static final Translation2d[] X1 = {
        new Translation2d(0.289, 0.2635), new Translation2d(0.289, -0.2635),
        new Translation2d(-0.289, 0.2635), new Translation2d(-0.289, -0.2635)};
    /** The X1's Falcons, m/s. */
    private static final double TOP = 5.24;

    /** The fastest module for a robot-relative translation and turn, m/s. */
    private static double fastest(double rx, double ry, double turn) {
        double worst = 0;
        for (double[] m : X1_XY) {
            worst = Math.max(worst, Math.hypot(rx - turn * m[1], ry + turn * m[0]));
        }
        return worst;
    }

    /** No acceleration limits, so each call answers only for the wheels. */
    private static SwerveSetpointGenerator unramped(Priority priority) {
        return new SwerveSetpointGenerator(TOP, 20.0, 1000.0, 1000.0, priority, X1);
    }

    @Test
    void theDefaultLeavesSaturationToTheDrivetrainAsItAlwaysHas() {
        // 5 m/s forward while turning at 3 rad/s: 5 + 3 x 0.39 is past 5.24, and the request goes out whole
        // for the drivetrain's own desaturation to shrink - the turn with it.
        ChassisVelocities request = new ChassisVelocities(5.0, 0.0, 3.0);
        ChassisVelocities byDefault = new SwerveSetpointGenerator(TOP, 20.0, 1000.0, 1000.0).generate(request, 0.02);
        ChassisVelocities proportional = unramped(Priority.PROPORTIONAL).generate(request, 0.02);
        assertEquals(5.0, byDefault.vx, 0.0);
        assertEquals(3.0, byDefault.omega, 0.0);
        assertEquals(byDefault.vx, proportional.vx, 0.0);
        assertEquals(byDefault.omega, proportional.omega, 0.0);

        // And the ramps are the ones they were: from rest, 8 m/s² and 8π rad/s² over 20 ms.
        SwerveSetpointGenerator old = new SwerveSetpointGenerator(4.0, 10.0, 8.0);
        ChassisVelocities ramped = old.generate(new ChassisVelocities(5.0, 0.0, 12.0), 0.02);
        assertEquals(0.16, ramped.vx, 1e-12);
        assertEquals(Math.PI * 8 * 0.02, ramped.omega, 1e-12);
        assertEquals(1.0, old.getTranslationScale(), 0.0);
    }

    @Test
    void theScaleFillsTheModulesExactlyAndNoFurther() {
        double scale = SwerveSetpointGenerator.translationScale(5.0, 0.0, 3.0, TOP, X1_XY);
        assertTrue(scale < 1.0);
        assertEquals(TOP, fastest(5.0 * scale, 0.0, 3.0), 1e-9, "the fastest module sits exactly at the limit");
        // Slow enough, nothing changes.
        assertEquals(1.0, SwerveSetpointGenerator.translationScale(3.0, 1.0, 1.0, TOP, X1_XY), 0.0);
        // A diagonal and a clockwise turn: still exact.
        double s2 = SwerveSetpointGenerator.translationScale(3.5, -3.0, -2.5, TOP, X1_XY);
        assertEquals(TOP, fastest(3.5 * s2, -3.0 * s2, -2.5), 1e-9);
        // A turn that alone saturates a module leaves nothing to translate with.
        assertEquals(0.0, SwerveSetpointGenerator.translationScale(2.0, 0.0, 20.0, TOP, X1_XY), 0.0);
    }

    @Test
    void rotationPriorityKeepsTheTurnAndShrinksOnlyTheTranslation() {
        SwerveSetpointGenerator gen = unramped(Priority.ROTATION);
        ChassisVelocities out = gen.generate(new ChassisVelocities(5.0, 0.0, 3.0), 0.02);
        assertEquals(3.0, out.omega, 0.0, "the turn is kept whole");
        assertEquals(0.0, out.vy, 0.0, "the direction is kept");
        assertTrue(out.vx < 5.0);
        assertEquals(TOP, fastest(out.vx, out.vy, out.omega), 1e-9);
        assertEquals(out.vx / 5.0, gen.getTranslationScale(), 1e-12);
    }

    @Test
    void noModuleIsAskedForMoreThanItHasAtAnyHeading() {
        SwerveSetpointGenerator gen = unramped(Priority.ROTATION);
        for (double heading = -3; heading < 3; heading += 0.37) {
            for (double turn = -3; turn <= 3; turn += 0.5) {
                gen.reset();
                ChassisVelocities out = gen.generate(new ChassisVelocities(5.0, 1.5, turn), 0.02,
                        Rotation2d.fromRadians(heading));
                double c = Math.cos(heading);
                double s = Math.sin(heading);
                double rx = out.vx * c + out.vy * s;
                double ry = -out.vx * s + out.vy * c;
                assertTrue(fastest(rx, ry, out.omega) <= TOP + 1e-9, "heading " + heading + ", turn " + turn);
                assertEquals(turn, out.omega, 0.0);
            }
        }
    }

    @Test
    void aFieldRelativeRequestIsJudgedWhereTheRobotFaces() {
        // Robot-relative (4, 2.5) is, for a robot facing +y, field-relative (-2.5, 4): the same wheels.
        SwerveSetpointGenerator robot = unramped(Priority.ROTATION);
        SwerveSetpointGenerator field = unramped(Priority.ROTATION);
        robot.generate(new ChassisVelocities(4.0, 2.5, 2.5), 0.02);
        field.generate(new ChassisVelocities(-2.5, 4.0, 2.5), 0.02, Rotation2d.fromDegrees(90));
        assertTrue(robot.getTranslationScale() < 1.0);
        assertEquals(robot.getTranslationScale(), field.getTranslationScale(), 1e-12);
    }

    @Test
    void aTurnThatAloneFillsAModuleLeavesNoTranslation() {
        ChassisVelocities out = unramped(Priority.ROTATION).generate(new ChassisVelocities(2.0, 0.0, 15.0), 0.02);
        // 15 rad/s at 0.39 m is 5.87 m/s: past the wheels' top speed before any translation.
        assertEquals(0.0, out.vx, 0.0);
        assertEquals(15.0, out.omega, 0.0);
    }

    @Test
    void rotationPriorityNeedsTheModules() {
        assertThrows(IllegalArgumentException.class,
                () -> new SwerveSetpointGenerator(TOP, 3.0, 8.0, 8.0, Priority.ROTATION));
        // The default needs none.
        new SwerveSetpointGenerator(TOP, 3.0, 8.0, 8.0, Priority.PROPORTIONAL);
    }

    @Test
    void aHeadingThatIsNotANumberLeavesTheTranslationToTheDrivetrain() {
        SwerveSetpointGenerator gen = unramped(Priority.ROTATION);
        ChassisVelocities out = gen.generate(new ChassisVelocities(5.0, 0.0, 3.0), 0.02,
                Rotation2d.fromRadians(Double.NaN));
        assertEquals(5.0, out.vx, 0.0);
        assertEquals(1.0, gen.getTranslationScale(), 0.0);
    }
}
