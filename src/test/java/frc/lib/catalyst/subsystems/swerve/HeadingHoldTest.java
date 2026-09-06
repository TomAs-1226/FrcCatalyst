package frc.lib.catalyst.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The Catalyst X1 on blocks: enabled, nobody touching the sticks, every module tangential and the
 * wheels spinning. That was the heading lock chasing a cardinal on a robot that could not turn.
 * The first test is that enable, and it fails against the old rule.
 */
class HeadingHoldTest {

    private static final double[] CARDINALS = {0, 90, 180, 270};

    private static PIDController pid() {
        PIDController p = new PIDController(5.0, 0, 0);
        p.enableContinuousInput(-Math.PI, Math.PI);
        p.setTolerance(Math.toRadians(1.5));
        return p;
    }

    @Test
    void aParkedRobotIsNotAskedToTurn() {
        // Heading 7 degrees off a cardinal, sticks at rest: the old rule snapped to 0 and drove.
        HeadingHold.Decision d = HeadingHold.decide(0.0, false, Rotation2d.fromDegrees(7), null,
                CARDINALS, 12, pid(), 10.0, 1.0);
        assertEquals(0.0, d.rotRadPerSec(), 1e-12, "nothing to hold straight while parked");
        assertNull(d.locked(), "and no lock is taken that would fire the moment the robot moves");
        assertFalse(d.driverRotating());
    }

    @Test
    void whileTranslatingTheHeadingIsHeldAndSnappedToANearbyPreset() {
        PIDController p = pid();
        HeadingHold.Decision d = HeadingHold.decide(0.0, true, Rotation2d.fromDegrees(7), null,
                CARDINALS, 12, p, 10.0, 1.0);
        assertNotNull(d.locked());
        assertEquals(0.0, d.locked().getDegrees(), 1e-9, "7 degrees is within 12 of the 0 preset");
        assertTrue(d.rotRadPerSec() < 0, "so it turns back toward 0");
        // The lock is kept across loops rather than re-snapped each time.
        HeadingHold.Decision d2 = HeadingHold.decide(0.0, true, Rotation2d.fromDegrees(3), d.locked(),
                CARDINALS, 12, p, 10.0, 1.0);
        assertEquals(0.0, d2.locked().getDegrees(), 1e-9);
    }

    @Test
    void farFromEveryPresetTheCurrentHeadingIsWhatIsHeld() {
        HeadingHold.Decision d = HeadingHold.decide(0.0, true, Rotation2d.fromDegrees(45), null,
                CARDINALS, 12, pid(), 10.0, 1.0);
        assertEquals(45.0, d.locked().getDegrees(), 1e-9);
        assertEquals(0.0, d.rotRadPerSec(), 1e-9, "already there");
    }

    @Test
    void aSmallErrorIsIgnoredRatherThanChased() {
        HeadingHold.Decision d = HeadingHold.decide(0.0, true, Rotation2d.fromDegrees(0.5),
                Rotation2d.fromDegrees(0), CARDINALS, 12, pid(), 10.0, 1.0);
        assertEquals(0.0, d.rotRadPerSec(), 1e-12, "half a degree is inside the loop's tolerance");
    }

    @Test
    void theCorrectionIsClampedToWhatTheStickCouldAsk() {
        HeadingHold.Decision d = HeadingHold.decide(0.0, true, Rotation2d.fromDegrees(170),
                Rotation2d.fromDegrees(0), null, 0, pid(), 4.0, 0.35);
        assertEquals(1.4, Math.abs(d.rotRadPerSec()), 1e-9, "max rate times the slow-mode multiplier");
    }

    @Test
    void theDriverTurningPassesThroughAndDropsTheLock() {
        HeadingHold.Decision d = HeadingHold.decide(-0.5, true, Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(90), CARDINALS, 12, pid(), 10.0, 1.0);
        assertEquals(-5.0, d.rotRadPerSec(), 1e-9);
        assertNull(d.locked());
        assertTrue(d.driverRotating());
    }
}
