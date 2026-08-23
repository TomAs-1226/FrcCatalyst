package frc.lib.catalyst.subsystems.vision;

import frc.lib.catalyst.subsystems.vision.GamePieceDetector.Detection;

import org.junit.jupiter.api.Test;

import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The detection geometry, which is pure maths and needs no camera.
 *
 * <p>Worth pinning down because the failure mode is silent: {@code tan()} of an angle just above the
 * horizon returns a large, plausible-looking number rather than an error, so a detector pointed
 * slightly wrong reports a game piece thirty metres away and a behaviour drives at it.
 */
class GamePieceDetectorTest {

    private static Detection at(double tyDegrees) {
        return new Detection("note", 0.9, 0.0, tyDegrees, 5.0, 1);
    }

    @Test
    void aTargetOnTheFloorRangesByTrigonometry() {
        // Camera 1 m up, level, target 45 degrees down: the range equals the height.
        Optional<Double> range = GamePieceDetector.groundRangeMeters(at(-45), 1.0, 0.0);
        assertTrue(range.isPresent());
        assertEquals(1.0, range.get(), 1e-6);
    }

    @Test
    void cameraPitchIsAddedToTheTargetAngle() {
        // Camera pitched 20 degrees down, target 25 degrees below the crosshair: 45 degrees total.
        Optional<Double> range = GamePieceDetector.groundRangeMeters(at(-25), 1.0, -20.0);
        assertTrue(range.isPresent());
        assertEquals(1.0, range.get(), 1e-6);
    }

    @Test
    void aShallowerAngleIsFurtherAway() {
        double near = GamePieceDetector.groundRangeMeters(at(-45), 1.0, 0.0).orElseThrow();
        double far = GamePieceDetector.groundRangeMeters(at(-20), 1.0, 0.0).orElseThrow();
        assertTrue(far > near, "a target closer to the horizon must range further, got "
                + far + " vs " + near);
    }

    @Test
    void aTargetAtOrAboveTheHorizonHasNoGroundRange() {
        // This is the case that matters. Above the horizon there is no floor intersection, and
        // returning a number here would be inventing one.
        assertTrue(GamePieceDetector.groundRangeMeters(at(0), 1.0, 0.0).isEmpty(),
                "a target exactly on the horizon does not meet the floor");
        assertTrue(GamePieceDetector.groundRangeMeters(at(5), 1.0, 0.0).isEmpty(),
                "a target above the horizon does not meet the floor");
        assertTrue(GamePieceDetector.groundRangeMeters(at(-10), 1.0, 15.0).isEmpty(),
                "an upward camera pitch can put a downward target above the horizon too");
    }

    @Test
    void rangeScalesWithCameraHeight() {
        double low = GamePieceDetector.groundRangeMeters(at(-30), 0.5, 0.0).orElseThrow();
        double high = GamePieceDetector.groundRangeMeters(at(-30), 1.0, 0.0).orElseThrow();
        assertEquals(2.0, high / low, 1e-9, "range is linear in camera height");
    }

    @Test
    void bearingIsCcwPositiveToMatchWpilib() {
        // Limelight reports tx positive to the right; WPILib angles are CCW positive, so a target
        // to the right is a negative rotation. Getting this backwards steers the wrong way.
        Detection tenDegreesRight = new Detection("note", 0.9, 10.0, -20.0, 5.0, 1);
        assertEquals(-10.0, tenDegreesRight.bearing().getDegrees(), 1e-9,
                "a target 10 degrees right should be -10 degrees CCW");
    }
}
