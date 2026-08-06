package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.catalyst.physics.contact.FieldHeightmap;
import org.junit.jupiter.api.Test;

/** Pure-Java tests for CAD-derived field collision. No HAL, no NetworkTables, no robot. */
class FieldHeightmapTest {

    /**
     * A 2 x 1 m test field on 10 cm cells: flat carpet, a ramp climbing across the middle, a wall, and
     * a stretch of low overhead to drive under.
     *
     * <pre>
     *   x:  0.0 .. 0.6   flat carpet
     *       0.6 .. 1.0   ramp, 3 cm per cell
     *       1.0 .. 1.4   flat top of the ramp
     *       1.4 .. 1.6   wall, 80 cm
     *       1.6 .. 2.0   carpet with a bar 60 cm overhead
     * </pre>
     */
    private static FieldHeightmap testField() {
        int cols = 20, rows = 10;
        StringBuilder h = new StringBuilder();
        StringBuilder c = new StringBuilder();
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                int height;
                int clearance = 9999;
                if (x < 6) height = 0;
                else if (x < 10) height = (x - 5) * 30;          // ramp: 30 mm per 10 cm cell
                else if (x < 14) height = 150;                    // flat top
                else if (x < 16) height = 800;                    // wall
                else { height = 0; clearance = 600; }             // trench: carpet under a low bar
                if (h.length() > 0) { h.append(','); c.append(','); }
                h.append(height);
                c.append(clearance);
            }
        }
        return FieldHeightmap.parse("{"
                + "\"cellMeters\":0.1,\"cols\":" + cols + ",\"rows\":" + rows + ","
                + "\"lengthMeters\":2.0,\"widthMeters\":1.0,"
                + "\"heightsMillimetres\":[" + h + "],"
                + "\"clearanceMillimetres\":[" + c + "]}");
    }

    private static final double HALF = 0.15;   // a small test robot, 30 cm square
    private static final double TALL = 0.9;
    private static final double SHORT = 0.4;

    @Test
    void itParsesTheGeneratedShape() {
        FieldHeightmap f = testField();
        assertEquals(2.0, f.lengthMeters(), 1e-9);
        assertEquals(1.0, f.widthMeters(), 1e-9);
        assertEquals(0.1, f.cellMeters(), 1e-9);
    }

    @Test
    void aBadHeightmapIsRejectedRatherThanHalfLoaded() {
        assertThrows(IllegalArgumentException.class,
                () -> FieldHeightmap.parse("{\"cellMeters\":0.1,\"cols\":4,\"rows\":4,"
                        + "\"lengthMeters\":1,\"widthMeters\":1,\"heightsMillimetres\":[0,0,0]}"));
    }

    @Test
    void flatCarpetIsClear() {
        var v = testField().test(new Pose2d(0.3, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0);
        assertFalse(v.blocked(), v.reason());
        assertEquals(0.0, v.groundHeight(), 1e-9);
    }

    // ------------------------------------------------------------------ ramps

    @Test
    void aGentleRampIsDrivable() {
        // 30 mm per 100 mm cell is a step well under the climb limit, so it should not block.
        var v = testField().test(new Pose2d(0.75, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0);
        assertFalse(v.blocked(), "a ramp must be drivable, got: " + v.reason());
    }

    @Test
    void theRobotRidesUpTheRamp() {
        FieldHeightmap f = testField();
        // Sampled below the point where the footprint reaches the flat top — a 30 cm robot at x=0.85
        // already has its nose on the 150 mm plateau, so it reports the top rather than the slope.
        double low = f.test(new Pose2d(0.3, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0).groundHeight();
        double mid = f.test(new Pose2d(0.75, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0).groundHeight();
        double top = f.test(new Pose2d(1.2, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0).groundHeight();

        assertEquals(0.0, low, 1e-9);
        assertTrue(mid > low, "ground height should rise along the ramp: " + low + " -> " + mid);
        assertTrue(top > mid, "and keep rising to the top: " + mid + " -> " + top);
        assertEquals(0.15, top, 1e-6, "the flat top is 150 mm up");
    }

    @Test
    void aRampTooSteepForTheRobotBlocks() {
        // Same ramp, but a robot that can only manage a 1 cm step.
        var v = testField().test(new Pose2d(0.75, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0.01);
        assertTrue(v.blocked(), "a 3 cm step should stop a robot with a 1 cm climb limit");
    }

    // ------------------------------------------------------------------ walls

    @Test
    void aWallBlocksAndPointsTheWayOut() {
        // Driven up against the wall, not standing on top of it. The reference height is whatever is
        // under the robot's centre, so a robot teleported inside the wall is "standing" on it and
        // sees no step at all — correct for a ramp top, meaningless for a wall, and not a state a
        // robot reaches by driving.
        var v = testField().test(new Pose2d(1.32, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0);
        assertTrue(v.blocked(), "the front of the robot is in an 80 cm wall");
        assertTrue(v.normal().getNorm() > 0.5, "a blocked verdict has to say which way is out");
        assertTrue(v.normal().getX() < 0, "the way out of a wall in +x is back toward -x");
    }

    @Test
    void drivingOffTheFieldIsBlocked() {
        var v = testField().test(new Pose2d(-0.2, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0);
        assertTrue(v.blocked());
        assertEquals("off the field", v.reason());
    }

    // ------------------------------------------------------------------ trenches

    @Test
    void aShortRobotFitsThroughTheTrench() {
        var v = testField().test(new Pose2d(1.8, 0.5, Rotation2d.kZero), HALF, HALF, SHORT, 0);
        assertFalse(v.blocked(),
                "40 cm of robot under a 60 cm bar should pass, got: " + v.reason());
        assertEquals(0.6, v.lowestClearance(), 1e-6);
    }

    @Test
    void aTallRobotDoesNotFitThroughTheTrench() {
        var v = testField().test(new Pose2d(1.8, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0);
        assertTrue(v.blocked(), "90 cm of robot must not fit under a 60 cm bar");
        assertTrue(v.reason().contains("clearance"), "and it should say why: " + v.reason());
    }

    @Test
    void theTrenchFloorIsNotMistakenForAWall() {
        // The whole point of the clearance layer: a height-only map calls this cell solid.
        FieldHeightmap f = testField();
        assertEquals(0.0, f.heightAt(1.8, 0.5), 1e-9);
        assertEquals(0.6, f.clearanceAt(1.8, 0.5), 1e-6);
    }

    @Test
    void openSkyIsNotAnObstruction() {
        assertTrue(testField().clearanceAt(0.3, 0.5) > 5.0);
    }
}
