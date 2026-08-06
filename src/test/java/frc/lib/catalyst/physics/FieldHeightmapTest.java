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

    /**
     * A field on 10 cm cells whose geometry varies only along x, given per-column millimetres. Every
     * scene below is a corridor, so a column profile is the whole map. Rows are fixed at ten, which
     * makes the field a metre wide — enough for a full-size robot to sit clear of both walls.
     */
    private static FieldHeightmap fromColumns(int[] heightMm, int[] clearanceMm) {
        int cols = heightMm.length, rows = 10;
        StringBuilder h = new StringBuilder();
        StringBuilder c = new StringBuilder();
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                if (h.length() > 0) { h.append(','); c.append(','); }
                h.append(heightMm[x]);
                c.append(clearanceMm[x]);
            }
        }
        return FieldHeightmap.parse("{"
                + "\"cellMeters\":0.1,\"cols\":" + cols + ",\"rows\":" + rows + ","
                + "\"lengthMeters\":" + (cols / 10.0) + ",\"widthMeters\":" + (rows / 10.0) + ","
                + "\"heightsMillimetres\":[" + h + "],"
                + "\"clearanceMillimetres\":[" + c + "]}");
    }

    /**
     * A 3 m corridor with a bar across the middle metre, 600 mm off the carpet. The height grid holds
     * 700 mm there — the top of the bar, which really is the highest surface in those cells. Nothing
     * records the carpet underneath, so a reader that trusts the height grid alone sees a 70 cm wall.
     */
    private static FieldHeightmap barOverCarpetField() {
        int[] h = new int[30], c = new int[30];
        for (int x = 0; x < 30; x++) {
            boolean underBar = x >= 10 && x < 20;
            h[x] = underBar ? 700 : 0;
            c[x] = underBar ? 600 : 9999;
        }
        return fromColumns(h, c);
    }

    /** A 4 m corridor climbing 200 mm over a metre — 20 mm a cell, longer than the robot. */
    private static FieldHeightmap rampField() {
        int[] h = new int[40], c = new int[40];
        for (int x = 0; x < 40; x++) {
            h[x] = x < 10 ? 0 : x < 20 ? (x - 9) * 20 : 200;
            c[x] = 9999;
        }
        return fromColumns(h, c);
    }

    /** The same 200 mm of climb as {@link #rampField()}, delivered as one lip in a single cell. */
    private static FieldHeightmap lipField() {
        int[] h = new int[40], c = new int[40];
        for (int x = 0; x < 40; x++) {
            h[x] = x < 15 ? 0 : 200;
            c[x] = 9999;
        }
        return fromColumns(h, c);
    }

    /** A 4 m corridor with a flat 80 cm wall across everything from x = 1.5 onward. */
    private static FieldHeightmap wallAt(double x0) {
        int[] h = new int[40], c = new int[40];
        for (int x = 0; x < 40; x++) {
            h[x] = x < (int) Math.round(x0 * 10) ? 0 : 800;
            c[x] = 9999;
        }
        return fromColumns(h, c);
    }

    /**
     * Open carpet in the low corner, 80 cm of wall at x &ge; 1.5 or y &ge; 0.6. Two flat faces meeting
     * at a right angle, which is the case a normal built from one sample cannot describe.
     */
    private static FieldHeightmap cornerField() {
        int cols = 40, rows = 20;
        StringBuilder h = new StringBuilder();
        StringBuilder c = new StringBuilder();
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                if (h.length() > 0) { h.append(','); c.append(','); }
                h.append(x >= 15 || y >= 6 ? 800 : 0);
                c.append(9999);
            }
        }
        return FieldHeightmap.parse("{"
                + "\"cellMeters\":0.1,\"cols\":" + cols + ",\"rows\":" + rows + ","
                + "\"lengthMeters\":4.0,\"widthMeters\":2.0,"
                + "\"heightsMillimetres\":[" + h + "],"
                + "\"clearanceMillimetres\":[" + c + "]}");
    }

    private static final double HALF = 0.15;   // a small test robot, 30 cm square
    private static final double BIG = 0.43;    // and a real one, 86 cm square
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
    void aRobotLengthRampIsClimbable() {
        // The climb limit is a step between adjacent ground, not the rise across the footprint. An
        // 86 cm robot on a 20 mm-per-cell ramp spans 180 mm of climb; read across the footprint that
        // is a wall three times the limit, and this test failed for exactly that reason.
        var v = rampField().test(new Pose2d(1.4, 0.5, Rotation2d.kZero), BIG, BIG, TALL, 0);
        assertFalse(v.blocked(), "a ramp gentler than the limit must be drivable, got: " + v.reason());
        assertEquals(0.18, v.groundHeight(), 1e-9, "and the robot rides up it");
    }

    @Test
    void aSingleLipOfTheSameHeightIsNotClimbable() {
        var v = lipField().test(new Pose2d(1.4, 0.5, Rotation2d.kZero), BIG, BIG, TALL, 0);
        assertTrue(v.blocked(), "200 mm in one cell is a wall however long the robot is");
        assertTrue(v.reason().contains("step"), "and it should say why: " + v.reason());
        assertTrue(v.normal().getX() < 0, "the way out of a lip in +x is back toward -x");
    }

    @Test
    void theRampAndTheLipClimbToTheSameHeight() {
        // Otherwise the pair above proves nothing: the difference has to be how the climb is spread.
        assertEquals(0.20, rampField().heightAt(1.95, 0.5), 1e-9);
        assertEquals(0.20, lipField().heightAt(1.95, 0.5), 1e-9);
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
        // Containment reports how far out it is rather than a bare label, because that depth is what
        // recovers the robot in one step instead of nudging it a cell at a time.
        assertTrue(v.reason().contains("outside the field"), v.reason());
        assertEquals(0.35, v.penetration(), 1e-9);
        assertEquals(1.0, v.normal().getX(), 1e-9);
    }

    @Test
    void theRobotCannotLeaveTheMapEvenWhereTheCadHasNoWall() {
        // The CAD's guardrails fall outside the cropped carpet, so on the real map the long sides
        // carry wall in 0.6% of their cells against 51% and 37% on the ends. Containment cannot
        // depend on the terrain having a wall in it.
        FieldHeightmap f = testField();
        for (double y : new double[] {-0.4, 0.5, 1.4}) {
            for (double x : new double[] {-0.4, 1.0, 2.4}) {
                if (x > 0 && x < 2.0 && y > 0 && y < 1.0) continue;
                var v = f.test(new Pose2d(x, y, Rotation2d.kZero), HALF, HALF, TALL, 0);
                assertTrue(v.blocked(), "escaped the map at " + x + ", " + y);
                assertTrue(v.penetration() > 0, "an escape needs a real depth to recover from");
                assertTrue(v.normal().getNorm() > 0.5, "and a direction back in");
            }
        }
    }

    // ------------------------------------------------------- how deep, and which way (see #14)

    @Test
    void thePenetrationIsMeasuredRatherThanAssumedToBeOneCell() {
        FieldHeightmap f = wallAt(1.5);
        // The front bumper is at x + BIG and the wall face is at 1.5, so the overlap is arithmetic and
        // the map should report exactly it. It used to report one cell — 0.1 m — for every one of
        // these, which is 100x too much at the top of the list and too little at the bottom.
        for (double overlap : new double[] {0.001, 0.01, 0.03, 0.08, 0.13}) {
            Pose2d at = new Pose2d(1.5 - BIG + overlap, 0.5, Rotation2d.kZero);
            var v = f.test(at, BIG, BIG, TALL, 0);
            assertTrue(v.blocked(), "the bumper is " + overlap + " m into an 80 cm wall");
            assertEquals(overlap, v.penetration(), 1e-6,
                    "the way out of a " + overlap + " m overlap is " + overlap + " m");
        }
    }

    @Test
    void aClearVerdictHasNoPenetrationToResolve() {
        var v = wallAt(1.5).test(new Pose2d(0.6, 0.5, Rotation2d.kZero), BIG, BIG, TALL, 0);
        assertFalse(v.blocked(), v.reason());
        assertEquals(0.0, v.penetration(), 1e-12);
    }

    @Test
    void theWayOutOfAFlatWallIsTheWallsOwnNormal() {
        FieldHeightmap f = wallAt(1.5);
        // Nudged around by fractions of a cell. The normal used to run from the worst sample to the
        // robot's centre, which on this geometry read 45 degrees out and flipped to its mirror image
        // when a tie changed hands — a 90 degree swing for a sub-millimetre move. That swing let a
        // second and third impulse land in one timestep, and it is what the jitter was made of.
        for (int i = 0; i < 12; i++) {
            double nudge = i * 0.0037;
            var v = f.test(new Pose2d(1.09 + nudge, 0.5 + nudge, Rotation2d.kZero), BIG, BIG, TALL, 0);
            assertTrue(v.blocked());
            assertEquals(-1.0, v.normal().getX(), 1e-12, "nudge " + nudge);
            assertEquals(0.0, v.normal().getY(), 1e-12, "nudge " + nudge + " must not steer sideways");
        }
    }

    @Test
    void theWayOutOfACornerIsTheBisectorOfBothWalls() {
        var v = cornerField().test(new Pose2d(1.4, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0);
        assertTrue(v.blocked());
        assertEquals(-Math.sqrt(0.5), v.normal().getX(), 1e-9);
        assertEquals(-Math.sqrt(0.5), v.normal().getY(), 1e-9);
    }

    @Test
    void theWayOffTheFieldPointsBackOntoIt() {
        // It used to point further out, so a robot that ended up outside was pushed away for ever at
        // a cell a step. Twelve metres past the wall was reachable.
        var v = testField().test(new Pose2d(-0.2, 0.5, Rotation2d.kZero), HALF, HALF, TALL, 0);
        assertTrue(v.blocked());
        assertTrue(v.normal().getX() > 0.5, "the way back on is +x, got " + v.normal());
        assertTrue(v.penetration() > 0.3, "and it is a real distance: " + v.penetration());
    }

    @Test
    void aWallTheBumperIsTouchingIsNotGroundToStandOn() {
        // groundHeight folded every sample's floor into one maximum, walls included, so brushing an
        // 80 cm wall reported a robot standing 80 cm above the carpet it was demonstrably sitting on.
        var v = wallAt(1.5).test(new Pose2d(1.09, 0.5, Rotation2d.kZero), BIG, BIG, TALL, 0);
        assertTrue(v.blocked());
        assertEquals(0.0, v.groundHeight(), 1e-9,
                "the robot is on carpet with its bumper in a wall, not levitating on top of it");
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

    @Test
    void theFilesOpenSkySentinelReadsAsOpenAndNeverBlocks() {
        // The generator writes 9999 mm for "nothing overhead"; the library used to answer 9.0 for
        // those cells, so anything comparing a verdict against the file's own sentinel missed.
        int[] h = new int[30];
        int[] c = new int[30];
        java.util.Arrays.fill(c, 9999);
        FieldHeightmap f = fromColumns(h, c);

        assertEquals(FieldHeightmap.OPEN_SKY, f.clearanceAt(1.5, 0.5), 1e-9,
                "the library's sentinel has to be the number the file writes");

        // Taller than the sentinel itself, which is the case a subtraction would get wrong.
        var v = f.test(new Pose2d(1.5, 0.5, Rotation2d.kZero), BIG, BIG, 40.0, 0);
        assertFalse(v.blocked(), "open sky cannot stop a robot of any height, got: " + v.reason());
        assertEquals(FieldHeightmap.OPEN_SKY, v.lowestClearance(), 1e-9,
                "and the verdict reports the sentinel, not a number of its own");
    }

    // The cases below drive *into* a bar from the carpet outside it, which is where the old ordering
    // showed: the height grid held the top of the bar, that read as a 70 cm step, and the verdict was
    // settled before clearance was ever looked at. Every drive-under passage on the field was solid.

    @Test
    void aShortRobotDrivesUnderABarTheHeightGridCallsSolid() {
        var v = barOverCarpetField().test(new Pose2d(0.95, 0.5, Rotation2d.kZero), BIG, BIG, SHORT, 0);
        assertFalse(v.blocked(),
                "40 cm of robot under a 60 cm bar should pass, got: " + v.reason());
        assertEquals(0.6, v.lowestClearance(), 1e-6);
        assertEquals(0.0, v.groundHeight(), 1e-9, "the bar overhead is not ground to ride up on");
    }

    @Test
    void aTallRobotIsStoppedByTheBarItCannotFitUnder() {
        var v = barOverCarpetField().test(new Pose2d(0.95, 0.5, Rotation2d.kZero), BIG, BIG, TALL, 0);
        assertTrue(v.blocked(), "90 cm of robot must not fit under a 60 cm bar");
        assertTrue(v.reason().contains("clearance"),
                "and it is the clearance that stops it, not a phantom step: " + v.reason());
    }

    @Test
    void theBarIsStillTheHighestSurfaceInTheMap() {
        // If this stops being true the two tests above stop testing the thing they were written for.
        assertEquals(0.7, barOverCarpetField().heightAt(1.5, 0.5), 1e-9);
        assertEquals(0.6, barOverCarpetField().clearanceAt(1.5, 0.5), 1e-6);
    }
}
