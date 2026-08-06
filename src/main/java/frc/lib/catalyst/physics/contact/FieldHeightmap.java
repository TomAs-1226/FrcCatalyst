package frc.lib.catalyst.physics.contact;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;

/**
 * Field collision taken from the field CAD rather than hand-placed boxes.
 *
 * <p>Two grids over the playing surface. <b>Height</b> is the highest surface in each cell, measured
 * from the carpet. <b>Clearance</b> is the underside of the lowest thing overhead. Between them they
 * describe everything a drivetrain cares about:
 *
 * <ul>
 *   <li>flat carpet — height ~0, clearance high</li>
 *   <li>a ramp — height rises gradually; drivable while the slope stays under what wheels can climb</li>
 *   <li>a trench — height ~0 but clearance low; drivable only by a robot short enough to fit</li>
 *   <li>structure — height above climbing height, or no clearance at all; blocked</li>
 * </ul>
 *
 * <p>The clearance grid matters more than it looks. An FRC trench is not dug into the floor: the floor
 * is carpet and there is a bar above it. A height-only model sees the bar, calls the cell solid, and
 * refuses to let anything through — which is the opposite of what happens on a real field.
 *
 * <p>Which is why the two grids are read together rather than one after the other. Depending on what
 * the CAD gave it, the extractor may record a trench cell's height as the carpet under the bar or as
 * the top of the bar itself — it is the highest surface either way. A cell whose height sits above its
 * own clearance is therefore not a floor at all, and must not be judged as one.
 *
 * <p>Generate the file with {@code npm run field-collision} in the Catalyst Console repo, which reads
 * the season's CAD. Hand-editing it defeats the purpose.
 *
 * <p>A cell with nothing above it carries a sentinel rather than a measurement: the generator writes
 * 9999 mm, which this class reads back as {@link #OPEN_SKY}. Any clearance at or above that value is
 * open sky and stops nothing, however tall the robot. The constant is public because the sentinel is
 * part of the file format, and a consumer comparing {@link Verdict#lowestClearance()} against the
 * value in the file needs both sides to be the same number. They were not: this class used to call
 * open sky 9.0 m, so a cell the file described as 9.999 came back as 9.0 and every such comparison
 * missed.
 *
 * <pre>{@code
 * FieldHeightmap field = FieldHeightmap.load(Path.of("src/main/deploy/field-collision.json"));
 *
 * var verdict = field.test(robotPose, 0.43, 0.43, ROBOT_HEIGHT, CLIMB_LIMIT);
 * if (verdict.blocked()) {
 *     // verdict.normal() is the way out
 * } else {
 *     robotZ = verdict.groundHeight();   // riding up the ramp
 * }
 * }</pre>
 *
 * <p>Pure Java: no HAL, no NetworkTables, and the parser is hand-rolled so it pulls in no JSON
 * dependency a robot project might not already have.
 *
 * @since 1.9.0
 */
public final class FieldHeightmap {

    /** Height difference across one cell that counts as a wall rather than a slope, in metres. */
    private static final double DEFAULT_CLIMB_LIMIT = 0.06;

    /**
     * Clearance that means "nothing overhead", in metres — the 9999 mm the generator writes.
     *
     * <p>Compare against this, not against a height of your own choosing, when asking whether a cell
     * is open. Read the class javadoc before changing it: the number is the file format's, not ours.
     */
    public static final double OPEN_SKY = 9.999;

    /**
     * Is this clearance the open-sky sentinel?
     *
     * <p>At or above, not equal to: a map may legitimately record more headroom than the sentinel, and
     * the file stores whole millimetres, so half of one is enough slack to survive the round trip
     * through the generator's floating-point crop without hinging on exact equality.
     */
    private static boolean isOpenSky(double clearance) {
        return clearance >= OPEN_SKY - 5e-4;
    }

    private final double cell;
    private final int cols;
    private final int rows;
    private final double length;
    private final double width;
    private final double[] height;
    private final double[] clearance;

    private FieldHeightmap(
            double cell, int cols, int rows, double length, double width,
            double[] height, double[] clearance) {
        this.cell = cell;
        this.cols = cols;
        this.rows = rows;
        this.length = length;
        this.width = width;
        this.height = height;
        this.clearance = clearance;
    }

    /**
     * What the field does to a robot standing at a particular place.
     *
     * <p>{@code lowestClearance} is {@link #OPEN_SKY} when nothing was found overhead, and is capped
     * there — a footprint entirely under open sky reports the sentinel rather than whatever larger
     * number happened to be in the file.
     */
    public record Verdict(
            boolean blocked,
            Translation2d normal,
            double penetration,
            double groundHeight,
            double lowestClearance,
            String reason) {}

    /** Field length in metres. */
    public double lengthMeters() {
        return length;
    }

    /** Field width in metres. */
    public double widthMeters() {
        return width;
    }

    /** Grid resolution in metres. */
    public double cellMeters() {
        return cell;
    }

    /** Surface height at a point, in metres above the carpet. Outside the field reads as a wall. */
    public double heightAt(double x, double y) {
        int index = indexOf(x, y);
        return index < 0 ? Double.POSITIVE_INFINITY : height[index];
    }

    /** Height of the lowest thing overhead, in metres above the carpet. */
    public double clearanceAt(double x, double y) {
        int index = indexOf(x, y);
        return index < 0 ? 0.0 : clearance[index];
    }

    private int indexOf(double x, double y) {
        return indexOf(cellX(x), cellY(y));
    }

    private int indexOf(int cx, int cy) {
        return onGrid(cx, cy) ? cy * cols + cx : -1;
    }

    private int cellX(double x) {
        return (int) Math.floor(x / cell);
    }

    private int cellY(double y) {
        return (int) Math.floor(y / cell);
    }

    private boolean onGrid(int cx, int cy) {
        return cx >= 0 && cy >= 0 && cx < cols && cy < rows;
    }

    /**
     * Can a robot of this size stand here, and if not, which way is out?
     *
     * <p>The footprint is sampled on the grid rather than tested analytically. At 5&nbsp;cm cells a
     * bumper covers well over a hundred samples, which is far more resolution than the model itself
     * carries, and it means ramps, trenches and walls all fall out of the same loop instead of needing
     * three separate intersection routines.
     *
     * <p>What counts as blocked is a <em>step</em>, not a height. A robot at the top of a ramp is two
     * metres above the carpet and perfectly happy; a robot against a 6&nbsp;cm lip is stuck. The step
     * is measured against the ground the chassis is riding on — see {@link #supportHeight} — so a
     * metre of 2&nbsp;cm-per-cell ramp is climbable while the same 20&nbsp;cm delivered in one cell is
     * not.
     *
     * <h2>The normal is a property of the geometry, not of the pose</h2>
     *
     * <p>This used to point from the worst sample toward the robot's centre. That describes where a
     * sample sits inside the footprint and carries no information about which way the surface faces.
     * Driven head-on into a flat, axis-aligned perimeter wall it was 22&deg; out on average and 45&deg;
     * out at worst, so a robot that hit a wall square came away moving sideways — and because the
     * winning sample was decided by a strict {@code >} over tied severities, a sub-millimetre change
     * of pose swapped it for its mirror image and swung the normal 90&deg; between one call and the
     * next. That swing is what the jitter was.
     *
     * <p>What replaces it is an occupancy gradient: every blocked cell votes with the directions in
     * which it has a passable neighbour. A flat wall votes unanimously and gives exactly the surface
     * normal; a corner gives the bisector of the two walls; standing off the grid gives a vector
     * pointing back onto it, which is what stops a stranded robot being flung further out.
     *
     * <h2>The penetration is measured</h2>
     *
     * <p>It used to be one cell, with a comment saying depth was not measurable from a heightmap. It
     * is: every cell is an axis-aligned box of known size, so walking the escape ray cell by cell
     * gives the exact distance a sample has to travel to get clear. The old constant was 45&times; too
     * large on a 1&nbsp;mm graze and 4&times; too small on a 12&nbsp;cm one, and a solver fed a
     * different wrong distance on every contact is a solver that oscillates.
     *
     * @param pose        where the robot is
     * @param halfLength  half its bumper-to-bumper length, in metres
     * @param halfWidth   half its bumper-to-bumper width, in metres
     * @param robotHeight how tall it is, in metres — what has to fit under a trench
     * @param climbLimit  the biggest step it can drive up, in metres; pass 0 for the default
     */
    public Verdict test(
            Pose2d pose, double halfLength, double halfWidth, double robotHeight, double climbLimit) {

        double limit = climbLimit > 0 ? climbLimit : DEFAULT_CLIMB_LIMIT;

        Rotation2d rotation = pose.getRotation();
        double c = rotation.getCos();
        double s = rotation.getSin();

        int stepsL = Math.max(1, (int) Math.ceil(halfLength / cell));
        int stepsW = Math.max(1, (int) Math.ceil(halfWidth / cell));
        int alongL = 2 * stepsL + 1;
        int alongW = 2 * stepsW + 1;
        int samples = alongL * alongW;

        double[] sampleX = new double[samples];
        double[] sampleY = new double[samples];
        double lowestClearance = OPEN_SKY;

        for (int i = 0; i < alongL; i++) {
            for (int j = 0; j < alongW; j++) {
                double lx = ((i - stepsL) / (double) stepsL) * halfLength;
                double ly = ((j - stepsW) / (double) stepsW) * halfWidth;

                int k = i * alongW + j;
                sampleX[k] = pose.getX() + lx * c - ly * s;
                sampleY[k] = pose.getY() + lx * s + ly * c;

                int index = indexOf(sampleX[k], sampleY[k]);
                if (index >= 0) {
                    lowestClearance = Math.min(lowestClearance, clearance[index]);
                }
            }
        }

        double support = supportHeight(pose, sampleX, sampleY, limit);

        int[] blocked = new int[samples];
        int blockedCount = 0;
        double severity = 0;
        String reason = "clear";

        for (int k = 0; k < samples; k++) {
            int cx = cellX(sampleX[k]);
            int cy = cellY(sampleY[k]);
            if (!impassable(cx, cy, support, robotHeight, limit)) {
                continue;
            }
            blocked[blockedCount++] = k;

            double candidate;
            String why;
            int index = indexOf(cx, cy);
            if (index < 0) {
                candidate = 1e3;   // nothing on the field outranks having left it
                why = "off the field";
            } else {
                double h = height[index];
                double over = clearance[index];
                double rise = (over >= h ? h : support) - support;
                double headroom = isOpenSky(over) ? Double.POSITIVE_INFINITY : over - support;
                // Both can be true of one cell — a low bar on top of a kerb — so report the worse.
                double asStep = rise > limit ? rise : 0;
                double asCeiling = headroom < robotHeight ? robotHeight - headroom : 0;
                candidate = Math.max(asStep, asCeiling);
                why = asStep >= asCeiling
                        ? String.format("%.0f cm step", rise * 100)
                        : String.format("%.0f cm clearance", over * 100);
            }
            if (candidate > severity) {
                severity = candidate;
                reason = why;
            }
        }

        if (blockedCount == 0) {
            return new Verdict(false, Translation2d.kZero, 0, support, lowestClearance, "clear");
        }

        Translation2d escape = escapeDirection(
                pose, sampleX, sampleY, blocked, blockedCount,
                support, robotHeight, limit, halfLength, halfWidth);

        // One translation along the normal has to free every blocked sample at once, so the depth of
        // the contact is the deepest of them, not the average and not a constant.
        double reach = 2 * Math.hypot(halfLength, halfWidth) + 4 * cell;
        double penetration = 0;
        for (int b = 0; b < blockedCount; b++) {
            int k = blocked[b];
            penetration = Math.max(penetration, exitDistance(
                    sampleX[k], sampleY[k], escape.getX(), escape.getY(),
                    support, robotHeight, limit, reach));
        }

        return new Verdict(true, escape, penetration, support, lowestClearance, reason);
    }

    /**
     * The ground the chassis is actually riding on, in metres above the carpet.
     *
     * <p>A rigid chassis rides on the highest thing under it, so this is a maximum — but only over
     * ground it can reach. Taking it over every sample let a wall the bumper was merely touching count
     * as floor: brush the 2&nbsp;m perimeter and {@link Verdict#groundHeight()} claimed the robot was
     * two metres up, and that inflated support then poisoned every headroom judgement measured
     * against it.
     *
     * <p>Swept rather than computed in one pass because the answer feeds its own test — the top of a
     * ramp is only within a climbable step of the support once the middle of the ramp has been
     * accepted. Two or three sweeps settle it; the loop bound is there so a pathological map cannot
     * spin, and the early return is the normal exit.
     */
    private double supportHeight(Pose2d pose, double[] sampleX, double[] sampleY, double limit) {
        // A cell whose height is above its own clearance never recorded a floor — the height grid
        // caught the bar, not the carpet under it — so carpet is the only honest seed.
        int centre = indexOf(pose.getX(), pose.getY());
        double support = centre >= 0 && clearance[centre] >= height[centre] ? height[centre] : 0.0;

        for (int sweep = 0; sweep < sampleX.length; sweep++) {
            double best = support;
            for (int k = 0; k < sampleX.length; k++) {
                int index = indexOf(sampleX[k], sampleY[k]);
                if (index < 0) {
                    continue;
                }
                double h = height[index];
                if (clearance[index] < h) {
                    continue;   // a bar overhead, not a floor to stand on
                }
                if (h - support > limit) {
                    continue;   // a wall is something to hit, not something to ride up
                }
                best = Math.max(best, h);
            }
            if (best == support) {
                return support;
            }
            support = best;
        }
        return support;
    }

    /**
     * Can a chassis riding on {@code support} occupy this cell?
     *
     * <p>Deliberately says nothing about where the robot is. The escape normal is built out of this
     * predicate, and a direction derived from the robot's own footprint is not a surface normal.
     */
    private boolean impassable(int cx, int cy, double support, double robotHeight, double limit) {
        int index = indexOf(cx, cy);
        if (index < 0) {
            return true;   // off the field is as solid as anything on it
        }
        double h = height[index];
        double over = clearance[index];
        double f = over >= h ? h : support;
        if (f - support > limit) {
            return true;
        }
        return !isOpenSky(over) && over - support < robotHeight;
    }

    /**
     * Which way is out, as a unit vector.
     *
     * <p>Every blocked cell votes once for each direction in which it has a passable neighbour, so the
     * result is the outward normal of the occupied region averaged over the footprint. A flat wall is
     * unanimous, a corner produces the bisector, and the vote is unchanged by a pose that moves within
     * a cell — which is what makes it stable enough to solve against.
     */
    private Translation2d escapeDirection(
            Pose2d pose, double[] sampleX, double[] sampleY, int[] blocked, int blockedCount,
            double support, double robotHeight, double limit, double halfLength, double halfWidth) {

        // A robot buried deeper than its immediate neighbourhood sees nothing at one cell out, so the
        // stencil widens until it finds free space or has searched its own footprint.
        int furthest = (int) Math.ceil(Math.hypot(halfLength, halfWidth) / cell) + 2;
        for (int radius = 1; radius <= furthest; radius++) {
            double gx = 0;
            double gy = 0;
            for (int b = 0; b < blockedCount; b++) {
                int k = blocked[b];
                int cx = cellX(sampleX[k]);
                int cy = cellY(sampleY[k]);
                if (!impassable(cx + radius, cy, support, robotHeight, limit)) gx += 1;
                if (!impassable(cx - radius, cy, support, robotHeight, limit)) gx -= 1;
                if (!impassable(cx, cy + radius, support, robotHeight, limit)) gy += 1;
                if (!impassable(cx, cy - radius, support, robotHeight, limit)) gy -= 1;
            }
            double norm = Math.hypot(gx, gy);
            if (norm > 1e-9) {
                return new Translation2d(gx / norm, gy / norm);
            }
        }

        // Nothing free anywhere near. If the robot is off the grid entirely, the grid itself is the
        // only direction worth having; otherwise fall back on the old centre-of-mass guess, which is
        // wrong but bounded.
        double toGridX = clamp(pose.getX(), 0, cols * cell) - pose.getX();
        double toGridY = clamp(pose.getY(), 0, rows * cell) - pose.getY();
        double norm = Math.hypot(toGridX, toGridY);
        if (norm > 1e-9) {
            return new Translation2d(toGridX / norm, toGridY / norm);
        }

        double bx = 0;
        double by = 0;
        for (int b = 0; b < blockedCount; b++) {
            bx += sampleX[blocked[b]];
            by += sampleY[blocked[b]];
        }
        double dx = pose.getX() - bx / blockedCount;
        double dy = pose.getY() - by / blockedCount;
        norm = Math.hypot(dx, dy);
        return norm < 1e-9 ? new Translation2d(1, 0) : new Translation2d(dx / norm, dy / norm);
    }

    /**
     * How far this sample has to travel along {@code (nx, ny)} before it is standing somewhere legal.
     *
     * <p>A heightmap has exact analytic geometry — each cell is an axis-aligned box of side
     * {@link #cellMeters()} — so this is a grid walk rather than an estimate, and it handles a wall
     * several cells thick without needing to know how thick it is.
     *
     * @return the exact distance, or zero if there is nothing passable within {@code reach}: a
     *     direction that leads nowhere means the normal is wrong, not that the robot is a metre deep,
     *     and answering {@code reach} there teleports it across the field
     */
    private double exitDistance(
            double sx, double sy, double nx, double ny,
            double support, double robotHeight, double limit, double reach) {

        double travelled = 0;
        double x = sx;
        double y = sy;
        // Bounded by the ray crossing at most two cell boundaries per cell of reach; the count is
        // generous so that a near-axis-aligned ray cannot exhaust it before the distance does.
        for (int guard = 0; guard < 256; guard++) {
            int cx = cellX(x);
            int cy = cellY(y);
            if (!impassable(cx, cy, support, robotHeight, limit)) {
                return travelled;
            }
            double toX = nx > 0 ? ((cx + 1) * cell - x) / nx
                    : nx < 0 ? (cx * cell - x) / nx : Double.POSITIVE_INFINITY;
            double toY = ny > 0 ? ((cy + 1) * cell - y) / ny
                    : ny < 0 ? (cy * cell - y) / ny : Double.POSITIVE_INFINITY;
            double stride = Math.min(toX, toY);
            if (!Double.isFinite(stride)) {
                return 0;   // no direction at all; nothing to measure along it
            }
            // Land inside the next cell rather than on its edge, where the floor could go either way.
            travelled += stride + 1e-9;
            if (travelled > reach) {
                return 0;
            }
            x = sx + nx * travelled;
            y = sy + ny * travelled;
        }
        return 0;
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    /** Load a heightmap written by {@code npm run field-collision}. */
    public static FieldHeightmap load(Path path) throws IOException {
        return parse(Files.readString(path, StandardCharsets.UTF_8));
    }

    /**
     * Parse the generated JSON.
     *
     * <p>Hand-rolled on purpose. The schema is fixed and generated by a script in this project, so a
     * full JSON library would be a dependency taken on for one file whose exact shape we control.
     */
    public static FieldHeightmap parse(String json) {
        double cell = number(json, "cellMeters");
        int cols = (int) number(json, "cols");
        int rows = (int) number(json, "rows");
        double length = number(json, "lengthMeters");
        double width = number(json, "widthMeters");
        double[] height = millimetreArray(json, "heightsMillimetres", cols * rows);
        double[] clearance = millimetreArray(json, "clearanceMillimetres", cols * rows);

        if (cols <= 0 || rows <= 0 || !(cell > 0)) {
            throw new IllegalArgumentException("heightmap has no usable grid: " + cols + "x" + rows);
        }
        return new FieldHeightmap(cell, cols, rows, length, width, height, clearance);
    }

    private static double number(String json, String key) {
        int at = json.indexOf('"' + key + '"');
        if (at < 0) {
            throw new IllegalArgumentException("heightmap is missing \"" + key + "\"");
        }
        // Skip whitespace BEFORE scanning the number, not after. The other way round, a single space
        // after the colon ends the digit scan immediately and then walks the start past the end, so
        // the substring throws. It only ever worked because the generator emits no whitespace — which
        // is a property of one script in another repository, not something to rely on.
        int start = json.indexOf(':', at) + 1;
        while (start < json.length() && Character.isWhitespace(json.charAt(start))) {
            start++;
        }
        int end = start;
        while (end < json.length() && "-+.eE0123456789".indexOf(json.charAt(end)) >= 0) {
            end++;
        }
        if (end == start) {
            throw new IllegalArgumentException("heightmap has no number after \"" + key + "\"");
        }
        return Double.parseDouble(json.substring(start, end));
    }

    private static double[] millimetreArray(String json, String key, int expected) {
        int at = json.indexOf('"' + key + '"');
        if (at < 0) {
            // Clearance is optional: a map generated before it existed still describes solid geometry.
            double[] open = new double[expected];
            java.util.Arrays.fill(open, OPEN_SKY);
            return open;
        }
        int open = json.indexOf('[', at);
        int close = json.indexOf(']', open);
        String body = json.substring(open + 1, close);

        double[] out = new double[expected];
        int index = 0;
        int start = 0;
        for (int i = 0; i <= body.length(); i++) {
            if (i == body.length() || body.charAt(i) == ',') {
                if (i > start && index < expected) {
                    out[index++] = Integer.parseInt(body.substring(start, i).trim()) / 1000.0;
                }
                start = i + 1;
            }
        }
        if (index != expected) {
            throw new IllegalArgumentException(
                    "heightmap \"" + key + "\" has " + index + " cells, expected " + expected);
        }
        return out;
    }

    /** Load a heightmap if one is there, without making a missing file an error. */
    public static Optional<FieldHeightmap> tryLoad(Path path) {
        try {
            return Files.exists(path) ? Optional.of(load(path)) : Optional.empty();
        } catch (IOException | RuntimeException e) {
            System.err.println("[Catalyst] could not read field heightmap: " + e.getMessage());
            return Optional.empty();
        }
    }
}
