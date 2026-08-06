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
        int cx = (int) Math.floor(x / cell);
        int cy = (int) Math.floor(y / cell);
        if (cx < 0 || cy < 0 || cx >= cols || cy >= rows) {
            return -1;
        }
        return cy * cols + cx;
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
     * is measured between neighbouring samples. This used to compare every sample against the ground
     * under the robot's centre, which made the limit apply across half the robot's length instead of
     * across one cell: a metre-long ramp rising 2&nbsp;cm per cell reads as a 10&nbsp;cm wall at the
     * front bumper, so nothing on the field was climbable and {@link Verdict#groundHeight()} never
     * left the carpet. Cell to cell, the same ramp is 2&nbsp;cm at a time and a lip is still a lip.
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

        // A cell whose height is above its own clearance never recorded a floor — the height grid
        // caught the bar, not the carpet under it — so it borrows the ground beneath the centre.
        // Off the field, or with the centre itself under a bar, carpet is the only honest guess.
        double centreHeight = heightAt(pose.getX(), pose.getY());
        double centreClearance = clearanceAt(pose.getX(), pose.getY());
        double reference =
                Double.isFinite(centreHeight) && centreClearance >= centreHeight ? centreHeight : 0.0;

        double[] floor = new double[samples];
        double[] roof = new double[samples];
        double[] sampleX = new double[samples];
        double[] sampleY = new double[samples];
        boolean[] offField = new boolean[samples];

        double highestGround = reference;
        double lowestClearance = OPEN_SKY;

        for (int i = 0; i < alongL; i++) {
            for (int j = 0; j < alongW; j++) {
                double lx = ((i - stepsL) / (double) stepsL) * halfLength;
                double ly = ((j - stepsW) / (double) stepsW) * halfWidth;
                double x = pose.getX() + lx * c - ly * s;
                double y = pose.getY() + lx * s + ly * c;

                int k = i * alongW + j;
                sampleX[k] = x;
                sampleY[k] = y;

                double h = heightAt(x, y);
                if (!Double.isFinite(h)) {
                    offField[k] = true;
                    floor[k] = reference;
                    roof[k] = OPEN_SKY;   // the field edge is the verdict; headroom is irrelevant
                    continue;
                }

                double over = clearanceAt(x, y);
                floor[k] = over >= h ? h : reference;
                roof[k] = over;

                highestGround = Math.max(highestGround, floor[k]);
                lowestClearance = Math.min(lowestClearance, over);
            }
        }

        Worst worst = new Worst();

        for (int k = 0; k < samples; k++) {
            if (offField[k]) {
                worst.offer(1e3, k, "off the field");
                continue;
            }
            if (isOpenSky(roof[k])) {
                continue;   // the sentinel is not a ceiling, so subtracting ground from it means nothing
            }
            // A chassis is rigid: it rides on the highest ground under it, so headroom is measured
            // from there rather than from each cell's own floor.
            double headroom = roof[k] - highestGround;
            if (headroom < robotHeight) {
                worst.offer(robotHeight - headroom, k,
                        String.format("%.0f cm clearance", roof[k] * 100));
            }
        }

        // Steps only exist between samples. The lattice is spaced at most one cell apart, so this is
        // the per-cell rise the climb limit is defined against.
        for (int i = 0; i < alongL; i++) {
            for (int j = 0; j < alongW; j++) {
                int k = i * alongW + j;
                if (offField[k]) {
                    continue;
                }
                if (i + 1 < alongL) {
                    offerStep(worst, floor, offField, k, (i + 1) * alongW + j, limit);
                }
                if (j + 1 < alongW) {
                    offerStep(worst, floor, offField, k, k + 1, limit);
                }
            }
        }

        if (worst.at < 0) {
            return new Verdict(false, Translation2d.kZero, 0, highestGround, lowestClearance, "clear");
        }
        // Push back toward the robot's centre — away from whatever it has run into.
        double dx = pose.getX() - sampleX[worst.at];
        double dy = pose.getY() - sampleY[worst.at];
        double norm = Math.hypot(dx, dy);
        Translation2d escape = norm < 1e-6
                ? new Translation2d(1, 0)
                : new Translation2d(dx / norm, dy / norm);
        // Penetration is not measurable from a heightmap, so back out by a cell at a time and let
        // repeated resolution converge rather than inventing a distance.
        return new Verdict(true, escape, cell, highestGround, lowestClearance, worst.reason);
    }

    /** A step between two neighbouring samples, blamed on whichever of the pair is higher. */
    private static void offerStep(
            Worst worst, double[] floor, boolean[] offField, int a, int b, double limit) {
        if (offField[b]) {
            return;   // already reported as off the field, and its floor is a stand-in, not a surface
        }
        double rise = Math.abs(floor[b] - floor[a]);
        if (rise > limit) {
            worst.offer(rise, floor[b] > floor[a] ? b : a, String.format("%.0f cm step", rise * 100));
        }
    }

    /** The worst thing found under the footprint, kept so the verdict can point away from it. */
    private static final class Worst {
        double severity;
        int at = -1;
        String reason;

        void offer(double candidate, int index, String why) {
            if (candidate > severity) {
                severity = candidate;
                at = index;
                reason = why;
            }
        }
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
