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
 * <p>Generate the file with {@code npm run field-collision} in the Catalyst Console repo, which reads
 * the season's CAD. Hand-editing it defeats the purpose.
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

    /** Clearance recorded for a cell with nothing above it at all. */
    private static final double OPEN_SKY = 9.0;

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

    /** What the field does to a robot standing at a particular place. */
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
     * metres above the carpet and perfectly happy; a robot against a 6&nbsp;cm lip is stuck. So the
     * test compares each cell against the ground under the robot's centre, which is the surface it is
     * actually standing on.
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
        double standing = heightAt(pose.getX(), pose.getY());
        if (!Double.isFinite(standing)) {
            standing = 0.0;   // centre is off the field; treat the carpet as the reference
        }

        Rotation2d rotation = pose.getRotation();
        double c = rotation.getCos();
        double s = rotation.getSin();

        int stepsL = Math.max(1, (int) Math.ceil(halfLength / cell));
        int stepsW = Math.max(1, (int) Math.ceil(halfWidth / cell));

        double worstStep = 0.0;
        double lowestClearance = OPEN_SKY;
        double highestGround = standing;
        Translation2d escape = null;
        String reason = null;

        for (int i = -stepsL; i <= stepsL; i++) {
            for (int j = -stepsW; j <= stepsW; j++) {
                double lx = (i / (double) stepsL) * halfLength;
                double ly = (j / (double) stepsW) * halfWidth;
                double x = pose.getX() + lx * c - ly * s;
                double y = pose.getY() + lx * s + ly * c;

                double h = heightAt(x, y);
                boolean offField = !Double.isFinite(h);
                double step = offField ? Double.POSITIVE_INFINITY : h - standing;

                double over = clearanceAt(x, y);
                if (over < lowestClearance) {
                    lowestClearance = over;
                }
                if (!offField && h > highestGround && step <= limit) {
                    highestGround = h;   // riding up onto something climbable
                }

                boolean tooTall = step > limit;
                boolean tooLow = over < robotHeight && over < step + robotHeight;

                if (tooTall || (tooLow && over < robotHeight)) {
                    double severity = offField ? 1e3 : Math.max(step, robotHeight - over);
                    if (severity > worstStep) {
                        worstStep = severity;
                        // Push back toward the robot's centre — away from whatever it has run into.
                        double dx = pose.getX() - x;
                        double dy = pose.getY() - y;
                        double norm = Math.hypot(dx, dy);
                        escape = norm < 1e-6
                                ? new Translation2d(1, 0)
                                : new Translation2d(dx / norm, dy / norm);
                        reason = offField ? "off the field"
                                : tooTall ? String.format("%.0f cm step", step * 100)
                                : String.format("%.0f cm clearance", over * 100);
                    }
                }
            }
        }

        if (escape == null) {
            return new Verdict(false, Translation2d.kZero, 0, highestGround, lowestClearance, "clear");
        }
        // Penetration is not measurable from a heightmap, so back out by a cell at a time and let
        // repeated resolution converge rather than inventing a distance.
        return new Verdict(true, escape, cell, highestGround, lowestClearance, reason);
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
        int colon = json.indexOf(':', at);
        int end = colon + 1;
        while (end < json.length() && "-+.eE0123456789".indexOf(json.charAt(end)) >= 0) {
            end++;
        }
        while (end < json.length() && json.charAt(colon + 1) == ' ') {
            colon++;
        }
        return Double.parseDouble(json.substring(colon + 1, end).trim());
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
