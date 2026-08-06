package frc.lib.catalyst.physics.contact;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * The solid parts of the field, and whether a robot is inside one of them.
 *
 * <p>A simulated robot with no collision drives through the perimeter wall and out of the venue, which
 * makes it useless for the questions autos actually raise — does this path clip the hub, does the
 * robot have room to turn there, what happens if it arrives at the wall at speed. This is the geometry
 * half of the answer; {@link ContactResolver} is the physics half.
 *
 * <p>The field is a rectangle with an optional list of static obstacles. The robot is an oriented box,
 * because a robot at 45° needs noticeably more room than one square to the wall, and a simulation that
 * models it as a circle either lets it clip corners or refuses to fit it through gaps it would really
 * fit through.
 *
 * <h2>How the test works</h2>
 *
 * <p>Perimeter contact is exact and cheap: project the robot's oriented box onto the world axes and
 * compare against the walls. Obstacle contact uses the separating axis theorem across four axes — the
 * two world axes and the robot's own two — and reports the axis of least overlap, which is the
 * direction the robot has to move to get out and therefore the contact normal.
 *
 * <pre>{@code
 * CollisionField field = CollisionField.rebuilt()
 *         .addObstacle("red hub", new Translation2d(12.5, 4.0), 0.6, 0.6, ContactMaterial.ALUMINIUM)
 *         .build();
 *
 * field.contact(robotPose, 0.43, 0.43).ifPresent(hit -> {
 *     // hit.normal() points out of the obstacle, hit.penetration() is how far in it is
 * });
 * }</pre>
 *
 * <p>Pure geometry: no HAL, no allocation beyond the returned {@link Optional}.
 *
 * @since 1.8.0
 */
public final class CollisionField {

    /** A static rectangular thing the robot cannot drive through. */
    public record Obstacle(
            String name,
            Translation2d center,
            double halfLength,
            double halfWidth,
            ContactMaterial material) {

        public Obstacle {
            if (!(halfLength > 0) || !(halfWidth > 0)) {
                throw new IllegalArgumentException(
                        "obstacle " + name + " must have positive half-extents");
            }
        }
    }

    /** Where the robot is touching something, and what. */
    public record Contact(
            String what, Translation2d normal, double penetration, ContactMaterial material) {}

    private final double length;
    private final double width;
    private final ContactMaterial wallMaterial;
    private final List<Obstacle> obstacles;

    private CollisionField(Builder builder) {
        this.length = builder.length;
        this.width = builder.width;
        this.wallMaterial = builder.wallMaterial;
        this.obstacles = List.copyOf(builder.obstacles);
    }

    /** The obstacles in this field, in the order they were added. */
    public List<Obstacle> obstacles() {
        return obstacles;
    }

    /** Field length in metres. */
    public double length() {
        return length;
    }

    /** Field width in metres. */
    public double width() {
        return width;
    }

    /**
     * The deepest thing the robot is currently inside, or empty if it is clear.
     *
     * <p>Only the worst contact is reported. Resolving one at a time and stepping again converges,
     * and resolving several simultaneously fights itself in the corners where two walls meet.
     *
     * @param pose       the robot's pose in field coordinates
     * @param halfLength half the bumper-to-bumper length, in metres
     * @param halfWidth  half the bumper-to-bumper width, in metres
     */
    public Optional<Contact> contact(Pose2d pose, double halfLength, double halfWidth) {
        Contact worst = null;

        Contact wall = perimeterContact(pose, halfLength, halfWidth);
        if (wall != null) {
            worst = wall;
        }

        for (Obstacle obstacle : obstacles) {
            Contact hit = obstacleContact(pose, halfLength, halfWidth, obstacle);
            if (hit != null && (worst == null || hit.penetration() > worst.penetration())) {
                worst = hit;
            }
        }
        return Optional.ofNullable(worst);
    }

    /**
     * How far the robot's oriented box reaches along the world axes.
     *
     * <p>This is what makes a diagonal robot need more room than a square one, which is the whole
     * reason the box is not modelled as a circle.
     */
    private static double[] axisExtents(Rotation2d rotation, double halfLength, double halfWidth) {
        double c = Math.abs(rotation.getCos());
        double s = Math.abs(rotation.getSin());
        return new double[] {halfLength * c + halfWidth * s, halfLength * s + halfWidth * c};
    }

    private Contact perimeterContact(Pose2d pose, double halfLength, double halfWidth) {
        double[] extent = axisExtents(pose.getRotation(), halfLength, halfWidth);
        double x = pose.getX();
        double y = pose.getY();

        double intoLeft = extent[0] - x;
        double intoRight = (x + extent[0]) - length;
        double intoNear = extent[1] - y;
        double intoFar = (y + extent[1]) - width;

        double deepest = 0;
        Translation2d normal = null;
        String what = null;

        if (intoLeft > deepest) { deepest = intoLeft; normal = new Translation2d(1, 0); what = "wall x-"; }
        if (intoRight > deepest) { deepest = intoRight; normal = new Translation2d(-1, 0); what = "wall x+"; }
        if (intoNear > deepest) { deepest = intoNear; normal = new Translation2d(0, 1); what = "wall y-"; }
        if (intoFar > deepest) { deepest = intoFar; normal = new Translation2d(0, -1); what = "wall y+"; }

        return normal == null ? null : new Contact(what, normal, deepest, wallMaterial);
    }

    /**
     * Separating axis test between the robot's oriented box and an axis-aligned obstacle.
     *
     * <p>Four axes are enough for two boxes in the plane: each box contributes its own two face
     * normals, and the obstacle's are the world axes. If any axis separates them there is no contact;
     * otherwise the axis of least overlap is the shortest way out.
     */
    private static Contact obstacleContact(
            Pose2d pose, double halfLength, double halfWidth, Obstacle obstacle) {

        Translation2d offset = pose.getTranslation().minus(obstacle.center());
        Rotation2d rotation = pose.getRotation();
        double c = rotation.getCos();
        double s = rotation.getSin();

        // The robot's own two axes, in world coordinates.
        Translation2d[] axes = {
            new Translation2d(1, 0),
            new Translation2d(0, 1),
            new Translation2d(c, s),
            new Translation2d(-s, c),
        };

        double bestOverlap = Double.POSITIVE_INFINITY;
        Translation2d bestAxis = null;

        for (Translation2d axis : axes) {
            double ax = Math.abs(axis.getX());
            double ay = Math.abs(axis.getY());

            // Half-extent of the obstacle projected on this axis.
            double obstacleReach = obstacle.halfLength() * ax + obstacle.halfWidth() * ay;

            // Half-extent of the robot: project each of its own axes onto this one.
            double alongLength = Math.abs(axis.getX() * c + axis.getY() * s);
            double alongWidth = Math.abs(axis.getX() * -s + axis.getY() * c);
            double robotReach = halfLength * alongLength + halfWidth * alongWidth;

            double distance = Math.abs(offset.getX() * axis.getX() + offset.getY() * axis.getY());
            double overlap = obstacleReach + robotReach - distance;
            if (overlap <= 0) {
                return null;   // this axis separates them: no contact, and no need to check the rest
            }
            if (overlap < bestOverlap) {
                bestOverlap = overlap;
                // Point the normal from the obstacle toward the robot.
                double sign = (offset.getX() * axis.getX() + offset.getY() * axis.getY()) < 0 ? -1 : 1;
                bestAxis = axis.times(sign);
            }
        }

        return new Contact(obstacle.name(), bestAxis, bestOverlap, obstacle.material());
    }

    /** An empty field of the given size, with polycarbonate walls. */
    public static Builder rectangular(double lengthMeters, double widthMeters) {
        return new Builder(lengthMeters, widthMeters);
    }

    /** A REBUILT-sized field: 16.54 m by 8.07 m, per the 2026 field drawings. */
    public static Builder rebuilt() {
        return new Builder(16.54, 8.07);
    }

    /** Builder for {@link CollisionField}. */
    public static final class Builder {
        private final double length;
        private final double width;
        private ContactMaterial wallMaterial = ContactMaterial.POLYCARBONATE;
        private final List<Obstacle> obstacles = new ArrayList<>();

        private Builder(double length, double width) {
            if (!(length > 0) || !(width > 0)) {
                throw new IllegalArgumentException("the field must have positive dimensions");
            }
            this.length = length;
            this.width = width;
        }

        /** What the perimeter is made of. Polycarbonate by default. */
        public Builder wallMaterial(ContactMaterial value) {
            this.wallMaterial = value;
            return this;
        }

        /** Add a static rectangular obstacle, centred at {@code center}, axis-aligned. */
        public Builder addObstacle(
                String name,
                Translation2d center,
                double halfLength,
                double halfWidth,
                ContactMaterial material) {
            obstacles.add(new Obstacle(name, center, halfLength, halfWidth, material));
            return this;
        }

        /** Build it. */
        public CollisionField build() {
            return new CollisionField(this);
        }
    }
}
