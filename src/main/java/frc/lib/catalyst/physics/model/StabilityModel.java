package frc.lib.catalyst.physics.model;

import java.util.Locale;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * How close the robot is to going over, computed from where its mass actually is right now.
 *
 * <h2>The check</h2>
 * A robot tips when the combined effect of gravity and its own acceleration pushes the resultant
 * ground reaction outside the rectangle its wheels sit on. Work in the robot frame, with the centre of
 * mass at {@code (cx, cy, cz)} and the chassis accelerating at {@code a}. The pseudo-force on the mass
 * is {@code -m·a}, gravity is {@code -m·g·ẑ}, and the line of the resultant meets the carpet at
 *
 * <pre>
 * zmp = (cx, cy) - (cz / g) · (ax, ay)
 * </pre>
 *
 * <p>That point — the zero-moment point — is where the ground has to push back. While it stays inside
 * the wheel rectangle every wheel carries load and the robot is planted. When it reaches an edge, the
 * wheels on the far side are carrying nothing, and past it the robot rotates about that edge.
 *
 * <p>{@link #tipMarginMeters} is the distance from the ZMP to the nearest edge: how much further the
 * robot could push before the inside wheels unload. It is in metres, so it is a number a driver can be
 * shown and a guard can threshold.
 *
 * <h2>Why this and not the simple formula</h2>
 * {@link DrivetrainModel#maxTippingAccelerationMpsSq()} uses {@code g·halfWidth/comHeight}. That is
 * this same calculation with two assumptions baked in: the centre of mass is exactly above the
 * geometric centre, and it never moves. Both are fine for a low robot with nothing extended and both
 * are wrong the moment an elevator goes up. Fed an {@link ArticulatedRobotModel}, this class sees the
 * mass move and the margin shrink.
 *
 * <pre>{@code
 * StabilityModel stability = new StabilityModel(articulatedRobot);
 *
 * double margin = stability.tipMarginMeters(state.fieldAcceleration(), state.pose().getRotation());
 * double limit  = stability.maxAccelerationMpsSq(new Translation2d(1, 0));   // hardest safe forward
 * double[] load = stability.wheelLoadsNewtons(robotRelativeAccel);           // FL, FR, BL, BR
 * }</pre>
 *
 * <p>Pure math over live suppliers — no hardware, no HAL.
 *
 * @since 1.6.0
 */
public final class StabilityModel {

    /** Wheel order used by every array this class returns: front-left, front-right, back-left, back-right. */
    public static final int FRONT_LEFT = 0;
    /** @see #FRONT_LEFT */
    public static final int FRONT_RIGHT = 1;
    /** @see #FRONT_LEFT */
    public static final int BACK_LEFT = 2;
    /** @see #FRONT_LEFT */
    public static final int BACK_RIGHT = 3;

    private final ArticulatedRobotModel robot;

    /**
     * @param robot the mass tree whose live centre of mass drives every result here
     */
    public StabilityModel(ArticulatedRobotModel robot) {
        if (robot == null) throw new IllegalArgumentException("robot model must not be null");
        this.robot = robot;
    }

    /** A stability model for a robot with no articulated links — a fixed centre of mass. */
    public static StabilityModel ofChassisOnly(RobotModel chassis) {
        return new StabilityModel(ArticulatedRobotModel.ofChassisOnly(chassis));
    }

    /** The mass tree behind these results. */
    public ArticulatedRobotModel robot() {
        return robot;
    }

    /** Half the footprint front-to-back, in metres — the {@code x} half-extent of the support rectangle. */
    public double halfLengthMeters() {
        return robot.chassis().wheelBaseMeters() / 2.0;
    }

    /** Half the footprint side-to-side, in metres — the {@code y} half-extent of the support rectangle. */
    public double halfWidthMeters() {
        return robot.chassis().trackWidthMeters() / 2.0;
    }

    /**
     * Where the ground reaction acts, in the robot frame, for a given robot-relative acceleration.
     * Inside the wheel rectangle means planted; outside means tipping.
     */
    public Translation2d zeroMomentPoint(Translation2d robotRelativeAccel) {
        Translation3d com = robot.centerOfMass();
        Translation2d accel = robotRelativeAccel == null ? Translation2d.kZero : robotRelativeAccel;
        double lever = com.getZ() / RobotModel.GRAVITY;
        return new Translation2d(com.getX() - lever * accel.getX(),
                                 com.getY() - lever * accel.getY());
    }

    /**
     * Distance from the zero-moment point to the nearest edge of the wheel rectangle, in metres.
     * Positive means planted with that much room; zero means the inside wheels have just unloaded;
     * negative means the robot is past the tipping point.
     *
     * @param robotRelativeAccel chassis acceleration in the robot frame, m/s^2
     */
    public double tipMarginMeters(Translation2d robotRelativeAccel) {
        Translation2d zmp = zeroMomentPoint(robotRelativeAccel);
        double marginX = halfLengthMeters() - Math.abs(zmp.getX());
        double marginY = halfWidthMeters() - Math.abs(zmp.getY());
        return Math.min(marginX, marginY);
    }

    /**
     * The same, taking the field-relative acceleration that {@link
     * frc.lib.catalyst.physics.PhysicalRobotState#fieldAcceleration()} reports, plus the heading to
     * rotate it into the robot frame.
     */
    public double tipMarginMeters(Translation2d fieldAccel, Rotation2d heading) {
        return tipMarginMeters(toRobotFrame(fieldAccel, heading));
    }

    /** True once the tipping margin has run out — at least one wheel is carrying no load. */
    public boolean isTipping(Translation2d robotRelativeAccel) {
        return tipMarginMeters(robotRelativeAccel) <= 0.0;
    }

    /**
     * The hardest the robot may accelerate in {@code direction} before a wheel lifts, in m/s^2.
     *
     * <p>Accelerating along {@code d} slides the zero-moment point along {@code -d}, so this is a ray
     * cast from the resting centre of mass to the edge of the wheel rectangle, scaled by
     * {@code g / comHeight}. Returns {@code 0} if the robot is already unstable standing still, and
     * {@link Double#POSITIVE_INFINITY} for a robot whose centre of mass is on the carpet — a
     * degenerate model, not a real robot.
     *
     * @param direction robot-relative direction of travel; magnitude is ignored
     */
    public double maxAccelerationMpsSq(Translation2d direction) {
        if (direction == null || direction.getNorm() < 1e-9) return 0.0;
        Translation3d com = robot.centerOfMass();
        if (com.getZ() <= 1e-9) return Double.POSITIVE_INFINITY;

        Translation2d unit = direction.div(direction.getNorm());
        // The ZMP starts at the resting CoM and travels along -unit as acceleration grows.
        double distance = distanceToBoundary(new Translation2d(com.getX(), com.getY()), unit.unaryMinus());
        if (distance <= 0) return 0.0;
        return distance * RobotModel.GRAVITY / com.getZ();
    }

    /**
     * The hardest safe acceleration in the worst direction, in m/s^2 — the number to use when the
     * direction is not known ahead of time. This is what a conservative dynamic limit should be
     * built from.
     */
    public double worstCaseAccelerationMpsSq() {
        double worst = Double.POSITIVE_INFINITY;
        for (Translation2d direction : new Translation2d[]{
                new Translation2d(1, 0), new Translation2d(-1, 0),
                new Translation2d(0, 1), new Translation2d(0, -1)}) {
            worst = Math.min(worst, maxAccelerationMpsSq(direction));
        }
        return worst;
    }

    /**
     * Normal load on each wheel in newtons, in {@link #FRONT_LEFT} order.
     *
     * <p>Four contact points on a rigid body are statically indeterminate — the real split depends on
     * suspension and frame flex, which Catalyst does not model. This uses the standard bilinear
     * distribution, which is the unique split that is symmetric and reproduces both the total weight
     * and the moment about each axis. It is exact for a symmetric, evenly-sprung robot and a good
     * approximation otherwise.
     *
     * <p>A load of zero means that wheel has just lifted; the calculation clamps at zero rather than
     * reporting a wheel pulling the robot down, which is what the unclamped formula would imply.
     */
    public double[] wheelLoadsNewtons(Translation2d robotRelativeAccel) {
        Translation2d zmp = zeroMomentPoint(robotRelativeAccel);
        double weight = robot.totalMassKg() * RobotModel.GRAVITY;
        double a = halfLengthMeters();
        double b = halfWidthMeters();

        double px = a > 0 ? zmp.getX() / a : 0.0;
        double py = b > 0 ? zmp.getY() / b : 0.0;

        double[] loads = new double[4];
        loads[FRONT_LEFT]  = weight / 4.0 * (1 + px) * (1 + py);
        loads[FRONT_RIGHT] = weight / 4.0 * (1 + px) * (1 - py);
        loads[BACK_LEFT]   = weight / 4.0 * (1 - px) * (1 + py);
        loads[BACK_RIGHT]  = weight / 4.0 * (1 - px) * (1 - py);
        for (int i = 0; i < loads.length; i++) loads[i] = Math.max(0.0, loads[i]);
        return loads;
    }

    /**
     * The lightest-loaded wheel's share of an even split, from 1.0 (perfectly balanced) down to 0.0
     * (that wheel is off the carpet). A single number for "how evenly is the robot planted".
     */
    public double loadBalance(Translation2d robotRelativeAccel) {
        double[] loads = wheelLoadsNewtons(robotRelativeAccel);
        double even = robot.totalMassKg() * RobotModel.GRAVITY / 4.0;
        if (even <= 0) return 0.0;
        double lightest = Double.POSITIVE_INFINITY;
        for (double load : loads) lightest = Math.min(lightest, load);
        return Math.max(0.0, Math.min(1.0, lightest / even));
    }

    /** Rotate a field-relative vector into the robot frame. */
    public static Translation2d toRobotFrame(Translation2d fieldVector, Rotation2d heading) {
        if (fieldVector == null) return Translation2d.kZero;
        return fieldVector.rotateBy(heading.unaryMinus());
    }

    /** One line naming the live centre of mass height and the worst-direction acceleration limit. */
    public String describe() {
        return String.format(Locale.ROOT,
                "StabilityModel[CoM h=%.3f m, worst-case tip limit %.1f m/s^2, margin at rest %.3f m]",
                robot.centerOfMassHeightMeters(), worstCaseAccelerationMpsSq(),
                tipMarginMeters(Translation2d.kZero));
    }

    /**
     * Distance from {@code origin} to the boundary of the wheel rectangle along {@code unitDirection}.
     * Negative when the origin already sits outside the rectangle.
     */
    private double distanceToBoundary(Translation2d origin, Translation2d unitDirection) {
        double a = halfLengthMeters();
        double b = halfWidthMeters();
        if (Math.abs(origin.getX()) > a || Math.abs(origin.getY()) > b) return -1.0;

        double distance = Double.POSITIVE_INFINITY;
        distance = Math.min(distance, axisDistance(origin.getX(), unitDirection.getX(), a));
        distance = Math.min(distance, axisDistance(origin.getY(), unitDirection.getY(), b));
        return distance;
    }

    /** Distance along one axis from {@code p} to the slab {@code [-half, half]} travelling at {@code d}. */
    private static double axisDistance(double p, double d, double half) {
        if (Math.abs(d) < 1e-12) return Double.POSITIVE_INFINITY;   // never crosses this pair of edges
        double target = d > 0 ? half : -half;
        return (target - p) / d;
    }
}
