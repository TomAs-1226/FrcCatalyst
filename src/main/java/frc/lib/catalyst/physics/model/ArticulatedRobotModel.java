package frc.lib.catalyst.physics.model;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The whole robot as a tree of masses, so the centre of mass moves when the mechanisms do.
 *
 * <p>{@link RobotModel} carries one {@code centerOfMassHeightMeters}, and for a robot that stays low
 * that is enough. It stops being enough the moment something heavy goes up: raise a 6&nbsp;kg elevator
 * carriage 1.2&nbsp;m on a 55&nbsp;kg robot and the true CoM climbs about 13&nbsp;cm, which is the
 * difference between a comfortable tipping margin and a marginal one. This model recomputes the CoM
 * from the live mechanism positions every time you ask.
 *
 * <p>It also answers the other question a mass tree is good for: where a mechanism actually is.
 * {@link #fieldPoseOf(String, Pose2d)} composes the chassis pose with the chain of joint transforms
 * and hands back the field-relative pose of any link — the end-effector pose that placement games
 * need, without hand-rolling a transform chain per season.
 *
 * <pre>{@code
 * ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
 *     .chassis(chassisModel)                      // mass and footprint of everything that never moves
 *     .add(elevatorLink)
 *     .add(armLink)                               // childOf("Elevator")
 *     .build();
 *
 * Translation3d com = robot.centerOfMass();       // robot frame, live
 * double h = robot.centerOfMassHeightMeters();    // what StabilityModel wants
 * Pose3d gripper = robot.fieldPoseOf("Arm", drive.getPose()).orElseThrow();
 * }</pre>
 *
 * <p>Everything is recomputed on call — there is no cached state to go stale, and the cost is a
 * handful of transform multiplications, which is nothing next to a 20&nbsp;ms loop. Pure math, no
 * hardware, no HAL.
 *
 * @since 1.6.0
 */
public final class ArticulatedRobotModel {

    private final RobotModel chassis;
    private final Translation3d chassisCenterOfMass;
    private final Map<String, MechanismModel> links;
    /** Link names ordered parents-before-children, so one pass resolves the whole tree. */
    private final List<String> resolutionOrder;

    private ArticulatedRobotModel(Builder builder) {
        this.chassis = builder.chassis;
        this.chassisCenterOfMass = builder.chassisCenterOfMass != null
                ? builder.chassisCenterOfMass
                : new Translation3d(0.0, 0.0, builder.chassis.centerOfMassHeightMeters());
        this.links = new LinkedHashMap<>(builder.links);
        this.resolutionOrder = topologicalOrder(this.links);
    }

    /** The static chassis model — mass, footprint, wheel radius, friction. */
    public RobotModel chassis() {
        return chassis;
    }

    /** The moving links, in the order they were added. */
    public Map<String, MechanismModel> links() {
        return Map.copyOf(links);
    }

    /**
     * Total mass in kilograms: the chassis plus every link.
     *
     * <p>Note that {@code chassis().massKg()} should be the mass of everything that does <em>not</em>
     * move. Counting the elevator in both places double-counts it and quietly biases every result
     * here toward the chassis.
     */
    public double totalMassKg() {
        double total = chassis.massKg();
        for (MechanismModel link : links.values()) total += link.massKg();
        return total;
    }

    /**
     * Pose of a link in the <b>robot</b> frame, resolved through its parent chain. Empty if no link by
     * that name was registered.
     */
    public Optional<Pose3d> poseOf(String linkName) {
        Map<String, Pose3d> frames = resolveFrames();
        return Optional.ofNullable(frames.get(linkName));
    }

    /**
     * Pose of a link in <b>field</b> coordinates, given where the chassis is.
     *
     * <p>The chassis pose is 2D, so this assumes the robot is flat on the carpet — true enough for
     * everything except the moment it is actually tipping, which is a case you would rather detect
     * than model.
     */
    public Optional<Pose3d> fieldPoseOf(String linkName, Pose2d chassisPose) {
        return poseOf(linkName).map(local -> new Pose3d(chassisPose).transformBy(
                new Transform3d(local.getTranslation(), local.getRotation())));
    }

    /**
     * Centre of mass of the whole robot in the robot frame, in metres — the mass-weighted mean over
     * the chassis and every link at its current position.
     */
    public Translation3d centerOfMass() {
        Map<String, Pose3d> frames = resolveFrames();
        double totalMass = chassis.massKg();
        Translation3d weighted = chassisCenterOfMass.times(chassis.massKg());

        for (MechanismModel link : links.values()) {
            Pose3d frame = frames.get(link.name());
            if (frame == null) continue;
            Translation3d com = frame.getTranslation()
                    .plus(link.centerOfMassOffset().rotateBy(frame.getRotation()));
            weighted = weighted.plus(com.times(link.massKg()));
            totalMass += link.massKg();
        }
        return totalMass > 0 ? weighted.div(totalMass) : chassisCenterOfMass;
    }

    /** Height of the live centre of mass above the carpet, in metres — what a tipping check needs. */
    public double centerOfMassHeightMeters() {
        return centerOfMass().getZ();
    }

    /**
     * How far the live centre of mass has moved from where the static {@link RobotModel} assumes it
     * is, in metres. A large value is the signal that the static model is no longer describing this
     * robot and the articulated one should be driving the stability limits.
     */
    public double centerOfMassShiftMeters() {
        return centerOfMass().minus(chassisCenterOfMass).getNorm();
    }

    /** One line naming the live centre of mass and total mass. */
    public String describe() {
        Translation3d com = centerOfMass();
        return String.format(Locale.ROOT,
                "ArticulatedRobotModel[%.1f kg total, %d link(s), CoM (%.3f, %.3f, %.3f) m]",
                totalMassKg(), links.size(), com.getX(), com.getY(), com.getZ());
    }

    /** Walks the tree once, composing each link's transform onto its parent's frame. */
    private Map<String, Pose3d> resolveFrames() {
        Map<String, Pose3d> frames = new LinkedHashMap<>();
        for (String name : resolutionOrder) {
            MechanismModel link = links.get(name);
            Pose3d parentFrame = link.parentName() == null
                    ? new Pose3d()
                    : frames.getOrDefault(link.parentName(), new Pose3d());
            frames.put(name, parentFrame.transformBy(link.localTransform()));
        }
        return frames;
    }

    /**
     * Orders links parents-before-children. A link naming a parent that was never registered is
     * treated as chassis-mounted rather than dropped, so a typo degrades to a slightly wrong position
     * instead of a silently missing mass.
     */
    private static List<String> topologicalOrder(Map<String, MechanismModel> links) {
        List<String> ordered = new ArrayList<>(links.size());
        List<String> pending = new ArrayList<>(links.keySet());

        while (!pending.isEmpty()) {
            boolean progressed = false;
            for (var iterator = pending.iterator(); iterator.hasNext(); ) {
                String name = iterator.next();
                String parent = links.get(name).parentName();
                boolean ready = parent == null || !links.containsKey(parent) || ordered.contains(parent);
                if (ready) {
                    ordered.add(name);
                    iterator.remove();
                    progressed = true;
                }
            }
            // A cycle (A childOf B childOf A) would loop forever. Break it by taking the rest in
            // declaration order; build() rejects cycles, so this is belt-and-braces.
            if (!progressed) {
                ordered.addAll(pending);
                break;
            }
        }
        return ordered;
    }

    /** Start building. A chassis model is required; links are optional. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link ArticulatedRobotModel}. */
    public static final class Builder {
        private RobotModel chassis;
        private Translation3d chassisCenterOfMass;
        private final Map<String, MechanismModel> links = new LinkedHashMap<>();

        /**
         * The static part of the robot. Its {@code massKg} should cover everything that does not move —
         * frame, drivetrain, battery, bumpers — and <b>not</b> the mechanisms added below.
         */
        public Builder chassis(RobotModel chassis) {
            this.chassis = chassis;
            return this;
        }

        /**
         * Where the chassis's own centre of mass sits in the robot frame. Defaults to directly above
         * the origin at the chassis model's {@code centerOfMassHeightMeters}, which is right for a
         * roughly symmetric drivetrain.
         */
        public Builder chassisCenterOfMass(Translation3d chassisCenterOfMass) {
            this.chassisCenterOfMass = chassisCenterOfMass;
            return this;
        }

        /** Add a moving link. Names must be unique. */
        public Builder add(MechanismModel link) {
            if (link == null) throw new IllegalArgumentException("link must not be null");
            if (links.containsKey(link.name())) {
                throw new IllegalStateException("duplicate mechanism name '" + link.name() + "'");
            }
            links.put(link.name(), link);
            return this;
        }

        /** Validate and build. Throws on a missing chassis or a parent cycle. */
        public ArticulatedRobotModel build() {
            if (chassis == null) {
                throw new IllegalStateException("a chassis RobotModel is required");
            }
            for (MechanismModel link : links.values()) {
                assertNoCycle(link);
            }
            return new ArticulatedRobotModel(this);
        }

        private void assertNoCycle(MechanismModel start) {
            String seen = start.name();
            String current = start.parentName();
            int guard = links.size() + 1;
            while (current != null && guard-- > 0) {
                if (current.equals(seen)) {
                    throw new IllegalStateException("mechanism '" + start.name()
                            + "' is its own ancestor - check the childOf() chain");
                }
                MechanismModel parent = links.get(current);
                if (parent == null) return;   // chassis-mounted, or an unknown parent
                current = parent.parentName();
            }
            if (guard <= 0) {
                throw new IllegalStateException("mechanism parent chain from '" + start.name()
                        + "' does not terminate - check the childOf() chain");
            }
        }
    }

    /** Convenience for a robot with no configured links: everything sits at the chassis CoM. */
    public static ArticulatedRobotModel ofChassisOnly(RobotModel chassis) {
        return builder().chassis(chassis).build();
    }

    /** The rotation-free identity frame, exposed so callers can compose their own transforms. */
    public static Transform3d identity() {
        return new Transform3d(Translation3d.kZero, Rotation3d.kZero);
    }
}
