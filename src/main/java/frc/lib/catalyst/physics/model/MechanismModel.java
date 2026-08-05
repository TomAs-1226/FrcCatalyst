package frc.lib.catalyst.physics.model;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * One moving link of the robot: how much it weighs, where its mass sits, and how it moves.
 *
 * <p>A robot's centre of mass is not a constant. Raise an elevator 1.2&nbsp;m with 6&nbsp;kg on it and
 * the whole robot's CoM climbs by roughly {@code 6 / totalMass * 1.2} — on a 55&nbsp;kg robot that is
 * 13&nbsp;cm, which is easily enough to turn a comfortable tipping margin into a marginal one. A
 * static {@code centerOfMassHeightMeters} cannot see that. A list of {@code MechanismModel}s can.
 *
 * <p>Each model is a link in a transform tree. It knows its parent, a live transform from the parent's
 * frame into its own, and where its centre of mass sits within its own frame.
 * {@link ArticulatedRobotModel} walks the tree every time you ask, so the answer always reflects where
 * the mechanisms are right now.
 *
 * <pre>{@code
 * MechanismModel elevator = MechanismModel.linear("Elevator", 6.0)
 *     .mountedAt(new Translation3d(0.10, 0.0, 0.15))   // where the rails start, robot frame
 *     .along(new Translation3d(0.0, 0.0, 1.0))         // straight up
 *     .position(elevatorMech::getPosition)             // live, in metres
 *     .build();
 *
 * MechanismModel arm = MechanismModel.rotational("Arm", 4.0)
 *     .childOf("Elevator")                             // rides on the carriage
 *     .mountedAt(new Translation3d(0.0, 0.0, 0.05))    // pivot, relative to the carriage
 *     .comAt(new Translation3d(0.30, 0.0, 0.0))        // CoM 30 cm out along the arm
 *     .angle(() -> Math.toRadians(armMech.getPosition()))
 *     .build();
 * }</pre>
 *
 * <p>Angles follow the robot frame: {@code +X} forward, {@code +Y} left, {@code +Z} up. A rotational
 * mechanism pitches about {@code +Y} by default, which is how an arm or a wrist moves.
 *
 * <p><b>Watch the sign.</b> Right-handed rotation about {@code +Y} takes {@code +X} toward
 * {@code -Z}, so a <em>positive</em> angle pitches the far end <em>down</em>. Most teams zero their
 * arm horizontal and count up as positive, which is the opposite. Two ways to match it, both fine:
 * pass {@code .about(new Translation3d(0, -1, 0))}, or negate in the supplier. Getting this backwards
 * does not throw — it silently lowers the centre of mass when the arm goes up — so check it once
 * against {@link ArticulatedRobotModel#centerOfMassHeightMeters()} with the arm raised.
 *
 * <p>Anything that does not fit the linear or rotational pattern can supply its own transform with
 * {@link #custom(String, double, Supplier)}.
 *
 * <p>Nothing here touches hardware. The suppliers are read on demand, so a unit test drives them from
 * a plain array.
 *
 * @since 1.6.0
 */
public final class MechanismModel {

    private final String name;
    private final String parentName;
    private final double massKg;
    private final Supplier<Transform3d> localTransform;
    private final Translation3d comOffset;

    private MechanismModel(String name, String parentName, double massKg,
                           Supplier<Transform3d> localTransform, Translation3d comOffset) {
        this.name = name;
        this.parentName = parentName;
        this.massKg = massKg;
        this.localTransform = localTransform;
        this.comOffset = comOffset;
    }

    /** The link's name, used as the key in {@link ArticulatedRobotModel} lookups. */
    public String name() {
        return name;
    }

    /** The name of the link this one is mounted on, or {@code null} when it rides on the chassis. */
    public String parentName() {
        return parentName;
    }

    /** Mass of this link in kilograms — the moving part only, not what it is bolted to. */
    public double massKg() {
        return massKg;
    }

    /** The current transform from the parent's frame into this link's frame. Read live. */
    public Transform3d localTransform() {
        return localTransform.get();
    }

    /** Where this link's centre of mass sits within its own frame. */
    public Translation3d centerOfMassOffset() {
        return comOffset;
    }

    /**
     * A link that slides along a fixed axis — an elevator, a telescoping arm, a linear slide.
     *
     * @param name   unique name for this link
     * @param massKg mass of the moving assembly, in kilograms
     */
    public static LinearBuilder linear(String name, double massKg) {
        return new LinearBuilder(name, massKg);
    }

    /**
     * A link that pivots about an axis — an arm, a wrist, a hood.
     *
     * @param name   unique name for this link
     * @param massKg mass of the rotating assembly, in kilograms
     */
    public static RotationalBuilder rotational(String name, double massKg) {
        return new RotationalBuilder(name, massKg);
    }

    /**
     * A link whose motion does not fit the linear or rotational patterns. You supply the transform
     * from the parent frame directly.
     *
     * @param name           unique name for this link
     * @param massKg         mass in kilograms
     * @param localTransform live transform from the parent frame into this link's frame
     */
    public static CustomBuilder custom(String name, double massKg, Supplier<Transform3d> localTransform) {
        return new CustomBuilder(name, massKg, localTransform);
    }

    /**
     * A link that never moves — a battery, a bumper set, a fixed superstructure. Contributes mass at a
     * fixed point, which is the honest way to account for a heavy subassembly that is not at the
     * chassis centre.
     *
     * @param name     unique name for this link
     * @param massKg   mass in kilograms
     * @param position where its centre of mass sits in the robot frame
     */
    public static MechanismModel fixed(String name, double massKg, Translation3d position) {
        requirePositiveMass(name, massKg);
        // The position goes in the transform, not the CoM offset, so that poseOf(name) reports where
        // the link actually is. Putting it in the offset would give the right centre of mass and a
        // link that appears to sit at the robot origin.
        Transform3d at = new Transform3d(position, Rotation3d.kZero);
        return new MechanismModel(name, null, massKg, () -> at, Translation3d.kZero);
    }

    private static void requirePositiveMass(String name, double massKg) {
        if (!(massKg > 0) || Double.isInfinite(massKg)) {
            throw new IllegalArgumentException("massKg for '" + name + "' must be a positive, finite "
                    + "number (got " + massKg + ")");
        }
    }

    /** Shared builder state for the three link flavours. */
    private abstract static class Base<T extends Base<T>> {
        final String name;
        final double massKg;
        String parentName;
        Translation3d mountOffset = Translation3d.kZero;
        Translation3d comOffset = Translation3d.kZero;

        Base(String name, double massKg) {
            this.name = name;
            this.massKg = massKg;
        }

        @SuppressWarnings("unchecked")
        T self() {
            return (T) this;
        }

        /** Mount this link on another one rather than on the chassis. */
        public T childOf(String parentName) {
            this.parentName = parentName;
            return self();
        }

        /** Where this link attaches, in its parent's frame. Defaults to the parent's origin. */
        public T mountedAt(Translation3d mountOffset) {
            this.mountOffset = mountOffset;
            return self();
        }

        /**
         * Where this link's centre of mass sits within its own frame, once it has moved. Defaults to
         * the link's own origin, which is right for an elevator carriage and wrong for an arm — an
         * arm's mass is out along its length, so set it.
         */
        public T comAt(Translation3d comOffset) {
            this.comOffset = comOffset;
            return self();
        }
    }

    /** Builder for a sliding link. */
    public static final class LinearBuilder extends Base<LinearBuilder> {
        private Translation3d axis = new Translation3d(0.0, 0.0, 1.0);
        private DoubleSupplier position;

        LinearBuilder(String name, double massKg) {
            super(name, massKg);
        }

        /**
         * Direction of travel in the parent's frame. Normalised for you, so {@code (0, 0, 1)} and
         * {@code (0, 0, 2)} mean the same thing. Defaults to straight up.
         */
        public LinearBuilder along(Translation3d axis) {
            this.axis = axis;
            return this;
        }

        /** Live extension along {@link #along(Translation3d)}, in metres from the mounting point. */
        public LinearBuilder position(DoubleSupplier position) {
            this.position = position;
            return this;
        }

        /** Validate and build. */
        public MechanismModel build() {
            requirePositiveMass(name, massKg);
            if (position == null) {
                throw new IllegalStateException("position supplier is required for linear mechanism '"
                        + name + "' - without it the link cannot move");
            }
            double norm = axis.getNorm();
            if (!(norm > 0)) {
                throw new IllegalStateException("travel axis for '" + name + "' must not be zero-length");
            }
            Translation3d unit = axis.div(norm);
            Translation3d mount = mountOffset;
            DoubleSupplier live = position;
            return new MechanismModel(name, parentName, massKg,
                    () -> new Transform3d(mount.plus(unit.times(live.getAsDouble())), Rotation3d.kZero),
                    comOffset);
        }
    }

    /** Builder for a pivoting link. */
    public static final class RotationalBuilder extends Base<RotationalBuilder> {
        private Translation3d axis = new Translation3d(0.0, 1.0, 0.0);
        private DoubleSupplier angleRadians;

        RotationalBuilder(String name, double massKg) {
            super(name, massKg);
        }

        /**
         * Axis of rotation in the parent's frame, normalised for you. Defaults to {@code +Y}, which
         * pitches the link the way an arm or wrist moves — remembering that a positive angle about
         * {@code +Y} takes the far end <em>down</em>. Pass {@code (0, -1, 0)} if your mechanism counts
         * up as positive.
         */
        public RotationalBuilder about(Translation3d axis) {
            this.axis = axis;
            return this;
        }

        /** Live joint angle in radians, measured from the link's zero position. */
        public RotationalBuilder angle(DoubleSupplier angleRadians) {
            this.angleRadians = angleRadians;
            return this;
        }

        /** Validate and build. */
        public MechanismModel build() {
            requirePositiveMass(name, massKg);
            if (angleRadians == null) {
                throw new IllegalStateException("angle supplier is required for rotational mechanism '"
                        + name + "' - without it the link cannot move");
            }
            double norm = axis.getNorm();
            if (!(norm > 0)) {
                throw new IllegalStateException("rotation axis for '" + name + "' must not be zero-length");
            }
            Translation3d unit = axis.div(norm);
            Translation3d mount = mountOffset;
            DoubleSupplier live = angleRadians;
            var axisVector = VecBuilder.fill(unit.getX(), unit.getY(), unit.getZ());
            return new MechanismModel(name, parentName, massKg,
                    () -> new Transform3d(mount, new Rotation3d(axisVector, live.getAsDouble())),
                    comOffset);
        }
    }

    /** Builder for a link with a caller-supplied transform. */
    public static final class CustomBuilder extends Base<CustomBuilder> {
        private final Supplier<Transform3d> localTransform;

        CustomBuilder(String name, double massKg, Supplier<Transform3d> localTransform) {
            super(name, massKg);
            this.localTransform = localTransform;
        }

        /** Validate and build. */
        public MechanismModel build() {
            requirePositiveMass(name, massKg);
            if (localTransform == null) {
                throw new IllegalStateException("a transform supplier is required for custom mechanism '"
                        + name + "'");
            }
            return new MechanismModel(name, parentName, massKg, localTransform, comOffset);
        }
    }
}
