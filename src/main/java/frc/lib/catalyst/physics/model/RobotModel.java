package frc.lib.catalyst.physics.model;

import edu.wpi.first.math.util.Units;

/**
 * The handful of physical facts about your robot that the rest of Physics Core reasons from.
 *
 * <p>Deliberately small. Every field here is something a team can measure in an afternoon with a
 * bathroom scale and a tape measure — mass, footprint, wheel radius, roughly how high the weight
 * sits. There is no CAD import, no inertia tensor, no per-link mass tree; those belong to the
 * articulated whole-robot model, which is a later phase. What is here is enough to answer the
 * questions Phase 1 actually asks: how hard can this robot accelerate before the wheels break loose,
 * and before it tips.
 *
 * <pre>{@code
 * RobotModel model = RobotModel.builder()
 *     .massKg(54.4)                       // robot + battery + bumpers, on a scale
 *     .footprintMeters(0.74, 0.74)        // frame perimeter, track width x wheelbase
 *     .wheelRadiusInches(2.0)
 *     .centerOfMassHeightMeters(0.20)     // measure it, or estimate a quarter of robot height
 *     .build();
 * }</pre>
 *
 * <p>The defaults describe a typical mid-weight FRC swerve robot, so a team that only knows its mass
 * still gets sensible numbers out. Every default is stated on the builder method that sets it, and
 * {@link #describe()} prints what was actually used.
 *
 * @since 1.5.0
 */
public final class RobotModel {

    /** Standard gravity, m/s^2. */
    public static final double GRAVITY = 9.80665;

    private final double massKg;
    private final double trackWidthMeters;
    private final double wheelBaseMeters;
    private final double wheelRadiusMeters;
    private final double centerOfMassHeightMeters;
    private final double coefficientOfFriction;
    private final double momentOfInertiaKgM2;

    private RobotModel(Builder builder) {
        this.massKg = builder.massKg;
        this.trackWidthMeters = builder.trackWidthMeters;
        this.wheelBaseMeters = builder.wheelBaseMeters;
        this.wheelRadiusMeters = builder.wheelRadiusMeters;
        this.centerOfMassHeightMeters = builder.centerOfMassHeightMeters;
        this.coefficientOfFriction = builder.coefficientOfFriction;
        this.momentOfInertiaKgM2 = builder.momentOfInertiaKgM2 > 0
                ? builder.momentOfInertiaKgM2
                : estimateMomentOfInertia(builder.massKg, builder.trackWidthMeters, builder.wheelBaseMeters);
    }

    /**
     * Rotational inertia of a uniform rectangular slab about its centre: {@code m(w^2 + l^2) / 12}.
     *
     * <p>A real robot is not uniform — the battery and the drivetrain sit low and outboard — so this
     * runs perhaps 10-20% light. It is a starting value, not a measurement; a team that has done a
     * rotational characterization should pass the measured figure to
     * {@link Builder#momentOfInertiaKgM2(double)} instead.
     */
    public static double estimateMomentOfInertia(double massKg, double trackWidthMeters, double wheelBaseMeters) {
        return massKg * (trackWidthMeters * trackWidthMeters + wheelBaseMeters * wheelBaseMeters) / 12.0;
    }

    /** Total robot mass in kilograms, including battery and bumpers. */
    public double massKg() {
        return massKg;
    }

    /** Left-to-right distance between wheel contact patches, in metres. */
    public double trackWidthMeters() {
        return trackWidthMeters;
    }

    /** Front-to-back distance between wheel contact patches, in metres. */
    public double wheelBaseMeters() {
        return wheelBaseMeters;
    }

    /** Drive wheel radius in metres. */
    public double wheelRadiusMeters() {
        return wheelRadiusMeters;
    }

    /** Height of the centre of mass above the carpet, in metres. Drives the tipping limit. */
    public double centerOfMassHeightMeters() {
        return centerOfMassHeightMeters;
    }

    /** Wheel-on-carpet coefficient of friction. Drives the traction limit. */
    public double coefficientOfFriction() {
        return coefficientOfFriction;
    }

    /** Rotational inertia about the vertical axis, kg&middot;m^2 — measured if given, estimated if not. */
    public double momentOfInertiaKgM2() {
        return momentOfInertiaKgM2;
    }

    /** Half the shorter footprint dimension: the tipping lever arm, in metres. */
    public double tippingHalfWidthMeters() {
        return Math.min(trackWidthMeters, wheelBaseMeters) / 2.0;
    }

    /** One line naming every value in use, so a log says what the model actually was. */
    public String describe() {
        return String.format(java.util.Locale.ROOT,
                "RobotModel[%.1f kg, %.2fx%.2f m, wheel r=%.3f m, CoM h=%.2f m, mu=%.2f, I=%.2f kg*m^2]",
                massKg, trackWidthMeters, wheelBaseMeters, wheelRadiusMeters,
                centerOfMassHeightMeters, coefficientOfFriction, momentOfInertiaKgM2);
    }

    /** Start building a model. Only {@link Builder#massKg(double)} has no usable default. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link RobotModel}. */
    public static final class Builder {
        private double massKg = Double.NaN;
        private double trackWidthMeters = 0.6;
        private double wheelBaseMeters = 0.6;
        private double wheelRadiusMeters = Units.inchesToMeters(2.0);
        private double centerOfMassHeightMeters = 0.20;
        private double coefficientOfFriction = 0.9;
        private double momentOfInertiaKgM2 = 0.0;

        /** Total mass in kilograms — robot, battery, and bumpers, as it competes. Required. */
        public Builder massKg(double massKg) {
            this.massKg = massKg;
            return this;
        }

        /** Total mass in pounds, converted for you. */
        public Builder massPounds(double massPounds) {
            this.massKg = Units.lbsToKilograms(massPounds);
            return this;
        }

        /**
         * Distance between wheel contact patches, side to side and front to back, in metres.
         * Defaults to a 0.6 x 0.6 m square — a common swerve module spacing inside a 28" frame.
         */
        public Builder footprintMeters(double trackWidthMeters, double wheelBaseMeters) {
            this.trackWidthMeters = trackWidthMeters;
            this.wheelBaseMeters = wheelBaseMeters;
            return this;
        }

        /** Drive wheel radius in metres. Defaults to 2 inches. */
        public Builder wheelRadiusMeters(double wheelRadiusMeters) {
            this.wheelRadiusMeters = wheelRadiusMeters;
            return this;
        }

        /** Drive wheel radius in inches. Defaults to 2 inches. */
        public Builder wheelRadiusInches(double wheelRadiusInches) {
            this.wheelRadiusMeters = Units.inchesToMeters(wheelRadiusInches);
            return this;
        }

        /**
         * Height of the centre of mass above the carpet, in metres. Defaults to 0.20 m, which suits a
         * low robot with a stowed superstructure. Raise it if you carry an elevator up — a tall
         * extension moves this a long way, and the tipping limit moves with it.
         */
        public Builder centerOfMassHeightMeters(double centerOfMassHeightMeters) {
            this.centerOfMassHeightMeters = centerOfMassHeightMeters;
            return this;
        }

        /**
         * Coefficient of friction between wheel and carpet. Defaults to 0.9, a deliberately
         * conservative figure for treaded wheels on FRC carpet — measured values run 0.9 to 1.1, and
         * guessing low means the derived acceleration limits err toward being safe.
         */
        public Builder coefficientOfFriction(double coefficientOfFriction) {
            this.coefficientOfFriction = coefficientOfFriction;
            return this;
        }

        /**
         * Measured rotational inertia about the vertical axis, kg&middot;m^2. Leave it unset and
         * {@link RobotModel#estimateMomentOfInertia} fills in a uniform-slab estimate from mass and
         * footprint.
         */
        public Builder momentOfInertiaKgM2(double momentOfInertiaKgM2) {
            this.momentOfInertiaKgM2 = momentOfInertiaKgM2;
            return this;
        }

        /** Validate and build. Throws if mass is missing, or if any dimension is not positive. */
        public RobotModel build() {
            require(massKg, "massKg", "weigh the robot as it competes");
            require(trackWidthMeters, "trackWidthMeters", "measure between wheel contact patches");
            require(wheelBaseMeters, "wheelBaseMeters", "measure between wheel contact patches");
            require(wheelRadiusMeters, "wheelRadiusMeters", "half the wheel diameter");
            require(centerOfMassHeightMeters, "centerOfMassHeightMeters", "height of the CoM above carpet");
            require(coefficientOfFriction, "coefficientOfFriction", "wheel-on-carpet friction, around 0.9");
            return new RobotModel(this);
        }

        private static void require(double value, String name, String hint) {
            if (Double.isNaN(value)) {
                throw new IllegalStateException(name + " is required - " + hint);
            }
            if (!(value > 0) || Double.isInfinite(value)) {
                throw new IllegalStateException(name + " must be a positive, finite number (got "
                        + value + ") - " + hint);
            }
        }
    }
}
