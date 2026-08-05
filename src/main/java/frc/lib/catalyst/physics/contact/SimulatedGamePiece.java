package frc.lib.catalyst.physics.contact;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * A game piece that falls, bounces, rolls, and gets shoved by a robot.
 *
 * <p>Enough physics to answer the questions a simulation actually gets asked — where does a shot land,
 * does that ball roll out of the zone, what happens when the robot drives through a pile — and no
 * more. It is a sphere with a material, in a rectangular field with a floor and four walls.
 *
 * <p>The behaviour that makes it worth having is that surfaces differ. The same piece dropped on
 * carpet stops in a bounce or two; thrown at the polycarbonate wall it comes back into play. That
 * falls out of {@link ContactMaterial} rather than being special-cased, so a team that measures their
 * own coefficients gets their own field.
 *
 * <pre>{@code
 * SimulatedGamePiece fuel = SimulatedGamePiece.builder()
 *         .position(new Translation3d(8.0, 4.0, 1.4))
 *         .velocity(new Translation3d(3.5, 0.0, 2.0))
 *         .radius(0.12)
 *         .massKg(0.27)
 *         .build();
 *
 * while (!fuel.isAtRest()) {
 *     fuel.step(0.02);
 * }
 * Translation2d landed = fuel.groundPosition();
 * }</pre>
 *
 * <p>Pure Java: no HAL, no NetworkTables, no scheduler. It runs in a unit test.
 *
 * @since 1.8.0
 */
public final class SimulatedGamePiece {

    private static final double GRAVITY = 9.80665;

    /** Below this speed, with the piece on the floor, it is called at rest and stops integrating. */
    private static final double AT_REST_SPEED = 0.05;

    private final double radius;
    private final double massKg;
    private final ContactMaterial material;
    private final ContactMaterial floorMaterial;
    private final ContactMaterial wallMaterial;
    private final double fieldLength;
    private final double fieldWidth;
    private final double dragCoefficient;

    private Translation3d position;
    private Translation3d velocity;
    private int contacts = 0;

    private SimulatedGamePiece(Builder builder) {
        this.radius = builder.radius;
        this.massKg = builder.massKg;
        this.material = builder.material;
        this.floorMaterial = builder.floorMaterial;
        this.wallMaterial = builder.wallMaterial;
        this.fieldLength = builder.fieldLength;
        this.fieldWidth = builder.fieldWidth;
        this.dragCoefficient = builder.dragCoefficient;
        this.position = builder.position;
        this.velocity = builder.velocity;
    }

    /**
     * Advance by one timestep.
     *
     * <p>Order matters: integrate first, then resolve whatever that motion drove the piece into. The
     * other way round leaves a piece that has already sunk through the floor by the time anything
     * looks at it.
     *
     * @param dt timestep in seconds
     */
    public void step(double dt) {
        if (!(dt > 0)) {
            throw new IllegalArgumentException("dt must be positive, got " + dt);
        }

        boolean onFloor = position.getZ() <= radius + 1e-6;
        if (isAtRest()) {
            return;
        }

        // Gravity, plus a linear drag term. Linear rather than quadratic because game pieces are slow
        // enough and light enough that the difference is far below the uncertainty in the coefficient.
        Translation3d acceleration = new Translation3d(0, 0, -GRAVITY).minus(velocity.times(dragCoefficient));
        velocity = velocity.plus(acceleration.times(dt));
        position = position.plus(velocity.times(dt));

        resolveFloor();
        resolveWalls();

        onFloor = position.getZ() <= radius + 1e-6;
        if (onFloor) {
            // Rolling resistance only applies while actually touching the floor, and only to the part
            // of the motion that is along it.
            double loss = material.rollingResistanceWith(floorMaterial) * dt;
            double keep = Math.max(0.0, 1.0 - loss);
            velocity = new Translation3d(velocity.getX() * keep, velocity.getY() * keep, velocity.getZ());
        }
    }

    private void resolveFloor() {
        if (position.getZ() > radius) {
            return;
        }
        position = new Translation3d(position.getX(), position.getY(), radius);
        var after = ContactResolver.resolveAgainstStatic(
                velocity, new Translation3d(0, 0, 1), massKg, material, floorMaterial);
        if (after.resolved()) {
            contacts++;
        }
        velocity = after.velocityA();
    }

    private void resolveWalls() {
        double x = position.getX();
        double y = position.getY();
        Translation3d normal = null;

        if (x < radius) {
            position = new Translation3d(radius, y, position.getZ());
            normal = new Translation3d(1, 0, 0);
        } else if (x > fieldLength - radius) {
            position = new Translation3d(fieldLength - radius, y, position.getZ());
            normal = new Translation3d(-1, 0, 0);
        }

        if (normal != null) {
            applyStatic(normal, wallMaterial);
            x = position.getX();
        }

        normal = null;
        if (y < radius) {
            position = new Translation3d(x, radius, position.getZ());
            normal = new Translation3d(0, 1, 0);
        } else if (y > fieldWidth - radius) {
            position = new Translation3d(x, fieldWidth - radius, position.getZ());
            normal = new Translation3d(0, -1, 0);
        }
        if (normal != null) {
            applyStatic(normal, wallMaterial);
        }
    }

    private void applyStatic(Translation3d normal, ContactMaterial surface) {
        var after = ContactResolver.resolveAgainstStatic(velocity, normal, massKg, material, surface);
        if (after.resolved()) {
            contacts++;
        }
        velocity = after.velocityA();
    }

    /**
     * Shove the piece with a robot, if the robot is touching it.
     *
     * <p>The robot is treated as an axis-aligned box in its own frame — good enough, because a bumper
     * corner and a bumper face push a ball in noticeably different directions and that is the part
     * worth getting right. The robot is not slowed in return: a 55&nbsp;kg robot barely notices a
     * 270&nbsp;g ball, and pretending otherwise adds a term smaller than the model's own error.
     *
     * @param robotPose     where the robot is, in field coordinates
     * @param robotVelocity the robot's field-relative velocity, in m/s
     * @param halfLength    half the robot's bumper-to-bumper length, in metres
     * @param halfWidth     half the robot's bumper-to-bumper width, in metres
     * @return true if contact happened
     */
    public boolean interactWithRobot(
            Pose2d robotPose, Translation2d robotVelocity, double halfLength, double halfWidth) {

        // Into the robot's frame, where the box is axis-aligned and the maths is trivial.
        Translation2d offset = new Translation2d(position.getX(), position.getY())
                .minus(robotPose.getTranslation())
                .rotateBy(robotPose.getRotation().unaryMinus());

        double dx = Math.abs(offset.getX()) - halfLength;
        double dy = Math.abs(offset.getY()) - halfWidth;
        if (dx > radius || dy > radius) {
            return false;
        }
        // Outside a corner, the true separation is the diagonal distance rather than either axis.
        if (dx > 0 && dy > 0 && Math.hypot(dx, dy) > radius) {
            return false;
        }

        // Push out along whichever face is nearest — the shallowest penetration.
        Translation2d localNormal = dx > dy
                ? new Translation2d(Math.signum(offset.getX()), 0)
                : new Translation2d(0, Math.signum(offset.getY()));
        Translation2d fieldNormal = localNormal.rotateBy(robotPose.getRotation());

        Translation3d normal = new Translation3d(fieldNormal.getX(), fieldNormal.getY(), 0);
        var after = ContactResolver.resolve(
                velocity,
                new Translation3d(robotVelocity.getX(), robotVelocity.getY(), 0),
                normal,
                massKg,
                ContactResolver.IMMOVABLE,
                material,
                ContactMaterial.BUMPER);

        velocity = after.velocityA();
        if (after.resolved()) {
            contacts++;
            // Lift it clear of the bumper so the next step does not resolve the same contact again.
            double push = radius - Math.max(0.0, Math.max(dx, dy)) + 1e-3;
            position = position.plus(new Translation3d(
                    fieldNormal.getX() * push, fieldNormal.getY() * push, 0));
            return true;
        }
        return false;
    }

    /** Where it is, in metres, with z measured from the carpet to the centre of the piece. */
    public Translation3d position() {
        return position;
    }

    /** Where it is on the carpet, ignoring height. */
    public Translation2d groundPosition() {
        return new Translation2d(position.getX(), position.getY());
    }

    /** How fast it is going, in m/s. */
    public Translation3d velocity() {
        return velocity;
    }

    /** How many contacts it has resolved since it was created or last reset. */
    public int contactCount() {
        return contacts;
    }

    /** Whether it has settled: on the floor and barely moving. */
    public boolean isAtRest() {
        return position.getZ() <= radius + 1e-6 && velocity.getNorm() < AT_REST_SPEED;
    }

    /** Put it somewhere and stop it. */
    public void reset(Translation3d where) {
        this.position = where;
        this.velocity = Translation3d.kZero;
        this.contacts = 0;
    }

    /** Start building a game piece. Everything has a default; a bare {@code build()} is a fuel ball. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link SimulatedGamePiece}. */
    public static final class Builder {
        private double radius = 0.12;
        private double massKg = 0.27;
        private ContactMaterial material = ContactMaterial.FOAM_GAME_PIECE;
        private ContactMaterial floorMaterial = ContactMaterial.CARPET;
        private ContactMaterial wallMaterial = ContactMaterial.POLYCARBONATE;
        private double fieldLength = 16.54;
        private double fieldWidth = 8.07;
        private double dragCoefficient = 0.08;
        private Translation3d position = new Translation3d(8.27, 4.03, 0.12);
        private Translation3d velocity = Translation3d.kZero;

        /** Radius in metres. */
        public Builder radius(double value) {
            this.radius = value;
            return this;
        }

        /** Mass in kilograms. */
        public Builder massKg(double value) {
            this.massKg = value;
            return this;
        }

        /** The piece's own surface. */
        public Builder material(ContactMaterial value) {
            this.material = value;
            return this;
        }

        /** What the floor is made of. Carpet, unless you are simulating a practice field on concrete. */
        public Builder floorMaterial(ContactMaterial value) {
            this.floorMaterial = value;
            return this;
        }

        /** What the perimeter is made of. */
        public Builder wallMaterial(ContactMaterial value) {
            this.wallMaterial = value;
            return this;
        }

        /** Field size in metres. Defaults to the REBUILT carpet, 16.54 by 8.07. */
        public Builder field(double lengthMeters, double widthMeters) {
            this.fieldLength = lengthMeters;
            this.fieldWidth = widthMeters;
            return this;
        }

        /** Linear drag, per second. Zero for a vacuum; the default suits a light foam ball. */
        public Builder dragCoefficient(double value) {
            this.dragCoefficient = value;
            return this;
        }

        /** Starting position, z measured to the centre of the piece. */
        public Builder position(Translation3d value) {
            this.position = value;
            return this;
        }

        /** Starting velocity. */
        public Builder velocity(Translation3d value) {
            this.velocity = value;
            return this;
        }

        /** Validate and build. */
        public SimulatedGamePiece build() {
            if (!(radius > 0)) {
                throw new IllegalStateException("radius must be positive, got " + radius);
            }
            if (!(massKg > 0)) {
                throw new IllegalStateException(
                        "massKg must be positive, got " + massKg + "; impulses divide by it");
            }
            if (!(fieldLength > 2 * radius) || !(fieldWidth > 2 * radius)) {
                throw new IllegalStateException("the field must be bigger than the game piece");
            }
            if (!(dragCoefficient >= 0)) {
                throw new IllegalStateException(
                        "dragCoefficient must be at least 0, got " + dragCoefficient);
            }
            return new SimulatedGamePiece(this);
        }
    }
}
