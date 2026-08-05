package frc.lib.catalyst.physics.contact;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Impulse-based contact resolution: what two velocities become after a collision.
 *
 * <p>This is the standard rigid-body treatment. A contact is resolved in one instant rather than as a
 * force over time, because at a 20&nbsp;ms loop period a collision is over long before the next
 * sample — spreading it across a timestep produces a robot that visibly sinks into a wall and then
 * gets spat out.
 *
 * <p>The normal impulse removes the approach speed and returns a fraction of it, set by the pair's
 * combined restitution. The tangential impulse tries to remove the sliding speed entirely and is
 * clamped by Coulomb friction — which is what makes a glancing hit on carpet behave differently from
 * the same hit on polycarbonate, rather than only differing in bounce.
 *
 * <h2>Two details that matter more than they look</h2>
 *
 * <p><b>Restitution is suppressed at low speed.</b> A ball resting on carpet is technically in contact
 * with a small approach speed every frame, and a naive resolver bounces it forever, microscopically.
 * Below {@link #RESTING_SPEED} the bounce is dropped to zero so things come to rest.
 *
 * <p><b>Separating contacts are ignored.</b> If the two bodies are already moving apart, there is
 * nothing to resolve; resolving anyway would glue them together.
 *
 * <p>Everything here is pure arithmetic on velocities — no HAL, no NetworkTables, no allocation beyond
 * the returned record.
 *
 * <pre>{@code
 * // A game piece hitting the field wall, which does not move.
 * ContactResolution after = ContactResolver.resolve(
 *         pieceVelocity, Translation3d.kZero,
 *         wallNormal,                       // unit vector, pointing out of the wall
 *         pieceMassKg, ContactResolver.IMMOVABLE,
 *         ContactMaterial.FOAM_GAME_PIECE, ContactMaterial.POLYCARBONATE);
 * pieceVelocity = after.velocityA();
 * }</pre>
 *
 * @since 1.8.0
 */
public final class ContactResolver {

    /**
     * Mass to pass for a body that does not move: the field, a wall, the floor.
     *
     * <p>It is infinity rather than a big number on purpose — the maths divides by mass, so infinity
     * gives exactly zero response and no accumulated rounding error.
     */
    public static final double IMMOVABLE = Double.POSITIVE_INFINITY;

    /**
     * Approach speed below which a contact is treated as resting rather than bouncing, in m/s.
     *
     * <p>Chosen at about the speed a game piece has after its last visible bounce. Lower and objects
     * jitter on the floor forever; much higher and a gentle contact that should bounce does not.
     */
    public static final double RESTING_SPEED = 0.12;

    private ContactResolver() {}

    /** The velocities after a contact. */
    public record ContactResolution(
            Translation3d velocityA,
            Translation3d velocityB,
            double normalImpulse,
            double tangentImpulse,
            boolean sliding) {

        /** Whether the contact actually did anything. */
        public boolean resolved() {
            return normalImpulse > 0.0;
        }
    }

    /**
     * Resolve a contact between two bodies.
     *
     * @param velocityA velocity of body A before the contact, in m/s
     * @param velocityB velocity of body B before the contact, in m/s
     * @param normal    unit vector pointing from B toward A; the direction A is pushed
     * @param massA     mass of A in kg, or {@link #IMMOVABLE}
     * @param massB     mass of B in kg, or {@link #IMMOVABLE}
     * @param materialA the surface on A
     * @param materialB the surface on B
     */
    public static ContactResolution resolve(
            Translation3d velocityA,
            Translation3d velocityB,
            Translation3d normal,
            double massA,
            double massB,
            ContactMaterial materialA,
            ContactMaterial materialB) {

        double normalLength = normal.getNorm();
        if (normalLength < 1e-9) {
            throw new IllegalArgumentException(
                    "contact normal has no direction; it must point from B toward A");
        }
        Translation3d n = normal.div(normalLength);

        double inverseA = Double.isInfinite(massA) ? 0.0 : 1.0 / massA;
        double inverseB = Double.isInfinite(massB) ? 0.0 : 1.0 / massB;
        double inverseSum = inverseA + inverseB;
        if (inverseSum <= 0.0) {
            // Two immovable objects met. Nothing can change, which is the correct answer.
            return new ContactResolution(velocityA, velocityB, 0, 0, false);
        }

        Translation3d relative = velocityA.minus(velocityB);
        double approach = dot(relative, n);
        if (approach >= 0.0) {
            // Already separating — leave it alone rather than pulling them back together.
            return new ContactResolution(velocityA, velocityB, 0, 0, false);
        }

        double restitution =
                -approach < RESTING_SPEED ? 0.0 : materialA.restitutionWith(materialB);
        double normalImpulse = -(1.0 + restitution) * approach / inverseSum;

        // Whatever is left of the relative velocity after removing the normal part is sliding.
        Translation3d tangential = relative.minus(n.times(approach));
        double slidingSpeed = tangential.getNorm();

        Translation3d impulse = n.times(normalImpulse);
        double tangentImpulse = 0.0;
        boolean sliding = false;

        if (slidingSpeed > 1e-9) {
            Translation3d t = tangential.div(slidingSpeed);
            // The impulse that would stop the slide outright.
            double needed = slidingSpeed / inverseSum;
            double limit = materialA.frictionWith(materialB) * normalImpulse;
            // Static if friction can absorb it, kinetic if it cannot — the Coulomb cone.
            tangentImpulse = Math.min(needed, limit);
            sliding = needed > limit;
            impulse = impulse.minus(t.times(tangentImpulse));
        }

        return new ContactResolution(
                velocityA.plus(impulse.times(inverseA)),
                velocityB.minus(impulse.times(inverseB)),
                normalImpulse,
                tangentImpulse,
                sliding);
    }

    /**
     * Resolve a contact against something that cannot move — the floor, a wall, the field perimeter.
     *
     * @param velocity the moving body's velocity in m/s
     * @param normal   unit vector pointing away from the surface
     * @param mass     the moving body's mass in kg
     * @param moving   the surface on the moving body
     * @param fixed    the surface it hit
     */
    public static ContactResolution resolveAgainstStatic(
            Translation3d velocity,
            Translation3d normal,
            double mass,
            ContactMaterial moving,
            ContactMaterial fixed) {
        return resolve(velocity, Translation3d.kZero, normal, mass, IMMOVABLE, moving, fixed);
    }

    /** Dot product. {@link Translation3d} does not offer one. */
    private static double dot(Translation3d a, Translation3d b) {
        return a.getX() * b.getX() + a.getY() * b.getY() + a.getZ() * b.getZ();
    }
}
