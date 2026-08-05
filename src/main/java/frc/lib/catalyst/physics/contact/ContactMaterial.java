package frc.lib.catalyst.physics.contact;

/**
 * What a surface does when something hits it.
 *
 * <p>Two numbers carry almost all of the behaviour a robot simulation cares about. <b>Restitution</b>
 * is how much of the approach speed survives the bounce — 0 for a beanbag, near 1 for a superball.
 * <b>Friction</b> is the Coulomb coefficient that decides how much sideways speed a contact can scrub
 * off before it gives up and slides.
 *
 * <p>The point of having materials at all is that the same collision resolves differently depending on
 * what hit what. A foam game piece dropped on carpet dies almost immediately; the same piece off the
 * polycarbonate wall comes back at you. Modelling both with one global "bounciness" gets one of those
 * two wrong, and it is usually the one that matters.
 *
 * <h2>Combining two materials</h2>
 *
 * <p>A contact has two surfaces, so the pair has to be reduced to one restitution and one friction.
 * The geometric mean is used for both, which is what Box2D and Bullet do and behaves sensibly at the
 * extremes: anything touching a dead surface is dead, and it never produces a pair bouncier than its
 * bounciest member.
 *
 * <pre>{@code
 * ContactMaterial ball = ContactMaterial.FOAM_GAME_PIECE;
 * double bounce = ball.restitutionWith(ContactMaterial.CARPET);       // ~0.24, thuds
 * double wall   = ball.restitutionWith(ContactMaterial.POLYCARBONATE); // ~0.55, comes back
 * }</pre>
 *
 * <p>The preset values are plausible starting points measured off nothing more authoritative than
 * published coefficients for similar materials. Treat them the way you would treat any other physical
 * constant in this library: a reasonable default that your own measurements should replace. See
 * {@code docs/advanced/physics-measurement.md} for how to take those measurements.
 *
 * @param name             a label, for diagnostics and {@link #toString()}
 * @param restitution      fraction of approach speed returned by a bounce, 0 to 1
 * @param friction         Coulomb coefficient of friction, 0 upward
 * @param rollingResistance fraction of speed lost per second while rolling in contact, 0 upward
 * @since 1.8.0
 */
public record ContactMaterial(
        String name, double restitution, double friction, double rollingResistance) {

    /** FRC carpet: grippy, and it absorbs nearly everything dropped on it. */
    public static final ContactMaterial CARPET =
            new ContactMaterial("carpet", 0.15, 1.05, 0.55);

    /** The clear field wall. Hard and lively — this is what sends game pieces back into play. */
    public static final ContactMaterial POLYCARBONATE =
            new ContactMaterial("polycarbonate", 0.60, 0.30, 0.0);

    /** Field structure: extrusion, plate, the frame of a goal. */
    public static final ContactMaterial ALUMINIUM =
            new ContactMaterial("aluminium", 0.35, 0.42, 0.0);

    /** A foam game piece. Soft, and it stops dead on carpet. */
    public static final ContactMaterial FOAM_GAME_PIECE =
            new ContactMaterial("foam game piece", 0.38, 0.72, 0.9);

    /** A robot bumper: fabric over pool noodle, which is to say a shock absorber. */
    public static final ContactMaterial BUMPER =
            new ContactMaterial("bumper", 0.22, 0.65, 0.0);

    /** A drive wheel against carpet. Very grippy, no bounce worth speaking of. */
    public static final ContactMaterial TREAD =
            new ContactMaterial("tread", 0.05, 1.20, 0.0);

    public ContactMaterial {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("name must not be blank — it is what diagnostics print");
        }
        if (!(restitution >= 0.0) || restitution > 1.0) {
            throw new IllegalArgumentException(
                    "restitution must be between 0 and 1, got " + restitution
                            + "; above 1 a bounce would leave faster than it arrived");
        }
        if (!(friction >= 0.0)) {
            throw new IllegalArgumentException("friction must be at least 0, got " + friction);
        }
        if (!(rollingResistance >= 0.0)) {
            throw new IllegalArgumentException(
                    "rollingResistance must be at least 0, got " + rollingResistance);
        }
    }

    /** Restitution for a contact between this material and {@code other}. */
    public double restitutionWith(ContactMaterial other) {
        return Math.sqrt(restitution * other.restitution);
    }

    /** Coulomb friction for a contact between this material and {@code other}. */
    public double frictionWith(ContactMaterial other) {
        return Math.sqrt(friction * other.friction);
    }

    /** Rolling resistance for a contact between this material and {@code other}. */
    public double rollingResistanceWith(ContactMaterial other) {
        return Math.sqrt(rollingResistance * other.rollingResistance);
    }

    /** A copy with a different restitution, for when you have measured your own. */
    public ContactMaterial withRestitution(double value) {
        return new ContactMaterial(name, value, friction, rollingResistance);
    }

    /** A copy with a different friction coefficient. */
    public ContactMaterial withFriction(double value) {
        return new ContactMaterial(name, restitution, value, rollingResistance);
    }

    @Override
    public String toString() {
        return String.format("%s(e=%.2f, mu=%.2f)", name, restitution, friction);
    }
}
