package frc.lib.catalyst.physics.contact;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * What a contact does when one of its inputs is not a number.
 *
 * <p>It resolved it, and handed back NaN velocities. Every check in {@code resolve} is a comparison,
 * so one non-finite input did not trip a single guard — it switched off all four at once: the
 * degenerate-normal check, the already-separating early return, the immovable-pair early return, and
 * the friction branch. Nothing threw, {@code resolved()} read false so it did not even count as a
 * contact, and the body's velocity became NaN with the bad input long gone.
 *
 * <p>The tests below assert on {@code resolve} throwing rather than on the NaN, because that is the
 * contract the javadoc already claimed ("unit vector pointing from B toward A") and the guard was
 * always meant to enforce.
 */
class ContactResolverNonFiniteTest {

    private static final Translation3d UP = new Translation3d(0, 0, 1);
    private static final Translation3d FALLING = new Translation3d(0, 0, -3);
    private static final ContactMaterial M = ContactMaterial.BUMPER;

    // --- the normal ----------------------------------------------------------

    @Test
    void aNaNNormalIsRejectedRatherThanResolved() {
        Translation3d bad = new Translation3d(Double.NaN, 0, 0);
        assertThrows(IllegalArgumentException.class,
                () -> ContactResolver.resolveAgainstStatic(FALLING, bad, 0.27, M, M));
    }

    @Test
    void anInfiniteNormalIsRejectedToo() {
        // Distinct from NaN: an infinite component passes a length test, then divides down to a zero
        // vector, so the contact silently does nothing instead of erroring.
        Translation3d bad = new Translation3d(Double.POSITIVE_INFINITY, 0, 0);
        assertThrows(IllegalArgumentException.class,
                () -> ContactResolver.resolveAgainstStatic(FALLING, bad, 0.27, M, M));
    }

    @Test
    void aZeroNormalIsStillRejected() {
        // The case the original guard was written for. It must survive the rewrite.
        assertThrows(IllegalArgumentException.class, () -> ContactResolver.resolveAgainstStatic(
                FALLING, Translation3d.ZERO, 0.27, M, M));
    }

    // --- velocity and mass ---------------------------------------------------

    @Test
    void aNaNVelocityIsRejected() {
        Translation3d bad = new Translation3d(0, 0, Double.NaN);
        assertThrows(IllegalArgumentException.class,
                () -> ContactResolver.resolveAgainstStatic(bad, UP, 0.27, M, M));
    }

    @Test
    void aNaNMassIsRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> ContactResolver.resolveAgainstStatic(FALLING, UP, Double.NaN, M, M));
    }

    @Test
    void immovableIsStillAcceptedBecauseItIsDeliberatelyInfinite() {
        // The mass guard has to reject NaN without rejecting IMMOVABLE, which IS +Infinity. A guard
        // written as Double.isFinite(massA) would pass every test above and break the library's own
        // most common call — every wall, floor and robot contact passes IMMOVABLE.
        var after = ContactResolver.resolve(
                FALLING, Translation3d.ZERO, UP,
                0.27, ContactResolver.IMMOVABLE, M, M);

        assertTrue(after.resolved(), "a ball landing on an immovable floor is a contact");
        assertTrue(Double.isFinite(after.velocityA().getZ()));
    }

    @Test
    void anOrdinaryContactStillBounces() {
        // The guards must not have cost the thing the class is for.
        var after = ContactResolver.resolveAgainstStatic(FALLING, UP, 0.27, M, M);

        assertTrue(after.velocityA().getZ() > FALLING.getZ(),
                "the downward velocity should have been removed or reversed, not left alone");
        assertEquals(0.0, after.velocityA().getX(), 1e-9);
    }
}
