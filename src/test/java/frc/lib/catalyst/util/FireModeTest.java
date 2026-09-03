package frc.lib.catalyst.util;

import frc.lib.catalyst.util.FireMode.Posture;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** AWP and MAC-10, and the boring cases that make the joke work. */
class FireModeTest {

    @Test
    void nothingHappeningIsHolstered() {
        FireMode f = FireMode.builder().build();
        f.update();
        assertEquals(Posture.HOLSTERED, f.current());
        assertEquals(0.0, f.shotsPerSecond(), 1e-9);
    }

    @Test
    void aimingWithoutFiringIsAwp() {
        FireMode f = FireMode.builder().aiming(() -> true).build();
        f.update();
        assertEquals(Posture.AWP, f.current());
    }

    @Test
    void dumpingShotsIsMac10EvenWhileTheTurretClaimsToBeTracking() {
        // Cadence wins on purpose. A turret still reporting "tracking" while shots leave four times
        // a second is describing an intention, not what anyone watching would call aiming.
        FireMode f = FireMode.builder().aiming(() -> true).build();
        for (int i = 0; i < 8; i++) {
            f.recordShot();
        }
        f.update();
        assertEquals(Posture.MAC_10, f.current());
        assertTrue(f.shotsPerSecond() >= 2.0);
    }

    @Test
    void oneDeliberateShotIsStillAwp() {
        // The distinguishing case: shooting, but not spraying. A single shot with no aiming signal
        // wired up must not read as MAC-10 just because a shot happened.
        FireMode f = FireMode.builder().build();
        f.recordShot();
        f.update();
        assertEquals(Posture.AWP, f.current());
    }

    @Test
    void justChangedFiresOnTheTransitionAndNotAfter() {
        FireMode f = FireMode.builder().aiming(() -> true).build();
        f.update();
        assertTrue(f.justChanged(), "HOLSTERED to AWP is a change");
        f.update();
        assertFalse(f.justChanged(), "staying in AWP is not");
    }

    @Test
    void aThrowingSupplierCannotTakeDownTheLoop() {
        // These suppliers come from team code and usually reach into a turret and a vision
        // solution. A null pose in somebody's isOnTarget must not throw out of a periodic loop
        // because of a dashboard readout.
        FireMode f = FireMode.builder()
                .aiming(() -> { throw new IllegalStateException("no pose yet"); })
                .build();
        f.update();
        assertEquals(Posture.HOLSTERED, f.current());
    }

    @Test
    void resetForgetsTheBurst() {
        FireMode f = FireMode.builder().build();
        for (int i = 0; i < 8; i++) f.recordShot();
        f.update();
        assertEquals(Posture.MAC_10, f.current());

        f.reset();
        f.update();
        assertEquals(Posture.HOLSTERED, f.current());
    }

    @Test
    void theDisplayNameHasTheHyphen() {
        assertEquals("MAC-10", Posture.MAC_10.display());
        assertEquals("AWP", Posture.AWP.display());
    }
}
