package frc.lib.catalyst.system;

import frc.lib.catalyst.util.Preflight;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.SystemServer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * That an absent Systemcore reads as absent.
 *
 * <p>It did not. {@code NtSource} decided availability with {@code resolved != null}, which looks
 * like the right test and is not: {@code SystemServer.getSystemServer()} is
 * {@code fromNativeHandle(getSystemServerHandle())}, and {@code fromNativeHandle} is a plain
 * constructor that wraps whatever {@code int} it is handed. On a desktop it wraps handle 0 and
 * returns a perfectly non-null object, so the check passed everywhere and {@code isAvailable()}
 * answered true in simulation.
 *
 * <p>Nothing crashed, which is why it survived: every reading still came back empty, because the
 * topics genuinely are not there. What broke was the diagnosis. Preflight told a student running in
 * simulation {@code Systemcore: reporting}, and HealthMonitor registered a set of Systemcore checks
 * that could never fire and whose detail line read {@code battery NaN V}.
 *
 * <p>These tests construct the real {@code NtSource}, which loads the HAL natives. That is safe in
 * this repository because {@code extractNativeLibs} runs before {@code test}, and it is the same
 * bet the Phoenix tests already make — but it is a bet, and it is why this file is the only one
 * that resolves the real source.
 */
class SystemCoreAvailabilityTest {

    @AfterEach
    void clear() {
        // Leave no real NtSource behind for whatever runs next.
        SystemCoreStatus.useSource(SystemCoreSource.unavailable());
    }

    @Test
    void anAbsentSystemServerReadsAsAbsent() {
        assertTrue(HAL.initialize(500, 0), "no HAL, nothing to resolve");

        SystemCoreStatus.useSource(null);
        assertFalse(SystemCoreStatus.getInstance().isAvailable(),
                "there is no Systemcore attached to a desktop test run");
    }

    @Test
    void theNullCheckThatUsedToStandInForItIsNotEnough() {
        // The point of the fix, stated as the thing that is actually true. If this assertion ever
        // starts failing, `resolved != null` became a sufficient test and the guard can be
        // simplified - but until then, removing it silently restores the bug.
        assertTrue(HAL.initialize(500, 0));

        var resolved = SystemServer.getSystemServer();
        assertNotEquals(null, resolved,
                "getSystemServer returns a non-null instance even with no system server present");
        assertEquals(0, resolved.getHandle(),
                "and the thing that distinguishes it is the handle, not nullness");
    }

    @Test
    void anAbsentServerIsNotSilentlyTheRobotsOwnTables() {
        // The failure mode this was NOT: if handle 0 aliased the default instance, every /sys read
        // would resolve against the robot program's own tables, and a team that happened to publish
        // under /sys would get plausible readings from the wrong place.
        assertTrue(HAL.initialize(500, 0));

        NetworkTableInstance.getDefault().getTable("sys").getEntry("battery").setDouble(12.34);

        SystemCoreStatus.useSource(null);
        var status = SystemCoreStatus.getInstance();
        assertTrue(status.batteryVolts().isEmpty(),
                "a value published on the robot's own NT must not read back as Systemcore's");
        assertNull(status.server(), "and no instance is handed out when there is nothing behind it");
    }

    @Test
    void preflightSaysNotPresentRatherThanReporting() {
        // The user-visible half. With the bug, teamNumber() was empty so the ternary fell through
        // to "reporting" - the one word that tells a student the opposite of the truth.
        assertTrue(HAL.initialize(500, 0));
        SystemCoreStatus.useSource(null);

        String line = Preflight.run().findings().stream()
                .filter(f -> f.what().startsWith("Systemcore"))
                .map(Preflight.Finding::detail)
                .findFirst()
                .orElseThrow(() -> new AssertionError("Preflight said nothing about Systemcore"));

        assertTrue(line.contains("not present"),
                "expected a 'not present' line off hardware, got: " + line);
    }

    @Test
    void aSimulatedSourceStillReportsAvailable() {
        // The fix must not have made SystemCoreSim useless - that is the supported way to exercise
        // every reading below, and it has no native handle at all.
        SystemCoreStatus.useSource(new SystemCoreSim());
        assertTrue(SystemCoreStatus.getInstance().isAvailable(),
                "an explicitly installed simulator is present by definition");
    }
}
