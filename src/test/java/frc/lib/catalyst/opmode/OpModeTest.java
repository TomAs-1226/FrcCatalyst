package frc.lib.catalyst.opmode;

import frc.lib.catalyst.command.Commands;
import frc.lib.catalyst.system.SystemCoreSim;
import frc.lib.catalyst.system.SystemCoreSource;
import frc.lib.catalyst.system.SystemCoreStatus;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.wpilib.command3.Command;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * OpMode lifecycle, run for real.
 *
 * <p>An OpMode is the thing a driver selects on the Driver Station, so the failures here happen at
 * the worst possible moment and look like the robot rather than like the code: a mode that never
 * ticks the scheduler does nothing at all, and a mode that does not cancel what it started keeps
 * driving into teleop while the driver wonders why the robot will not respond.
 */
class OpModeTest {

    @BeforeEach
    void noMachine() {
        // These tests are about the lifecycle, not about Systemcore. An absent machine keeps
        // publish() from doing anything, which is what a desktop run is anyway.
        SystemCoreStatus.useSource(SystemCoreSim.healthy().withAvailable(false));
    }

    @AfterEach
    void tidy() {
        SystemCoreStatus.useSource(SystemCoreSource.unavailable());
    }

    /** An OpMode on its own scheduler, so tests do not inherit each other's commands. */
    private static final class Recording extends CatalystOpMode {
        final List<String> calls = new ArrayList<>();

        Recording() {
            super(false);
        }

        @Override protected void onStart()           { calls.add("start"); }
        @Override protected void onPeriodic()        { calls.add("periodic"); }
        @Override protected void onEnd()             { calls.add("end"); }
        @Override protected void onDisabledPeriodic() { calls.add("disabled"); }
    }

    // --- lifecycle -----------------------------------------------------------

    @Test
    void theHooksFireInOrder() {
        Recording mode = new Recording();
        mode.start();
        mode.periodic();
        mode.periodic();
        mode.end();

        assertEquals(List.of("start", "periodic", "periodic", "end"), mode.calls);
    }

    @Test
    void endWithoutStartDoesNothing() {
        // WPILib calls end() on the selected mode even when it never ran. A subclass releasing
        // something it never acquired is a confusing crash at exactly the wrong moment.
        Recording mode = new Recording();
        mode.end();
        assertEquals(List.of(), mode.calls);
    }

    @Test
    void aModeCanBeRunTwice() {
        // Two matches, one robot program. The second start must behave like the first.
        Recording mode = new Recording();
        mode.start();
        mode.end();
        mode.start();
        mode.end();

        assertEquals(List.of("start", "end", "start", "end"), mode.calls);
    }

    @Test
    void theSchedulerTicksEveryLoop() {
        // The failure this exists to prevent: an OpMode whose periodic does not run the scheduler
        // compiles, runs, and does nothing whatsoever - no command, no default command, no
        // subsystem periodic - with nothing anywhere saying why.
        AtomicInteger ticks = new AtomicInteger();
        Recording mode = new Recording();
        mode.scheduler().schedule(Commands.run(ticks::incrementAndGet).withName("work"));

        for (int i = 0; i < 4; i++) {
            mode.periodic();
        }
        assertTrue(ticks.get() >= 3, "the scheduler should have ticked, saw " + ticks.get());
    }

    @Test
    void theSchedulerAlsoTicksWhileDisabled() {
        // Between matches is when a pit crew reads telemetry, and stopping the scheduler here would
        // stop the periodic callbacks producing it.
        AtomicInteger ticks = new AtomicInteger();
        Recording mode = new Recording();
        mode.scheduler().schedule(Commands.run(ticks::incrementAndGet).withName("work"));

        for (int i = 0; i < 3; i++) {
            mode.disabledPeriodic();
        }
        assertTrue(ticks.get() >= 2, "saw " + ticks.get());
        assertEquals(List.of("disabled", "disabled", "disabled"), mode.calls);
    }

    @Test
    void theNameDefaultsToTheClass() {
        assertEquals("Recording", new Recording().name());
    }

    // --- CommandOpMode -------------------------------------------------------

    private static final class Auto extends CommandOpMode {
        Auto(java.util.function.Supplier<Command> supplier) {
            // Its own scheduler, so the tests do not inherit each other's commands.
            super(supplier, false);
        }
    }

    @Test
    void theCommandIsBuiltWhenTheModeStartsNotWhenItIsConstructed() {
        // The whole reason the constructor takes a supplier. An OpMode is constructed when the
        // Driver Station lists the modes, which is at startup - a command built then captures the
        // pose the robot had at boot and the alliance before the FMS said, and drives the wrong way
        // while looking entirely plausible.
        AtomicInteger built = new AtomicInteger();
        Auto auto = new Auto(() -> {
            built.incrementAndGet();
            return Commands.none().withName("routine");
        });

        assertEquals(0, built.get(), "constructing the mode must not build the command");

        auto.start();
        assertEquals(1, built.get());
    }

    @Test
    void eachRunRebuildsTheCommand() {
        AtomicInteger built = new AtomicInteger();
        Auto auto = new Auto(() -> {
            built.incrementAndGet();
            return Commands.none().withName("routine");
        });

        auto.start();
        auto.end();
        auto.start();
        assertEquals(2, built.get());
    }

    @Test
    void endingTheModeCancelsWhatItStarted() {
        // The match ending does not cancel what the mode scheduled. Without this a fifteen-second
        // auto keeps driving into teleop, still holding the drivetrain, and the driver cannot work
        // out why the robot will not respond to a stick.
        Auto auto = new Auto(() -> Commands.idle().withName("holds forever"));

        auto.start();
        auto.periodic();
        Command running = auto.runningCommand();
        assertTrue(auto.scheduler().isScheduledOrRunning(running), "it should be running");

        auto.end();
        auto.periodic();
        assertFalse(auto.scheduler().isScheduledOrRunning(running), "ending must cancel it");
        assertNull(auto.runningCommand());
    }

    @Test
    void aSupplierThatReturnsNothingDoesNotTakeTheRobotDown() {
        // A bug in robot code, but the mode still has to end cleanly - and the difference between
        // "the auto did not run" and knowing why is the logged error.
        Auto auto = new Auto(() -> null);
        auto.start();
        auto.periodic();
        auto.end();
        assertNull(auto.runningCommand());
    }
}
