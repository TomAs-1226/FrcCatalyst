package frc.lib.catalyst.opmode;

import frc.lib.catalyst.command.Commands;
import frc.lib.catalyst.system.SystemCoreSim;
import frc.lib.catalyst.system.SystemCoreSource;
import frc.lib.catalyst.system.SystemCoreStatus;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.wpilib.command3.Command;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

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
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    /**
     * Put the simulated Driver Station in the state the framework calls {@code periodic()} in.
     *
     * <p>These tests used to call {@code periodic()} with no Driver Station at all, which meant they
     * exercised it while disabled — a state the framework's own contract says cannot happen, since
     * {@code periodic()} is documented to run only while enabled. That is not a harmless shortcut:
     * it is exactly why the double-tick defect was invisible here. The shipped alpha-6 OpModeRobot
     * registers {@code periodic()} with no enabled gate, so while disabled it fired alongside
     * {@code disabledPeriodic()} and the scheduler ran twice per robot loop — and a test that drives
     * the hooks by hand can never see that, because it is the framework's loop that calls both.
     *
     * <p>So these now assert against the real enabled state, and {@code CatalystOpMode.periodic()}
     * returns early when disabled. The disabled window belongs to {@code disabledPeriodic()}, which
     * {@code theSchedulerAlsoTicksWhileDisabled} still covers.
     */
    private static void enabled() {
        assertTrue(HAL.initialize(), "no HAL, no Driver Station state");
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }


    /** An OpMode on its own scheduler, so tests do not inherit each other's commands. */
    private static final class Recording extends CatalystOpMode {
        final List<String> calls = new ArrayList<>();

        /** What onStart() should schedule, if a test wants a handle on it. */
        private Command next;

        Recording() {
            super(false);
        }

        /**
         * Start the mode having it schedule {@code command}, the ordinary way a mode starts work:
         * {@code onStart() { run(auto.threePiece()); }}. The test keeps the same handle the mode
         * has, which is what lets it assert on the command's fate afterwards.
         */
        void startWith(Command command) {
            this.next = command;
            start();
        }

        @Override protected void onStart() {
            calls.add("start");
            if (next != null) {
                run(next);
            }
        }
        @Override protected void onPeriodic()        { calls.add("periodic"); }
        @Override protected void onEnd()             { calls.add("end"); }
        @Override protected void onDisabledPeriodic() { calls.add("disabled"); }
    }

    // --- lifecycle -----------------------------------------------------------

    @Test
    void theHooksFireInOrder() {
        enabled();
        Recording mode = new Recording();
        mode.start();
        mode.periodic();
        mode.periodic();
        mode.end();

        assertEquals(List.of("start", "periodic", "periodic", "end"), mode.calls);
    }

    @Test
    void periodicDoesNothingWhileDisabled() {
        // The scheduler must tick exactly once per robot loop, and while disabled it is
        // disabledPeriodic() that does it. periodic() staying out of that window is what stops the
        // two from both firing.
        //
        // Why this matters and why it was missed: periodic() is documented to be called only while
        // enabled, and the shipped alpha-6 OpModeRobot does not honour that - it registers the
        // callback at opmode-selection time with no enabled gate, so while disabled BOTH hooks
        // fired and everything that counts loops counted double. Command timeouts expired at half
        // their stated duration.
        //
        // What this test does NOT prove: that the framework calls both. That needs a real
        // OpModeRobot loop, because driving the hooks by hand is exactly what cannot see it. What
        // it does pin is the half that is Catalyst's to get right - that periodic() declines the
        // disabled window - and reverting the guard fails it.
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        AtomicInteger ticks = new AtomicInteger();
        Recording mode = new Recording();
        mode.scheduler().schedule(Commands.run(ticks::incrementAndGet).withName("work"));

        for (int i = 0; i < 4; i++) {
            mode.periodic();
        }

        assertEquals(0, ticks.get(),
                "periodic() ticked the scheduler while disabled; disabledPeriodic() owns that window");
        assertEquals(List.of(), mode.calls, "and onPeriodic must not fire on a disabled loop");
    }

    // --- an OpMode owns what it schedules -------------------------------------

    @Test
    void endingTheModeCancelsWhatRunStarted() {
        enabled();
        Recording mode = new Recording();
        var work = Commands.run(() -> { }).withName("owned");
        mode.startWith(work);

        mode.periodic();
        assertTrue(mode.scheduler().isScheduledOrRunning(work), "precondition: it is running");

        mode.end();
        mode.periodic();

        assertFalse(mode.scheduler().isScheduledOrRunning(work),
                "a command started by the mode must not outlive it");
    }

    @Test
    void aLeftoverDoesNotKeepTickingThroughTheDisabledWindow() {
        // The case that costs a match and shows nothing. On a disable, OpModeRobot ends the mode and
        // forces a fresh instance of the SAME one - and the opmode id has not changed, so v3's
        // ForOpmode scope does not reap anything. That instance's disabledPeriodic() runs the
        // scheduler, so the leftover auto command keeps advancing: timers expire, sequential groups
        // step forward, all with the outputs held neutral so nothing looks wrong.
        enabled();
        AtomicInteger ticks = new AtomicInteger();
        Recording mode = new Recording();
        mode.startWith(Commands.run(ticks::incrementAndGet).withName("auto"));

        for (int i = 0; i < 5; i++) {
            mode.periodic();
        }
        int atEnd = ticks.get();
        assertTrue(atEnd > 0, "precondition: it ran while enabled");

        mode.end();
        for (int i = 0; i < 10; i++) {
            mode.disabledPeriodic();
        }

        assertEquals(atEnd, ticks.get(),
                "the leftover advanced through the disabled window; it should have been released");
    }

    @Test
    void theSameModeCanBeRunTwice() {
        // The pit routine: enable auto, disable, enable the same auto again. With the leftover still
        // holding mechanisms, the second run fights it - and the symptom is "we ran auto twice and
        // the second time it did nothing", with nothing anywhere saying why.
        enabled();
        Recording mode = new Recording();

        var first = Commands.run(() -> { }).withName("run1");
        mode.startWith(first);
        mode.periodic();
        mode.end();

        var second = Commands.run(() -> { }).withName("run2");
        mode.startWith(second);
        mode.periodic();

        assertFalse(mode.scheduler().isScheduledOrRunning(first), "the first run must be gone");
        assertTrue(mode.scheduler().isScheduledOrRunning(second), "and the second must be running");
    }

    @Test
    void closeReleasesEvenWithoutEnd() {
        // WPILib calls close() on every instance it discards, and one path calls close() WITHOUT
        // end(): a mode deselected while disabled. end() is guarded behind having started; close()
        // is the only hook that covers that instance.
        enabled();
        Recording mode = new Recording();
        var work = Commands.run(() -> { }).withName("owned");
        mode.startWith(work);
        mode.periodic();

        mode.close();
        mode.periodic();

        assertFalse(mode.scheduler().isScheduledOrRunning(work));
    }

    @Test
    void aCommandScheduledDirectlyIsNotOwned() {
        // The escape hatch, and it has to keep working: something scheduled on the scheduler rather
        // than through run() is meant to outlive the mode.
        enabled();
        Recording mode = new Recording();
        var global = Commands.run(() -> { }).withName("global");
        mode.scheduler().schedule(global);
        mode.start();
        mode.periodic();

        mode.end();
        mode.periodic();

        assertTrue(mode.scheduler().isScheduledOrRunning(global),
                "only what run() started is owned; anything else is the team's to manage");
        assertTrue(mode.scheduledCommands().isEmpty());
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
        enabled();
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
