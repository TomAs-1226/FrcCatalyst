package frc.lib.catalyst.command;

import org.junit.jupiter.api.Test;

import org.wpilib.command3.Scheduler;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * One throw out of a {@code periodic()} must not end the robot.
 *
 * <p>v3 runs periodic callbacks as sideloaded coroutines. An unchecked exception escaping one
 * destroys that coroutine, and the scheduler re-mounts the dead continuation on every subsequent
 * {@code run()} — so every later loop throws before reaching a single command, and it does not
 * recover on disable and re-enable.
 *
 * <p>What that looks like from the driver's station: the robot works, then freezes completely at
 * some arbitrary moment — the first null vision frame, the first CAN read on a browned-out device —
 * with every command, every default command and all telemetry stopping at once. The log fills with
 * {@code IllegalStateException: Mounted!!!!} from inside WPILib, naming nothing the team wrote.
 *
 * <p>This is strictly worse than v2, where the same exception recurred but the scheduler carried on
 * once the condition cleared. A subsystem that reads a sensor which is briefly absent is an ordinary
 * subsystem, not a broken one.
 */
class PeriodicFaultTest {

    /** A subsystem whose periodic throws once, the way a real one meets a null vision frame. */
    private static final class Flaky extends CatalystSubsystem {
        final AtomicInteger calls = new AtomicInteger();
        int throwOnCall = -1;

        @Override
        public void periodic() {
            int n = calls.incrementAndGet();
            if (n == throwOnCall) {
                throw new NullPointerException("vision frame was null");
            }
        }
    }

    @Test
    void aThrowingPeriodicDoesNotStopTheScheduler() {
        Scheduler scheduler = Scheduler.createIndependentScheduler();
        Flaky subsystem = new Flaky();
        subsystem.throwOnCall = 3;
        subsystem.registerPeriodic(scheduler);

        AtomicInteger commandTicks = new AtomicInteger();
        scheduler.schedule(Commands.run(commandTicks::incrementAndGet).withName("armHold"));

        assertDoesNotThrow(() -> {
            for (int i = 0; i < 12; i++) {
                scheduler.run();
            }
        }, "a subsystem that throws once must not take the scheduler down");

        // The real assertion. Commands have to keep running *after* the throw, not just before it.
        assertTrue(commandTicks.get() >= 9,
                "commands should have kept running through the fault, saw " + commandTicks.get());
    }

    @Test
    void theSubsystemKeepsBeingPolledAfterItThrows() {
        // A sensor that is briefly absent comes back. If the callback were dropped on first throw,
        // the subsystem would go quiet for the rest of the match instead.
        Scheduler scheduler = Scheduler.createIndependentScheduler();
        Flaky subsystem = new Flaky();
        subsystem.throwOnCall = 2;
        subsystem.registerPeriodic(scheduler);

        for (int i = 0; i < 10; i++) {
            scheduler.run();
        }

        assertTrue(subsystem.calls.get() >= 9,
                "periodic should still be called after it threw, saw " + subsystem.calls.get());
    }

    @Test
    void aPeriodicThatThrowsEveryTimeIsStillSurvivable() {
        // The worst case: a sensor that never comes back. The robot has to keep driving.
        Scheduler scheduler = Scheduler.createIndependentScheduler();
        CatalystSubsystem always = new CatalystSubsystem() {
            @Override
            public void periodic() {
                throw new IllegalStateException("device never enumerated");
            }
        };
        always.registerPeriodic(scheduler);

        AtomicInteger commandTicks = new AtomicInteger();
        scheduler.schedule(Commands.run(commandTicks::incrementAndGet).withName("drive"));

        assertDoesNotThrow(() -> {
            for (int i = 0; i < 20; i++) {
                scheduler.run();
            }
        });
        assertTrue(commandTicks.get() >= 15,
                "the drivetrain should still be driving, saw " + commandTicks.get());
    }
}
