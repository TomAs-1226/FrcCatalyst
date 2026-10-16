package frc.lib.catalyst.behavior;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.catalyst.command.Commands;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;

/** Real scheduler coverage for the adopted action's prerequisite during phase dwell. */
class AutopilotPreconditionTest {
    private Scheduler scheduler;

    @BeforeEach
    void freshSchedulerAndHal() {
        assertTrue(HAL.initialize(), "Autopilot telemetry needs HAL");
        scheduler = Scheduler.createIndependentScheduler();
    }

    private void tick(int count) {
        for (int i = 0; i < count; i++) scheduler.run();
    }

    @Test
    void ineligibleAdoptedActionDoesNotRunWhileAnotherPhaseDwells() {
        AtomicInteger acquireRuns = new AtomicInteger();
        AtomicInteger scoreRuns = new AtomicInteger();
        Action acquire = Action.named("Acquire")
                .when(() -> false)
                .run(() -> Commands.runOnce(acquireRuns::incrementAndGet))
                .build();
        Action score = Action.named("Score")
                .when(() -> true)
                .run(() -> Commands.runOnce(scoreRuns::incrementAndGet))
                .build();
        var command = Autopilot.builder().name("DwellPrecondition")
                .phases(() -> 1, acquire, score).dwellSeconds(60).build().run();
        scheduler.schedule(command);
        try {
            tick(8);
            assertEquals(0, acquireRuns.get(),
                    "dwell must not authorize the old phase when its prerequisite is false");
            assertEquals(0, scoreRuns.get(), "the desired phase must still respect dwell");
        } finally {
            scheduler.cancel(command);
            tick(2);
        }
    }

    @Test
    void eligibleAdoptedActionStillRunsDuringDwell() {
        AtomicInteger acquireRuns = new AtomicInteger();
        Action acquire = Action.named("Acquire")
                .run(() -> Commands.runOnce(acquireRuns::incrementAndGet)).build();
        Action score = Action.named("Score").run(Commands::none).build();
        var command = Autopilot.builder().name("EligibleDwell")
                .phases(() -> 1, acquire, score).dwellSeconds(60).build().run();
        scheduler.schedule(command);
        try {
            tick(8);
            assertTrue(acquireRuns.get() > 0, "eligible old phase remains available during dwell");
        } finally {
            scheduler.cancel(command);
            tick(2);
        }
    }

    @Test
    void adoptedActionCanResumeWhenItsPrerequisiteRecoversDuringDwell() {
        AtomicBoolean eligible = new AtomicBoolean(false);
        AtomicInteger acquireRuns = new AtomicInteger();
        Action acquire = Action.named("Acquire")
                .when(eligible::get)
                .run(() -> Commands.runOnce(acquireRuns::incrementAndGet)).build();
        Action score = Action.named("Score").run(Commands::none).build();
        var command = Autopilot.builder().name("RecoveredDwell")
                .phases(() -> 1, acquire, score).dwellSeconds(60).build().run();
        scheduler.schedule(command);
        try {
            tick(5);
            assertEquals(0, acquireRuns.get(), "the ineligible phase must be held first");
            eligible.set(true);
            tick(8);
            assertTrue(acquireRuns.get() > 0, "a recovered prerequisite permits the adopted phase");
        } finally {
            scheduler.cancel(command);
            tick(2);
        }
    }
}
