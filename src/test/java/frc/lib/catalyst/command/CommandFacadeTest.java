package frc.lib.catalyst.command;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The command facade, run on a real scheduler.
 *
 * <p>This is the most load-bearing new code in the port — every command in every mechanism is built
 * through {@link Commands} and decorated with {@link CatalystCommand}, and it is a reimplementation
 * of the v2 vocabulary on a coroutine engine that works nothing like v2 did. Until now it had no
 * tests at all, because constructing a scheduler needs JVM flags the build was not passing.
 *
 * <p>The failure mode that matters is not a compile error. A command that never yields hangs the
 * scheduler for the rest of the match; a decorator that drops requirements lets two commands drive
 * one mechanism at once; a {@code finallyDo} that fires on both paths runs its cleanup twice. None
 * of those are visible without running the thing.
 *
 * <p>The queries here use {@code isScheduledOrRunning}, not {@code isScheduled}. v3 splits the two:
 * {@code isScheduled} means queued and not yet started, so it reads false for a command that is
 * running right now. v2's {@code isScheduled} meant "active", which is what nearly every caller
 * wants, and {@link CatalystCommand#isScheduled()} preserves that meaning by delegating to
 * {@code isScheduledOrRunning}. Anything reaching for the raw scheduler needs to know the difference.
 */
class CommandFacadeTest {

    private Scheduler scheduler;

    /** A mechanism with no behaviour, to check that requirements are carried and honoured. */
    private static final class Mech implements Mechanism {
        private final String name;
        Mech(String name) { this.name = name; }
        @Override public String getName() { return name; }
    }

    @BeforeEach
    void freshScheduler() {
        // Independent rather than the default: the tests must not inherit commands from each other.
        scheduler = Scheduler.createIndependentScheduler();
    }

    /** Run the scheduler n times, which is n robot loops. */
    private void tick(int n) {
        for (int i = 0; i < n; i++) {
            scheduler.run();
        }
    }

    // --- the factories ------------------------------------------------------

    @Test
    void runOnceRunsExactlyOnceAndFinishes() {
        AtomicInteger n = new AtomicInteger();
        scheduler.schedule(Commands.runOnce(n::incrementAndGet).withName("once"));
        tick(5);
        assertEquals(1, n.get(), "runOnce must not repeat");
    }

    @Test
    void runRepeatsAndYields() {
        // The important half is that the scheduler comes back at all. A body that loops without
        // yielding never returns control, and the robot stops responding with no error anywhere.
        AtomicInteger n = new AtomicInteger();
        scheduler.schedule(Commands.run(n::incrementAndGet).withName("repeat"));
        tick(4);
        assertTrue(n.get() >= 3, "expected roughly one call per loop, got " + n.get());
    }

    @Test
    void noneCompletesImmediatelyWithoutDoingAnything() {
        CatalystCommand c = Commands.none();
        scheduler.schedule(c);
        tick(2);
        assertFalse(scheduler.isScheduledOrRunning(c), "none() should not stay resident");
    }

    @Test
    void startEndRunsItsEndActionWhenCancelled() {
        AtomicInteger started = new AtomicInteger();
        AtomicInteger ended = new AtomicInteger();
        CatalystCommand c = Commands.startEnd(started::incrementAndGet, ended::incrementAndGet)
                .withName("startEnd");

        scheduler.schedule(c);
        tick(2);
        assertEquals(1, started.get());
        assertEquals(0, ended.get(), "it should still be running");

        scheduler.cancel(c);
        tick(1);
        assertEquals(1, ended.get(), "cancelling must run the end action - this is how subsystems stop");
    }

    @Test
    void waitUntilBlocksUntilTheConditionHolds() {
        AtomicInteger after = new AtomicInteger();
        boolean[] go = {false};

        scheduler.schedule(Commands.sequence(
                Commands.waitUntil(() -> go[0]).withName("gate"),
                Commands.runOnce(after::incrementAndGet).withName("after")).withName("seq"));

        tick(5);
        assertEquals(0, after.get(), "the gate is shut");

        go[0] = true;
        tick(3);
        assertEquals(1, after.get(), "the gate opened");
    }

    // --- composition --------------------------------------------------------

    @Test
    void sequenceRunsInOrder() {
        List<String> order = new ArrayList<>();
        scheduler.schedule(Commands.sequence(
                Commands.runOnce(() -> order.add("a")).withName("a"),
                Commands.runOnce(() -> order.add("b")).withName("b"),
                Commands.runOnce(() -> order.add("c")).withName("c")).withName("seq"));
        tick(8);
        assertEquals(List.of("a", "b", "c"), order);
    }

    @Test
    void parallelWaitsForTheSlowest() {
        AtomicInteger fastRuns = new AtomicInteger();
        int[] slowTicks = {0};

        CatalystCommand slow = Commands.of("slow", co -> {
            while (slowTicks[0]++ < 4) {
                co.yield();
            }
        });

        CatalystCommand group = Commands.parallel(
                Commands.runOnce(fastRuns::incrementAndGet).withName("fast"), slow).withName("both");

        scheduler.schedule(group);
        tick(2);
        assertEquals(1, fastRuns.get(), "the fast one is done");
        assertTrue(scheduler.isScheduledOrRunning(group), "but the group waits for the slow one");

        tick(6);
        assertFalse(scheduler.isScheduledOrRunning(group), "and ends once the slow one finishes");
    }

    @Test
    void raceEndsWithTheFirstToFinish() {
        int[] slowTicks = {0};
        CatalystCommand slow = Commands.of("slow", co -> {
            while (slowTicks[0]++ < 50) {
                co.yield();
            }
        });

        CatalystCommand group = Commands.race(Commands.none().withName("instant"), slow)
                .withName("race");
        scheduler.schedule(group);
        tick(3);

        assertFalse(scheduler.isScheduledOrRunning(group), "the instant branch should have ended the race");
        assertTrue(slowTicks[0] < 50, "the slow branch must have been cut short, reached " + slowTicks[0]);
    }

    @Test
    void eitherPicksItsBranchWhenScheduled() {
        List<String> ran = new ArrayList<>();
        boolean[] pick = {true};

        CatalystCommand c = Commands.either(
                Commands.runOnce(() -> ran.add("true")).withName("t"),
                Commands.runOnce(() -> ran.add("false")).withName("f"),
                () -> pick[0]).withName("either");

        scheduler.schedule(c);
        tick(4);
        assertEquals(List.of("true"), ran);

        // The selector is read at schedule time, so the same command object follows a changed
        // condition on its next run rather than remembering the first answer.
        pick[0] = false;
        scheduler.schedule(c);
        tick(4);
        assertEquals(List.of("true", "false"), ran);
    }

    @Test
    void deferBuildsItsCommandWhenScheduledNotWhenConstructed() {
        // The whole reason defer exists. Building at construction time captures state from robotInit
        // and every autonomous run then replays that stale plan.
        AtomicInteger built = new AtomicInteger();
        CatalystCommand c = Commands.defer(() -> {
            built.incrementAndGet();
            return Commands.none().withName("inner");
        }).withName("deferred");

        assertEquals(0, built.get(), "nothing should be built before it is scheduled");

        scheduler.schedule(c);
        tick(3);
        assertEquals(1, built.get());

        scheduler.schedule(c);
        tick(3);
        assertEquals(2, built.get(), "each scheduling must rebuild");
    }

    // --- decorators ---------------------------------------------------------

    @Test
    void withNameOverridesTheName() {
        assertEquals("renamed", Commands.none().withName("renamed").name());
    }

    @Test
    void beforeStartingRunsAheadOfTheBody() {
        List<String> order = new ArrayList<>();
        scheduler.schedule(Commands.runOnce(() -> order.add("body")).withName("b")
                .beforeStarting(() -> order.add("before")));
        tick(3);
        assertEquals(List.of("before", "body"), order);
    }

    @Test
    void finallyDoRunsOnceWhenTheCommandFinishesNormally() {
        // Firing on both the completion and cancellation paths would run every cleanup twice - and
        // cleanup here means things like zeroing a motor or dropping a game piece.
        List<Boolean> calls = new ArrayList<>();
        scheduler.schedule(Commands.runOnce(() -> { }).withName("x")
                .finallyDo((Boolean interrupted) -> calls.add(interrupted)));
        tick(4);
        assertEquals(List.of(false), calls, "exactly one call, reporting not-interrupted");
    }

    @Test
    void finallyDoReportsInterruptionWhenCancelled() {
        List<Boolean> calls = new ArrayList<>();
        CatalystCommand c = Commands.idle().withName("idle")
                .finallyDo((Boolean interrupted) -> calls.add(interrupted));

        scheduler.schedule(c);
        tick(2);
        assertEquals(List.of(), calls, "still running");

        scheduler.cancel(c);
        tick(1);
        assertEquals(List.of(true), calls, "one call, reporting interrupted");
    }

    @Test
    void untilTrueEndsTheCommandEarly() {
        AtomicInteger ticks = new AtomicInteger();
        boolean[] stop = {false};

        CatalystCommand c = Commands.run(ticks::incrementAndGet).withName("work")
                .untilTrue(() -> stop[0]);
        scheduler.schedule(c);
        tick(3);
        assertTrue(scheduler.isScheduledOrRunning(c));

        stop[0] = true;
        tick(2);
        assertFalse(scheduler.isScheduledOrRunning(c), "the condition should have ended it");
    }

    @Test
    void thenChainsASecondCommand() {
        List<String> order = new ArrayList<>();
        scheduler.schedule(Commands.runOnce(() -> order.add("first")).withName("f")
                .then(Commands.runOnce(() -> order.add("second")).withName("s")));
        tick(6);
        assertEquals(List.of("first", "second"), order);
    }

    // --- requirements, which is where two commands fight over one motor -----

    @Test
    void requirementsSurviveTheFactories() {
        Mech arm = new Mech("arm");
        assertTrue(Commands.run(() -> { }, arm).requirements().contains(arm));
        assertTrue(Commands.runOnce(() -> { }, arm).requirements().contains(arm));
        assertTrue(Commands.startEnd(() -> { }, () -> { }, arm).requirements().contains(arm));
        assertTrue(Commands.idle(arm).requirements().contains(arm));
    }

    @Test
    void requirementsSurviveDecoration() {
        // A decorator that quietly drops requirements is the worst kind of bug here: everything
        // still runs, and two commands drive the same motor with opposite intentions.
        Mech arm = new Mech("arm");
        CatalystCommand base = Commands.run(() -> { }, arm).withName("hold");

        assertTrue(base.withName("renamed").requirements().contains(arm), "withName");
        assertTrue(base.finallyDo(() -> { }).requirements().contains(arm), "finallyDo");
        assertTrue(base.beforeStarting(() -> { }).requirements().contains(arm), "beforeStarting");
    }

    @Test
    void aSecondCommandOnTheSameMechanismDisplacesTheFirst() {
        Mech arm = new Mech("arm");
        AtomicInteger firstEnded = new AtomicInteger();

        CatalystCommand first = Commands.idle(arm).withName("first")
                .finallyDo(firstEnded::incrementAndGet);
        CatalystCommand second = Commands.idle(arm).withName("second");

        scheduler.schedule(first);
        tick(2);
        assertTrue(scheduler.isScheduledOrRunning(first));

        scheduler.schedule(second);
        tick(2);

        assertFalse(scheduler.isScheduledOrRunning(first), "the first must be displaced, not run alongside");
        assertTrue(scheduler.isScheduledOrRunning(second));
        assertEquals(1, firstEnded.get(), "and it must be told it ended, so its cleanup runs");
    }
}
