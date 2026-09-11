package frc.lib.catalyst.goal;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.catalyst.logging.RecordingSink;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The goal package had no tests at all. These three are the ones that would have caught what was
 * actually wrong with it: a team's readiness lambda could take down the command that called it, two
 * directors on one robot overwrote each other's dashboard keys, and every write went straight at
 * NetworkTables where nothing could observe it - including a test.
 */
class GoalDirectorTelemetryTest {

    private RecordingSink sink;

    @BeforeEach
    void install() {
        sink = RecordingSink.install();
    }

    @AfterEach
    void restore() {
        sink.restore();
    }

    @Test
    void aReadinessTestThatThrowsLeavesTheGoalNotReadyAndSaysWhy() {
        // Team code. A sensor that is not there yet, a device that has not enumerated - this is the
        // ordinary shape of a readiness lambda on a robot that is still being built.
        Goal goal = Goal.named("SCORE")
                .readyWhen(() -> {
                    throw new IllegalStateException("shooter encoder not present");
                })
                .build();
        GoalDirector director = GoalDirector.builder().build();

        // Before the guard this propagated out of the pursue command's monitor and took it down.
        director.updateReadiness(goal);

        assertFalse(director.isReady(), "a goal that cannot say whether it is ready is not ready");
        String why = sink.string("Goal/WhyNotReady");
        assertNotNull(why);
        assertTrue(why.contains("readiness test failed"), why);
        assertTrue(why.contains("shooter encoder not present"), "the reason names the cause: " + why);
    }

    @Test
    void aWorkingReadinessTestStillDecidesNormally() {
        boolean[] ready = {false};
        Goal goal = Goal.named("SCORE").readyWhen(() -> ready[0]).build();
        GoalDirector director = GoalDirector.builder().build();

        director.updateReadiness(goal);
        assertFalse(director.isReady());
        assertEquals("setup not complete", sink.string("Goal/WhyNotReady"));

        ready[0] = true;
        director.updateReadiness(goal);
        assertTrue(director.isReady());
        assertEquals("", sink.string("Goal/WhyNotReady"));
    }

    @Test
    void twoDirectorsOnOneRobotDoNotOverwriteEachOther() {
        GoalDirector scoring = GoalDirector.builder()
                .name("Scoring").defaultGoal(Goal.named("STOW").build()).build();
        GoalDirector climbing = GoalDirector.builder()
                .name("Climbing").defaultGoal(Goal.named("PARK").build()).build();

        assertEquals("STOW", sink.string("Goal/Scoring/Active"));
        assertEquals("PARK", sink.string("Goal/Climbing/Active"));
        assertNull(sink.string("Goal/Active"), "a named director does not write the shared key");

        // And an unnamed one keeps the key path it has always had, so nobody's dashboard moves.
        GoalDirector legacy = GoalDirector.builder().defaultGoal(Goal.named("IDLE").build()).build();
        assertEquals("IDLE", sink.string("Goal/Active"));
        assertNotNull(legacy);
    }

    @Test
    void unchangedValuesAreNotRepublishedEveryLoop() {
        Goal goal = Goal.named("SCORE").readyWhen(() -> false).build();
        GoalDirector director = GoalDirector.builder().build();
        sink.clear();

        // The monitor runs this every loop. Fifty loops of an unchanging answer used to be fifty
        // writes of each key, across the wire, every second.
        for (int i = 0; i < 50; i++) {
            director.updateReadiness(goal);
        }

        // Zero, not one: the constructor already published the idle state, and "not ready" has not
        // changed since. Fifty loops of an unchanging answer used to be fifty writes of each key.
        assertEquals(0, sink.writeCount("Goal/Ready"), "nothing changed, so nothing is written");
        assertEquals(1, sink.writeCount("Goal/WhyNotReady"),
                "except the reason, which moved from \"idle\" to \"setup not complete\" once");
    }

    @Test
    void aChangedValueIsStillPublished() {
        boolean[] ready = {false};
        Goal goal = Goal.named("SCORE").readyWhen(() -> ready[0]).build();
        GoalDirector director = GoalDirector.builder().build();
        sink.clear();

        director.updateReadiness(goal);   // false, and it was already false from construction
        ready[0] = true;
        director.updateReadiness(goal);   // -> true, a real change
        ready[0] = false;
        director.updateReadiness(goal);   // -> false, a real change

        assertEquals(2, sink.writeCount("Goal/Ready"), "both real changes are published");
        assertEquals(Boolean.FALSE, sink.bool("Goal/Ready"), "and the latest value is right");
    }
}
