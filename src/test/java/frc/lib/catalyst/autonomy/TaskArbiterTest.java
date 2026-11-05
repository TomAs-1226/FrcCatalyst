package frc.lib.catalyst.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.Test;

/**
 * The thing the library could not express until now: two behaviours that do not need the same
 * mechanisms running at the same time.
 */
class TaskArbiterTest {

    private static TaskArbiter.Candidate<String, String> c(String name, double score, boolean canStart,
                                                           String... needs) {
        return new TaskArbiter.Candidate<>(name, name, score, Set.of(needs), canStart);
    }

    @Test
    void twoTasksThatShareNothingBothRun() {
        var sel = TaskArbiter.select(List.of(
                c("Shoot", 0.9, true, "shooter", "feeder"),
                c("Intake", 0.8, true, "intake")), 0.0);

        assertEquals(List.of("Shoot", "Intake"), sel.tasks(),
                "no shared mechanism, so there was never a reason to pick one");
        assertTrue(sel.skipped().isEmpty());
    }

    @Test
    void aConflictIsResolvedInFavourOfTheHigherScore() {
        var sel = TaskArbiter.select(List.of(
                c("Chase", 0.4, true, "drivetrain"),
                c("AlignToScore", 0.9, true, "drivetrain")), 0.0);

        assertEquals(List.of("AlignToScore"), sel.tasks());
        assertEquals(1, sel.skipped().size());
        assertEquals("Chase", sel.skipped().get(0).name());
        assertTrue(sel.skipped().get(0).reason().contains("drivetrain"),
                "and it says which resource it lost: " + sel.skipped().get(0).reason());
    }

    @Test
    void aLowerScoringTaskStillRunsIfItFitsAroundTheWinner() {
        // The whole point. Chase loses the drivetrain but Feed does not need it.
        var sel = TaskArbiter.select(List.of(
                c("AlignToScore", 0.9, true, "drivetrain"),
                c("Chase", 0.5, true, "drivetrain"),
                c("Feed", 0.3, true, "feeder")), 0.0);

        assertEquals(List.of("AlignToScore", "Feed"), sel.tasks());
        assertEquals(List.of("Chase"), sel.skipped().stream().map(TaskArbiter.Skipped::name).toList());
    }

    @Test
    void aTaskThatCannotStartIsNotConsideredAndDoesNotHoldItsResources() {
        var sel = TaskArbiter.select(List.of(
                c("AlignToScore", 0.9, false, "drivetrain"),
                c("Chase", 0.5, true, "drivetrain")), 0.0);

        assertEquals(List.of("Chase"), sel.tasks(),
                "the higher scorer could not run, so it must not block the one that can");
        assertEquals("cannot start", sel.skipped().get(0).reason());
    }

    @Test
    void theScoreFloorIsHonoured() {
        var sel = TaskArbiter.select(List.of(
                c("Marginal", 0.1, true, "intake"),
                c("Worthwhile", 0.7, true, "shooter")), 0.5);

        assertEquals(List.of("Worthwhile"), sel.tasks());
        assertTrue(sel.skipped().get(0).reason().contains("below the 0.50 floor"),
                sel.skipped().get(0).reason());
    }

    @Test
    void tiesResolveTheSameWayEveryTime() {
        // Two behaviours on the same score must not swap between loops; a selector nobody can
        // predict is worse than one that is occasionally suboptimal.
        var a = TaskArbiter.select(List.of(
                c("Zebra", 0.5, true, "drivetrain"),
                c("Alpha", 0.5, true, "drivetrain")), 0.0);
        var b = TaskArbiter.select(List.of(
                c("Alpha", 0.5, true, "drivetrain"),
                c("Zebra", 0.5, true, "drivetrain")), 0.0);

        assertEquals(List.of("Alpha"), a.tasks());
        assertEquals(a.tasks(), b.tasks(), "input order must not change the answer");
    }

    @Test
    void nothingToChooseFromIsNotAnError() {
        var sel = TaskArbiter.<String, String>select(List.of(), 0.0);
        assertTrue(sel.tasks().isEmpty());
        assertEquals("nothing to choose from", sel.explain());
    }

    @Test
    void aTaskNeedingNothingAlwaysFits() {
        var sel = TaskArbiter.select(List.of(
                c("Drive", 0.9, true, "drivetrain"),
                c("Blink", 0.1, true)), 0.0);
        assertEquals(List.of("Drive", "Blink"), sel.tasks());
    }

    @Test
    void theExplanationSaysWhatRanAndHowMuchWasHeld() {
        var sel = TaskArbiter.select(List.of(
                c("AlignToScore", 0.9, true, "drivetrain"),
                c("Chase", 0.5, true, "drivetrain"),
                c("Feed", 0.3, true, "feeder")), 0.0);
        assertEquals("running AlignToScore, Feed; held 1", sel.explain());
    }

    @Test
    void aWholeSetIsChosenInOnePassWithoutBacktracking() {
        // Greedy: taking the best always, then the best that still fits. Adding a candidate can
        // never displace a higher-scoring one, which is what makes this predictable at a comp.
        var sel = TaskArbiter.select(List.of(
                c("A", 1.0, true, "x"),
                c("B", 0.9, true, "x", "y"),
                c("C", 0.8, true, "y"),
                c("D", 0.7, true, "z")), 0.0);

        assertEquals(List.of("A", "C", "D"), sel.tasks(),
                "A takes x; B wanted x so it is out; C takes y; D takes z");
    }
}
