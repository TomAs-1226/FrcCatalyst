package frc.lib.catalyst.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

/** Cycle arithmetic, at a desk. */
class ChaseCoreTest {

    private static ChaseCore.Target<String> t(String name, double value, double seconds) {
        return new ChaseCore.Target<>(name, name, value, seconds, true, "");
    }

    private static ChaseCore.Target<String> unreachable(String name, double value, double seconds) {
        return new ChaseCore.Target<>(name, name, value, seconds, false, "no path");
    }

    @Test
    void theBestCycleWinsRatherThanTheBiggestPrize() {
        // Twice the points, four times the drive. It is a worse cycle and the robot should know it.
        var choice = ChaseCore.choose(List.of(
                t("Far", 10, 8.0),      // 1.25 / s
                t("Near", 5, 2.0)),     // 2.5  / s
                Double.NaN, 0);

        assertEquals("Near", choice.chosen().orElseThrow().name());
        assertTrue(choice.reason().contains("2.50/s"), choice.reason());
    }

    @Test
    void andNotSimplyTheNearestEither() {
        var choice = ChaseCore.choose(List.of(
                t("Near", 1, 2.0),      // 0.5 / s
                t("Far", 10, 5.0)),     // 2.0 / s
                Double.NaN, 0);
        assertEquals("Far", choice.chosen().orElseThrow().name());
    }

    @Test
    void nothingIsStartedThatTheMatchWillNotLetYouFinish() {
        // Most of the way to a piece when the buzzer goes is worth nothing, and it cost the cycle
        // that was actually available.
        var choice = ChaseCore.choose(List.of(
                t("Far", 20, 9.0),
                t("Near", 3, 2.0)),
                5.0, 0);

        assertEquals("Near", choice.chosen().orElseThrow().name(),
                "the better rate does not fit in the time left");
        assertEquals(List.of("Far"), choice.rejected().stream().map(ChaseCore.Target::name).toList());
    }

    @Test
    void anUnreachableTargetIsPassedOver() {
        var choice = ChaseCore.choose(List.of(
                unreachable("Blocked", 50, 1.0),
                t("Open", 5, 2.0)),
                Double.NaN, 0);
        assertEquals("Open", choice.chosen().orElseThrow().name());
    }

    @Test
    void aTargetBelowTheValueFloorIsNotWorthACycle() {
        var choice = ChaseCore.choose(List.of(t("Scrap", 0.5, 0.1)), Double.NaN, 1.0);
        assertTrue(choice.chosen().isEmpty());
        assertEquals("nothing worth chasing", choice.reason());
    }

    @Test
    void aTargetWithNoTimeEstimateIsRankedOnValueRatherThanIgnored() {
        // Not knowing how long something takes is a reason to be careful, not a reason to drive
        // past a piece sitting in front of the robot.
        var choice = ChaseCore.choose(List.of(
                new ChaseCore.Target<>("Unknown", "Unknown", 10, Double.NaN, true, ""),
                t("Known", 1, 1.0)),
                Double.NaN, 0);
        assertEquals("Unknown", choice.chosen().orElseThrow().name());
    }

    @Test
    void anEmptyFieldIsNotAnError() {
        var choice = ChaseCore.<String>choose(List.of(), 15.0, 0);
        assertTrue(choice.chosen().isEmpty());
        assertEquals("nothing in sight", choice.reason());
    }

    @Test
    void equalTargetsResolveTheSameWayEveryTime() {
        // Two identical pieces must not alternate between loops, or the robot drives at the point
        // halfway between them.
        var a = ChaseCore.choose(List.of(t("Zebra", 5, 2.0), t("Alpha", 5, 2.0)), Double.NaN, 0);
        var b = ChaseCore.choose(List.of(t("Alpha", 5, 2.0), t("Zebra", 5, 2.0)), Double.NaN, 0);
        assertEquals("Alpha", a.chosen().orElseThrow().name());
        assertEquals(a.chosen().orElseThrow().name(), b.chosen().orElseThrow().name());
    }

    @Test
    void theRejectedListExplainsTheChoiceRatherThanJustOmittingThem() {
        var choice = ChaseCore.choose(List.of(
                unreachable("Blocked", 50, 1.0),
                t("Chosen", 5, 2.0),
                t("Slow", 5, 20.0)),
                Double.NaN, 0);

        assertEquals("Chosen", choice.chosen().orElseThrow().name());
        assertEquals(List.of("Blocked"), choice.rejected().stream().map(ChaseCore.Target::name).toList(),
                "only what was considered and passed over before the winner");
    }
}
