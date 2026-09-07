package frc.lib.catalyst.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

/** The guess, the hysteresis that keeps it readable, and the score that decides whether to believe it. */
class IntentCoreTest {

    private static IntentCore.Guess g(String name, double confidence) {
        return new IntentCore.Guess(name, confidence, "");
    }

    @Test
    void theLeadingGuessIsHeld() {
        IntentCore ias = new IntentCore(0.3, 0.1);
        var r = ias.read(List.of(g("Score", 0.8), g("Intake", 0.4)));
        assertEquals("Score", r.best().orElseThrow().name());
        assertEquals("Score", ias.held());
    }

    @Test
    void nothingConfidentEnoughIsNoGuessRatherThanAWeakOne() {
        IntentCore ias = new IntentCore(0.5, 0.0);
        var r = ias.read(List.of(g("Score", 0.2), g("Intake", 0.1)));
        assertTrue(r.best().isEmpty());
        assertEquals("no idea", r.explain());
    }

    @Test
    void aMarginalLeadDoesNotDisplaceTheHeldGuess() {
        // Two intents trading places every loop makes the dashboard useless and the hit rate
        // meaningless, because the thing being scored keeps changing.
        IntentCore ias = new IntentCore(0.3, 0.15);
        ias.read(List.of(g("Score", 0.7), g("Intake", 0.5)));
        assertEquals("Score", ias.held());

        var r = ias.read(List.of(g("Intake", 0.75), g("Score", 0.7)));
        assertEquals("Score", r.best().orElseThrow().name(), "0.05 ahead is not enough to take over");
    }

    @Test
    void aClearLeadDoesDisplaceIt() {
        IntentCore ias = new IntentCore(0.3, 0.15);
        ias.read(List.of(g("Score", 0.7), g("Intake", 0.5)));
        var r = ias.read(List.of(g("Intake", 0.9), g("Score", 0.7)));
        assertEquals("Intake", r.best().orElseThrow().name());
    }

    @Test
    void theHitRateIsUnknownUntilThereIsEnoughHistoryToMeanAnything() {
        IntentCore ias = new IntentCore(0.3, 0.0);
        for (int i = 0; i < 9; i++) {
            ias.read(List.of(g("Score", 0.9)));
            ias.observe("Score");
        }
        assertTrue(Double.isNaN(ias.hitRate()), "nine samples is noise, not a measurement");
        assertTrue(ias.read(List.of(g("Score", 0.9))).explain().contains("not enough history"));

        ias.observe("Score");
        assertEquals(1.0, ias.hitRate(), 1e-9);
    }

    @Test
    void beingWrongCountsAgainstIt() {
        // The whole deliverable. If this number is poor, nothing should ever be wired to the guess.
        IntentCore ias = new IntentCore(0.3, 0.0);
        for (int i = 0; i < 10; i++) {
            ias.read(List.of(g("Score", 0.9)));
            ias.observe(i % 2 == 0 ? "Score" : "Intake");
        }
        assertEquals(0.5, ias.hitRate(), 1e-9);
        assertEquals(10, ias.samples());
    }

    @Test
    void theExplanationCarriesTheConfidenceAndTheScore() {
        IntentCore ias = new IntentCore(0.3, 0.0);
        for (int i = 0; i < 10; i++) {
            ias.read(List.of(new IntentCore.Guess("Score", 0.8, "driver is lined up")));
            ias.observe("Score");
        }
        String explain = ias.read(List.of(new IntentCore.Guess("Score", 0.8, "driver is lined up"))).explain();
        assertTrue(explain.contains("Score"), explain);
        assertTrue(explain.contains("80%"), explain);
        assertTrue(explain.contains("driver is lined up"), explain);
        assertTrue(explain.contains("right 100% of 10"), explain);
    }

    @Test
    void nothingHereCanCommandAnything() {
        // Asserted rather than assumed: the class exposes a reading, a rate and a reset, and no
        // way to act. A future change that adds one should have to argue for it here.
        var methods = java.util.Arrays.stream(IntentCore.class.getDeclaredMethods())
                .filter(m -> java.lang.reflect.Modifier.isPublic(m.getModifiers()))
                .map(java.lang.reflect.Method::getName)
                .sorted()
                .toList();
        assertEquals(List.of("held", "hitRate", "observe", "read", "reset", "samples"), methods);
    }

    @Test
    void resetForgetsTheMatch() {
        IntentCore ias = new IntentCore(0.3, 0.0);
        ias.read(List.of(g("Score", 0.9)));
        ias.observe("Score");
        ias.reset();
        assertEquals("", ias.held());
        assertEquals(0, ias.samples());
    }

    @Test
    void tiesResolveTheSameWayEveryTime() {
        IntentCore a = new IntentCore(0.3, 0.0);
        IntentCore b = new IntentCore(0.3, 0.0);
        assertEquals(a.read(List.of(g("Zebra", 0.5), g("Alpha", 0.5))).best().orElseThrow().name(),
                b.read(List.of(g("Alpha", 0.5), g("Zebra", 0.5))).best().orElseThrow().name());
    }
}
