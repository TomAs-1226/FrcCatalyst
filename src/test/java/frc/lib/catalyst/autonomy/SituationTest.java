package frc.lib.catalyst.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.catalyst.physics.prediction.PowerPredictor;
import org.junit.jupiter.api.Test;

/**
 * The seam every later phase reads from. Its whole job is to be honest about what it does not know,
 * so these tests are mostly about what it refuses to claim.
 */
class SituationTest {

    @Test
    void aBlindSituationClaimsNothing() {
        Situation s = Situation.blind();

        assertTrue(s.blindfolded());
        assertFalse(s.localization().valid());
        assertFalse(s.motion().valid());
        assertFalse(s.traction().valid());
        assertFalse(s.power().valid());
        assertFalse(s.match().valid());

        // And the convenience predicates stay false rather than reading zeros as good news.
        assertFalse(s.localization().trustworthy());
        assertFalse(s.traction().slipping(), "no measurement is not the same as gripping");
        assertFalse(s.traction().nearTipping());
        assertFalse(s.power().overBudget(), "no measurement is not the same as inside budget");
        assertFalse(s.match().endgame(30.0));
        assertTrue(s.motion().atRest(), "unknown motion is treated as at rest, the cautious reading");
    }

    @Test
    void withNoPhysicsEveryDerivedFacetIsInvalidRatherThanZero() {
        SituationSource source = PhysicsSituationSource.builder().build();
        Situation s = source.sample(1.0);

        assertEquals(1.0, s.timestampSeconds());
        assertFalse(s.localization().valid());
        assertFalse(s.motion().valid());
        assertFalse(s.traction().valid());
        assertFalse(s.power().valid());
    }

    @Test
    void aPredictorWithoutABreakerBudgetDoesNotCountAsAPowerMeasurement() {
        // The trap this facet exists to avoid. A predictor answers headroomAmps() whether or not it
        // has been told the breaker budget, and without one the answer is a battery-sag figure -
        // 245 A on a robot allowed 120. An allocator reading valid() must not see that as a budget.
        PowerPredictor unbudgeted = PowerPredictor.builder()
                .presentVoltage(() -> 12.4)
                .presentCurrent(() -> 40)
                .build();
        Situation s = PhysicsSituationSource.builder().power(unbudgeted).build().sample(0.0);

        assertFalse(s.power().valid(), "no breaker budget means no budget to report");
        assertEquals("no breaker budget set", s.power().bindingLimit());
        assertEquals(0.0, s.power().headroomAmps(), 1e-9, "and no number that could be mistaken for one");
    }

    @Test
    void aBudgetedPredictorIsAMeasurement() {
        PowerPredictor budgeted = PowerPredictor.builder()
                .presentVoltage(() -> 12.4)
                .presentCurrent(() -> 40)
                .breakerBudgetAmps(120)
                .build();
        Situation s = PhysicsSituationSource.builder().power(budgeted).build().sample(0.0);

        assertTrue(s.power().valid());
        assertEquals(80.0, s.power().headroomAmps(), 1e-9);
        assertEquals(40.0, s.power().drawAmps(), 1e-9);
        assertEquals("breaker", s.power().bindingLimit());
        assertFalse(s.power().overBudget());
    }

    @Test
    void aSourceThatThrowsCostsItsFacetAndNotTheLoop() {
        PowerPredictor exploding = PowerPredictor.builder()
                .presentVoltage(() -> 12.0)
                .presentCurrent(() -> {
                    throw new IllegalStateException("PDH not on the bus");
                })
                .breakerBudgetAmps(120)
                .build();

        // No exception escapes; the facet is simply unmeasured.
        Situation s = PhysicsSituationSource.builder().power(exploding).build().sample(0.0);
        assertFalse(s.power().valid());
    }

    @Test
    void cachingSamplesOncePerLoopAndReSamplesOnTheNext() {
        int[] calls = {0};
        SituationSource counted = now -> {
            calls[0]++;
            return Situation.blind(now);
        };
        SituationSource cached = counted.cachedPerLoop();

        Situation a = cached.sample(1.0);
        Situation b = cached.sample(1.0);
        Situation c = cached.sample(1.0);
        assertSame(a, b, "one loop, one snapshot - so consumers cannot disagree with each other");
        assertSame(a, c);
        assertEquals(1, calls[0]);

        Situation next = cached.sample(1.02);
        assertEquals(2, calls[0], "a new loop re-samples");
        assertEquals(1.02, next.timestampSeconds());
    }

    @Test
    void samplingIsCheapEnoughForTheLoopBudget() {
        // The plan flagged this as the one unmeasured risk in the phase: a record graph allocated at
        // 50 Hz inside a 20 ms budget. Measure it before anything depends on the shape.
        SituationSource source = PhysicsSituationSource.builder().build();
        for (int i = 0; i < 20_000; i++) {
            source.sample(i * 0.02);   // warm up
        }

        long start = System.nanoTime();
        final int samples = 100_000;
        for (int i = 0; i < samples; i++) {
            Situation s = source.sample(i * 0.02);
            if (s.timestampSeconds() < 0) {
                throw new AssertionError("unreachable, but keeps the sample from being optimised away");
            }
        }
        double perSampleMicros = (System.nanoTime() - start) / 1000.0 / samples;

        // A robot loop is 20 000 us. Anything under 10 us is three orders of magnitude clear; the
        // bound is deliberately loose so this measures an order-of-magnitude regression, not jitter.
        assertTrue(perSampleMicros < 10.0,
                "one snapshot took " + perSampleMicros + " us; the loop has 20000 us for everything");
    }
}
