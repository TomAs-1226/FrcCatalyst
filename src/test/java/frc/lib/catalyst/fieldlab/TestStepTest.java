package frc.lib.catalyst.fieldlab;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The individual steps: what they cost the time estimate, and what {@link TestStep.Measure} thinks
 * is plausible.
 */
class TestStepTest {

    @Test
    void aBatteryGateThatPassesCostsNothingAndASettleCostsItsSeconds() {
        assertEquals(0.0, TestStep.BatteryGate.at(12.0).estimatedSeconds(), 0.0,
                "a gate's own estimate should be zero; the wait it might cause isn't the campaign's to predict");
        assertEquals(45.0, TestStep.Settle.of("settle", 45.0).estimatedSeconds(), 0.0,
                "a settle should read as its own dead time");
    }

    @Test
    void plausibleAcceptsTheClosedRangeAndRejectsBeyondEitherEnd() {
        TestStep.Measure measure = TestStep.Measure.of("measure", "k", "m", 2.0, 5.0);

        assertTrue(measure.plausible(2.0), "the low boundary itself is inside the range");
        assertTrue(measure.plausible(5.0), "the high boundary itself is inside the range");
        assertTrue(measure.plausible(3.5), "the middle of the range is plausible");
        assertFalse(measure.plausible(1.999), "just below the low boundary should be rejected");
        assertFalse(measure.plausible(5.001), "just above the high boundary should be rejected");
    }

    @Test
    void aNaNBoundMeansNoOpinionAndAcceptsEverythingOnThatSide() {
        TestStep.Measure noMin = new TestStep.Measure("measure", "k", "m", Double.NaN, 5.0, 60.0);
        assertTrue(noMin.plausible(-1_000_000.0), "no lower opinion should accept anything below the max");
        assertTrue(noMin.plausible(5.0));
        assertFalse(noMin.plausible(5.001), "the declared max should still apply");

        TestStep.Measure noMax = new TestStep.Measure("measure", "k", "m", 2.0, Double.NaN, 60.0);
        assertTrue(noMax.plausible(1_000_000.0), "no upper opinion should accept anything above the min");
        assertFalse(noMax.plausible(1.999), "the declared min should still apply");

        TestStep.Measure noOpinion = TestStep.Measure.of("measure", "k", "m");
        assertTrue(noOpinion.plausible(Double.NEGATIVE_INFINITY), "no opinion at all must accept everything");
        assertTrue(noOpinion.plausible(Double.POSITIVE_INFINITY));
        assertTrue(noOpinion.plausible(0.0));
    }
}
