package frc.lib.catalyst.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Deterministic eligibility and handback checks while phase adoption is delayed by dwell. */
class CycleCoreEligibilityTest {
    @Test
    void ineligibleRetainedPhaseStallsAndHandsBackDuringDwell() {
        CycleCore cycle = new CycleCore(2, 1.0, 0.25);
        assertEquals(CycleCore.Act.HOLD, cycle.decide(0.0, 1, phase -> phase == 1).act());
        assertEquals(CycleCore.Act.HOLD, cycle.decide(0.2, 1, phase -> phase == 1).act());
        CycleCore.Decision handback = cycle.decide(0.3, 1, phase -> phase == 1);
        assertEquals(0, handback.phase());
        assertEquals(CycleCore.Act.HAND_BACK, handback.act());
        assertTrue(handback.reason().contains("handing back"));
        assertEquals(0.3, cycle.stalledSeconds(0.3), 1e-9);
    }

    @Test
    void recoveredRetainedPhaseResetsStallWithoutSkippingDwell() {
        CycleCore cycle = new CycleCore(2, 1.0, 0.5);
        cycle.decide(0.0, 1, phase -> false);
        CycleCore.Decision recovered = cycle.decide(0.3, 1, phase -> true);
        assertEquals(0, recovered.phase());
        assertEquals(CycleCore.Act.RUN, recovered.act());
        assertEquals(0.0, cycle.stalledSeconds(0.3), 1e-9);
        assertEquals(CycleCore.Act.HOLD, cycle.decide(0.4, 1, phase -> false).act());
        assertEquals(CycleCore.Act.HOLD, cycle.decide(0.8, 1, phase -> false).act());
        assertEquals(CycleCore.Act.HAND_BACK, cycle.decide(0.95, 1, phase -> false).act());
    }

    @Test
    void eligibilityIsCheckedOnceForTheAdoptedPhaseAndChangesAfterDwell() {
        CycleCore cycle = new CycleCore(2, 1.0, 0.5);
        List<Integer> checked = new ArrayList<>();
        var eligible = (java.util.function.IntPredicate) phase -> {
            checked.add(phase);
            return phase == 1;
        };
        assertEquals(CycleCore.Act.HOLD, cycle.decide(0.0, 1, eligible).act());
        CycleCore.Decision adopted = cycle.decide(1.0, 1, eligible);
        assertEquals(1, adopted.phase());
        assertEquals(CycleCore.Act.RUN, adopted.act());
        assertEquals(List.of(0, 1), checked);
        assertEquals(0.0, cycle.stalledSeconds(1.0), 1e-9);
    }

    @Test
    void compatibilityOverloadRetainsItsDesiredOnlyEligibilitySemantics() {
        CycleCore cycle = new CycleCore(2, 1.0, 0.5);
        assertEquals(CycleCore.Act.RUN, cycle.decide(0.0, 1, false).act());
        assertEquals(CycleCore.Act.HOLD, cycle.decide(1.0, 1, false).act());
    }
}
