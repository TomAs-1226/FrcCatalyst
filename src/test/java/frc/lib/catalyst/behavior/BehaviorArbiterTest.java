package frc.lib.catalyst.behavior;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Whether the selector can be made to stop handing control back and forth.
 *
 * <p>Utility selection has no memory: every loop the highest scorer wins. Two behaviours whose
 * scores cross repeatedly trade control at the loop rate, and each handover cancels a command and
 * schedules another — so the mechanism restarts fifty times a second and finishes nothing. The
 * driver sees a robot that twitches, and no individual score is wrong.
 *
 * <p>The first test pins today's behaviour as the baseline rather than assuming it, because both
 * knobs default to zero and that has to keep being true: a non-zero default would change what every
 * existing robot does on a library upgrade.
 */
class BehaviorArbiterTest {

    private static BehaviorArbiter.Candidate c(String name, double score) {
        return new BehaviorArbiter.Candidate(name, score, true);
    }

    private static BehaviorArbiter.Candidate ineligible(String name, double score) {
        return new BehaviorArbiter.Candidate(name, score, false);
    }

    /** Alternate two near-equal scores for n loops and count the handovers. */
    private static int alternate(BehaviorArbiter a, int loops) {
        String incumbent = null;
        for (int i = 0; i < loops; i++) {
            List<BehaviorArbiter.Candidate> cs = (i % 2 == 0)
                    ? List.of(c("shoot", 10.1), c("intake", 10.0))
                    : List.of(c("shoot", 10.0), c("intake", 10.1));
            var d = a.decide(cs, incumbent, i * 0.02);
            incumbent = d.winner();
        }
        return a.switches();
    }

    @Test
    void atDefaultsItThrashes_whichIsTodaysBehaviourAndTheBaseline() {
        // Not an endorsement. This is the regression baseline: with both knobs at zero the arbiter
        // must reproduce what the library does today, or upgrading changes every robot silently.
        BehaviorArbiter a = new BehaviorArbiter(0.0, 0.0, 0.0);

        int switches = alternate(a, 100);

        assertTrue(switches >= 95,
                "with no margin and no dwell, near-equal alternating scores should hand over almost "
                        + "every loop - that is the problem, and it is today's behaviour: " + switches);
    }

    @Test
    void aMarginStopsIt() {
        // 0.5 in the team's own units, against a 0.1 lead. Nobody should hand over for 0.1.
        BehaviorArbiter a = new BehaviorArbiter(0.0, 0.5, 0.0);

        int switches = alternate(a, 100);

        assertTrue(switches <= 1,
                "one handover to get started, and then it should settle: " + switches);
    }

    @Test
    void aDwellStopsItToo() {
        // Margin left at zero on purpose: dwell alone must be sufficient, since the two knobs solve
        // the same problem from different directions and a team may reasonably set only one.
        BehaviorArbiter a = new BehaviorArbiter(0.0, 0.0, 1.0);

        int switches = alternate(a, 100);   // 100 loops x 20 ms = 2.0 s, so few dwells elapse

        assertTrue(switches <= 3, "a one-second dwell over two seconds: " + switches);
    }

    @Test
    void anIneligibleIncumbentIsReleasedImmediatelyDespiteDwell() {
        // A dwell timer must never hold on to something that cannot run. Here waiting is not
        // conservative, it is broken.
        BehaviorArbiter a = new BehaviorArbiter(0.0, 5.0, 10.0);

        var first = a.decide(List.of(c("shoot", 10.0), c("intake", 1.0)), null, 0.0);
        assertEquals("shoot", first.winner());

        var second = a.decide(List.of(ineligible("shoot", 10.0), c("intake", 1.0)), "shoot", 0.02);

        assertTrue(second.switched(), "an unrunnable incumbent must be released at once");
        assertEquals("intake", second.winner());
    }

    @Test
    void anIncumbentThatDropsBelowMinScoreIsAlsoReleased() {
        BehaviorArbiter a = new BehaviorArbiter(5.0, 5.0, 10.0);

        a.decide(List.of(c("shoot", 10.0), c("intake", 6.0)), null, 0.0);
        var d = a.decide(List.of(c("shoot", 1.0), c("intake", 6.0)), "shoot", 0.02);

        assertTrue(d.switched());
        assertEquals("intake", d.winner());
    }

    @Test
    void aFlickeringGateIsCountedAsASwitch() {
        // The likelier case in practice, and the one the original framing missed: a vision-gated
        // scorer whose Limelight tv drops for a single frame. The selector goes to "nothing
        // runnable", cancels, and re-schedules next loop. Same restart cost as trading places.
        BehaviorArbiter a = new BehaviorArbiter(0.0, 0.0, 0.0);

        a.decide(List.of(c("aim", 10.0)), null, 0.00);
        var gone = a.decide(List.of(ineligible("aim", 10.0)), "aim", 0.02);
        var back = a.decide(List.of(c("aim", 10.0)), "", 0.04);

        assertTrue(gone.switched(), "losing the only candidate is a handover");
        assertNull(gone.winner());
        assertTrue(back.switched(), "and picking it back up is another");
        assertEquals(3, a.switches(), "start, release, restart");
    }

    @Test
    void anExactTieKeepsTheIncumbent() {
        // The one deliberate difference from the old scan at default settings. It used registration
        // order on a tie, so a tie could switch or not depending which behaviour was added first.
        BehaviorArbiter a = new BehaviorArbiter(0.0, 0.0, 0.0);

        a.decide(List.of(c("shoot", 10.0), c("intake", 1.0)), null, 0.0);
        var d = a.decide(List.of(c("shoot", 10.0), c("intake", 10.0)), "shoot", 0.02);

        assertFalse(d.switched(), "an exact tie is not a reason to restart a mechanism");
        assertEquals("shoot", d.winner());
    }

    @Test
    void heldSecondsAndReasonAreVisibleForBothFailureDirections() {
        // A margin set too high freezes the robot on its first choice, which is as silent as
        // thrashing. These two keys are how that shows up.
        BehaviorArbiter a = new BehaviorArbiter(0.0, 1000.0, 0.0);

        a.decide(List.of(c("shoot", 10.0), c("intake", 1.0)), null, 0.0);
        a.decide(List.of(c("shoot", 1.0), c("intake", 900.0)), "shoot", 30.0);

        assertEquals(1, a.switches(), "the huge margin froze it after the first pick");
        assertEquals(30.0, a.heldSeconds(30.0), 1e-9, "and the held time says so");
        assertTrue(a.lastSwitchReason().contains("shoot"));
    }
}
