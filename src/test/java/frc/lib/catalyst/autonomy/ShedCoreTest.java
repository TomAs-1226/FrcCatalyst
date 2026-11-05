package frc.lib.catalyst.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

/**
 * Shedding is the only power authority this library will take, and the floors are the reason it is
 * safe. The elevator test below is the one that matters: get it wrong and the robot drops whatever
 * it is holding at the exact moment it is already in trouble.
 */
class ShedCoreTest {

    private static ShedCore.Claim claim(String name, int priority, double now, double floor) {
        return new ShedCore.Claim(name, priority, now, floor);
    }

    @Test
    void anElevatorHoldingItsHeightIsNeverShedBelowWhatHoldsIt() {
        // 40 A limit, 18 A of that is the gravity feedforward keeping the carriage up.
        var plan = ShedCore.plan(List.of(claim("Elevator", 1, 40, 18)), 100, 1);

        assertEquals(1, plan.cuts().size());
        assertEquals(18.0, plan.cuts().get(0).toAmps(), 1e-9, "shed to the floor and not one amp further");
        assertFalse(plan.sufficient(), "and it says it could not find the rest rather than taking it anyway");
        assertEquals(78.0, plan.shortfall(), 1e-9);
    }

    @Test
    void aMechanismWithNoHeadroomIsNotTouched() {
        var plan = ShedCore.plan(List.of(claim("Wrist", 1, 20, 20)), 50, 1);
        assertTrue(plan.cuts().isEmpty(), "floor equals limit, so there is nothing to give");
        assertTrue(plan.explain().contains("nothing can give it up safely"), plan.explain());
    }

    @Test
    void lowPriorityShedsFirst() {
        var plan = ShedCore.plan(List.of(
                claim("Shooter", 9, 60, 0),
                claim("Intake", 1, 30, 0)), 20, 1);

        assertEquals(List.of("Intake"), plan.cuts().stream().map(ShedCore.Cut::name).toList());
        assertEquals(10.0, plan.cuts().get(0).toAmps(), 1e-9);
        assertTrue(plan.sufficient());
    }

    @Test
    void withinAPriorityTheBiggestGiverGoesFirst() {
        // Taking 25 A from one mechanism disturbs the robot less than 5 A from five of them.
        var plan = ShedCore.plan(List.of(
                claim("Small", 1, 10, 0),
                claim("Big", 1, 60, 0)), 25, 1);

        assertEquals(1, plan.cuts().size());
        assertEquals("Big", plan.cuts().get(0).name());
        assertEquals(35.0, plan.cuts().get(0).toAmps(), 1e-9);
    }

    @Test
    void itStopsAsSoonAsTheDeficitIsCovered() {
        // A brownout is not a reason to slow down everything on the robot.
        var plan = ShedCore.plan(List.of(
                claim("A", 1, 40, 0),
                claim("B", 2, 40, 0),
                claim("C", 3, 40, 0)), 15, 1);

        assertEquals(1, plan.cuts().size(), "one cut covered it; B and C are left alone");
        assertEquals(15.0, plan.shedAmps(), 1e-9);
        assertTrue(plan.sufficient());
    }

    @Test
    void aCutTooSmallToBeWorthACanWriteIsSkipped() {
        var plan = ShedCore.plan(List.of(claim("Nearly", 1, 20.5, 20)), 10, 2.0);
        assertTrue(plan.cuts().isEmpty(), "half an amp is not worth a bus write");
    }

    @Test
    void noDeficitMeansNoCuts() {
        var plan = ShedCore.plan(List.of(claim("Anything", 1, 40, 0)), 0, 1);
        assertTrue(plan.cuts().isEmpty());
        assertTrue(plan.sufficient());
        assertEquals("nothing to shed", plan.explain());
    }

    @Test
    void everyCutIsAReductionAndNeverAnIncrease() {
        // The property the whole design rests on. Fuzzed across a spread of shapes.
        for (int i = 0; i < 200; i++) {
            double now = 5 + (i % 17) * 3.0;
            double floor = (i % 5) * 2.0;
            var plan = ShedCore.plan(List.of(claim("M", 1, now, Math.min(floor, now))), i % 40, 0.5);
            for (ShedCore.Cut cut : plan.cuts()) {
                assertTrue(cut.toAmps() <= now + 1e-9, "a cut raised a limit: " + cut);
                assertTrue(cut.toAmps() >= Math.min(floor, now) - 1e-9, "a cut went below the floor: " + cut);
            }
        }
    }

    @Test
    void theSameRobotInTheSameStateShedsTheSameThingsTwice() {
        List<ShedCore.Claim> claims = List.of(
                claim("Zebra", 1, 30, 0),
                claim("Alpha", 1, 30, 0));
        var a = ShedCore.plan(claims, 10, 1);
        var b = ShedCore.plan(List.of(claims.get(1), claims.get(0)), 10, 1);
        assertEquals(a.cuts().get(0).name(), b.cuts().get(0).name(), "input order must not decide");
        assertEquals("Alpha", a.cuts().get(0).name());
    }

    @Test
    void aPartialShedSaysHowShortItIs() {
        var plan = ShedCore.plan(List.of(
                claim("A", 1, 20, 10),
                claim("B", 2, 20, 15)), 40, 1);

        assertEquals(15.0, plan.shedAmps(), 1e-9, "10 from A and 5 from B is all the floors allow");
        assertEquals(25.0, plan.shortfall(), 1e-9);
        assertTrue(plan.explain().contains("still 25 A short"), plan.explain());
    }
}
