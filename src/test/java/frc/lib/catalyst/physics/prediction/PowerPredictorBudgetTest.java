package frc.lib.catalyst.physics.prediction;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * {@link PowerPredictor} shipped with no tests, and its headline number is easy to misread.
 *
 * <p>{@code headroomAmps()} answered a battery question and was reached for as a power budget. On a
 * robot sitting still at 12.4 V it reports 245 A of room - true of the battery, false of a 120 A
 * main breaker - so anything allocating against it would have authorised roughly twice the current
 * the robot is allowed to draw.
 */
class PowerPredictorBudgetTest {

    private static PowerPredictor at(double volts, double amps) {
        return PowerPredictor.builder()
                .presentVoltage(() -> volts)
                .presentCurrent(() -> amps)
                .internalResistance(0.020)
                .minimumVoltage(7.5)
                .build();
    }

    private static PowerPredictor at(double volts, double amps, double breaker) {
        return PowerPredictor.builder()
                .presentVoltage(() -> volts)
                .presentCurrent(() -> amps)
                .internalResistance(0.020)
                .minimumVoltage(7.5)
                .breakerBudgetAmps(breaker)
                .build();
    }

    @Test
    void theSagHeadroomDoesNotDependOnThePresentCurrent() {
        // V_open = V + I·R, and headroom = (V_open − V_min)/R − I, so the current cancels exactly.
        // Worth pinning: it looks like a bug every time someone reads it, and it is not one.
        double idle = at(12.4, 0).sagHeadroomAmps();
        double loaded = at(12.4, 90).sagHeadroomAmps();
        assertEquals(idle, loaded, 1e-9, "the same bus voltage means the same room left, by construction");
        assertEquals((12.4 - 7.5) / 0.020, idle, 1e-9);
    }

    @Test
    void withoutABreakerBudgetItStillAnswersTheBatteryQuestion() {
        PowerPredictor p = at(12.4, 40);
        assertEquals(245.0, p.headroomAmps(), 1e-9, "unchanged for anyone already relying on it");
        assertTrue(p.breakerBudgetAmps().isEmpty());
        assertEquals("battery (no breaker budget set)", p.bindingLimit());
    }

    @Test
    void aBreakerBudgetMakesItANumberAnAllocatorCanSpend() {
        // The same robot, now told what it is actually allowed to draw.
        PowerPredictor p = at(12.4, 40, 120);
        assertEquals(80.0, p.headroomAmps(), 1e-9, "120 A allowed minus 40 A already flowing");
        assertEquals("breaker", p.bindingLimit());
        assertEquals(245.0, p.sagHeadroomAmps(), 1e-9, "the battery answer is still available");
    }

    @Test
    void theBatteryBindsOnceItHasSaggedFarEnough() {
        // Deep into a match: the bus is low, so the battery becomes the tighter of the two.
        PowerPredictor p = at(8.5, 40, 120);
        assertEquals(50.0, p.headroomAmps(), 1e-9, "(8.5 − 7.5)/0.020 = 50, tighter than 120 − 40");
        assertEquals("battery", p.bindingLimit());
    }

    @Test
    void pastTheLimitTheAnswerGoesNegativeAndSaysHowMuchToShed() {
        PowerPredictor p = at(12.0, 150, 120);
        assertEquals(-30.0, p.headroomAmps(), 1e-9, "30 A over the budget");
        assertTrue(p.headroomAmps() < 0);
    }

    @Test
    void aNonsenseBudgetIsRefusedAtBuildRatherThanAtTheFirstBrownout() {
        assertThrows(IllegalStateException.class, () -> PowerPredictor.builder()
                .presentVoltage(() -> 12.0).presentCurrent(() -> 0)
                .breakerBudgetAmps(0).build());
        assertThrows(IllegalStateException.class, () -> PowerPredictor.builder()
                .presentVoltage(() -> 12.0).presentCurrent(() -> 0)
                .breakerBudgetAmps(-10).build());
    }

    @Test
    void planRespectsTheBudgetRatherThanTheBattery() {
        // Three demands that fit inside 245 A of battery room but not inside a 120 A breaker.
        PowerPredictor p = at(12.4, 40, 120);
        PowerPredictor.PowerPlan plan = p.plan(
                new PowerPredictor.PowerDemand("Elevator", 60),
                new PowerPredictor.PowerDemand("Shooter", 45),
                new PowerPredictor.PowerDemand("Intake", 25));
        assertTrue(plan.waves().size() > 1,
                "with only 80 A to spend these cannot all go at once; got " + plan.waves().size() + " wave(s)");
    }
}
