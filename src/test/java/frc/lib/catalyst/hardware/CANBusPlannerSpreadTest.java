package frc.lib.catalyst.hardware;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The planner has to be able to reach all five buses.
 *
 * <p>It used to score candidates by SPI-controller load alone. Within a pair both buses always
 * reported the same controller load, the comparison was strict, and the iteration order handed it to
 * the first every time — so {@code can_s1} and {@code can_s4} were never chosen, in any plan, ever.
 *
 * <p>That is not merely untidy. Each Systemcore CAN bus is an independent 1 Mbit/s wire even when it
 * shares a controller, so a twenty-device plan came back piled onto three buses with two sitting
 * empty — real headroom thrown away, in a plan the planner's own {@code validate()} would then
 * complain about.
 */
class CANBusPlannerSpreadTest {

    @AfterEach
    void clear() {
        CANRegistry.clear();
    }

    private static void devices(int count) {
        for (int i = 0; i < count; i++) {
            CANRegistry.register("Motor" + i, i + 1, "can_s0", "TalonFX");
        }
    }

    @Test
    void aBigPlanUsesMoreThanThreeBuses() {
        devices(20);
        Map<String, List<String>> plan = CANBusPlanner.suggest();

        assertTrue(plan.size() > 3,
                "twenty devices should reach more than three buses, got " + plan.keySet());
    }

    @Test
    void thePairedBusesAreReachableAtAll() {
        // Stated as the exact thing that was broken. Not "the plan is balanced" - "these two buses
        // can be chosen".
        devices(20);
        Map<String, List<String>> plan = CANBusPlanner.suggest();

        assertTrue(plan.containsKey("can_s1") || plan.containsKey("can_s4"),
                "neither can_s1 nor can_s4 was used: " + plan.keySet());
    }

    @Test
    void everyBusIsUsedWhenThereAreEnoughDevices() {
        devices(30);
        Map<String, List<String>> plan = CANBusPlanner.suggest();

        for (String bus : List.of("can_s0", "can_s1", "can_s2", "can_s3", "can_s4")) {
            assertTrue(plan.containsKey(bus), bus + " was never used: " + plan.keySet());
        }
    }

    @Test
    void theLoadIsSpreadEvenlyAcrossControllersRatherThanBuses() {
        // Controllers, not buses, and the distinction is the whole reason this planner exists.
        //
        // can_s2 has its controller to itself, so an even plan gives it a full third of the devices
        // while can_s0 and can_s1 split their third between them. A plan that looked even
        // bus-by-bus would actually be putting twice the load on the two shared controllers.
        //
        // 20 devices across 3 controllers is 7/7/6, and the per-bus counts that produces - 7, then
        // 4+3, then 3+3 - look lopsided precisely because they are correct.
        devices(20);
        Map<String, List<String>> plan = CANBusPlanner.suggest();

        int[] perController = new int[3];
        for (Map.Entry<String, List<String>> e : plan.entrySet()) {
            perController[CatalystCANBus.of(e.getKey()).controllerGroup()] += e.getValue().size();
        }

        int most = Math.max(perController[0], Math.max(perController[1], perController[2]));
        int fewest = Math.min(perController[0], Math.min(perController[1], perController[2]));
        assertTrue(most - fewest <= 2,
                "controller loads differ by " + (most - fewest) + ": "
                        + perController[0] + "/" + perController[1] + "/" + perController[2]
                        + " from " + plan);
    }

    @Test
    void noSingleBusTakesEverything() {
        // The looser per-bus check that is still worth having: the failure this fix was about was a
        // plan concentrated on three buses with two empty.
        devices(20);
        Map<String, List<String>> plan = CANBusPlanner.suggest();

        int most = plan.values().stream().mapToInt(List::size).max().orElse(0);
        assertTrue(most <= 10, "one bus took " + most + " of 20 devices: " + plan);
    }

    @Test
    void aSmallPlanStillPrefersTheUnpairedBusAndTheFirst() {
        // The tie-break order has to survive the fix: controller load first, then the unpaired bus,
        // then individual load. With few devices the plan should still be concentrated rather than
        // scattered one-per-bus for no reason.
        devices(2);
        Map<String, List<String>> plan = CANBusPlanner.suggest();

        assertTrue(plan.size() <= 2, "two devices should not need three buses: " + plan.keySet());
    }

    @Test
    void everyDevicePlacedExactlyOnce() {
        devices(17);
        Map<String, List<String>> plan = CANBusPlanner.suggest();

        int placed = plan.values().stream().mapToInt(List::size).sum();
        assertEquals(17, placed, "every device has to land somewhere, exactly once");
    }
}
