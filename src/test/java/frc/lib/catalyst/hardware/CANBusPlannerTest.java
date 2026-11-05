package frc.lib.catalyst.hardware;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The bus-planning rules, which are pure arithmetic over the registry and need no hardware.
 *
 * <p>Worth testing carefully because the thing being encoded is a hardware fact that is invisible
 * from robot code and easy to get backwards: Systemcore's five CAN buses sit on three SPI
 * controllers, not five. Getting the pairing wrong produces a planner that confidently recommends
 * the arrangement it is supposed to prevent.
 */
class CANBusPlannerTest {

    @BeforeEach
    void reset() {
        CANRegistry.clear();
    }

    @Test
    void busesPairOntoSharedSpiControllersAsTheImageDescribes() {
        CatalystCANBus s0 = CatalystCANBus.systemcore(0);
        CatalystCANBus s1 = CatalystCANBus.systemcore(1);
        CatalystCANBus s2 = CatalystCANBus.systemcore(2);
        CatalystCANBus s3 = CatalystCANBus.systemcore(3);
        CatalystCANBus s4 = CatalystCANBus.systemcore(4);

        // From config.txt: s0+s1 on SPI2, s2 alone on SPI3, s3+s4 on SPI1.
        assertTrue(s0.sharesControllerWith(s1), "can_s0 and can_s1 share SPI2");
        assertTrue(s3.sharesControllerWith(s4), "can_s3 and can_s4 share SPI1");

        assertFalse(s0.sharesControllerWith(s3), "can_s0 and can_s3 are on different controllers");
        assertFalse(s2.sharesControllerWith(s0), "can_s2 has its controller to itself");
        assertFalse(s2.sharesControllerWith(s4), "can_s2 has its controller to itself");
    }

    @Test
    void aBusDoesNotContendWithItself() {
        CatalystCANBus s0 = CatalystCANBus.systemcore(0);
        assertFalse(s0.sharesControllerWith(s0),
                "the question is 'will these two fight', and one bus does not fight itself");
    }

    @Test
    void everythingOnOneBusIsFlagged() {
        // Twelve devices is under the estimated utilisation target - about 40% - and still worth
        // flagging, because four buses are sitting idle. The advice does not depend on the load
        // model being accurate, which is why the concentration rule is independent of it.
        for (int id = 0; id < 12; id++) {
            CANRegistry.register("Motor" + id, id, "can_s0", "TalonFX");
        }
        assertTrue(CANBusPlanner.utilizationOf("can_s0") < CANBusPlanner.TARGET_BUS_UTILIZATION,
                "this case is meant to be under the utilisation target, or it tests the wrong rule");

        List<String> problems = CANBusPlanner.validate();
        assertFalse(problems.isEmpty(), "12 devices on one bus with four idle should be flagged");
        assertTrue(problems.stream().anyMatch(p -> p.contains("can_s0")),
                "the offending bus should be named: " + problems);
    }

    @Test
    void aGenuinelySaturatedBusIsFlaggedOnUtilisation() {
        // Enough to exceed the utilisation target on its own, not merely to look lopsided.
        for (int id = 0; id < 20; id++) {
            CANRegistry.register("Motor" + id, id, "can_s0", "TalonFX");
        }
        assertTrue(CANBusPlanner.utilizationOf("can_s0") > CANBusPlanner.TARGET_BUS_UTILIZATION);
        assertTrue(CANBusPlanner.validate().stream().anyMatch(p -> p.contains("utilisation")),
                "a saturated bus should be called out as saturated, not merely as concentrated");
    }

    @Test
    void aLoadedPairIsFlaggedEvenWhenNeitherBusAloneWouldBe() {
        // Fifteen per bus sits under the single-bus target and takes the shared controller over the
        // pair target. This is the failure that hides: both buses look fine on their own.
        for (int id = 0; id < 15; id++) {
            CANRegistry.register("Left" + id, id, "can_s0", "TalonFX");
            CANRegistry.register("Right" + id, id, "can_s1", "TalonFX");
        }

        assertTrue(CANBusPlanner.utilizationOf("can_s0") <= CANBusPlanner.TARGET_BUS_UTILIZATION,
                "each bus alone should be within its own target, or this test proves nothing");

        List<String> problems = CANBusPlanner.validate();
        assertTrue(problems.stream().anyMatch(p -> p.contains("share an SPI controller")),
                "the pair should be flagged: " + problems);
    }

    @Test
    void theSamePairSplitAcrossControllersIsFine() {
        for (int id = 0; id < 15; id++) {
            CANRegistry.register("Left" + id, id, "can_s0", "TalonFX");
            CANRegistry.register("Right" + id, id, "can_s3", "TalonFX");
        }
        assertTrue(CANBusPlanner.validate().stream().noneMatch(p -> p.contains("SPI controller")),
                "can_s0 and can_s3 are on different controllers, so the same load must not be "
                        + "flagged as contention: " + CANBusPlanner.validate());
    }

    @Test
    void measurementOverridesTheEstimate() {
        for (int id = 0; id < 20; id++) {
            CANRegistry.register("Motor" + id, id, "can_s0", "TalonFX");
        }
        assertTrue(CANBusPlanner.utilizationOf("can_s0") > CANBusPlanner.TARGET_BUS_UTILIZATION,
                "the estimate should consider this saturated");

        // A real measurement always beats the model.
        CANBusPlanner.calibrate("can_s0", 0.20);
        assertEquals(0.20, CANBusPlanner.utilizationOf("can_s0"), 1e-9);
    }

    @Test
    void suggestionSpreadsDevicesAcrossControllersRatherThanWithinAPair() {
        for (int id = 0; id < 4; id++) {
            CANRegistry.register("Drive" + id, id, "can_s0", "TalonFX");
        }
        Map<String, List<String>> plan = CANBusPlanner.suggest();

        assertTrue(plan.size() > 1, "four motors should not all land on one bus: " + plan);

        // With four equal devices and three controllers, the first two must not share one.
        List<String> buses = List.copyOf(plan.keySet());
        CatalystCANBus first = CatalystCANBus.of(buses.get(0));
        CatalystCANBus second = CatalystCANBus.of(buses.get(1));
        assertNotEquals(first.controllerGroup(), second.controllerGroup(),
                "the planner should reach for a different SPI controller before a paired bus: " + plan);
    }

    @Test
    void everyRegisteredDeviceAppearsExactlyOnceInASuggestion() {
        for (int id = 0; id < 9; id++) {
            CANRegistry.register("Device" + id, id, "can_s0", id % 3 == 0 ? "CANcoder" : "TalonFX");
        }
        long placed = CANBusPlanner.suggest().values().stream().mapToLong(List::size).sum();
        assertEquals(9, placed, "a plan that loses or duplicates a device is worse than no plan");
    }
}
