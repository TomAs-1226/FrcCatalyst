package frc.lib.catalyst.util;

import frc.lib.catalyst.hardware.CANRegistry;
import frc.lib.catalyst.system.SystemCoreSim;
import frc.lib.catalyst.system.SystemCoreSource;
import frc.lib.catalyst.system.SystemCoreStatus;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The boot check, against machines in the states nobody can arrange on purpose.
 *
 * <p>Every one of these is a robot that boots normally, connects normally, and then fails at the
 * worst moment as something that does not resemble its cause. That is the whole reason to look for
 * them before anyone walks to the field, and it is why the wording matters as much as the verdict —
 * a line that says what to do is worth ten that say something is wrong.
 */
class PreflightTest {

    @AfterEach
    void tidy() {
        SystemCoreStatus.useSource(SystemCoreSource.unavailable());
        CANRegistry.clear();
    }

    private static Preflight.Report on(SystemCoreSim machine) {
        SystemCoreStatus.useSource(machine);
        return Preflight.run();
    }

    // --- the verdict ---------------------------------------------------------

    @Test
    void aHealthyMachineIsReady() {
        Preflight.Report report = on(SystemCoreSim.healthy());

        assertTrue(report.canEnable(), report.summary());
        assertEquals("Ready", report.summary());
    }

    @Test
    void noSystemcoreIsNotAFailure() {
        // Simulation and bench work are legitimate places to be. A preflight that called them broken
        // would be wrong every time anybody ran the simulator.
        Preflight.Report report = on(SystemCoreSim.healthy().withAvailable(false));

        assertTrue(report.canEnable());
        assertTrue(report.findings().stream()
                        .anyMatch(f -> f.detail() != null && f.detail().contains("simulation")),
                "it should say why it can see nothing");
    }

    // --- the states that stop a robot ----------------------------------------

    @Test
    void aFullDiskIsABlocker() {
        // The failure that takes a robot out quietly: logging stops, then the robot program stops,
        // and nothing about the symptom points at the disk.
        Preflight.Report report = on(SystemCoreSim.healthy()
                .withStorage(31_500_000_000.0, 32_000_000_000.0));

        assertFalse(report.canEnable());
        assertTrue(report.summary().startsWith("NOT READY"), report.summary());
    }

    @Test
    void aBatteryBelowTheBrownoutThresholdIsABlocker() {
        // Judged against the machine's own floor, not a roboRIO constant. A robot already browning
        // out on the cart will not survive being driven.
        Preflight.Report report = on(SystemCoreSim.healthy()
                .withBrownoutThresholds(6.75, 7.5)
                .withBattery(6.5));

        assertFalse(report.canEnable());
    }

    @Test
    void aLowButWorkableBatteryIsOnlyAWarning() {
        Preflight.Report report = on(SystemCoreSim.healthy().withBattery(11.4));

        assertTrue(report.canEnable(), "11.4 V is not a reason to refuse");
        assertTrue(report.summary().contains("warning"), report.summary());
    }

    @Test
    void wornFlashWarnsRatherThanBlocks() {
        // It will still run this match. It is a thing to order a part for, not a thing to stop for.
        Preflight.Report report = on(SystemCoreSim.healthy().withEmmc(10, 10, 2));

        assertTrue(report.canEnable());
        assertTrue(report.findings().stream().anyMatch(f -> f.what().contains("Flash")));
    }

    @Test
    void aHotMachineWarnsAndSaysWhatItWillLookLike() {
        // The point of the wording: a thermal problem does not present as a temperature. The cores
        // throttle and the symptom that reaches robot code is a loop overrun.
        Preflight.Report report = on(SystemCoreSim.healthy().withTemperature(85));

        assertTrue(report.canEnable());
        assertTrue(report.findings().stream()
                        .anyMatch(f -> f.detail() != null && f.detail().contains("loop overrun")),
                "it should say what a hot machine actually looks like");
    }

    // --- ordering and wording ------------------------------------------------

    @Test
    void theWorstThingIsReportedFirst() {
        // Somebody reading three lines should read the three that matter.
        Preflight.Report report = on(SystemCoreSim.healthy()
                .withBattery(6.2)
                .withTemperature(85)
                .withStorage(31_500_000_000.0, 32_000_000_000.0));

        assertEquals(Preflight.Level.BLOCKER, report.findings().get(0).level());
        assertFalse(report.canEnable());
    }

    @Test
    void severalBlockersStillGiveOneActionableLine() {
        Preflight.Report report = on(SystemCoreSim.healthy()
                .withBattery(6.2)
                .withStorage(31_500_000_000.0, 32_000_000_000.0));

        // Not "3 problems". The first thing to fix, named.
        assertTrue(report.summary().startsWith("NOT READY: "), report.summary());
        assertTrue(report.summary().length() > "NOT READY: ".length());
    }

    @Test
    void everyFindingReadsAsSomethingAPersonCanActOn() {
        Preflight.Report report = on(SystemCoreSim.healthy()
                .withStorage(31_500_000_000.0, 32_000_000_000.0));

        for (Preflight.Finding f : report.atLeast(Preflight.Level.WARN)) {
            assertTrue(f.detail() != null && !f.detail().isBlank(),
                    "a warning with no detail is a warning nobody can act on: " + f.what());
        }
    }

    @Test
    void printingNeverThrows() {
        on(SystemCoreSim.healthy()).printToConsole();
        on(SystemCoreSim.healthy().withAvailable(false)).printToConsole();
        on(SystemCoreSim.healthy().withBattery(6.0).withEmmc(11, 11, 3)).printToConsole();
    }
}
