package frc.lib.catalyst.driverstation;

import frc.lib.catalyst.system.SystemCoreSim;
import frc.lib.catalyst.system.SystemCoreSource;
import frc.lib.catalyst.system.SystemCoreStatus;
import frc.lib.catalyst.util.HealthCheck;
import frc.lib.catalyst.util.HealthMonitor;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.wpilib.command3.Scheduler;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * What the Driver Station board says.
 *
 * <p>This is the one surface a driver is definitely looking at in the thirty seconds before a match,
 * so the wording is the feature. Two lines and a handful of characters each: it has to say the worst
 * true thing first, and it has to be wrong in the safe direction.
 *
 * <p>{@link org.wpilib.driverstation.DriverStationDisplay} itself is not exercised here — writing to
 * it needs a Driver Station attached. What is tested is everything that decides <em>what</em> gets
 * written, which is where the judgement lives.
 */
class DriverBoardTest {

    @BeforeEach
    void freshMonitor() {
        HealthMonitor.getInstance().clear();
        SystemCoreStatus.useSource(SystemCoreSim.healthy());
    }

    @AfterEach
    void tidy() {
        HealthMonitor.getInstance().clear();
        SystemCoreStatus.useSource(SystemCoreSource.unavailable());
    }

    /**
     * Register a check whose condition is already true.
     *
     * <p>Debounce zeroed deliberately. Checks default to holding for a quarter second before firing,
     * which is right on a robot - it stops one noisy reading raising a fault - and is not what these
     * tests are about.
     */
    private static void arm(String id, HealthCheck.Severity severity) {
        HealthMonitor.getInstance().register(
                HealthCheck.builder("Test", id)
                        .severity(severity)
                        .when(() -> true)
                        .debounce(0)
                        .build());
    }

    /**
     * Let the monitor evaluate.
     *
     * <p>The wait is not padding. {@code HealthMonitor.update()} throttles itself to 200 Hz, so
     * registering a check and calling update in the same breath evaluates nothing - which is
     * correct on a robot and makes a test that registers several checks in a loop silently observe
     * only whichever one happened to land on a tick.
     */
    private static void settle() {
        try {
            Thread.sleep(8);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        HealthMonitor.getInstance().update();
    }

    // --- health ---------------------------------------------------------------

    @Test
    void aHealthyRobotSaysSoInTwoCharacters() {
        assertEquals("OK", DriverBoard.healthSummary());
    }

    @Test
    void aFailingCheckIsNamed() {
        // Naming it is the difference between a driver knowing what to look at and a driver knowing
        // only that something is wrong.
        arm("ArmEncoder", HealthCheck.Severity.ERROR);
        settle();
        assertEquals("FAULT: ArmEncoder", DriverBoard.healthSummary());
    }

    @Test
    void anErrorIsReportedAheadOfWarnings() {
        // One line, so it has to carry the worst true thing. A driver told "WARN: LowBattery" while
        // a motor is unreachable has been told the least useful of the two facts.
        arm("LooseBelt", HealthCheck.Severity.WARN);
        arm("ArmEncoder", HealthCheck.Severity.ERROR);
        settle();

        String summary = DriverBoard.healthSummary();
        assertTrue(summary.startsWith("FAULT:"), summary);
        assertTrue(summary.contains("ArmEncoder"), summary);
        assertTrue(!summary.contains("LooseBelt"), "the warning should not crowd out the fault: " + summary);
    }

    @Test
    void warningsAreShownWhenThereIsNothingWorse() {
        arm("LooseBelt", HealthCheck.Severity.WARN);
        settle();
        assertEquals("WARN: LooseBelt", DriverBoard.healthSummary());
    }

    @Test
    void pastAHandfulItCountsInsteadOfListing() {
        // A robot with nine problems has one problem, and it is not any of the nine. The count is
        // the useful fact, and a display that scrolls is a display nobody reads.
        for (int i = 0; i < 9; i++) {
            arm("Check" + i, HealthCheck.Severity.ERROR);
        }
        settle();
        assertEquals("FAULT: 9 problems", DriverBoard.healthSummary());
    }

    @Test
    void threeFitButFourDoNot() {
        for (int i = 0; i < 3; i++) {
            arm("C" + i, HealthCheck.Severity.WARN);
        }
        settle();
        assertTrue(DriverBoard.healthSummary().contains("C0"), "three should still be named");

        arm("C3", HealthCheck.Severity.WARN);
        settle();
        assertEquals("WARN: 4 problems", DriverBoard.healthSummary());
    }

    // --- battery --------------------------------------------------------------

    @Test
    void aGoodBatteryIsJustTheVoltage() {
        SystemCoreStatus.useSource(SystemCoreSim.healthy().withBattery(12.4));
        assertEquals("12.40 V", DriverBoard.batterySummary());
    }

    @Test
    void lowIsJudgedAgainstTheMachinesOwnThreshold() {
        // Not the roboRIO's 6.8 V. Systemcore publishes its own brownout voltage and it is not the
        // same number, so a fixed constant would call "low" at the wrong point on the one reading a
        // driver actually acts on.
        SystemCoreStatus.useSource(SystemCoreSim.healthy()
                .withBrownoutThresholds(6.75, 7.5)
                .withBattery(7.2));
        assertTrue(DriverBoard.batterySummary().endsWith("LOW"), DriverBoard.batterySummary());

        SystemCoreStatus.useSource(SystemCoreSim.healthy()
                .withBrownoutThresholds(6.75, 7.5)
                .withBattery(9.0));
        assertEquals("9.00 V", DriverBoard.batterySummary());
    }

    @Test
    void anActualBrownoutOutranksLow() {
        SystemCoreStatus.useSource(SystemCoreSim.healthy().withBattery(6.2).withBrownedOut(true));
        assertTrue(DriverBoard.batterySummary().contains("BROWNOUT"), DriverBoard.batterySummary());
    }

    @Test
    void noReadingIsADashRatherThanZero() {
        // A board showing "0.00 V" would have a driver chasing a battery that is fine. Absent is
        // absent.
        SystemCoreStatus.useSource(SystemCoreSim.healthy().withAvailable(false));
        assertEquals("-", DriverBoard.batterySummary());
    }

    // --- assembly -------------------------------------------------------------

    @Test
    void theStandardBoardCarriesTheTwoLinesThatMatter() {
        DriverBoard board = DriverBoard.standard();
        assertEquals(2, board.lineCount());
    }

    @Test
    void aLineThatThrowsDoesNotTakeTheBoardDown() {
        // A driver seeing one line read "error" still has the others. A board that throws mid-match
        // takes the whole display with it.
        DriverBoard board = DriverBoard.empty()
                .line("Bad", () -> { throw new IllegalStateException("boom"); })
                .line("Good", "fine");

        assertEquals("error", board.valueOf("Bad"));
        assertEquals("fine", board.valueOf("Good"));
    }

    @Test
    void startingItActuallyMakesItUpdate() {
        // It previously did not. start() set the display mode, returned, and the javadoc claimed a
        // periodic callback had been registered - so a board built exactly as documented would have
        // shown its first values and then never changed again, which reads as a frozen robot rather
        // than as a frozen display.
        Scheduler scheduler = Scheduler.createIndependentScheduler();
        int[] reads = {0};

        DriverBoard board = DriverBoard.empty()
                .line("Count", () -> String.valueOf(++reads[0]))
                .start(scheduler);

        assertTrue(board.isStarted());
        scheduler.run();
        scheduler.run();
        assertTrue(reads[0] >= 2, "the line should be re-read each loop, saw " + reads[0]);
    }

    @Test
    void startingTwiceDoesNotWriteEveryLineTwice() {
        Scheduler scheduler = Scheduler.createIndependentScheduler();
        int[] reads = {0};

        DriverBoard board = DriverBoard.empty()
                .line("Count", () -> String.valueOf(++reads[0]))
                .start(scheduler)
                .start(scheduler);

        scheduler.run();
        assertEquals(1, reads[0], "one callback, not two");
    }

    @Test
    void aNullValueBecomesADash() {
        DriverBoard board = DriverBoard.empty().line("Nothing", () -> null);
        assertEquals("-", board.valueOf("Nothing"));
    }
}
