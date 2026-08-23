package frc.lib.catalyst.system;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Reading Systemcore, against a simulated machine.
 *
 * <p>These could not exist before {@link SystemCoreSource}. Constructing the real reader forces a
 * HAL JNI load, and WPILib's loader terminates the JVM when the natives are missing — so a test that
 * touched it killed the Gradle worker and lost every other result. Splitting the reading from the
 * interpreting made the interpreting testable, which is where all the rules live.
 *
 * <p>What is worth testing here is mostly the trouble states. A healthy machine is easy to observe
 * on real hardware; a full disk or a brownout is not, because reproducing those means deliberately
 * breaking a robot during a practice match.
 */
class SystemCoreStatusTest {

    @AfterEach
    void leaveNoFakeMachineBehind() {
        // Deliberately restored to "no Systemcore" rather than to the real source.
        //
        // Resolving the real one constructs an NtSource, which loads HAL natives, which terminates
        // this JVM. "Absent" is also the truthful state for a desktop test run, so this leaves the
        // suite in a correct state rather than a convenient one - and nothing downstream inherits a
        // machine that reports 12.4 V when there is no machine.
        SystemCoreStatus.useSource(SystemCoreSource.unavailable());
    }

    private static SystemCoreStatus with(SystemCoreSim sim) {
        SystemCoreStatus.useSource(sim);
        return SystemCoreStatus.getInstance();
    }

    // --- absent hardware -----------------------------------------------------

    @Test
    void aMachineThatIsNotThereReportsEverythingEmpty() {
        SystemCoreStatus s = with(SystemCoreSim.healthy().withAvailable(false));

        assertFalse(s.isAvailable());
        assertTrue(s.batteryVolts().isEmpty(), "batteryVolts");
        assertTrue(s.cpuUtilization().isEmpty(), "cpuUtilization");
        assertTrue(s.ramFraction().isEmpty(), "ramFraction");
        assertTrue(s.storageFraction().isEmpty(), "storageFraction");
        assertTrue(s.teamNumber().isEmpty(), "teamNumber");
        assertFalse(s.isBrownedOut(), "an unknown brownout state must read as not-browned-out");
    }

    @Test
    void aBootedButSilentMachineIsAvailableWithNoReadings() {
        // Systemcore is up and the server exists, but nothing has published yet. Every reading must
        // be empty rather than zero - a robot reading 0 V should not conclude the battery is flat.
        SystemCoreStatus s = with(new SystemCoreSim());

        assertTrue(s.isAvailable(), "the server is there");
        assertTrue(s.batteryVolts().isEmpty());
        assertTrue(s.cpuUtilization().isEmpty());
    }

    @Test
    void publishingNeverThrows() {
        with(SystemCoreSim.healthy()).publish();
        with(new SystemCoreSim().withAvailable(false)).publish();
        with(new SystemCoreSim()).publish();
    }

    // --- unit conversion -----------------------------------------------------

    @Test
    void brownoutThresholdsConvertFromMillivolts() {
        // The OS publishes millivolts. Getting this wrong by 1000x would make BrownoutMonitor
        // predict against 6750 V, which no comparison would ever trip.
        SystemCoreStatus s = with(SystemCoreSim.healthy().withBrownoutThresholds(6.75, 7.5));

        assertEquals(6.75, s.brownoutVolts().orElseThrow(), 1e-9);
        assertEquals(7.5, s.recoveryVolts().orElseThrow(), 1e-9);
    }

    @Test
    void batteryIsAlreadyInVoltsAndIsNotConverted() {
        assertEquals(12.4, with(SystemCoreSim.healthy().withBattery(12.4))
                .batteryVolts().orElseThrow(), 1e-9);
    }

    // --- derived ratios ------------------------------------------------------

    @Test
    void fractionsAreComputedFromUsedAndTotal() {
        SystemCoreStatus s = with(SystemCoreSim.healthy()
                .withMemory(2_000_000_000.0, 8_000_000_000.0)
                .withStorage(24_000_000_000.0, 32_000_000_000.0));

        assertEquals(0.25, s.ramFraction().orElseThrow(), 1e-9);
        assertEquals(0.75, s.storageFraction().orElseThrow(), 1e-9);
    }

    @Test
    void halfARatioIsNoRatio() {
        // A fraction built from a missing denominator would be a guess wearing the clothes of a
        // measurement, and "0% storage free" reads as an emergency.
        SystemCoreStatus s = with(SystemCoreSim.healthy().clear("ramtotal").clear("storage"));

        assertTrue(s.ramFraction().isEmpty(), "no total means no ratio");
        assertTrue(s.storageFraction().isEmpty(), "no used means no ratio");
    }

    @Test
    void aZeroTotalDoesNotDivideByZero() {
        SystemCoreStatus s = with(SystemCoreSim.healthy().withMemory(1000, 0));
        assertTrue(s.ramFraction().isEmpty(), "a zero total is bad data, not an infinite ratio");
    }

    // --- the states nobody can reproduce on hardware -------------------------

    @Test
    void aBrownoutIsReported() {
        SystemCoreStatus s = with(SystemCoreSim.healthy().withBrownedOut(true).withBattery(6.2));

        assertTrue(s.isBrownedOut());
        assertEquals(6.2, s.batteryVolts().orElseThrow(), 1e-9);
    }

    @Test
    void aNearlyFullDiskIsVisibleAsAFraction() {
        // The failure that takes a robot out quietly: a full disk stops logging, then stops the
        // robot program, and nothing about it points at the disk.
        SystemCoreStatus s = with(SystemCoreSim.healthy()
                .withStorage(31_500_000_000.0, 32_000_000_000.0));

        assertTrue(s.storageFraction().orElseThrow() > 0.98,
                "a disk this full should read as nearly full");
    }

    @Test
    void aPinnedCpuIsVisible() {
        assertEquals(97.0, with(SystemCoreSim.healthy().withCpuPercent(97))
                .cpuUtilization().orElseThrow(), 1e-9);
    }

    // --- eMMC wear, which is the failure nobody watches for ------------------

    @Test
    void eMmcLifeIsReadAsBandsBecauseThatIsAllTheDeviceReports() {
        // JEDEC gives a code in 10% steps, not a percentage. Band 1 is 0-10% used, so the honest
        // answer is the middle of the band - claiming 10% or 0% would be inventing a digit the
        // device never provided.
        assertEquals(0.05, with(SystemCoreSim.healthy().withEmmc(1, 1, 1))
                .emmcLifeUsedFraction().orElseThrow(), 1e-9);
        assertEquals(0.55, with(SystemCoreSim.healthy().withEmmc(6, 6, 1))
                .emmcLifeUsedFraction().orElseThrow(), 1e-9);
        assertEquals(0.95, with(SystemCoreSim.healthy().withEmmc(10, 10, 1))
                .emmcLifeUsedFraction().orElseThrow(), 1e-9);
    }

    @Test
    void theWorseOfTheTwoRegionsIsTheOneThatMatters() {
        // Two regions with different wear. The flash fails when the worse one does, so averaging
        // them would report a healthy device right up until it stopped writing.
        assertEquals(0.85, with(SystemCoreSim.healthy().withEmmc(2, 9, 1))
                .emmcLifeUsedFraction().orElseThrow(), 1e-9);
    }

    @Test
    void aDeviceBeyondItsRatedLifeReportsFullyUsed() {
        assertEquals(1.0, with(SystemCoreSim.healthy().withEmmc(11, 11, 3))
                .emmcLifeUsedFraction().orElseThrow(), 1e-9);
    }

    @Test
    void jedecZeroMeansUnknownNotBrandNew() {
        // 0 is "not defined". Reading it as 0% used would report a worn-out card as factory fresh.
        assertTrue(with(SystemCoreSim.healthy().withEmmc(0, 0, 0))
                .emmcLifeUsedFraction().isEmpty());
    }

    @Test
    void preEolIsPassedThroughAsItsCode() {
        assertEquals(2, with(SystemCoreSim.healthy().withEmmc(3, 3, 2)).emmcPreEol().orElseThrow());
    }

    @Test
    void attentionIsFlaggedByEitherSignalIndependently() {
        // The two are independent: pre-EOL reflects blocks actually retired, lifetime reflects
        // writes estimated. Either alone is worth acting on, so requiring both would mean waiting
        // for the second one.
        assertFalse(with(SystemCoreSim.healthy().withEmmc(2, 2, 1)).emmcNeedsAttention(),
                "a healthy card should not nag");
        assertTrue(with(SystemCoreSim.healthy().withEmmc(2, 2, 2)).emmcNeedsAttention(),
                "pre-EOL warning alone");
        assertTrue(with(SystemCoreSim.healthy().withEmmc(9, 9, 1)).emmcNeedsAttention(),
                "high wear alone");
    }

    @Test
    void aMachineThatReportsNoEmmcDataSaysNothingRatherThanFine() {
        SystemCoreStatus s = with(SystemCoreSim.healthy()
                .clear("emmc/lifetime_a").clear("emmc/lifetime_b").clear("emmc/pre_eol"));
        assertTrue(s.emmcLifeUsedFraction().isEmpty());
        assertTrue(s.emmcPreEol().isEmpty());
        assertFalse(s.emmcNeedsAttention(), "unknown is not the same as bad");
    }

    // --- per-bus CAN ---------------------------------------------------------

    @Test
    void canUtilizationComesBackPerBus() {
        SystemCoreStatus s = with(SystemCoreSim.healthy().withCanUtilization(0.5, 0.1, 0, 0, 0));

        assertEquals(5, s.canBusUtilization().orElseThrow().length);
        assertEquals(0.5, s.canBusUtilization(0).orElseThrow(), 1e-9);
        assertEquals(0.1, s.canBusUtilization(1).orElseThrow(), 1e-9);
    }

    @Test
    void anIndexOutsideTheReadingIsEmptyNotZero() {
        // A CANivore is not can_s5. Reporting 0% for a bus that was never measured would read as an
        // idle bus rather than as no bus.
        SystemCoreStatus s = with(SystemCoreSim.healthy().withCanUtilization(0.5, 0.1));
        assertTrue(s.canBusUtilization(4).isEmpty());
        assertTrue(s.canBusUtilization(-1).isEmpty());
    }

    @Test
    void noCanReadingAtAllIsEmpty() {
        assertTrue(with(SystemCoreSim.healthy().clear("/diagnostics/canbusutil"))
                .canBusUtilization().isEmpty());
    }

    @Test
    void canFaultsAreCountsSinceBootNotACurrentState() {
        SystemCoreStatus s = with(SystemCoreSim.healthy().withCanFaults(3, 1, false));

        assertEquals(3, s.canBusDownCount().orElseThrow(), 1e-9);
        assertEquals(1, s.canBusUnavailableCount().orElseThrow(), 1e-9);
        assertFalse(s.canBusDown(), "three drops earlier does not mean it is down now");

        assertTrue(with(SystemCoreSim.healthy().withCanFaults(3, 1, true)).canBusDown());
    }

    // --- thermal, rail, interfaces -------------------------------------------

    @Test
    void temperatureAndRailAreReadStraightThrough() {
        SystemCoreStatus s = with(SystemCoreSim.healthy().withTemperature(71.5).withRail3v3(0.9));
        assertEquals(71.5, s.cpuTemperatureCelsius().orElseThrow(), 1e-9);
        assertEquals(0.9, s.rail3v3Amps().orElseThrow(), 1e-9);
    }

    @Test
    void networkInterfacesArePassedThroughUnparsed() {
        String[] got = with(SystemCoreSim.healthy().withNetworkInterfaces("eth0", "wlan0"))
                .networkInterfaces().orElseThrow();
        assertEquals(2, got.length);
        assertEquals("eth0", got[0]);
    }

    @Test
    void anAbsentMachineReportsNoneOfTheNewReadings() {
        SystemCoreStatus s = with(SystemCoreSim.healthy().withAvailable(false));

        assertTrue(s.cpuTemperatureCelsius().isEmpty());
        assertTrue(s.emmcLifeUsedFraction().isEmpty());
        assertTrue(s.canBusUtilization().isEmpty());
        assertTrue(s.networkInterfaces().isEmpty());
        assertFalse(s.canBusDown());
    }

    // --- identity ------------------------------------------------------------

    @Test
    void teamNumberComesBackAsAnInt() {
        assertEquals(5805, with(SystemCoreSim.healthy().withTeamNumber(5805))
                .teamNumber().orElseThrow());
    }
}
