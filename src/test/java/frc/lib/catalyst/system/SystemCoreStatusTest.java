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

    // --- identity ------------------------------------------------------------

    @Test
    void teamNumberComesBackAsAnInt() {
        assertEquals(5805, with(SystemCoreSim.healthy().withTeamNumber(5805))
                .teamNumber().orElseThrow());
    }
}
