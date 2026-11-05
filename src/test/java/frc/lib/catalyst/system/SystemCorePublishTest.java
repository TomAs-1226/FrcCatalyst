package frc.lib.catalyst.system;

import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.logging.LogSink;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * What {@link SystemCoreStatus#publish()} puts on the wire, and how often.
 *
 * <p>Twenty-two readings at 50 Hz would be about 1100 NetworkTables writes a second, most of them
 * repeating that the team number is still 5805. The bandwidth is shared with everything else the
 * robot reports and the driver's station link pays for it, so the ones that move go out every loop
 * and the rest go out about once a second.
 *
 * <p>That split is easy to get subtly wrong in a way nothing notices: put a fast-moving reading in
 * the slow tier and a dashboard shows a one-second-old battery voltage, which is exactly the reading
 * nobody should be looking at stale. So this pins down which tier each one is in.
 */
class SystemCorePublishTest {

    /** Records what was logged, so the test can see the wire rather than infer it. */
    private static final class RecordingSink implements LogSink {
        final List<String> keys = new ArrayList<>();

        @Override public void log(String key, double value)    { keys.add(key); }
        @Override public void log(String key, boolean value)   { keys.add(key); }
        @Override public void log(String key, long value)      { keys.add(key); }
        @Override public void log(String key, String value)    { keys.add(key); }
        @Override public void log(String key, double[] value)  { keys.add(key); }
        @Override public void log(String key, boolean[] value) { keys.add(key); }
        @Override public void log(String key, long[] value)    { keys.add(key); }
        @Override public void log(String key, String[] value)  { keys.add(key); }

        Set<String> unique() {
            return new LinkedHashSet<>(keys);
        }

        int countOf(String key) {
            return (int) keys.stream().filter(key::equals).count();
        }

        void clear() {
            keys.clear();
        }
    }

    private RecordingSink sink;
    private LogSink previous;

    @BeforeEach
    void captureTheLog() {
        previous = CatalystLog.getSink();
        sink = new RecordingSink();
        CatalystLog.setSink(sink);
        SystemCoreStatus.useSource(SystemCoreSim.healthy());
    }

    @AfterEach
    void putItBack() {
        CatalystLog.setSink(previous);
        SystemCoreStatus.useSource(SystemCoreSource.unavailable());
    }

    private SystemCoreStatus status() {
        return SystemCoreStatus.getInstance();
    }

    // --- the first call has to be complete -----------------------------------

    @Test
    void everythingIsPublishedOnTheFirstCall() {
        // A dashboard that connects mid-match must fill in immediately. Waiting a second for the
        // slow tier would be defensible; waiting a second and showing blanks meanwhile is not.
        status().publish();

        for (String key : List.of(
                "Systemcore/BatteryVolts", "Systemcore/CpuPercent", "Systemcore/TempCelsius",
                "Systemcore/RamFraction", "Systemcore/StorageFraction", "Systemcore/TeamNumber",
                "Systemcore/EmmcLifeUsed", "Systemcore/CanUtilization",
                "Systemcore/NetworkInterfaces", "Systemcore/RamTotalBytes")) {
            assertTrue(sink.unique().contains(key), "missing on the first publish: " + key);
        }
    }

    // --- the split -----------------------------------------------------------

    @Test
    void theReadingsThatMoveGoOutEveryLoop() {
        SystemCoreStatus s = status();
        for (int i = 0; i < 10; i++) {
            s.publish();
        }

        for (String key : List.of(
                "Systemcore/BatteryVolts", "Systemcore/BrownedOut", "Systemcore/CpuPercent",
                "Systemcore/TempCelsius", "Systemcore/RamFraction", "Systemcore/Rail3v3Amps",
                "Systemcore/CanUtilization", "Systemcore/CanDown")) {
            assertEquals(10, sink.countOf(key), key + " should be published every loop");
        }
    }

    @Test
    void theReadingsThatDoNotMoveGoOutAboutOnceASecond() {
        SystemCoreStatus s = status();
        for (int i = 0; i < 10; i++) {
            s.publish();
        }

        // Ten loops is a fifth of a second. Everything slow was published once, on the first call.
        for (String key : List.of(
                "Systemcore/TeamNumber", "Systemcore/EmmcLifeUsed", "Systemcore/StorageFraction",
                "Systemcore/RamTotalBytes", "Systemcore/NetworkInterfaces",
                "Systemcore/BrownoutVolts", "Systemcore/HardwareSubRev")) {
            assertEquals(1, sink.countOf(key), key + " should not repeat every loop");
        }
    }

    @Test
    void theSlowTierComesRoundAgain() {
        // Slow must not mean once. A disk that fills during a match has to be visible before the
        // match ends.
        SystemCoreStatus s = status();
        for (int i = 0; i < 51; i++) {
            s.publish();
        }

        assertEquals(2, sink.countOf("Systemcore/StorageFraction"),
                "the slow tier should tick again after 50 loops");
        assertEquals(51, sink.countOf("Systemcore/BatteryVolts"),
                "and the fast tier should be unaffected");
    }

    @Test
    void aFullSecondOfLoopsCostsFarLessThanPublishingEverythingEveryTime() {
        // The point of the split, stated as the number it changes. Twenty-two readings at 50 Hz is
        // about 1100 writes a second; this should be nearer 400.
        SystemCoreStatus s = status();
        for (int i = 0; i < 50; i++) {
            s.publish();
        }

        assertTrue(sink.keys.size() < 500,
                "expected well under 500 writes per second, got " + sink.keys.size());
    }

    // --- nothing is published when there is nothing to publish ---------------

    @Test
    void anAbsentMachineWritesNothingAtAll() {
        // Simulation and desktop runs. Publishing zeroes here would put a fake healthy machine into
        // every replay log.
        SystemCoreStatus.useSource(SystemCoreSim.healthy().withAvailable(false));
        for (int i = 0; i < 60; i++) {
            SystemCoreStatus.getInstance().publish();
        }

        assertTrue(sink.keys.isEmpty(), "wrote " + sink.keys);
    }

    @Test
    void aReadingTheMachineDoesNotSendIsNotInvented() {
        SystemCoreStatus.useSource(SystemCoreSim.healthy()
                .clear("temp").clear("emmc/lifetime_a").clear("emmc/lifetime_b"));
        SystemCoreStatus.getInstance().publish();

        assertFalse(sink.unique().contains("Systemcore/TempCelsius"),
                "an absent sensor must not publish a zero");
        assertFalse(sink.unique().contains("Systemcore/EmmcLifeUsed"));
        assertTrue(sink.unique().contains("Systemcore/CpuPercent"),
                "but the readings that are present still go out");
    }
}
