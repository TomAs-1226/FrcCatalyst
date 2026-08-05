package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.lib.catalyst.physics.data.SignalSnapshot;
import frc.lib.catalyst.physics.data.TimestampSynchronizer;

/** Pure-Java tests for {@link TimestampSynchronizer}. */
class TimestampSynchronizerTest {

    @Test
    void readingsAreBackDatedByTheirRegisteredLatency() {
        TimestampSynchronizer sync = new TimestampSynchronizer(10);
        sync.register("fast", 0.0);
        sync.register("slow", 0.020);   // reports 20 ms late

        sync.record("fast", 1.0, 1.000);
        sync.record("slow", 2.0, 1.000);

        assertEquals(1.000, sync.buffer("fast").latestTimestamp(), 1e-9);
        assertEquals(0.980, sync.buffer("slow").latestTimestamp(), 1e-9);
    }

    @Test
    void latestCommonTimestampIsSetBySlowestSource() {
        TimestampSynchronizer sync = new TimestampSynchronizer(10);
        sync.register("fast", 0.0);
        sync.register("slow", 0.020);

        sync.record("fast", 1.0, 1.000);
        sync.record("slow", 1.0, 1.000);

        assertEquals(0.980, sync.latestCommonTimestamp(), 1e-9);
    }

    @Test
    void signalsAreInterpolatedOntoOneInstant() {
        TimestampSynchronizer sync = new TimestampSynchronizer(10);
        sync.register("a");
        sync.register("b");

        sync.record("a", 0.0, 0.0);
        sync.record("b", 100.0, 0.0);
        sync.record("a", 10.0, 1.0);
        sync.record("b", 200.0, 1.0);

        SignalSnapshot at = sync.snapshotAt(0.5);
        assertEquals(0.5, at.timestampSeconds(), 1e-9);
        assertEquals(5.0, at.get("a").getAsDouble(), 1e-9);
        assertEquals(150.0, at.get("b").getAsDouble(), 1e-9);
        assertEquals(2, at.size());
    }

    @Test
    void aSignalThatCannotCoverTheInstantIsAbsentRatherThanZero() {
        TimestampSynchronizer sync = new TimestampSynchronizer(10);
        sync.register("present");
        sync.register("late");

        sync.record("present", 1.0, 0.0);
        sync.record("present", 2.0, 1.0);
        sync.record("late", 9.0, 1.0);   // only has data at t=1.0

        SignalSnapshot at = sync.snapshotAt(0.5);
        assertTrue(at.has("present"));
        assertFalse(at.has("late"));
        assertTrue(at.get("late").isEmpty());
        assertEquals(-1.0, at.getOr("late", -1.0), 1e-9);
    }

    @Test
    void commonTimestampIsZeroUntilEverySignalHasData() {
        TimestampSynchronizer sync = new TimestampSynchronizer(10);
        sync.register("a");
        sync.register("b");

        sync.record("a", 1.0, 1.0);
        assertEquals(0.0, sync.latestCommonTimestamp(), 1e-9);

        sync.record("b", 1.0, 1.0);
        assertEquals(1.0, sync.latestCommonTimestamp(), 1e-9);
    }

    @Test
    void captureTimeRecordingIgnoresTheRegisteredLatency() {
        TimestampSynchronizer sync = new TimestampSynchronizer(10);
        sync.register("camera", 0.040);

        sync.recordAtCaptureTime("camera", 5.0, 0.900);   // frame carries its own timestamp

        assertEquals(0.900, sync.buffer("camera").latestTimestamp(), 1e-9);
    }

    @Test
    void unregisteredSignalsAndNegativeLatenciesAreRejected() {
        TimestampSynchronizer sync = new TimestampSynchronizer(10);
        assertThrows(IllegalArgumentException.class, () -> sync.record("nope", 1.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> sync.buffer("nope"));
        assertThrows(IllegalArgumentException.class, () -> sync.register("bad", -0.01));
    }
}
