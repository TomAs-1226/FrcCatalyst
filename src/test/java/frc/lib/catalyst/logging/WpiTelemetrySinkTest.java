package frc.lib.catalyst.logging;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.telemetry.TelemetryBackend;
import org.wpilib.telemetry.TelemetryEntry;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.util.protobuf.Protobuf;
import org.wpilib.util.struct.Struct;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Logging that happens before WPILib has anywhere to put it.
 *
 * <p>Hardware is claimed in constructors, and a container that owns hardware is built once — often
 * from a {@code static final} field, since building it twice would have two objects claiming the
 * same CAN ids. Class initialisation runs long before {@code RobotBase.startRobot} installs the
 * telemetry backend, so every publish made during construction lands in a registry that has nothing
 * behind it.
 *
 * <p>The trap that makes this worth a test file of its own: the registry is not empty in that
 * window. It answers every path with a {@link org.wpilib.telemetry.DiscardTelemetryBackend} — a
 * backend that exists, is not null, and throws away everything given to it. Four plausible ways of
 * asking "is there a backend yet" all say yes to it.
 */
class WpiTelemetrySinkTest {

    /** A backend that keeps what it is given, standing in for the NetworkTables one. */
    private static final class Recording implements TelemetryBackend {
        final Map<String, Object> written = new LinkedHashMap<>();
        final List<String> order = new ArrayList<>();

        @Override
        public TelemetryEntry getEntry(String path) {
            return new TelemetryEntry() {
                @Override public void keepDuplicates() {}
                @Override public void setProperty(String key, String value) {}

                @Override public void logBoolean(boolean value)  { put(path, value); }
                @Override public void logLong(long value)        { put(path, value); }
                @Override public void logFloat(float value)      { put(path, value); }
                @Override public void logDouble(double value)    { put(path, value); }
                @Override public void logString(String value, String type) { put(path, value); }

                @Override public void logBooleanArray(boolean[] value) { put(path, value); }
                @Override public void logShortArray(short[] value)     { put(path, value); }
                @Override public void logIntArray(int[] value)         { put(path, value); }
                @Override public void logLongArray(long[] value)       { put(path, value); }
                @Override public void logFloatArray(float[] value)     { put(path, value); }
                @Override public void logDoubleArray(double[] value)   { put(path, value); }
                @Override public void logStringArray(String[] value)   { put(path, value); }
                @Override public void logRaw(byte[] value, String type) { put(path, value); }

                @Override public <T> void logStruct(T value, Struct<? super T> struct) { put(path, value); }
                @Override public <T> void logStructArray(T[] value, Struct<? super T> struct) { put(path, value); }
                @Override public <T> void logProtobuf(T value, Protobuf<? super T, ?> proto) { put(path, value); }
            };
        }

        @Override
        public void close() {}

        private void put(String path, Object value) {
            if (!written.containsKey(path)) {
                order.add(path);
            }
            written.put(path, value);
        }
    }

    @AfterEach
    void clearRegistry() {
        TelemetryRegistry.reset();
    }

    /** Whatever the registry hands back for an unbacked path. */
    private static Object backendForRoot() {
        return TelemetryRegistry.getBackend("");
    }

    @Test
    void beforeAnyRobotStartsTheRegistryStillHandsBackABackend() {
        // The whole reason this class is not three lines. A null check here passes, and everything
        // written afterwards is silently thrown away.
        assertTrue(backendForRoot() != null,
                "if this ever becomes null, the sink's discard check can be simplified");
        assertEquals("DiscardTelemetryBackend", backendForRoot().getClass().getSimpleName(),
                "the sink recognises this type by identity; a rename breaks it silently");
    }

    @Test
    void writesMadeBeforeABackendExistsAreNotLost() {
        WpiTelemetrySink sink = new WpiTelemetrySink("CatalystTest");

        sink.log("Early/Name", "Rebuilt");
        sink.log("Early/Season", 2027L);

        Recording backend = new Recording();
        TelemetryRegistry.registerBackend("", backend);

        // The next write is what notices the backend and replays what was held.
        sink.log("Later/Ready", true);

        assertTrue(backend.written.containsKey("/CatalystTest/Early/Name"),
                "a value published during construction has to survive to the dashboard: "
                        + backend.written.keySet());
        assertTrue(backend.written.containsKey("/CatalystTest/Early/Season"));
        assertTrue(backend.written.containsKey("/CatalystTest/Later/Ready"));
    }

    @Test
    void theReplayKeepsTheOrderThingsWerePublishedIn() {
        WpiTelemetrySink sink = new WpiTelemetrySink("CatalystTest");
        sink.log("A", 1.0);
        sink.log("B", 2.0);

        Recording backend = new Recording();
        TelemetryRegistry.registerBackend("", backend);
        sink.log("C", 3.0);

        assertEquals(List.of("/CatalystTest/A", "/CatalystTest/B", "/CatalystTest/C"), backend.order,
                "a replay should look like what would have been written had a backend been there");
    }

    @Test
    void onlyTheNewestValuePerKeySurvives() {
        // Telemetry is state, not a stream of events. A dashboard connecting late wants the current
        // value, not a robot's boot sequence replayed at it - and unbounded history is a leak.
        WpiTelemetrySink sink = new WpiTelemetrySink("CatalystTest");
        for (int i = 0; i < 500; i++) {
            sink.log("Counter", (double) i);
        }

        Recording backend = new Recording();
        TelemetryRegistry.registerBackend("", backend);
        sink.log("Done", true);

        assertEquals(499.0, backend.written.get("/CatalystTest/Counter"));
        assertEquals(2, backend.order.size(), "500 writes to one key is one entry: " + backend.order);
    }

    @Test
    void anArrayTheCallerReusesIsNotCorruptedByTheReplay() {
        // A caller is entitled to reuse its buffer once log() returns. Holding the reference and
        // replaying it seconds later would publish whatever the array happens to contain by then,
        // which is a wrong number rather than a missing one - much harder to notice.
        WpiTelemetrySink sink = new WpiTelemetrySink("CatalystTest");
        double[] reused = { 1.0, 2.0, 3.0 };
        sink.log("Pose", reused);
        reused[0] = 99.0;

        Recording backend = new Recording();
        TelemetryRegistry.registerBackend("", backend);
        sink.log("Tick", true);

        assertEquals(1.0, ((double[]) backend.written.get("/CatalystTest/Pose"))[0],
                "the value at the time of the call is the one that was logged");
    }

    @Test
    void aSinkThatNeverSeesABackendDoesNotHoldValuesForever() {
        // The failure mode this design could hide, and it very nearly shipped. Buffering is a bridge
        // across robot startup; if no backend ever arrives - a unit test, a desktop tool, a harness
        // that never installs one - holding values indefinitely turns "logged nowhere, loudly" into
        // "logged nowhere, silently", which is the worse of the two and far harder to diagnose.
        //
        // Grace of zero rather than a test that sleeps for five seconds.
        WpiTelemetrySink sink = new WpiTelemetrySink("CatalystTest", 0);
        sink.log("Dropped", 1.0);

        assertFalse(sink.isBuffering(), "past the grace period nothing may be held");

        Recording backend = new Recording();
        TelemetryRegistry.registerBackend("", backend);
        sink.log("Live", 2.0);

        assertFalse(backend.written.containsKey("/CatalystTest/Dropped"),
                "a value written before giving up went to the discard backend and is gone - the "
                        + "point is that it was not held forever, not that it survived");
        assertTrue(backend.written.containsKey("/CatalystTest/Live"),
                "writes after a real backend appears must still reach it");
    }

    @Test
    void insideTheGracePeriodValuesAreHeld() {
        WpiTelemetrySink sink = new WpiTelemetrySink("CatalystTest");
        sink.log("Held", 1.0);
        assertTrue(sink.isBuffering(), "a fresh sink with no backend should be holding");
    }
}
