package frc.lib.catalyst.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.catalyst.subsystems.vision.VisionHealth.Level;
import frc.lib.catalyst.subsystems.vision.VisionHealth.Outcome;
import frc.lib.catalyst.subsystems.vision.VisionHealth.State;
import frc.lib.catalyst.subsystems.vision.VisionHealth.Summary;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

/**
 * VisionHealth as a pure function of what the cameras report. No NetworkTables session, no Driver
 * Station: probes are hand-driven and alerts are recorded.
 */
class VisionHealthTest {

    /** A camera whose every signal the test sets directly. */
    static final class FakeProbe implements VisionHealth.CameraProbe {
        final String name;
        boolean connected = true;
        OptionalDouble sinceFrame = OptionalDouble.empty();
        OptionalDouble fps = OptionalDouble.of(50);
        OptionalDouble temp = OptionalDouble.of(60);

        FakeProbe(String name) {
            this.name = name;
        }

        @Override public String name() { return name; }
        @Override public boolean connected() { return connected; }
        @Override public OptionalDouble secondsSinceFrame() { return sinceFrame; }
        @Override public OptionalDouble fps() { return fps; }
        @Override public OptionalDouble temperatureC() { return temp; }
    }

    static final class Recorder implements VisionHealth.Alerts {
        final List<String> log = new ArrayList<>();

        @Override public void warning(String m) { log.add("warn:" + m); }
        @Override public void clearWarning(String m) { log.add("clearwarn:" + m); }
        @Override public void error(String m) { log.add("error:" + m); }
        @Override public void clearError(String m) { log.add("clearerror:" + m); }
    }

    FakeProbe a;
    FakeProbe b;
    Recorder alerts;
    VisionHealth health;

    @BeforeEach
    void setUp() {
        a = new FakeProbe("limelight-left");
        b = new FakeProbe("limelight-right");
        alerts = new Recorder();
        health = new VisionHealth(List.of(a, b), VisionHealth.Thresholds.defaults(), alerts);
    }

    /** Run n loops at 20 ms, with the given outcome for both cameras. Returns the last summary. */
    private Summary loops(int n, double from, Outcome outcome, String reason) {
        Summary s = null;
        for (int i = 0; i < n; i++) {
            double now = from + i * 0.02;
            health.observe(0, outcome, reason, now);
            health.observe(1, outcome, reason, now);
            s = health.update(now);
        }
        return s;
    }

    @Test
    void twoAcceptingCamerasAreHealthy() {
        Summary s = loops(5, 100.0, Outcome.ACCEPTED, null);
        assertEquals(Level.OK, s.level());
        assertEquals(2, s.camerasOk());
        assertEquals(State.OK, s.reports().get(0).state());
        assertEquals("all 2 cameras healthy", s.text());
        assertTrue(alerts.log.isEmpty(), alerts.log.toString());
    }

    @Test
    void seeingNothingIsNotAFault() {
        Summary s = loops(5, 100.0, Outcome.NONE, null);
        assertEquals(Level.OK, s.level());
        assertEquals(State.NO_TARGETS, s.reports().get(0).state());
        assertEquals("nothing seen yet", s.reports().get(0).detail());
    }

    @Test
    void anEstimateNobodyFusesStillCountsAsWorking() {
        Summary s = loops(5, 100.0, Outcome.SEEN, null);
        assertEquals(State.OK, s.reports().get(0).state());
    }

    @Test
    void disconnectedCameraDegradesVision() {
        a.connected = false;
        Summary s = loops(3, 100.0, Outcome.ACCEPTED, null);
        assertEquals(Level.DEGRADED, s.level());
        assertEquals(State.DISCONNECTED, s.reports().get(0).state());
        assertEquals(State.OK, s.reports().get(1).state());
        assertEquals("1 of 2 cameras healthy: limelight-left disconnected", s.text());
    }

    @Test
    void alertWaitsForTheFaultToLast() {
        a.connected = false;
        loops(10, 100.0, Outcome.ACCEPTED, null);           // 0.18 s of fault
        assertTrue(alerts.log.isEmpty(), "too early to alert: " + alerts.log);
        loops(60, 100.2, Outcome.ACCEPTED, null);           // now past a second
        assertEquals(List.of("warn:limelight-left: no data from the camera"), alerts.log);
    }

    @Test
    void alertClearsOnlyAfterRecoveryLasts() {
        a.connected = false;
        loops(70, 100.0, Outcome.ACCEPTED, null);
        assertEquals(1, alerts.log.size());
        a.connected = true;
        loops(10, 102.0, Outcome.ACCEPTED, null);
        assertEquals(1, alerts.log.size(), "cleared too soon: " + alerts.log);
        loops(60, 102.2, Outcome.ACCEPTED, null);
        assertEquals("clearwarn:limelight-left: no data from the camera", alerts.log.get(1));
    }

    @Test
    void aFlappingCameraNeverAlerts() {
        // Toggles every 0.3 s for six seconds: never a full second in either state.
        double t = 100.0;
        for (int burst = 0; burst < 20; burst++) {
            a.connected = burst % 2 == 0;
            loops(15, t, Outcome.ACCEPTED, null);
            t += 0.3;
        }
        assertTrue(alerts.log.isEmpty(), alerts.log.toString());
    }

    @Test
    void everyCameraDeadIsBlindAndAnError() {
        a.connected = false;
        b.connected = false;
        Summary s = loops(70, 100.0, Outcome.NONE, null);
        assertEquals(Level.BLIND, s.level());
        assertTrue(alerts.log.contains("error:No camera is delivering usable poses"), alerts.log.toString());
        a.connected = true;
        b.connected = true;
        loops(70, 102.0, Outcome.ACCEPTED, null);
        assertTrue(alerts.log.contains("clearerror:No camera is delivering usable poses"), alerts.log.toString());
    }

    @Test
    void hotCameraIsReported() {
        a.temp = OptionalDouble.of(85);
        Summary s = loops(3, 100.0, Outcome.ACCEPTED, null);
        assertEquals(State.HOT, s.reports().get(0).state());
        assertTrue(s.reports().get(0).detail().startsWith("85 C"), s.reports().get(0).detail());
    }

    @Test
    void lowFrameRateIsReported() {
        a.fps = OptionalDouble.of(4);
        Summary s = loops(3, 100.0, Outcome.ACCEPTED, null);
        assertEquals(State.LOW_FPS, s.reports().get(0).state());
    }

    @Test
    void mostlyRejectedCameraNamesTheReason() {
        for (int i = 0; i < 18; i++) {
            health.observe(0, Outcome.REJECTED, "TooFar(" + (1 + i * 0.1) + "m)", 100 + i * 0.02);
            health.observe(1, Outcome.ACCEPTED, null, 100 + i * 0.02);
        }
        health.observe(0, Outcome.ACCEPTED, null, 100.4);
        Summary s = health.update(100.4);
        assertEquals(State.REJECTING, s.reports().get(0).state());
        assertTrue(s.reports().get(0).detail().contains("TooFar"), s.reports().get(0).detail());
        assertEquals(Level.DEGRADED, s.level());
    }

    @Test
    void frozenFramesOnAConnectedCameraAreStale() {
        loops(3, 100.0, Outcome.ACCEPTED, null);
        a.sinceFrame = OptionalDouble.of(2.5);
        Summary s = health.update(100.1);
        assertEquals(State.STALE, s.reports().get(0).state());
        assertTrue(s.reports().get(0).detail().contains("2.5"), s.reports().get(0).detail());
    }

    @Test
    void aProbeThatThrowsReadsAsDisconnected() {
        VisionHealth.CameraProbe bad = new VisionHealth.CameraProbe() {
            @Override public String name() { return "bad"; }
            @Override public boolean connected() { throw new IllegalStateException("boom"); }
            @Override public OptionalDouble secondsSinceFrame() { return OptionalDouble.empty(); }
            @Override public OptionalDouble fps() { return OptionalDouble.empty(); }
            @Override public OptionalDouble temperatureC() { return OptionalDouble.empty(); }
        };
        VisionHealth h = new VisionHealth(List.of(bad), VisionHealth.Thresholds.defaults(), alerts);
        assertEquals(State.DISCONNECTED, h.update(1.0).reports().get(0).state());
    }

    @Test
    void noCamerasIsOkAndSaysSo() {
        VisionHealth h = new VisionHealth(List.of(), VisionHealth.Thresholds.defaults(), alerts);
        Summary s = h.update(1.0);
        assertEquals(Level.OK, s.level());
        assertEquals("no cameras configured", s.text());
    }

    @Test
    void reasonsAreGroupedWithoutTheirNumbers() {
        assertEquals("TooFar", VisionHealth.normalise("TooFar(1.2m)"));
        assertEquals("NoTags", VisionHealth.normalise("NoTags"));
        assertEquals("unknown", VisionHealth.normalise(null));
    }

    @Test
    void alertTextIsFixedPerState() {
        assertEquals(VisionHealth.alertText("x", State.HOT), VisionHealth.alertText("x", State.HOT));
        assertFalse(VisionHealth.alertText("x", State.HOT).matches(".*\\d.*"),
                "no live numbers in a message that has to be clearable");
    }
}
