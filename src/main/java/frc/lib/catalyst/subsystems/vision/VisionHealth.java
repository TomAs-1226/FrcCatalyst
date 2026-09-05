package frc.lib.catalyst.subsystems.vision;

import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.util.AlertManager;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;

/**
 * Whether vision is actually working, camera by camera, in words a driver can act on.
 *
 * <h2>What was missing</h2>
 *
 * <p>Vision telemetry was a set of numbers - accepted counts, rejection reasons, per-camera poses -
 * that were all correct and none of which said "the left camera has been dead since the second
 * match". A camera that loses its NetworkTables session produces no rejections, because it produces
 * nothing; the numbers it used to publish simply stop changing. Nothing on the robot noticed, and a
 * dashboard cannot notice a number that is not moving.
 *
 * <p>This turns the per-camera signals Catalyst already has into a state per camera, a level for
 * vision as a whole, and alerts that appear when a camera has been in trouble long enough to mean it.
 *
 * <h2>States</h2>
 *
 * <ul>
 *   <li>{@code OK} - delivering estimates that are being accepted.
 *   <li>{@code NO_TARGETS} - alive, sees nothing. Normal for most of a match; not a fault.
 *   <li>{@code DISCONNECTED} - no data from the camera at all.
 *   <li>{@code STALE} - the camera is connected but its frames stopped advancing.
 *   <li>{@code HOT} - reported temperature at or above the configured ceiling. Limelight 4s
 *       throttle their pipeline when hot, and a throttled camera is a slow, then absent, one.
 *   <li>{@code LOW_FPS} - reporting a frame rate below the configured floor.
 *   <li>{@code REJECTING} - most of what it delivers is being rejected, and the detail says why.
 *       Usually a wrong transform, a wrong field map, or a tag that is not where the map says.
 * </ul>
 *
 * <p>The first two are healthy. The rest are faults. Vision as a whole is {@code OK} when nothing is
 * faulting, {@code DEGRADED} when something is, and {@code BLIND} when every camera is.
 *
 * <h2>Alerts are debounced; states are not</h2>
 *
 * <p>The published state changes the loop the evidence does, so a dashboard can be as jumpy as it
 * likes. The alert manager only hears about a fault that has lasted {@link #DEBOUNCE_SECONDS}, and
 * only hears it has cleared after the same again - a camera that drops one frame in ten does not
 * produce forty alerts a match. Alert text is fixed per state on purpose: AlertManager de-duplicates
 * and clears by exact message, so a message that carried a live temperature could never be cleared.
 * The numbers are on their own keys.
 *
 * <h2>Keys</h2>
 *
 * <pre>
 *   /Catalyst/Vision/Health/Level        int      0 ok, 1 degraded, 2 blind
 *   /Catalyst/Vision/Health/LevelName    string
 *   /Catalyst/Vision/Health/Summary      string   "3 of 4 cameras healthy: limelight-left disconnected"
 *   /Catalyst/Vision/Health/Cameras      int
 *   /Catalyst/Vision/Health/CamerasOk    int      OK or NO_TARGETS
 *   /Catalyst/Vision/Health/Names        string[]
 *   /Catalyst/Vision/Health/Rows         string[] "name|state|detail|fps|tempC|connected"
 *   /Catalyst/Vision/Health/&lt;name&gt;/State | Detail | Connected | Fps | TempC | SecondsSinceFrame | AcceptRatio
 * </pre>
 *
 * @since 2.0.0
 */
public final class VisionHealth {

    /** A camera's condition this loop. */
    public enum State {
        OK(false), NO_TARGETS(false), DISCONNECTED(true), STALE(true), HOT(true), LOW_FPS(true),
        REJECTING(true);

        private final boolean fault;

        State(boolean fault) {
            this.fault = fault;
        }

        /** Whether this state should count against vision as a whole. */
        public boolean isFault() {
            return fault;
        }
    }

    /** Vision as a whole. Ordinal is what gets published. */
    public enum Level { OK, DEGRADED, BLIND }

    /** What happened to a camera's output this loop, as told by {@link VisionSubsystem}. */
    public enum Outcome { NONE, SEEN, ACCEPTED, REJECTED }

    /** The signals a camera can answer about itself. Empty answers are allowed everywhere. */
    public interface CameraProbe {
        String name();

        /** Whether data is arriving from the camera at all. */
        boolean connected();

        /** Age of the newest frame, if the camera can say. */
        OptionalDouble secondsSinceFrame();

        OptionalDouble fps();

        OptionalDouble temperatureC();
    }

    /** Where alerts go. Exists so a test can listen without a Driver Station. */
    public interface Alerts {
        void warning(String message);

        void clearWarning(String message);

        void error(String message);

        void clearError(String message);
    }

    /** The thresholds, all in the units the keys use. */
    public record Thresholds(double hotCelsius, double minFps, double staleFrameSeconds,
                             double rejectingBelowRatio) {
        /** 80 degrees, 10 fps, one second, one estimate in five accepted. */
        public static Thresholds defaults() {
            return new Thresholds(80.0, 10.0, 1.0, 0.2);
        }
    }

    /** One camera's report, as published. */
    public record CameraReport(String name, State state, String detail, boolean connected,
                               OptionalDouble fps, OptionalDouble temperatureC,
                               OptionalDouble secondsSinceFrame, double acceptRatio,
                               int windowSamples) {}

    /** Vision as a whole, as published. */
    public record Summary(Level level, String text, int cameras, int camerasOk,
                          List<CameraReport> reports) {}

    /** How long a fault must persist before an alert, and how long it must be gone before it clears. */
    public static final double DEBOUNCE_SECONDS = 1.0;

    private static final int WINDOW = 100;
    private static final int MIN_WINDOW_FOR_RATIO = 10;
    private static final double NO_TARGET_AFTER_SECONDS = 1.0;
    private static final String SUBSYSTEM = "Vision";
    private static final String BLIND_TEXT = "No camera is delivering usable poses";

    private final List<CameraProbe> probes;
    private final Thresholds thresholds;
    private final Alerts alerts;
    private final Track[] tracks;

    private Summary last;
    private boolean blindAlerted = false;
    private double blindCandidateSince = Double.NaN;
    private double clearCandidateSince = Double.NaN;

    /** Per-camera bookkeeping. */
    private static final class Track {
        final Deque<Boolean> window = new ArrayDeque<>();      // true = accepted
        final Map<String, Integer> reasons = new HashMap<>();
        final Deque<String> reasonWindow = new ArrayDeque<>();
        int accepted = 0;
        double lastSeenTs = Double.NaN;      // an estimate was produced
        double lastAcceptedTs = Double.NaN;
        State state = State.NO_TARGETS;
        String detail = "";
        // Alert debouncing.
        State candidate = null;
        double candidateSince = Double.NaN;
        State alerted = null;
    }

    public VisionHealth(List<CameraProbe> probes, Thresholds thresholds) {
        this(probes, thresholds, alertManager());
    }

    VisionHealth(List<CameraProbe> probes, Thresholds thresholds, Alerts alerts) {
        this.probes = List.copyOf(probes);
        this.thresholds = thresholds;
        this.alerts = alerts;
        this.tracks = new Track[this.probes.size()];
        for (int i = 0; i < tracks.length; i++) tracks[i] = new Track();
    }

    /** Probes for the sources {@link VisionSubsystem} owns. */
    static List<CameraProbe> probesFor(List<CameraSource> cameras) {
        List<CameraProbe> out = new ArrayList<>(cameras.size());
        for (CameraSource c : cameras) {
            if (c instanceof LimelightSource ll) {
                out.add(new CameraProbe() {
                    @Override public String name() { return ll.getName(); }
                    @Override public boolean connected() { return ll.isConnected(); }
                    @Override public OptionalDouble secondsSinceFrame() { return ll.secondsSinceLastFrame(); }
                    @Override public OptionalDouble fps() { return ll.cameraFps(); }
                    @Override public OptionalDouble temperatureC() { return ll.cameraTemperatureC(); }
                });
            } else {
                out.add(new CameraProbe() {
                    @Override public String name() { return c.getName(); }
                    @Override public boolean connected() { return c.isConnected(); }
                    @Override public OptionalDouble secondsSinceFrame() { return OptionalDouble.empty(); }
                    @Override public OptionalDouble fps() { return OptionalDouble.empty(); }
                    @Override public OptionalDouble temperatureC() { return OptionalDouble.empty(); }
                });
            }
        }
        return out;
    }

    private static Alerts alertManager() {
        return new Alerts() {
            @Override public void warning(String m) { AlertManager.getInstance().warning(SUBSYSTEM, m); }
            @Override public void clearWarning(String m) { AlertManager.getInstance().clearWarning(SUBSYSTEM, m); }
            @Override public void error(String m) { AlertManager.getInstance().error(SUBSYSTEM, m); }
            @Override public void clearError(String m) { AlertManager.getInstance().clearError(SUBSYSTEM, m); }
        };
    }

    /**
     * Record what became of one camera's output this loop.
     *
     * @param camera  index into the probe list
     * @param outcome what happened
     * @param reason  the rejection reason, or null
     * @param now     seconds, monotonic
     */
    public void observe(int camera, Outcome outcome, String reason, double now) {
        Track t = tracks[camera];
        if (outcome == Outcome.NONE) return;
        t.lastSeenTs = now;
        if (outcome == Outcome.SEEN) {
            // Delivered a usable estimate that nobody is fusing. Healthy, by any measure.
            t.lastAcceptedTs = now;
            return;
        }
        boolean ok = outcome == Outcome.ACCEPTED;
        if (ok) t.lastAcceptedTs = now;
        t.window.addLast(ok);
        if (ok) t.accepted++;
        String key = ok ? null : normalise(reason);
        t.reasonWindow.addLast(key == null ? "" : key);
        if (key != null) t.reasons.merge(key, 1, Integer::sum);
        while (t.window.size() > WINDOW) {
            if (t.window.removeFirst()) t.accepted--;
            String old = t.reasonWindow.removeFirst();
            if (!old.isEmpty()) {
                t.reasons.merge(old, -1, Integer::sum);
                if (t.reasons.get(old) <= 0) t.reasons.remove(old);
            }
        }
    }

    /** Evaluate every camera, publish, and raise or clear alerts. Once per loop, after observing. */
    public Summary update(double now) {
        List<CameraReport> reports = new ArrayList<>(tracks.length);
        int ok = 0;
        List<String> problems = new ArrayList<>();
        for (int i = 0; i < tracks.length; i++) {
            CameraReport r = evaluate(i, now);
            reports.add(r);
            if (!r.state().isFault()) ok++;
            else problems.add(r.name() + " " + r.state().name().toLowerCase().replace('_', ' '));
            debounce(tracks[i], r, now);
        }

        Level level;
        if (tracks.length == 0 || problems.isEmpty()) level = Level.OK;
        else if (ok == 0) level = Level.BLIND;
        else level = Level.DEGRADED;

        String text;
        if (tracks.length == 0) text = "no cameras configured";
        else if (level == Level.OK) text = "all " + tracks.length + " camera" + (tracks.length == 1 ? "" : "s") + " healthy";
        else text = ok + " of " + tracks.length + " cameras healthy: " + String.join(", ", problems);

        debounceBlind(level, now);
        last = new Summary(level, text, tracks.length, ok, reports);
        publish(last);
        return last;
    }

    /** The most recent summary, or null before the first update. */
    public Summary summary() {
        return last;
    }

    private CameraReport evaluate(int i, double now) {
        CameraProbe p = probes.get(i);
        Track t = tracks[i];
        boolean connected;
        OptionalDouble fps;
        OptionalDouble temp;
        OptionalDouble sinceFrame;
        try {
            connected = p.connected();
            fps = p.fps();
            temp = p.temperatureC();
            sinceFrame = p.secondsSinceFrame();
        } catch (RuntimeException e) {
            connected = false;
            fps = OptionalDouble.empty();
            temp = OptionalDouble.empty();
            sinceFrame = OptionalDouble.empty();
        }
        // A probe that cannot date its frames is dated by what this class saw of it.
        if (sinceFrame.isEmpty() && !Double.isNaN(t.lastSeenTs)) {
            sinceFrame = OptionalDouble.of(now - t.lastSeenTs);
        }

        int samples = t.window.size();
        double ratio = samples == 0 ? 1.0 : (double) t.accepted / samples;

        State state;
        String detail;
        if (!connected) {
            state = State.DISCONNECTED;
            detail = "no data from the camera";
        } else if (sinceFrame.isPresent() && sinceFrame.getAsDouble() > thresholds.staleFrameSeconds()
                && !Double.isNaN(t.lastSeenTs)) {
            state = State.STALE;
            detail = String.format("frames stopped %.1f s ago", sinceFrame.getAsDouble());
        } else if (temp.isPresent() && temp.getAsDouble() >= thresholds.hotCelsius()) {
            state = State.HOT;
            detail = String.format("%.0f C, ceiling %.0f C", temp.getAsDouble(), thresholds.hotCelsius());
        } else if (fps.isPresent() && fps.getAsDouble() < thresholds.minFps()) {
            state = State.LOW_FPS;
            detail = String.format("%.0f fps, floor %.0f", fps.getAsDouble(), thresholds.minFps());
        } else if (samples >= MIN_WINDOW_FOR_RATIO && ratio < thresholds.rejectingBelowRatio()) {
            state = State.REJECTING;
            detail = String.format("%.0f%% accepted, mostly %s", ratio * 100, topReason(t));
        } else if (Double.isNaN(t.lastAcceptedTs) || now - t.lastAcceptedTs > NO_TARGET_AFTER_SECONDS) {
            state = State.NO_TARGETS;
            detail = Double.isNaN(t.lastSeenTs) ? "nothing seen yet" : "no usable target";
        } else {
            state = State.OK;
            detail = String.format("%.0f%% accepted", ratio * 100);
        }
        t.state = state;
        t.detail = detail;
        return new CameraReport(p.name(), state, detail, connected, fps, temp, sinceFrame, ratio, samples);
    }

    private void debounce(Track t, CameraReport r, double now) {
        State s = r.state().isFault() ? r.state() : null;    // null = healthy
        if (s != t.candidate) {
            t.candidate = s;
            t.candidateSince = now;
            return;
        }
        if (now - t.candidateSince < DEBOUNCE_SECONDS) return;
        if (s == t.alerted) return;
        // The candidate has held long enough and differs from what was alerted.
        if (t.alerted != null) alerts.clearWarning(alertText(r.name(), t.alerted));
        if (s != null) alerts.warning(alertText(r.name(), s));
        t.alerted = s;
    }

    private void debounceBlind(Level level, double now) {
        if (level == Level.BLIND) {
            clearCandidateSince = Double.NaN;
            if (Double.isNaN(blindCandidateSince)) blindCandidateSince = now;
            if (!blindAlerted && now - blindCandidateSince >= DEBOUNCE_SECONDS) {
                alerts.error(BLIND_TEXT);
                blindAlerted = true;
            }
        } else {
            blindCandidateSince = Double.NaN;
            if (Double.isNaN(clearCandidateSince)) clearCandidateSince = now;
            if (blindAlerted && now - clearCandidateSince >= DEBOUNCE_SECONDS) {
                alerts.clearError(BLIND_TEXT);
                blindAlerted = false;
            }
        }
    }

    /** Fixed text per state, so it can be cleared. */
    static String alertText(String camera, State state) {
        return switch (state) {
            case DISCONNECTED -> camera + ": no data from the camera";
            case STALE -> camera + ": frames have stopped";
            case HOT -> camera + ": running hot";
            case LOW_FPS -> camera + ": frame rate is low";
            case REJECTING -> camera + ": most estimates are being rejected";
            default -> camera + ": " + state.name().toLowerCase();
        };
    }

    private static String topReason(Track t) {
        String best = "unknown";
        int n = -1;
        for (Map.Entry<String, Integer> e : t.reasons.entrySet()) {
            if (e.getValue() > n) {
                n = e.getValue();
                best = e.getKey();
            }
        }
        return best;
    }

    /** "TooFar(1.2m)" and "TooFar(3.4m)" are one reason. */
    static String normalise(String reason) {
        if (reason == null || reason.isEmpty()) return "unknown";
        int paren = reason.indexOf('(');
        return paren < 0 ? reason : reason.substring(0, paren);
    }

    private void publish(Summary s) {
        String base = "Vision/Health/";
        CatalystLog.log(base + "Level", (long) s.level().ordinal());
        CatalystLog.log(base + "LevelName", s.level().name());
        CatalystLog.log(base + "Summary", s.text());
        CatalystLog.log(base + "Cameras", (long) s.cameras());
        CatalystLog.log(base + "CamerasOk", (long) s.camerasOk());
        String[] names = new String[s.reports().size()];
        String[] rows = new String[s.reports().size()];
        for (int i = 0; i < names.length; i++) {
            CameraReport r = s.reports().get(i);
            names[i] = r.name();
            rows[i] = r.name() + "|" + r.state().name() + "|" + r.detail() + "|"
                    + optional(r.fps()) + "|" + optional(r.temperatureC()) + "|" + r.connected();
            String cam = base + r.name() + "/";
            CatalystLog.log(cam + "State", r.state().name());
            CatalystLog.log(cam + "Detail", r.detail());
            CatalystLog.log(cam + "Connected", r.connected());
            CatalystLog.log(cam + "Fps", r.fps().orElse(Double.NaN));
            CatalystLog.log(cam + "TempC", r.temperatureC().orElse(Double.NaN));
            CatalystLog.log(cam + "SecondsSinceFrame", r.secondsSinceFrame().orElse(Double.NaN));
            CatalystLog.log(cam + "AcceptRatio", r.acceptRatio());
        }
        CatalystLog.log(base + "Names", names);
        CatalystLog.log(base + "Rows", rows);
    }

    private static String optional(OptionalDouble d) {
        return d.isPresent() ? String.format("%.1f", d.getAsDouble()) : "";
    }
}
