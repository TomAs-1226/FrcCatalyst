package frc.lib.catalyst.identity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.catalyst.hardware.CatalystCANBus;
import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.util.MiniJson;
import org.wpilib.driverstation.RobotState;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

/**
 * What every motor on the robot has been through, by serial number, kept on the robot.
 *
 * <h2>Why</h2>
 *
 * <p>A motor's CAN id and name are what the code knows it by, and both are one Tuner X session
 * away from changing - the Catalyst X1 arrived with a "front left steer" that was the front-right
 * drive and an "intake roller" that was a back drive motor, all correctly labelled for a robot they
 * were no longer on. The serial number is the motor. This keeps, per serial: every id, name, bus
 * and firmware it has ever been seen with and when; how long it has been powered, turning and
 * under load; how far it has turned; the hottest and hardest it has ever been driven; and a short
 * log of recent boots. So "which of these eight Falcons is the tired one" and "was this motor on
 * last year's robot" have answers, and a motor that gets renumbered does not lose its past.
 *
 * <h2>How</h2>
 *
 * <p>Identity comes from the robot program's own Phoenix diagnostic server (the one Tuner X talks
 * to), asked every {@link Config#rescanSeconds} for the devices on every bus - so a motor needs no
 * code of its own to be tracked, and the ones the team's code never constructs (a spare still
 * wired in, the bench rig's) are tracked too. Usage comes from the status frames every Talon FX
 * already sends: velocity, currents, voltage, temperature, sticky faults, read at
 * {@link Config#samplePeriodSeconds} through Phoenix objects this class makes for itself.
 *
 * <p>Everything is kept in one JSON file on the robot ({@link Config#path}, by default
 * {@code catalyst/motor-history.json} beside the program), read once at boot and written when it
 * changes, at most every {@link Config#flushSeconds} and whenever the robot disables. The file is
 * the record: nothing is re-derived at boot, and a bench laptop can copy it off, the Console reads
 * a live summary from {@code /Catalyst/MotorHistory}, and the Systemcore agent serves the whole
 * file on {@code /api/motor-history}.
 *
 * <p>Runs on {@link frc.lib.catalyst.util.HealthMonitor#update()}'s tick, like the device roster,
 * so any robot that has a HealthMonitor has this. Nothing here can throw into the robot loop.
 *
 * @since 2.0.0
 */
public final class MotorHistory {
    private MotorHistory() {}

    /** Settings. Change them before the first {@link #update} with {@link #configure}. */
    public static final class Config {
        /** Where the file lives. Relative paths are under the program's working directory. */
        public String path = "catalyst/motor-history.json";
        /** A motor at or above this counts hot seconds. Falcons throttle in the 90s; 70 is "working hard". */
        public double hotCelsius = 70.0;
        /** How often usage is sampled. Currents and temperature update at 4 Hz on the bus. */
        public double samplePeriodSeconds = 0.1;
        /** How often the diagnostic server is asked for the device list. */
        public double rescanSeconds = 30.0;
        /** How long a change may wait before it is on disk (also written on every disable). */
        public double flushSeconds = 30.0;
        /** Boots kept per motor. Totals are kept forever. */
        public int maxSessions = 40;
        /** The diagnostic server. The robot program's own; port 1250 is Phoenix's. */
        public String diagnosticServer = "http://127.0.0.1:1250/";
        /** A rotor slower than this is "not turning". */
        public double runningRps = 0.5;
        /** Less stator current than this is "not under load". */
        public double loadedAmps = 5.0;
    }

    /** One reading of a motor. */
    public record Sample(double velocityRps, double statorAmps, double supplyAmps, double motorVolts,
                         double tempC, long stickyFaults) {}

    /** An id / name / bus / firmware a device has been seen with, and when. */
    public static final class Identity {
        public int id;
        public String name = "";
        public String bus = "";
        public String firmware = "";
        public long firstSeenMs;
        public long lastSeenMs;

        boolean matches(int id, String name, String bus, String firmware) {
            return this.id == id && this.name.equals(name) && this.bus.equals(bus) && this.firmware.equals(firmware);
        }

        Map<String, Object> toJson() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("id", id);
            m.put("name", name);
            m.put("bus", bus);
            m.put("firmware", firmware);
            m.put("firstSeenMs", firstSeenMs);
            m.put("lastSeenMs", lastSeenMs);
            return m;
        }

        static Identity fromJson(Map<String, Object> m) {
            Identity i = new Identity();
            i.id = (int) num(m, "id");
            i.name = str(m, "name");
            i.bus = str(m, "bus");
            i.firmware = str(m, "firmware");
            i.firstSeenMs = (long) num(m, "firstSeenMs");
            i.lastSeenMs = (long) num(m, "lastSeenMs");
            return i;
        }
    }

    /** One boot's worth of use. */
    public static final class Session {
        public long startMs;
        public double seconds;
        public double runningSeconds;
        public double revolutions;
        public double peakAmps;
        public double peakTempC;
        public double hotSeconds;

        Map<String, Object> toJson() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("startMs", startMs);
            m.put("seconds", round(seconds));
            m.put("runningSeconds", round(runningSeconds));
            m.put("revolutions", round(revolutions));
            m.put("peakAmps", round(peakAmps));
            m.put("peakTempC", round(peakTempC));
            m.put("hotSeconds", round(hotSeconds));
            return m;
        }

        static Session fromJson(Map<String, Object> m) {
            Session s = new Session();
            s.startMs = (long) num(m, "startMs");
            s.seconds = num(m, "seconds");
            s.runningSeconds = num(m, "runningSeconds");
            s.revolutions = num(m, "revolutions");
            s.peakAmps = num(m, "peakAmps");
            s.peakTempC = num(m, "peakTempC");
            s.hotSeconds = num(m, "hotSeconds");
            return s;
        }
    }

    /** Everything known about one device, keyed by its serial number. */
    public static final class Record {
        public String serial = "";
        public String model = "";
        /** motor, encoder, imu or device. Only motors have usage. */
        public String kind = "device";
        public String hardwareRev = "";
        public String manufactured = "";
        public long firstSeenMs;
        public long lastSeenMs;
        public int boots;
        public double poweredSeconds;
        public double runningSeconds;
        public double loadedSeconds;
        public double revolutions;
        public double energyJoules;
        public double peakStatorAmps;
        public double peakTempC;
        public double hotSeconds;
        public long stickyFaults;
        public final List<Identity> identities = new ArrayList<>();
        public final List<Session> sessions = new ArrayList<>();
        /** This boot's session, once the device has been seen this boot. */
        Session current;

        /** The identity currently in force. */
        public Identity latest() {
            return identities.isEmpty() ? null : identities.get(identities.size() - 1);
        }

        /** Note the device on the bus with these particulars. A change of any of them is a new identity. */
        void observe(int id, String name, String bus, String firmware, long nowMs, int maxSessions) {
            if (firstSeenMs == 0) {
                firstSeenMs = nowMs;
            }
            lastSeenMs = nowMs;
            Identity last = latest();
            if (last == null || !last.matches(id, name, bus, firmware)) {
                Identity i = new Identity();
                i.id = id;
                i.name = name;
                i.bus = bus;
                i.firmware = firmware;
                i.firstSeenMs = nowMs;
                i.lastSeenMs = nowMs;
                identities.add(i);
            } else {
                last.lastSeenMs = nowMs;
            }
            if (current == null) {
                current = new Session();
                current.startMs = nowMs;
                sessions.add(current);
                boots++;
                while (sessions.size() > maxSessions) {
                    sessions.remove(0);
                }
            }
        }

        /** Account {@code dt} seconds of the motor being as {@code s} says. */
        void sample(Sample s, double dt, Config cfg) {
            if (current == null || dt <= 0) {
                return;
            }
            poweredSeconds += dt;
            current.seconds += dt;
            boolean running = Math.abs(s.velocityRps()) >= cfg.runningRps;
            if (running) {
                runningSeconds += dt;
                current.runningSeconds += dt;
                double revs = Math.abs(s.velocityRps()) * dt;
                revolutions += revs;
                current.revolutions += revs;
            }
            double amps = Math.abs(s.statorAmps());
            if (amps >= cfg.loadedAmps) {
                loadedSeconds += dt;
            }
            energyJoules += Math.abs(s.motorVolts() * s.statorAmps()) * dt;
            if (amps > peakStatorAmps) {
                peakStatorAmps = amps;
            }
            if (amps > current.peakAmps) {
                current.peakAmps = amps;
            }
            if (s.tempC() > peakTempC) {
                peakTempC = s.tempC();
            }
            if (s.tempC() > current.peakTempC) {
                current.peakTempC = s.tempC();
            }
            if (s.tempC() >= cfg.hotCelsius) {
                hotSeconds += dt;
                current.hotSeconds += dt;
            }
            stickyFaults |= s.stickyFaults();
        }

        Map<String, Object> toJson() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("serial", serial);
            m.put("model", model);
            m.put("kind", kind);
            m.put("hardwareRev", hardwareRev);
            m.put("manufactured", manufactured);
            m.put("firstSeenMs", firstSeenMs);
            m.put("lastSeenMs", lastSeenMs);
            m.put("boots", boots);
            Map<String, Object> t = new LinkedHashMap<>();
            t.put("poweredSeconds", round(poweredSeconds));
            t.put("runningSeconds", round(runningSeconds));
            t.put("loadedSeconds", round(loadedSeconds));
            t.put("revolutions", round(revolutions));
            t.put("energyJoules", round(energyJoules));
            t.put("peakStatorAmps", round(peakStatorAmps));
            t.put("peakTempC", round(peakTempC));
            t.put("hotSeconds", round(hotSeconds));
            t.put("stickyFaults", stickyFaults);
            m.put("totals", t);
            List<Object> ids = new ArrayList<>();
            for (Identity i : identities) {
                ids.add(i.toJson());
            }
            m.put("identities", ids);
            List<Object> ss = new ArrayList<>();
            for (Session s : sessions) {
                ss.add(s.toJson());
            }
            m.put("sessions", ss);
            return m;
        }

        @SuppressWarnings("unchecked")
        static Record fromJson(Map<String, Object> m) {
            Record r = new Record();
            r.serial = str(m, "serial");
            r.model = str(m, "model");
            r.kind = str(m, "kind").isEmpty() ? "device" : str(m, "kind");
            r.hardwareRev = str(m, "hardwareRev");
            r.manufactured = str(m, "manufactured");
            r.firstSeenMs = (long) num(m, "firstSeenMs");
            r.lastSeenMs = (long) num(m, "lastSeenMs");
            r.boots = (int) num(m, "boots");
            Object totals = m.get("totals");
            if (totals instanceof Map<?, ?> t) {
                Map<String, Object> tm = (Map<String, Object>) t;
                r.poweredSeconds = num(tm, "poweredSeconds");
                r.runningSeconds = num(tm, "runningSeconds");
                r.loadedSeconds = num(tm, "loadedSeconds");
                r.revolutions = num(tm, "revolutions");
                r.energyJoules = num(tm, "energyJoules");
                r.peakStatorAmps = num(tm, "peakStatorAmps");
                r.peakTempC = num(tm, "peakTempC");
                r.hotSeconds = num(tm, "hotSeconds");
                r.stickyFaults = (long) num(tm, "stickyFaults");
            }
            if (m.get("identities") instanceof List<?> l) {
                for (Object o : l) {
                    if (o instanceof Map<?, ?> im) {
                        r.identities.add(Identity.fromJson((Map<String, Object>) im));
                    }
                }
            }
            if (m.get("sessions") instanceof List<?> l) {
                for (Object o : l) {
                    if (o instanceof Map<?, ?> sm) {
                        r.sessions.add(Session.fromJson((Map<String, Object>) sm));
                    }
                }
            }
            return r;
        }
    }

    /** A device as the diagnostic server describes it. */
    record DeviceInfo(String model, int id, String name, String serial, String bus, String firmware,
                      String hardwareRev, String manufactured) {
        String key() {
            return serial.isEmpty() ? bus + ":" + model + ":" + id : serial;
        }

        String kind() {
            String m = model.toLowerCase(Locale.ROOT);
            if (m.contains("talon")) {
                return "motor";
            }
            if (m.contains("cancoder")) {
                return "encoder";
            }
            if (m.contains("pigeon")) {
                return "imu";
            }
            return "device";
        }
    }

    // ------------------------------------------------------------------ state

    private static Config config = new Config();
    private static final Map<String, Record> RECORDS = new LinkedHashMap<>();
    private static final Map<String, Tracked> TRACKED = new LinkedHashMap<>();
    private static final AtomicReference<List<DeviceInfo>> DISCOVERED = new AtomicReference<>(null);
    private static final HttpClient HTTP = HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(2)).build();
    private static boolean loaded = false;
    private static boolean dirty = false;
    private static volatile boolean scanning = false;
    private static double lastScan = Double.NEGATIVE_INFINITY;
    private static double lastSample = Double.NaN;
    private static double lastFlush = Double.NEGATIVE_INFINITY;
    private static double lastPublish = Double.NEGATIVE_INFINITY;
    private static boolean wasEnabled = false;
    private static boolean warnedOnce = false;
    private static String lastFailure = "";

    /** A motor being read: its Phoenix object and the signals, refreshed together. */
    private static final class Tracked {
        final Record record;
        final TalonFX motor;
        final StatusSignal<?> velocity;
        final StatusSignal<?> stator;
        final StatusSignal<?> supply;
        final StatusSignal<?> volts;
        final StatusSignal<?> temp;
        final StatusSignal<?> sticky;

        Tracked(Record record, TalonFX motor) {
            this.record = record;
            this.motor = motor;
            velocity = motor.getVelocity();
            stator = motor.getStatorCurrent();
            supply = motor.getSupplyCurrent();
            volts = motor.getMotorVoltage();
            temp = motor.getDeviceTemp();
            sticky = motor.getStickyFaultField();
        }

        Sample sample() {
            return new Sample(velocity.getValueAsDouble(), stator.getValueAsDouble(), supply.getValueAsDouble(),
                    volts.getValueAsDouble(), temp.getValueAsDouble(), (long) sticky.getValueAsDouble());
        }
    }

    /** Replace the settings. Before the first {@link #update}, or the path change is ignored. */
    public static synchronized void configure(Config cfg) {
        config = cfg;
    }

    /**
     * Discover, sample, persist and publish, rate-limited inside. Called at loop rate by
     * {@link frc.lib.catalyst.util.HealthMonitor#update()}.
     *
     * @param now monotonic seconds
     */
    public static synchronized void update(double now) {
        try {
            if (!loaded) {
                load();
                loaded = true;
                lastSample = now;
            }
            adoptDiscovered(now);
            if (!scanning && now - lastScan >= config.rescanSeconds) {
                lastScan = now;
                scanning = true;
                Thread t = new Thread(MotorHistory::scan, "Catalyst motor history discovery");
                t.setDaemon(true);
                t.start();
            }
            if (now - lastSample >= config.samplePeriodSeconds) {
                double dt = Math.min(now - lastSample, 1.0);   // a stalled loop is not a minute of use
                lastSample = now;
                sampleAll(dt);
            }
            boolean enabled = RobotState.isEnabled();
            boolean justDisabled = wasEnabled && !enabled;
            wasEnabled = enabled;
            if (dirty && (justDisabled || now - lastFlush >= config.flushSeconds)) {
                lastFlush = now;
                flushAsync();
            }
            if (now - lastPublish >= 2.0) {
                lastPublish = now;
                publish();
            }
        } catch (Throwable t) {
            // A history is nice to have; a robot loop is not the place for its problems.
            if (!warnedOnce) {
                warnedOnce = true;
                System.err.println("[Catalyst] MotorHistory: " + t);
            }
        }
    }

    /** Every device known, motors first, in the order first seen. */
    public static synchronized Collection<Record> records() {
        return new ArrayList<>(RECORDS.values());
    }

    /** The whole store as the file would hold it. */
    public static synchronized String toJson() {
        return MiniJson.pretty(document());
    }

    /** Write now, on this thread. For a shutdown hook, or a test. */
    public static synchronized void flushNow() {
        writeFile(toJson());
        dirty = false;
    }

    /** Where the file is. */
    public static File file() {
        File f = new File(config.path);
        return f.isAbsolute() ? f : new File(System.getProperty("user.dir"), config.path);
    }

    // ------------------------------------------------------------------ discovery

    private static void scan() {
        try {
            HttpRequest req = HttpRequest.newBuilder(URI.create(config.diagnosticServer + "?action=getdevices"))
                    .timeout(Duration.ofSeconds(4)).GET().build();
            String body = HTTP.send(req, HttpResponse.BodyHandlers.ofString()).body();
            DISCOVERED.set(parseDevices(body));
            lastFailure = "";
        } catch (Exception e) {
            lastFailure = e.getClass().getSimpleName();
        } finally {
            scanning = false;
        }
    }

    /** The diagnostic server's device list, as {@link DeviceInfo}s. Package-private for tests. */
    @SuppressWarnings("unchecked")
    static List<DeviceInfo> parseDevices(String json) {
        List<DeviceInfo> out = new ArrayList<>();
        Map<String, Object> doc = MiniJson.readObject(json);
        if (!(doc.get("DeviceArray") instanceof List<?> arr)) {
            return out;
        }
        for (Object o : arr) {
            if (!(o instanceof Map<?, ?> d)) {
                continue;
            }
            Map<String, Object> m = (Map<String, Object>) d;
            out.add(new DeviceInfo(str(m, "Model"), (int) num(m, "ID"), str(m, "Name"), str(m, "SerialNo"),
                    str(m, "CANbus"), str(m, "CurrentVers"), str(m, "HardwareRev"), str(m, "ManDate")));
        }
        return out;
    }

    private static void adoptDiscovered(double now) {
        List<DeviceInfo> found = DISCOVERED.getAndSet(null);
        if (found == null) {
            return;
        }
        long nowMs = System.currentTimeMillis();
        for (DeviceInfo d : found) {
            observe(d, nowMs);
        }
    }

    /** Record a device as present with these particulars. Package-private for tests. */
    static synchronized Record observe(DeviceInfo d, long nowMs) {
        Record r = RECORDS.computeIfAbsent(d.key(), k -> {
            Record n = new Record();
            n.serial = d.serial();
            n.model = d.model();
            n.kind = d.kind();
            n.hardwareRev = d.hardwareRev();
            n.manufactured = d.manufactured();
            return n;
        });
        if (r.model.isEmpty()) {
            r.model = d.model();
        }
        r.kind = d.kind();
        r.observe(d.id(), d.name(), d.bus(), d.firmware(), nowMs, config.maxSessions);
        dirty = true;
        if (r.kind.equals("motor") && !TRACKED.containsKey(d.key()) && onRobot()) {
            try {
                TalonFX motor = new TalonFX(d.id(), CatalystCANBus.of(d.bus()).phoenix());
                TRACKED.put(d.key(), new Tracked(r, motor));
            } catch (RuntimeException e) {
                lastFailure = "TalonFX " + d.id() + ": " + e.getClass().getSimpleName();
            }
        }
        return r;
    }

    private static boolean onRobot() {
        // Phoenix objects want a HAL. Tests run the accounting without one.
        return !Boolean.getBoolean("catalyst.motorhistory.noPhoenix");
    }

    // ------------------------------------------------------------------ sampling

    private static void sampleAll(double dt) {
        if (TRACKED.isEmpty()) {
            return;
        }
        List<BaseStatusSignal> all = new ArrayList<>(TRACKED.size() * 6);
        for (Tracked t : TRACKED.values()) {
            all.add(t.velocity);
            all.add(t.stator);
            all.add(t.supply);
            all.add(t.volts);
            all.add(t.temp);
            all.add(t.sticky);
        }
        BaseStatusSignal.refreshAll(all);
        for (Tracked t : TRACKED.values()) {
            if (!t.motor.isConnected()) {
                continue;
            }
            t.record.sample(t.sample(), dt, config);
        }
        dirty = true;
    }

    /** Account a sample for a record directly. Package-private for tests. */
    static synchronized void sampleFor(String key, Sample s, double dt) {
        Record r = RECORDS.get(key);
        if (r != null) {
            r.sample(s, dt, config);
            dirty = true;
        }
    }

    // ------------------------------------------------------------------ the file

    private static Map<String, Object> document() {
        Map<String, Object> doc = new LinkedHashMap<>();
        doc.put("format", "catalyst-motor-history");
        doc.put("version", 1);
        doc.put("updatedMs", System.currentTimeMillis());
        doc.put("clockTrusted", clockTrusted());
        List<Object> devices = new ArrayList<>();
        for (Record r : RECORDS.values()) {
            devices.add(r.toJson());
        }
        doc.put("devices", devices);
        return doc;
    }

    /** A Systemcore with no time source boots in 1970; dates before this are relative, not real. */
    private static boolean clockTrusted() {
        return System.currentTimeMillis() > 1_700_000_000_000L;
    }

    private static void load() {
        File f = file();
        if (!f.exists()) {
            return;
        }
        try {
            String text = Files.readString(f.toPath(), StandardCharsets.UTF_8);
            loadJson(text);
        } catch (IOException | RuntimeException e) {
            System.err.println("[Catalyst] MotorHistory: could not read " + f + " (" + e + "); starting fresh, the old file is kept as .bad");
            try {
                Files.move(f.toPath(), new File(f.getPath() + ".bad").toPath(), StandardCopyOption.REPLACE_EXISTING);
            } catch (IOException ignored) {
                // Then it stays; the next write replaces it.
            }
        }
    }

    /** Load a document's records, replacing what is held. Package-private for tests. */
    @SuppressWarnings("unchecked")
    static synchronized void loadJson(String text) {
        Map<String, Object> doc = MiniJson.readObject(text);
        RECORDS.clear();
        if (doc.get("devices") instanceof List<?> l) {
            for (Object o : l) {
                if (o instanceof Map<?, ?> m) {
                    Record r = Record.fromJson((Map<String, Object>) m);
                    String key = r.serial.isEmpty() && r.latest() != null
                            ? r.latest().bus + ":" + r.model + ":" + r.latest().id : r.serial;
                    RECORDS.put(key, r);
                }
            }
        }
    }

    private static void flushAsync() {
        String text = toJson();
        dirty = false;
        Thread t = new Thread(() -> writeFile(text), "Catalyst motor history write");
        t.setDaemon(true);
        t.start();
    }

    private static void writeFile(String text) {
        File f = file();
        try {
            File dir = f.getParentFile();
            if (dir != null) {
                Files.createDirectories(dir.toPath());
            }
            File tmp = new File(f.getPath() + ".tmp");
            Files.writeString(tmp.toPath(), text, StandardCharsets.UTF_8);
            Files.move(tmp.toPath(), f.toPath(), StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
        } catch (IOException e) {
            try {
                // Not every filesystem does atomic moves; the plain one is still better than nothing.
                Files.writeString(f.toPath(), text, StandardCharsets.UTF_8);
            } catch (IOException e2) {
                lastFailure = "write: " + e2.getMessage();
            }
        }
    }

    // ------------------------------------------------------------------ dashboard

    private static void publish() {
        List<String> rows = new ArrayList<>();
        int motors = 0;
        double hours = 0;
        Record hottest = null;
        for (Record r : RECORDS.values()) {
            Identity i = r.latest();
            rows.add(String.join("|",
                    r.serial, r.model, r.kind,
                    i == null ? "" : i.bus, i == null ? "" : Integer.toString(i.id), i == null ? "" : i.name,
                    i == null ? "" : i.firmware,
                    fmt(r.poweredSeconds), fmt(r.runningSeconds), fmt(r.loadedSeconds), fmt(r.revolutions),
                    fmt(r.peakStatorAmps), fmt(r.peakTempC), fmt(r.hotSeconds), fmt(r.energyJoules),
                    Integer.toString(r.boots), Long.toString(r.firstSeenMs), Long.toString(r.lastSeenMs),
                    Integer.toString(r.identities.size()), Long.toString(r.stickyFaults)));
            if (r.kind.equals("motor")) {
                motors++;
                hours += r.poweredSeconds / 3600.0;
                if (hottest == null || r.peakTempC > hottest.peakTempC) {
                    hottest = r;
                }
            }
        }
        CatalystLog.log("MotorHistory/Rows", rows.toArray(new String[0]));
        CatalystLog.log("MotorHistory/Count", (long) motors);
        CatalystLog.log("MotorHistory/Devices", (long) RECORDS.size());
        CatalystLog.log("MotorHistory/File", file().getPath());
        CatalystLog.log("MotorHistory/UpdatedMs", System.currentTimeMillis());
        CatalystLog.log("MotorHistory/ClockTrusted", clockTrusted());
        CatalystLog.log("MotorHistory/Discovery", lastFailure.isEmpty() ? "ok" : lastFailure);
        String summary;
        if (motors == 0) {
            summary = RECORDS.isEmpty() ? "no devices seen yet" : RECORDS.size() + " devices, no motors";
        } else {
            summary = String.format("%d motors, %.1f h powered in total", motors, hours);
            if (hottest != null && hottest.peakTempC > 0) {
                Identity hi = hottest.latest();
                summary += String.format(", hottest ever %.0f C (%s)", hottest.peakTempC,
                        hi == null || hi.name.isEmpty() ? hottest.serial : hi.name);
            }
        }
        CatalystLog.log("MotorHistory/Summary", summary);
    }

    // ------------------------------------------------------------------ helpers

    private static String fmt(double v) {
        return Double.toString(round(v));
    }

    private static double round(double v) {
        return Math.round(v * 100.0) / 100.0;
    }

    private static String str(Map<String, Object> m, String k) {
        Object v = m.get(k);
        return v == null ? "" : String.valueOf(v);
    }

    private static double num(Map<String, Object> m, String k) {
        Object v = m.get(k);
        return v instanceof Number n ? n.doubleValue() : 0.0;
    }

    /** Forget everything held in memory (not the file). For tests. */
    static synchronized void clearForTest() {
        RECORDS.clear();
        TRACKED.clear();
        DISCOVERED.set(null);
        loaded = false;
        dirty = false;
        lastSample = Double.NaN;
        lastScan = Double.NEGATIVE_INFINITY;
        lastFlush = Double.NEGATIVE_INFINITY;
        lastPublish = Double.NEGATIVE_INFINITY;
        config = new Config();
    }
}
