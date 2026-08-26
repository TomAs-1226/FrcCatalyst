package frc.lib.catalyst.system;

import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.SystemServer;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

/**
 * What Systemcore knows about itself.
 *
 * <p>Systemcore runs its own NetworkTables server, separate from the robot program's, and publishes
 * live system state onto it: battery voltage, brownout state, CPU load, memory and storage use, and
 * the configured team number. WPILib 2027 hands you the instance through
 * {@link SystemServer#getSystemServer()}. This class is the Catalyst-side reader for it.
 *
 * <p><b>Why this is worth having.</b> Catalyst already measures a lot about the robot, but it has
 * always had to infer things about the machine underneath it — and on a roboRIO there was mostly
 * nothing to read. Systemcore measures its own CPU, RAM, storage and power and publishes them, which
 * turns a class of mystery failure into a number on a dashboard. A robot that browns out because a
 * log filled the disk, or that misses loop deadlines because something else is pinning a core, now
 * says so.
 *
 * <p><b>Topic names are verified, not assumed.</b> The list below was read out of the
 * {@code MrcCommDaemon} binary in the Systemcore OS image rather than taken from documentation:
 * {@code /sys/battery}, {@code /sys/brownout}, {@code /sys/cpu}, {@code /sys/ram},
 * {@code /sys/ramtotal}, {@code /sys/storage}, {@code /sys/storagetotal}, {@code /sys/team}.
 * Brownout thresholds ({@code vbrownout}, {@code vrecovery}) are published by the IO daemon; their
 * exact table is not certain from the image alone, so they are probed rather than relied upon and
 * every reader here returns an empty Optional when a topic is absent.
 *
 * <p>Everything degrades quietly off-hardware. In simulation, or on any machine without a system
 * server, {@link #isAvailable()} is false and every reading is empty — no exceptions, no logs, no
 * behaviour change. Code can call this unconditionally.
 *
 * <p><b>Not unit-tested, and it cannot be.</b> Resolving the system server forces a HAL JNI load,
 * and WPILib's {@code RuntimeLoader} terminates the JVM when the natives are missing rather than
 * throwing. A test that constructs this class kills the Gradle test worker and takes every other
 * result with it, so no amount of {@code catch (Throwable)} here can make it testable on a desktop
 * JVM — the process is gone before the catch runs. The graceful-degradation paths below are written
 * defensively for the same reason, and want verifying on real hardware.
 *
 * @since 2.0.0
 */
public final class SystemCoreStatus {

    /** Table the Systemcore OS publishes its own state under. */
    private static final String SYS_TABLE = "sys";

    private static SystemCoreStatus instance;

    private final SystemCoreSource source;

    private SystemCoreStatus(SystemCoreSource source) {
        this.source = source;
    }

    /**
     * Replace where readings come from — a {@link SystemCoreSim} in tests and simulation.
     *
     * <p>Passing null clears the override so the next {@link #getInstance()} resolves the real
     * system server. It does <em>not</em> resolve it here: that would construct an {@code NtSource}
     * immediately, and doing so in a JVM without the HAL natives terminates the process. Clearing
     * has to stay lazy so a caller can undo an override without triggering the very thing the
     * override existed to avoid.
     */
    public static synchronized void useSource(SystemCoreSource source) {
        instance = source == null ? null : new SystemCoreStatus(source);
    }

    /** Reads the live system NetworkTables server. Constructed only on real hardware. */
    private static final class NtSource implements SystemCoreSource {
        private final NetworkTable sys;
        private final boolean available;
        private final NetworkTableInstance server;

        NtSource() {
            NetworkTableInstance resolved = null;
            try {
                resolved = SystemServer.getSystemServer();
            } catch (Throwable ignored) {
                // No system server: simulation, a unit test, or a desktop build. Not an error.
            }
            this.server = resolved;
            this.sys = resolved == null ? null : resolved.getTable(SYS_TABLE);
            this.available = resolved != null;
        }

        NetworkTableInstance server() {
            return server;
        }

        @Override public boolean isAvailable() { return available; }

        @Override
        public OptionalDouble number(String key) {
            if (!available) {
                return OptionalDouble.empty();
            }
            try {
                var entry = entry(key);
                return entry.exists() ? OptionalDouble.of(entry.getDouble(Double.NaN))
                                      : OptionalDouble.empty();
            } catch (Throwable ignored) {
                return OptionalDouble.empty();
            }
        }

        @Override
        public boolean bool(String key) {
            if (!available) {
                return false;
            }
            try {
                return entry(key).getBoolean(false);
            } catch (Throwable ignored) {
                return false;
            }
        }

        @Override
        public Optional<double[]> numberArray(String key) {
            if (!available) {
                return Optional.empty();
            }
            try {
                var entry = entry(key);
                // An absent topic and a topic holding an empty array are different facts: "no
                // reading" against "every bus at zero". Only the first is empty here.
                return entry.exists() ? Optional.of(entry.getDoubleArray(new double[0]))
                                      : Optional.empty();
            } catch (Throwable ignored) {
                return Optional.empty();
            }
        }

        @Override
        public Optional<String[]> stringArray(String key) {
            if (!available) {
                return Optional.empty();
            }
            try {
                var entry = entry(key);
                return entry.exists() ? Optional.of(entry.getStringArray(new String[0]))
                                      : Optional.empty();
            } catch (Throwable ignored) {
                return Optional.empty();
            }
        }

        /**
         * Resolve a key against the right table.
         *
         * <p>Most readings live on {@code /sys}. The CAN diagnostics do not - they are under
         * {@code /diagnostics} - so an absolute key goes to the server root instead.
         */
        private NetworkTableEntry entry(String key) {
            return key.startsWith("/") ? server.getEntry(key) : sys.getEntry(key);
        }
    }

    /** The shared reader. Cheap to call; the underlying source is resolved once. */
    public static synchronized SystemCoreStatus getInstance() {
        if (instance == null) {
            instance = new SystemCoreStatus(new NtSource());
        }
        return instance;
    }

    /**
     * Whether a Systemcore system server was found.
     *
     * <p>False in simulation and on desktop. Every reading below is empty when this is false, so
     * callers do not need to branch on it except to decide whether to show the data at all.
     */
    public boolean isAvailable() {
        return source.isAvailable();
    }

    /**
     * The raw system NetworkTables instance, for anything this class does not wrap.
     *
     * @return the instance, or null when reading a simulated source or off hardware
     */
    public NetworkTableInstance server() {
        return source instanceof NtSource nt ? nt.server() : null;
    }

    // --- Why there is no HAL fallback here -----------------------------------
    //
    // Tried and reverted, deliberately. WPILib exposes battery voltage, brownout state and CPU
    // temperature through RobotController as well, and it is tempting to use those when Systemcore
    // publishes nothing - the topic names here were read out of a binary rather than a
    // specification, so a renamed one silently empties a reading for a whole match.
    //
    // It breaks the invariant this class is built on, and five existing tests say so. An absent
    // Systemcore currently reports empty, DriverBoard shows a dash, and Preflight says it cannot
    // see the machine. With a fallback it reports 12.00 V - the simulator's default, presented as a
    // measurement - and every one of those surfaces starts showing a healthy battery for a robot
    // that is not answering. That is precisely the plausible-wrong-number failure the rest of this
    // file goes to some length to avoid, and it is worse than a dash because nobody investigates a
    // dashboard that looks fine.
    //
    // If this is ever wanted, it has to arrive as a separate reading that says where it came from,
    // not as a silent substitution behind the same accessor.

    // --- Power ---------------------------------------------------------------

    /** Battery voltage as Systemcore measures it, in volts. */
    public OptionalDouble batteryVolts() {
        return number("battery");
    }

    /** Whether Systemcore currently considers itself browned out. */
    public boolean isBrownedOut() {
        return bool("brownout");
    }

    /**
     * Voltage at which Systemcore browns out, in volts.
     *
     * <p>Published in millivolts and converted here. The OS default is 6750 mV. Empty when the
     * topic is absent, which is the honest answer rather than substituting a roboRIO-era constant.
     */
    public OptionalDouble brownoutVolts() {
        return millivoltsAsVolts("vbrownout");
    }

    /** Voltage at which Systemcore considers itself recovered, in volts. OS default 7500 mV. */
    public OptionalDouble recoveryVolts() {
        return millivoltsAsVolts("vrecovery");
    }

    // --- Compute -------------------------------------------------------------

    /** CPU utilisation, as Systemcore reports it. */
    public OptionalDouble cpuUtilization() {
        return number("cpu");
    }

    /** RAM in use, in the units Systemcore publishes (bytes). */
    public OptionalDouble ramUsed() {
        return number("ram");
    }

    /** Total RAM. */
    public OptionalDouble ramTotal() {
        return number("ramtotal");
    }

    /** Storage in use. */
    public OptionalDouble storageUsed() {
        return number("storage");
    }

    /** Total storage. */
    public OptionalDouble storageTotal() {
        return number("storagetotal");
    }

    /**
     * Fraction of RAM in use, 0–1.
     *
     * <p>Empty unless both halves are present, since a ratio built from one of them would be a
     * guess dressed up as a measurement.
     */
    public OptionalDouble ramFraction() {
        return fraction(ramUsed(), ramTotal());
    }

    /** Fraction of storage in use, 0–1. The one that fills up quietly and browns nothing out. */
    public OptionalDouble storageFraction() {
        return fraction(storageUsed(), storageTotal());
    }

    // --- Thermal and power ---------------------------------------------------

    /**
     * SoC temperature in degrees Celsius.
     *
     * <p>Read off {@code thermal_zone0}, which is the CM5's own sensor. Worth watching because the
     * failure it causes does not look thermal: Linux throttles the cores rather than reporting
     * anything, so a Systemcore in a sealed electronics box slows down and the symptom is a loop
     * overrun.
     */
    public OptionalDouble cpuTemperatureCelsius() {
        return number("temp");
    }

    /**
     * Current drawn from the 3.3 V rail, in amps.
     *
     * <p>That rail powers the IO pins. A climb here usually means something is being asked for more
     * current than the pins can supply - the reason servos cannot be driven from Systemcore at all.
     */
    public OptionalDouble rail3v3Amps() {
        return number("current3v3");
    }

    // --- Storage health ------------------------------------------------------

    /**
     * How much of the eMMC's rated write life is used, 0-1.
     *
     * <p>The flash wears out, and unlike a full disk it does not recover when you delete something.
     * A robot that logs every match for four seasons is doing exactly what wears it, and the first
     * symptom is usually a filesystem that has gone read-only mid-match.
     *
     * <p>Reported by the device as a JEDEC {@code DEVICE_LIFE_TIME_EST} code in 10% steps: 1 means
     * 0-10% used, 10 means 90-100%, 11 means past its rated life. Systemcore exposes two, for the
     * SLC and MLC regions; the worse of the two is what matters, so that is what this returns. The
     * midpoint of the band is used rather than an edge, because the code carries no more precision
     * than that and pretending otherwise would be inventing a digit.
     *
     * @return fraction of rated life used, or empty if the device does not report it
     */
    public OptionalDouble emmcLifeUsedFraction() {
        OptionalDouble a = number("emmc/lifetime_a");
        OptionalDouble b = number("emmc/lifetime_b");
        double worst = Math.max(a.orElse(0), b.orElse(0));
        if (worst < 1) {
            // 0 is JEDEC's "not defined", not "brand new".
            return OptionalDouble.empty();
        }
        if (worst >= 11) {
            return OptionalDouble.of(1.0);
        }
        return OptionalDouble.of((worst - 0.5) / 10.0);
    }

    /**
     * The eMMC's pre-EOL state, which is the device's own opinion of whether it is wearing out.
     *
     * <p>JEDEC {@code PRE_EOL_INFO}: 1 normal, 2 warning (80% of the reserved blocks are gone),
     * 3 urgent. Independent of {@link #emmcLifeUsedFraction()} and often the earlier signal, since
     * it reflects blocks actually retired rather than writes estimated.
     *
     * @return the raw code, or empty if not reported
     */
    public OptionalInt emmcPreEol() {
        OptionalDouble raw = number("emmc/pre_eol");
        return raw.isPresent() ? OptionalInt.of((int) raw.getAsDouble()) : OptionalInt.empty();
    }

    /** Whether the eMMC is reporting anything that wants attention before the next event. */
    public boolean emmcNeedsAttention() {
        return emmcPreEol().orElse(1) >= 2 || emmcLifeUsedFraction().orElse(0) >= 0.8;
    }

    // --- CAN -----------------------------------------------------------------

    /**
     * Bus utilisation per CAN bus, 0-1, indexed to match {@code can_s0} through {@code can_s4}.
     *
     * <p>Measured by the OS rather than estimated from a device list, which is the difference
     * between knowing and guessing. {@code CANBusPlanner} predicts this from a plan; comparing the
     * two is how a plan gets corrected.
     *
     * @return one entry per bus, or empty when there is no reading at all
     */
    public Optional<double[]> canBusUtilization() {
        return source.numberArray("/diagnostics/canbusutil");
    }

    /** Utilisation of one bus, 0-1, by index. Empty if that bus is not in the reading. */
    public OptionalDouble canBusUtilization(int busIndex) {
        return canBusUtilization()
                .filter(u -> busIndex >= 0 && busIndex < u.length)
                .map(u -> OptionalDouble.of(u[busIndex]))
                .orElse(OptionalDouble.empty());
    }

    /** How many times a CAN bus has gone down since boot. Counts, not a current state. */
    public OptionalDouble canBusDownCount() {
        return number("faultcounts/canbus_down");
    }

    /** How many times a CAN bus has been unavailable since boot. */
    public OptionalDouble canBusUnavailableCount() {
        return number("faultcounts/canbus_unavail");
    }

    /** Whether a CAN bus is down right now. */
    public boolean canBusDown() {
        return source.bool("faults/canbus_down");
    }

    // --- Identity ------------------------------------------------------------

    /**
     * The network interfaces the machine reports, as it words them.
     *
     * <p>Useful mostly for one question a pit crew asks constantly: is it on the radio, or only on
     * USB? Passed through unparsed - the format is the OS's, and reformatting it would mean
     * guessing at a shape that has no documentation.
     */
    public Optional<String[]> networkInterfaces() {
        return source.stringArray("networkInterfaces");
    }

    /** Hardware sub-revision, as the device reports it. */
    public OptionalDouble hardwareSubRevision() {
        return number("hsub");
    }

    /** Team number configured on the Systemcore itself, which is not always the one in your code. */
    public OptionalInt teamNumber() {
        OptionalDouble raw = number("team");
        return raw.isPresent() ? OptionalInt.of((int) raw.getAsDouble()) : OptionalInt.empty();
    }

    // --- Publishing ----------------------------------------------------------

    /**
     * How often the slow-moving readings are republished, in calls to {@link #publish()}.
     *
     * <p>Fifty is one second at a 20 ms loop. Nothing here needs to be fresher than that: a disk
     * does not fill in under a second, flash wear is measured in months, and the team number is
     * fixed before the robot is switched on.
     */
    private static final int SLOW_PUBLISH_EVERY = 50;

    private int publishCount = 0;

    /**
     * Copy the current readings onto Catalyst's own log, under {@code Systemcore/}.
     *
     * <p>The system server is a second NetworkTables instance, which most dashboards will not be
     * connected to and which nothing downstream of Catalyst knows to look at. Mirroring the values
     * through {@link CatalystLog} puts them in the same place as everything else Catalyst records,
     * so Catalyst Console, the Health Dashboard, AdvantageScope and a {@code .wpilog} replay all get
     * them for free, and a post-match investigation has machine state sitting next to robot state.
     *
     * <p>Call once per loop. No-op when there is no system server.
     *
     * <h2>Two rates, on purpose</h2>
     *
     * <p>There are twenty-two readings here and only about a third of them move loop to loop.
     * Publishing all of them at 50 Hz would put roughly 1100 NetworkTables writes per second on the
     * wire to say, over and over, that the team number is still 5805 and the eMMC is still 15% worn.
     * That bandwidth is shared with everything else the robot reports, and it is the driver's
     * station link that pays for it.
     *
     * <p>So the ones that actually change every loop go out every loop, and the rest go out about
     * once a second. Both are published on the first call, so a dashboard that connects mid-match
     * fills in immediately rather than waiting for the slow tick.
     */
    public void publish() {
        if (!isAvailable()) {
            return;
        }

        // --- every loop: the readings that genuinely move -------------------
        batteryVolts().ifPresent(v -> CatalystLog.log("Systemcore/BatteryVolts", v));
        CatalystLog.log("Systemcore/BrownedOut", isBrownedOut());
        cpuUtilization().ifPresent(v -> CatalystLog.log("Systemcore/CpuPercent", v));
        cpuTemperatureCelsius().ifPresent(v -> CatalystLog.log("Systemcore/TempCelsius", v));
        ramFraction().ifPresent(v -> CatalystLog.log("Systemcore/RamFraction", v));
        rail3v3Amps().ifPresent(v -> CatalystLog.log("Systemcore/Rail3v3Amps", v));
        canBusUtilization().ifPresent(u -> CatalystLog.log("Systemcore/CanUtilization", u));
        CatalystLog.log("Systemcore/CanDown", canBusDown());

        boolean slow = publishCount % SLOW_PUBLISH_EVERY == 0;
        publishCount++;
        if (!slow) {
            return;
        }

        // --- about once a second: everything else ---------------------------
        // A disk does not fill in under a second, flash wear is measured in months, and the team
        // number was fixed before the robot was switched on.
        brownoutVolts().ifPresent(v -> CatalystLog.log("Systemcore/BrownoutVolts", v));
        recoveryVolts().ifPresent(v -> CatalystLog.log("Systemcore/RecoveryVolts", v));
        storageFraction().ifPresent(v -> CatalystLog.log("Systemcore/StorageFraction", v));

        // Absolute byte counts as well as the ratios: "88% of 8 GB" and "88% of 512 MB" are the
        // same number and different problems, and a dashboard cannot tell them apart from a ratio.
        ramUsed().ifPresent(v -> CatalystLog.log("Systemcore/RamUsedBytes", v));
        ramTotal().ifPresent(v -> CatalystLog.log("Systemcore/RamTotalBytes", v));
        storageUsed().ifPresent(v -> CatalystLog.log("Systemcore/StorageUsedBytes", v));
        storageTotal().ifPresent(v -> CatalystLog.log("Systemcore/StorageTotalBytes", v));

        emmcLifeUsedFraction().ifPresent(v -> CatalystLog.log("Systemcore/EmmcLifeUsed", v));
        var preEol = emmcPreEol();
        if (preEol.isPresent()) {
            CatalystLog.log("Systemcore/EmmcPreEol", preEol.getAsInt());
        }

        // Counts since boot rather than a live state, so a slow tick loses nothing.
        canBusDownCount().ifPresent(v -> CatalystLog.log("Systemcore/CanDownCount", v));
        canBusUnavailableCount().ifPresent(v -> CatalystLog.log("Systemcore/CanUnavailCount", v));

        var team = teamNumber();
        if (team.isPresent()) {
            CatalystLog.log("Systemcore/TeamNumber", team.getAsInt());
        }
        networkInterfaces().ifPresent(n -> CatalystLog.log("Systemcore/NetworkInterfaces", n));
        hardwareSubRevision().ifPresent(v -> CatalystLog.log("Systemcore/HardwareSubRev", v));
    }

    // --- Internals -----------------------------------------------------------

    private OptionalDouble number(String key) {
        return source.number(key);
    }

    private boolean bool(String key) {
        return source.bool(key);
    }

    private OptionalDouble millivoltsAsVolts(String key) {
        OptionalDouble mv = number(key);
        return mv.isPresent() ? OptionalDouble.of(mv.getAsDouble() / 1000.0) : OptionalDouble.empty();
    }

    private static OptionalDouble fraction(OptionalDouble used, OptionalDouble total) {
        if (used.isEmpty() || total.isEmpty() || total.getAsDouble() <= 0) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(used.getAsDouble() / total.getAsDouble());
    }

    /** Test hook: drop the cached instance so a fresh one is resolved. */
    static synchronized void reset() {
        instance = null;
    }
}
