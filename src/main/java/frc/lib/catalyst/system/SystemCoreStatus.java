package frc.lib.catalyst.system;

import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.SystemServer;

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
                var entry = sys.getEntry(key);
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
                return sys.getEntry(key).getBoolean(false);
            } catch (Throwable ignored) {
                return false;
            }
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

    // --- Identity ------------------------------------------------------------

    /** Team number configured on the Systemcore itself, which is not always the one in your code. */
    public OptionalInt teamNumber() {
        OptionalDouble raw = number("team");
        return raw.isPresent() ? OptionalInt.of((int) raw.getAsDouble()) : OptionalInt.empty();
    }

    // --- Publishing ----------------------------------------------------------

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
     */
    public void publish() {
        if (!isAvailable()) {
            return;
        }
        batteryVolts().ifPresent(v -> CatalystLog.log("Systemcore/BatteryVolts", v));
        CatalystLog.log("Systemcore/BrownedOut", isBrownedOut());
        brownoutVolts().ifPresent(v -> CatalystLog.log("Systemcore/BrownoutVolts", v));
        recoveryVolts().ifPresent(v -> CatalystLog.log("Systemcore/RecoveryVolts", v));
        cpuUtilization().ifPresent(v -> CatalystLog.log("Systemcore/CpuPercent", v));
        ramFraction().ifPresent(v -> CatalystLog.log("Systemcore/RamFraction", v));
        storageFraction().ifPresent(v -> CatalystLog.log("Systemcore/StorageFraction", v));
        var team = teamNumber();
        if (team.isPresent()) {
            CatalystLog.log("Systemcore/TeamNumber", team.getAsInt());
        }
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
