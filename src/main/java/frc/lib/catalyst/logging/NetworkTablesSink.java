package frc.lib.catalyst.logging;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.HashMap;
import java.util.Map;

/**
 * Default {@link LogSink} that publishes everything to NetworkTables under a
 * configurable root table (default {@code "Catalyst"}).
 *
 * <p>Entries are cached per key so each call reuses the same
 * {@link NetworkTableEntry} rather than re-resolving the path on every loop.
 *
 * <p>Most teams never need to touch this class — it is installed automatically
 * by {@link CatalystLog}. Teams running AdvantageScope or Shuffleboard get all
 * Catalyst telemetry for free under {@code /Catalyst/...}.
 */
public final class NetworkTablesSink implements LogSink {

    private final NetworkTable root;
    private final Map<String, NetworkTableEntry> entryCache = new HashMap<>();

    /** Build a sink that publishes under {@code "Catalyst"} on the default NT instance. */
    public NetworkTablesSink() {
        this("Catalyst");
    }

    /** Build a sink that publishes under {@code rootTableName} on the default NT instance. */
    public NetworkTablesSink(String rootTableName) {
        this.root = NetworkTableInstance.getDefault().getTable(rootTableName);
    }

    private NetworkTableEntry entry(String key) {
        // Cache lookups — getEntry() on a hot path is fine but caching avoids
        // string parsing each loop and is consistent with how AK's own NT
        // publisher behaves.
        return entryCache.computeIfAbsent(key, root::getEntry);
    }

    @Override public void log(String key, double value)     { entry(key).setDouble(value); }
    @Override public void log(String key, boolean value)    { entry(key).setBoolean(value); }
    @Override public void log(String key, long value)       { entry(key).setInteger(value); }
    @Override public void log(String key, String value)     { entry(key).setString(value); }
    @Override public void log(String key, double[] value)   { entry(key).setDoubleArray(value); }
    @Override public void log(String key, boolean[] value)  { entry(key).setBooleanArray(value); }
    @Override public void log(String key, long[] value)     { entry(key).setIntegerArray(value); }
    @Override public void log(String key, String[] value)   { entry(key).setStringArray(value); }
}
