package frc.lib.catalyst.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * A numeric constant that can be edited live from the dashboard via NetworkTables.
 *
 * <p>Useful for tuning PID gains, feedforward values, speed limits, and
 * any other constant you want to adjust without redeploying code.
 * Values are cached so repeated reads in the periodic loop are free.
 *
 * <p>In competition builds, call {@link #disableTuning()} once at startup
 * to lock all values and skip NetworkTables reads entirely.
 *
 * <p>Example:
 * <pre>{@code
 * // Define tunable constants (typically as class fields)
 * TunableNumber kP = new TunableNumber("Elevator/kP", 50.0);
 * TunableNumber kD = new TunableNumber("Elevator/kD", 0.5);
 * TunableNumber maxSpeed = new TunableNumber("Elevator/MaxSpeed", 2.0);
 *
 * // In periodic — reads are cached, nearly zero cost
 * if (kP.hasChanged() || kD.hasChanged()) {
 *     pid.setP(kP.get());
 *     pid.setD(kD.get());
 * }
 * }</pre>
 */
public class TunableNumber {

    private static final String TABLE_NAME = "Catalyst/Tuning";
    private static boolean tuningEnabled = true;

    private final NetworkTableEntry entry;
    private final double defaultValue;
    private double cachedValue;
    private double lastPolledValue;

    /**
     * Create a tunable number.
     *
     * @param key unique key (use "Subsystem/Name" format)
     * @param defaultValue the default value
     */
    public TunableNumber(String key, double defaultValue) {
        this.defaultValue = defaultValue;
        this.cachedValue = defaultValue;
        this.lastPolledValue = defaultValue;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
        this.entry = table.getEntry(key);
        entry.setDouble(defaultValue);
    }

    /**
     * Get the current value. Reads from NetworkTables on each call
     * (when tuning is enabled), otherwise returns the cached default.
     */
    public double get() {
        if (tuningEnabled) {
            cachedValue = entry.getDouble(defaultValue);
        }
        return cachedValue;
    }

    /**
     * Check if the value has changed since the last call to {@code hasChanged()}.
     * Useful for only reconfiguring PID when gains actually change.
     */
    public boolean hasChanged() {
        double current = get();
        if (current != lastPolledValue) {
            lastPolledValue = current;
            return true;
        }
        return false;
    }

    /** Get the default value this was created with. */
    public double getDefault() {
        return defaultValue;
    }

    /** Set the value programmatically (also updates the dashboard). */
    public void set(double value) {
        cachedValue = value;
        lastPolledValue = value;
        entry.setDouble(value);
    }

    /**
     * Disable tuning globally. Call once at robot startup for competition.
     * All TunableNumbers will return their cached value without reading NT.
     */
    public static void disableTuning() {
        tuningEnabled = false;
    }

    /** Re-enable tuning (for practice/testing). */
    public static void enableTuning() {
        tuningEnabled = true;
    }

    /** Check if tuning is currently enabled. */
    public static boolean isTuningEnabled() {
        return tuningEnabled;
    }
}
