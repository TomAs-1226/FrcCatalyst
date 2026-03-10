package frc.lib.catalyst.util;

import java.util.Map;
import java.util.TreeMap;

/**
 * A lookup table that linearly interpolates between entries.
 * Perfect for shooter RPM lookup tables, hood angle tables,
 * or any relationship where you have discrete measured data points.
 *
 * <p>Example usage:
 * <pre>{@code
 * // Distance (meters) -> Shooter RPM
 * InterpolatingTable shooterTable = new InterpolatingTable()
 *     .add(1.0, 2000)
 *     .add(2.0, 2800)
 *     .add(3.0, 3500)
 *     .add(4.0, 4000)
 *     .add(5.0, 4200);
 *
 * double rpm = shooterTable.get(2.5); // returns 3150.0 (interpolated)
 * }</pre>
 */
public class InterpolatingTable {

    private final TreeMap<Double, Double> table = new TreeMap<>();

    /** Add a data point to the table. Returns this for chaining. */
    public InterpolatingTable add(double key, double value) {
        table.put(key, value);
        return this;
    }

    /**
     * Get the interpolated value for a given key.
     * If the key is below the lowest entry, returns the lowest value.
     * If the key is above the highest entry, returns the highest value.
     * Otherwise, linearly interpolates between the two nearest entries.
     */
    public double get(double key) {
        if (table.isEmpty()) return 0;

        Double exact = table.get(key);
        if (exact != null) return exact;

        Double lowerKey = table.floorKey(key);
        Double upperKey = table.ceilingKey(key);

        if (lowerKey == null) return table.firstEntry().getValue();
        if (upperKey == null) return table.lastEntry().getValue();

        double lowerVal = table.get(lowerKey);
        double upperVal = table.get(upperKey);

        double t = (key - lowerKey) / (upperKey - lowerKey);
        return lowerVal + t * (upperVal - lowerVal);
    }

    /** Get the number of entries in the table. */
    public int size() {
        return table.size();
    }

    /** Remove all entries. */
    public void clear() {
        table.clear();
    }
}
