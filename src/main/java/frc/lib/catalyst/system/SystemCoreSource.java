package frc.lib.catalyst.system;

import java.util.Optional;
import java.util.OptionalDouble;

/**
 * Where {@link SystemCoreStatus} gets its readings.
 *
 * <p>This exists because the real source cannot be used in a test, and the reason is worth stating:
 * resolving Systemcore's system NetworkTables server forces a HAL JNI load, and WPILib's
 * {@code RuntimeLoader} <em>terminates the JVM</em> when the natives are missing rather than
 * throwing. No amount of {@code catch (Throwable)} helps when the process is already gone. A test
 * that so much as constructs the real reader takes the whole Gradle worker down and loses every
 * other result with it.
 *
 * <p>So the reading and the interpreting are separated. {@code SystemCoreStatus} keeps all the
 * logic — unit conversion, ratios, what counts as absent — and this supplies raw values.
 * {@link SystemCoreSim} supplies them from memory, which makes every rule in that class testable
 * and gives simulation something to drive.
 *
 * <p>Values are raw, exactly as the OS publishes them: bytes for memory and storage, millivolts for
 * the brownout thresholds, volts for the battery, JEDEC codes for the eMMC health registers.
 * Conversion belongs to the reader, not here.
 *
 * <h2>Keys</h2>
 *
 * <p>A plain key names an entry on the {@code /sys} table. A key starting with {@code /} is an
 * absolute topic path on the system server, which is how the CAN diagnostics are reached — they live
 * under {@code /diagnostics}, not {@code /sys}.
 *
 * @since 2.0.0
 */
public interface SystemCoreSource {

    /** Whether this source has anything to report. False means every reading below is empty. */
    boolean isAvailable();

    /**
     * A numeric value by key.
     *
     * <p>Keys were read out of the {@code MrcCommDaemon} and IO daemon binaries in the OS image
     * rather than taken from documentation, which does not list them. On the {@code /sys} table:
     * {@code battery}, {@code current3v3}, {@code brownout}, {@code vbrownout}, {@code vrecovery},
     * {@code cpu}, {@code temp}, {@code ram}, {@code ramtotal}, {@code ramAvail}, {@code ramutil},
     * {@code storage}, {@code storagetotal}, {@code storageavail}, {@code storageutil},
     * {@code team}, {@code hsub}, {@code emmc/lifetime_a}, {@code emmc/lifetime_b},
     * {@code emmc/pre_eol}, {@code emmc/status}, and the {@code faults/} and {@code faultcounts/}
     * subtables.
     *
     * @return the value, or empty when the topic is absent
     */
    OptionalDouble number(String key);

    /** A boolean value by key. Absent reads as false, never as a fault. */
    boolean bool(String key);

    /**
     * An array of numbers by key, for the per-bus readings.
     *
     * <p>{@code /diagnostics/canbusutil} is the one that matters: one entry per CAN bus, which is
     * the only way to see utilisation per bus rather than as a single figure.
     *
     * @return the array, or empty when absent. Never a zero-length array standing in for absent —
     *         "five buses all at 0%" and "no reading" are different facts.
     */
    default Optional<double[]> numberArray(String key) {
        return Optional.empty();
    }

    /**
     * An array of strings by key.
     *
     * <p>{@code networkInterfaces} is the one Systemcore publishes.
     *
     * @return the array, or empty when absent
     */
    default Optional<String[]> stringArray(String key) {
        return Optional.empty();
    }

    /** A source that reports nothing, for anywhere there is no Systemcore. */
    static SystemCoreSource unavailable() {
        return new SystemCoreSource() {
            @Override public boolean isAvailable() { return false; }
            @Override public OptionalDouble number(String key) { return OptionalDouble.empty(); }
            @Override public boolean bool(String key) { return false; }
        };
    }
}
