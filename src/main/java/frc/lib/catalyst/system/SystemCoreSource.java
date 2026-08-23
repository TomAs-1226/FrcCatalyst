package frc.lib.catalyst.system;

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
 * the brownout thresholds, volts for the battery. Conversion belongs to the reader, not here.
 *
 * @since 2.0.0
 */
public interface SystemCoreSource {

    /** Whether this source has anything to report. False means every reading below is empty. */
    boolean isAvailable();

    /**
     * A numeric value by its key on the {@code /sys} table.
     *
     * <p>Keys, read out of the {@code MrcCommDaemon} binary in the OS image rather than taken from
     * documentation: {@code battery}, {@code cpu}, {@code ram}, {@code ramtotal}, {@code storage},
     * {@code storagetotal}, {@code team}. The brownout thresholds {@code vbrownout} and
     * {@code vrecovery} come from the IO daemon.
     *
     * @return the value, or empty when the topic is absent
     */
    OptionalDouble number(String key);

    /** A boolean value by key. Absent reads as false, never as a fault. */
    boolean bool(String key);

    /** A source that reports nothing, for anywhere there is no Systemcore. */
    static SystemCoreSource unavailable() {
        return new SystemCoreSource() {
            @Override public boolean isAvailable() { return false; }
            @Override public OptionalDouble number(String key) { return OptionalDouble.empty(); }
            @Override public boolean bool(String key) { return false; }
        };
    }
}
