package frc.lib.catalyst.system;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.OptionalDouble;

/**
 * A Systemcore that only exists in memory.
 *
 * <p>Two jobs. It makes {@link SystemCoreStatus} testable at all — see {@link SystemCoreSource} for
 * why the real one cannot be — and it lets a simulated robot exercise the code paths that only run
 * when the machine is in trouble. Those paths are the ones nobody tests, because reproducing them
 * on real hardware means deliberately filling a disk or pinning a core during a practice match.
 *
 * <p>Values are set in the OS's own units, so the conversions under test are the real ones:
 *
 * <pre>{@code
 * SystemCoreSim sim = new SystemCoreSim()
 *         .withBattery(12.4)
 *         .withCpuPercent(35)
 *         .withMemory(2_000_000_000L, 8_000_000_000L)
 *         .withStorage(30_000_000_000L, 32_000_000_000L);   // nearly full
 * SystemCoreStatus.useSource(sim);
 *
 * // ... and now the low-storage health check can actually be shown to fire.
 * sim.withStorage(31_500_000_000L, 32_000_000_000L);
 * }</pre>
 *
 * <p>A fresh instance reports available with no values, which is the honest model of a Systemcore
 * that has booted but not published yet — and is itself a case worth testing, since every reading
 * has to come back empty rather than zero.
 *
 * @since 2.0.0
 */
public final class SystemCoreSim implements SystemCoreSource {

    private final Map<String, Double> numbers = new LinkedHashMap<>();
    private final Map<String, Boolean> booleans = new LinkedHashMap<>();
    private boolean available = true;

    /** Set a raw value by key, in the OS's units. */
    public SystemCoreSim set(String key, double value) {
        numbers.put(key, value);
        return this;
    }

    /** Set a raw boolean by key. */
    public SystemCoreSim set(String key, boolean value) {
        booleans.put(key, value);
        return this;
    }

    /** Remove a key, so it reads as absent. */
    public SystemCoreSim clear(String key) {
        numbers.remove(key);
        booleans.remove(key);
        return this;
    }

    /**
     * Model a machine that is not there.
     *
     * <p>Simulation and desktop builds, where every reading must come back empty and nothing may
     * throw.
     */
    public SystemCoreSim withAvailable(boolean present) {
        this.available = present;
        return this;
    }

    /** Battery voltage, in volts. */
    public SystemCoreSim withBattery(double volts) {
        return set("battery", volts);
    }

    /** Whether the machine reports itself browned out. */
    public SystemCoreSim withBrownedOut(boolean brownedOut) {
        return set("brownout", brownedOut);
    }

    /**
     * Brownout and recovery thresholds, in volts.
     *
     * <p>Stored as millivolts, because that is what the OS publishes and the conversion is part of
     * what is being tested. Systemcore defaults to 6750 mV and 7500 mV.
     */
    public SystemCoreSim withBrownoutThresholds(double brownoutVolts, double recoveryVolts) {
        return set("vbrownout", brownoutVolts * 1000.0).set("vrecovery", recoveryVolts * 1000.0);
    }

    /** CPU utilisation, in percent. */
    public SystemCoreSim withCpuPercent(double percent) {
        return set("cpu", percent);
    }

    /** Memory used and total, in bytes. */
    public SystemCoreSim withMemory(double usedBytes, double totalBytes) {
        return set("ram", usedBytes).set("ramtotal", totalBytes);
    }

    /** Storage used and total, in bytes. */
    public SystemCoreSim withStorage(double usedBytes, double totalBytes) {
        return set("storage", usedBytes).set("storagetotal", totalBytes);
    }

    /** Team number as configured on the device, which is not always the one in robot code. */
    public SystemCoreSim withTeamNumber(int team) {
        return set("team", team);
    }

    /**
     * A machine in good order: 12.4 V, 25% CPU, quarter of memory and storage used.
     *
     * <p>A starting point to perturb, so a test states only the thing it is about.
     */
    public static SystemCoreSim healthy() {
        return new SystemCoreSim()
                .withBattery(12.4)
                .withBrownedOut(false)
                .withBrownoutThresholds(6.75, 7.5)
                .withCpuPercent(25)
                .withMemory(2_000_000_000.0, 8_000_000_000.0)
                .withStorage(8_000_000_000.0, 32_000_000_000.0)
                .withTeamNumber(5805);
    }

    @Override
    public boolean isAvailable() {
        return available;
    }

    @Override
    public OptionalDouble number(String key) {
        if (!available) {
            return OptionalDouble.empty();
        }
        Double v = numbers.get(key);
        return v == null ? OptionalDouble.empty() : OptionalDouble.of(v);
    }

    @Override
    public boolean bool(String key) {
        return available && Boolean.TRUE.equals(booleans.get(key));
    }
}
