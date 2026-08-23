package frc.lib.catalyst.hardware;

import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.util.AlertManager;

import org.wpilib.hardware.bus.I2C;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * I²C on Systemcore, with the parts that catch people out handled.
 *
 * <p>I²C was never much used in FRC, mostly because the roboRIO's onboard port had a reputation for
 * locking up the robot and the MXP port meant a breakout board. Systemcore's is a plain, external,
 * Qwiic-compatible port, which puts a large catalogue of cheap sensor breakouts — SparkFun Qwiic,
 * Seeed Grove, M5Stack Unit — one cable away from a robot. That is worth having a wrapper for.
 *
 * <p><b>The cable is wrong. Read this before wiring anything.</b> Systemcore's I²C pinout does not
 * match the roboRIO's: <b>SCL and SDA are swapped</b>. An existing FRC I²C cable will not work and
 * will not obviously fail either — it just never finds the device, which reads as a dead sensor.
 * Systemcore matches the Qwiic and REV Control/Expansion Hub pinout instead, so a Qwiic cable is
 * correct as-is and an FRC one needs two wires moved.
 *
 * <p>{@link #scan()} exists mainly because of that. Before writing a driver, scan the bus: if the
 * device answers, the cable is right and the problem is elsewhere; if nothing answers at all, check
 * the cable before suspecting anything else.
 *
 * <pre>{@code
 * // In robot init, once:
 * CatalystI2C.scan().forEach(addr ->
 *         System.out.printf("I2C device at 0x%02X%n", addr));
 *
 * CatalystI2C sensor = new CatalystI2C(0x29, "DistanceSensor");
 * sensor.readRegister(0x00).ifPresent(id -> ...);
 * }</pre>
 *
 * <p>This is a transport, not a driver library. Catalyst does not ship per-device drivers because
 * the useful set is unbounded and each one is a small, well-documented protocol; what it ships is
 * the layer that makes writing one uneventful.
 *
 * @since 2.0.0
 */
public final class CatalystI2C implements AutoCloseable {

    /**
     * Systemcore's external I²C port.
     *
     * <p>Fixed rather than configurable: Systemcore exposes one I²C port, on GPIO 10/11, and the OS
     * loads {@code i2c-dev} for it at boot. WPILib 2027 enumerates {@code PORT_0} and {@code PORT_1};
     * {@code PORT_0} is the external connector.
     */
    public static final I2C.Port PORT = I2C.Port.PORT_0;

    /** Lowest 7-bit address that is not reserved. */
    private static final int MIN_ADDRESS = 0x08;

    /** Highest 7-bit address that is not reserved. */
    private static final int MAX_ADDRESS = 0x77;

    private final I2C device;
    private final int address;
    private final String name;

    /**
     * Open a device.
     *
     * @param address 7-bit device address, 0x08–0x77
     * @param name    human-readable name, used in logs and alerts
     */
    public CatalystI2C(int address, String name) {
        if (address < MIN_ADDRESS || address > MAX_ADDRESS) {
            throw new IllegalArgumentException(String.format(
                    "I2C address must be 0x%02X-0x%02X (7-bit, non-reserved), got 0x%02X",
                    MIN_ADDRESS, MAX_ADDRESS, address));
        }
        this.address = address;
        this.name = name == null ? String.format("I2C@0x%02X", address) : name;
        this.device = new I2C(PORT, address);
    }

    /**
     * Every address that answers on the bus.
     *
     * <p>Addresses each candidate without transferring data, which is the standard way to probe an
     * I²C bus and is harmless to devices that are present.
     *
     * <p>An empty result is the signature of the swapped-cable problem described on this class. It
     * is common enough that an empty scan raises an alert saying so, rather than leaving a team to
     * work it out.
     */
    public static List<Integer> scan() {
        List<Integer> found = new ArrayList<>();
        for (int addr = MIN_ADDRESS; addr <= MAX_ADDRESS; addr++) {
            try (I2C probe = new I2C(PORT, addr)) {
                if (probe.addressOnly()) {
                    found.add(addr);
                }
            } catch (RuntimeException ignored) {
                // An address that cannot even be opened is simply not present.
            }
        }

        String[] rows = found.stream().map(a -> String.format("0x%02X", a)).toArray(String[]::new);
        CatalystLog.log("I2C/Devices", rows);

        if (found.isEmpty()) {
            AlertManager.getInstance().warning("I2C",
                    "No I2C devices answered. Systemcore swaps SCL and SDA relative to the roboRIO, "
                            + "so a roboRIO I2C cable will silently find nothing - check the cable "
                            + "before the sensor.");
        }
        return List.copyOf(found);
    }

    /** Whether this device answers on the bus right now. */
    public boolean isPresent() {
        try {
            return device.addressOnly();
        } catch (RuntimeException ignored) {
            return false;
        }
    }

    /**
     * Read one byte from a register.
     *
     * @return the value, or empty if the transfer aborted
     */
    public Optional<Integer> readRegister(int register) {
        byte[] buffer = new byte[1];
        // WPILib returns true on a failed transfer, which is the opposite of what reads naturally -
        // hence the negation. Getting this backwards silently inverts every read in a driver.
        boolean aborted = device.read(register, 1, buffer);
        return aborted ? Optional.empty() : Optional.of(buffer[0] & 0xFF);
    }

    /**
     * Read several consecutive bytes starting at a register.
     *
     * @return the bytes, or empty if the transfer aborted
     */
    public Optional<byte[]> readRegisters(int register, int count) {
        if (count <= 0) {
            throw new IllegalArgumentException("count must be positive, got " + count);
        }
        byte[] buffer = new byte[count];
        boolean aborted = device.read(register, count, buffer);
        return aborted ? Optional.empty() : Optional.of(buffer);
    }

    /**
     * Write one byte to a register.
     *
     * @return true when the transfer completed
     */
    public boolean writeRegister(int register, int value) {
        return !device.write(register, value);
    }

    /** The underlying WPILib device, for protocols this wrapper does not cover. */
    public I2C getDevice() {
        return device;
    }

    /** The device's 7-bit address. */
    public int address() {
        return address;
    }

    /** The name given at construction. */
    public String name() {
        return name;
    }

    @Override
    public void close() {
        device.close();
    }
}
