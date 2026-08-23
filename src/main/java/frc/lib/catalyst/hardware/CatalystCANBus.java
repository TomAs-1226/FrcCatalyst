package frc.lib.catalyst.hardware;

import com.ctre.phoenix6.CANBus;

import java.util.Objects;
import java.util.OptionalInt;

/**
 * One CAN bus, on a control system that now has five of them.
 *
 * <p>On the roboRIO there was a single bus plus whatever CANivores a team added, so Catalyst could
 * treat "which bus" as a bare string and default it to {@code ""}. Systemcore has five —
 * {@code can_s0} through {@code can_s4} — and Motioncore adds twenty more. Which bus a device sits
 * on is now a real design decision with real consequences, so it gets a real type.
 *
 * <p><b>The five buses are not five independent lanes.</b> Read out of the Systemcore OS image:
 * they are MCP2518FD controllers on shared SPI hosts — {@code can_s0} and {@code can_s1} share one,
 * {@code can_s3} and {@code can_s4} share another, and {@code can_s2} sits alone. That is why the
 * Systemcore web UI warns about frame-rate limits "for any individual CAN Bus <i>or CAN Bus pair</i>".
 * Two heavily loaded buses that happen to be paired will throttle each other in a way two unpaired
 * ones will not, which is exactly the sort of thing that looks like a mystery brownout at an event.
 * {@link #sharesControllerWith(CatalystCANBus)} exposes it so the CAN ID planner and the health
 * dashboard can account for it instead of teams discovering it on the field.
 *
 * <p>Bus naming is verified against the Phoenix 6 jar, not the documentation: the factory is
 * {@code CANBus.systemcore(int)}, lower-case, even though the Systemcore testing notes write it
 * {@code systemCore}.
 *
 * @since 2.0.0
 */
public final class CatalystCANBus {

    /** How many CAN buses Systemcore exposes. */
    public static final int SYSTEMCORE_BUS_COUNT = 5;

    /** How many CAN buses Motioncore exposes. */
    public static final int MOTIONCORE_BUS_COUNT = 20;

    /**
     * The bus a device lands on when nothing says otherwise: {@code can_s0}.
     *
     * <p>This is the one behavioural change teams may notice. On the roboRIO the default bus was
     * {@code ""}, the rio bus. There is no rio bus any more, so the default is the first Systemcore
     * bus. Code that never specified a bus keeps working and keeps all its devices together.
     */
    public static final CatalystCANBus DEFAULT = systemcore(0);

    private enum Kind { SYSTEMCORE, MOTIONCORE, CANIVORE }

    private final Kind kind;
    private final int index;
    private final String name;
    private final CANBus phoenix;

    private CatalystCANBus(Kind kind, int index, String name, CANBus phoenix) {
        this.kind = kind;
        this.index = index;
        this.name = name;
        this.phoenix = phoenix;
    }

    /**
     * One of Systemcore's own buses.
     *
     * @param index 0–4, numbered left to right on the device
     */
    public static CatalystCANBus systemcore(int index) {
        require(index, SYSTEMCORE_BUS_COUNT, "Systemcore");
        return new CatalystCANBus(Kind.SYSTEMCORE, index, "can_s" + index, CANBus.systemcore(index));
    }

    /**
     * One of Motioncore's buses. These are the CAN FD capable ones; Systemcore's five run CAN 2.0
     * at 1 Mbit/s in the shipped OS configuration.
     *
     * @param index 0–19
     */
    public static CatalystCANBus motioncore(int index) {
        require(index, MOTIONCORE_BUS_COUNT, "Motioncore");
        return new CatalystCANBus(Kind.MOTIONCORE, index, "can_d" + index, CANBus.motioncore(index));
    }

    /**
     * A CANivore, by the name given to it in Tuner.
     *
     * <p>Requires the {@code canivore-usb-kernel} and {@code canivore-usb} packages installed on
     * Systemcore and a power cycle; a CANivore that was never installed simply never appears.
     */
    public static CatalystCANBus canivore(String name) {
        Objects.requireNonNull(name, "CANivore name must not be null");
        if (name.isBlank()) {
            throw new IllegalArgumentException("CANivore name must not be blank");
        }
        return new CatalystCANBus(Kind.CANIVORE, -1, name, new CANBus(name));
    }

    /**
     * Resolve a bus from the string forms Catalyst accepted before this type existed, so existing
     * robot code and generated {@code CANIds.java} keep working.
     *
     * <p>Accepts {@code ""} or {@code null} (the default bus), a bare index {@code "0"}–{@code "4"},
     * an interface name {@code "can_s2"} or {@code "can_d7"}, or anything else as a CANivore name.
     */
    public static CatalystCANBus of(String spec) {
        if (spec == null || spec.isBlank()) {
            return DEFAULT;
        }
        String s = spec.trim();
        if (s.matches("\\d+")) {
            return systemcore(Integer.parseInt(s));
        }
        if (s.matches("can_s\\d+")) {
            return systemcore(Integer.parseInt(s.substring(5)));
        }
        if (s.matches("can_d\\d+")) {
            return motioncore(Integer.parseInt(s.substring(5)));
        }
        return canivore(s);
    }

    private static void require(int index, int count, String what) {
        if (index < 0 || index >= count) {
            throw new IllegalArgumentException(
                    what + " bus index must be 0-" + (count - 1) + ", got " + index);
        }
    }

    /** The Phoenix handle, for constructing CTRE devices. */
    public CANBus phoenix() {
        return phoenix;
    }

    /**
     * The WPILib handle, for constructing WPILib devices such as pneumatics.
     *
     * <p>WPILib and Phoenix each ship their own {@code CANBus} type and they are not
     * interchangeable — Phoenix's is a class built from a name, WPILib's is an enum of the fixed
     * Systemcore and Motioncore interfaces. Holding both behind one Catalyst type is the whole
     * reason this class exists; call sites should not have to know which library wants which.
     *
     * @throws IllegalStateException for a CANivore, which WPILib's enum cannot represent
     */
    public org.wpilib.hardware.bus.CANBus wpilib() {
        return switch (kind) {
            case SYSTEMCORE -> org.wpilib.hardware.bus.CANBus.valueOf("CAN_S" + index);
            case MOTIONCORE -> org.wpilib.hardware.bus.CANBus.valueOf("CAN_D" + index);
            case CANIVORE -> throw new IllegalStateException(
                    "WPILib's CANBus enum has no CANivore entry, so " + name
                            + " cannot be used for WPILib devices. CANivores are Phoenix-only.");
        };
    }

    /** Interface name, e.g. {@code "can_s2"}, or the CANivore's name. */
    public String name() {
        return name;
    }

    /** Whether this is one of Systemcore's own five buses. */
    public boolean isSystemcore() {
        return kind == Kind.SYSTEMCORE;
    }

    /** Whether this is one of Motioncore's buses. */
    public boolean isMotioncore() {
        return kind == Kind.MOTIONCORE;
    }

    /** Bus index, present for Systemcore and Motioncore buses and empty for a CANivore. */
    public OptionalInt index() {
        return index >= 0 ? OptionalInt.of(index) : OptionalInt.empty();
    }

    /** Whether the bus is running CAN FD. Systemcore's five report false on the shipped OS. */
    public boolean isFD() {
        return phoenix.isNetworkFD();
    }

    /** Live status from Phoenix: utilisation, bus-off count, TX-full count, REC and TEC. */
    public CANBus.CANBusStatus status() {
        return phoenix.getStatus();
    }

    /**
     * Which physical SPI controller this bus hangs off, for Systemcore buses.
     *
     * <p>Buses sharing a group contend for the same SPI host. Returns -1 for anything that is not a
     * Systemcore bus, since the grouping is a fact about Systemcore's board and nothing else.
     */
    public int controllerGroup() {
        if (!isSystemcore()) {
            return -1;
        }
        return switch (index) {
            case 0, 1 -> 0;   // SPI2
            case 2 -> 1;      // SPI3, on its own
            case 3, 4 -> 2;   // SPI1
            default -> -1;
        };
    }

    /**
     * Whether two buses share an SPI controller, and so share bandwidth.
     *
     * <p>A bus never counts as sharing with itself — this answers "will loading these two fight
     * each other", and one bus does not fight itself.
     */
    public boolean sharesControllerWith(CatalystCANBus other) {
        if (other == null || !isSystemcore() || !other.isSystemcore() || equals(other)) {
            return false;
        }
        return controllerGroup() == other.controllerGroup();
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof CatalystCANBus b && kind == b.kind && index == b.index
                && name.equals(b.name);
    }

    @Override
    public int hashCode() {
        return Objects.hash(kind, index, name);
    }

    @Override
    public String toString() {
        return name;
    }
}
