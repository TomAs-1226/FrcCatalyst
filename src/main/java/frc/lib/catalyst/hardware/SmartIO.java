package frc.lib.catalyst.hardware;

import frc.lib.catalyst.logging.CatalystLog;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * What each Systemcore IO pin is configured to be, declared in robot code.
 *
 * <p>Systemcore's IO pins are typed, and the type lives <em>on the device</em>, set in its web UI.
 * A pin configured as {@code analog_in} does not behave like one configured as {@code digital_in},
 * and robot code has no way to tell them apart — it just reads the wrong thing. Worse, the types
 * survive a code deploy: a pin someone retyped while debugging in the pit stays that way through
 * every subsequent build, and nothing in the robot program mentions it.
 *
 * <p>This is the same idea as {@link CANRegistry}, applied to IO. Declare what the robot expects,
 * get it published where a human can compare it against the device, and catch the mistakes that can
 * be caught from the declaration alone.
 *
 * <pre>{@code
 * SmartIO.declare(0, SmartIO.Type.DIGITAL_IN,  "IntakeBeamBreak");
 * SmartIO.declare(1, SmartIO.Type.WS2812,      "UnderglowStrip");
 * SmartIO.declare(2, SmartIO.Type.QUADRATURE,  "ArmEncoder");
 * }</pre>
 *
 * <p>The declared map is published to {@code /Catalyst/IO/Pins}, so a pit checklist becomes reading
 * two lists side by side instead of remembering what a pin was supposed to be.
 *
 * <p><b>What this does not do.</b> It does not read the pin types back off the device and compare
 * them automatically. Systemcore exposes them over its web API at {@code /api/io}, but nothing in
 * the robot-side libraries surfaces them, and inventing an endpoint call here would be guessing at
 * a contract that is not published. Declaring and publishing is the honest half; the comparison is
 * a human reading {@code /Catalyst/IO/Pins} against the Config tab, until the platform offers
 * something better.
 *
 * <p>The type names below were read out of the {@code diagnosticsprocess} binary in the Systemcore
 * OS image, so they match what the device actually calls them.
 *
 * @since 2.0.0
 */
public final class SmartIO {

    /**
     * The modes a Systemcore IO pin can be configured into.
     *
     * <p>{@link #wireName()} is the string the OS itself uses, so a published declaration reads the
     * same as the web UI rather than needing translation.
     */
    public enum Type {
        /** Analog input, reported in millivolts (0–3300). */
        ANALOG_IN("analog_in"),
        /** Digital input. */
        DIGITAL_IN("digital_in"),
        /** Digital output. */
        DIGITAL_OUT("digital_out"),
        /** PWM input, with period available. */
        PWM_IN("pwm_in"),
        /** PWM output. Note that Systemcore forces these to centre when the robot is disabled. */
        PWM_OUT("pwm_out"),
        /** Quadrature encoder. */
        QUADRATURE("quadrature"),
        /** Counter, rising edge. */
        COUNTER_RISING("single_counter_rising"),
        /** Counter, falling edge. */
        COUNTER_FALLING("single_counter_falling"),
        /** Addressable LED strip. */
        WS2812("ws2812"),
        /** Fabric pulse output. */
        FAB_PULSE("fabpulse");

        private final String wireName;

        Type(String wireName) {
            this.wireName = wireName;
        }

        /** The name Systemcore uses for this mode, as shown in its web UI. */
        public String wireName() {
            return wireName;
        }

        /** Resolve a Systemcore wire name back to a type. */
        public static Optional<Type> fromWireName(String name) {
            for (Type t : values()) {
                if (t.wireName.equalsIgnoreCase(name)) {
                    return Optional.of(t);
                }
            }
            return Optional.empty();
        }
    }

    /** One declared pin. */
    public record Pin(int pin, Type type, String name) {
        /** Pipe-delimited line used for NT serialisation, matching the CAN registry's shape. */
        String serialize() {
            return String.format("%d|%s|%s", pin, type.wireName(), name);
        }
    }

    /** Thrown when two different things claim the same pin. */
    public static class PinConflictException extends RuntimeException {
        public PinConflictException(String message) {
            super(message);
        }
    }

    private static final Map<Integer, Pin> byPin = new LinkedHashMap<>();

    private SmartIO() {}

    /**
     * Declare what a pin is expected to be.
     *
     * <p>Re-declaring the same pin identically is a no-op, so a constant and a subsystem can both
     * name it without tripping over each other. Declaring it as something else throws, because that
     * is a wiring plan that cannot be satisfied.
     *
     * @param pin  IO pin number
     * @param type mode the pin must be configured into on the device
     * @param name human-readable name, e.g. {@code "IntakeBeamBreak"}
     */
    public static synchronized void declare(int pin, Type type, String name) {
        if (pin < 0) {
            throw new IllegalArgumentException("IO pin must not be negative, got " + pin);
        }
        Pin candidate = new Pin(pin, type, name == null ? "" : name);
        Pin existing = byPin.get(pin);

        if (existing != null) {
            if (existing.equals(candidate)) {
                return;
            }
            throw new PinConflictException(
                    "IO pin " + pin + " declared twice:\n"
                            + "  was: " + existing.name() + " (" + existing.type().wireName() + ")\n"
                            + "  now: " + candidate.name() + " (" + candidate.type().wireName() + ")");
        }
        byPin.put(pin, candidate);
        republish();
    }

    /** Every declared pin, in pin order. */
    public static synchronized List<Pin> all() {
        List<Pin> out = new ArrayList<>(byPin.values());
        out.sort((a, b) -> Integer.compare(a.pin(), b.pin()));
        return List.copyOf(out);
    }

    /** What the robot expects a given pin to be, if anything declared it. */
    public static synchronized Optional<Pin> lookup(int pin) {
        return Optional.ofNullable(byPin.get(pin));
    }

    /** How many pins have been declared. */
    public static synchronized int size() {
        return byPin.size();
    }

    /**
     * Problems visible from the declaration alone.
     *
     * <p>Currently one, and it is the one that actually bites: more WS2812 pins than Systemcore can
     * drive. The firmware caps total addressable LEDs, and a robot that declares four strips has
     * usually not checked the total against that limit.
     *
     * <p>Advisory — nothing throws. Intended for {@code SystemCheck} and the pit dashboard.
     */
    public static synchronized List<String> warnings() {
        List<String> out = new ArrayList<>();

        long ledPins = byPin.values().stream().filter(p -> p.type() == Type.WS2812).count();
        if (ledPins > 1) {
            out.add(ledPins + " pins are declared as WS2812. Systemcore drives at most "
                    + frc.lib.catalyst.subsystems.leds.LEDConfig.MAX_LEDS
                    + " LEDs in total across all pins - check the combined length, and give each "
                    + "strip a distinct startIndex.");
        }
        return out;
    }

    /** Drop every declaration. Mostly for tests. */
    public static synchronized void clear() {
        byPin.clear();
        republish();
    }

    private static void republish() {
        List<Pin> sorted = all();
        String[] rows = new String[sorted.size()];
        for (int i = 0; i < sorted.size(); i++) {
            rows[i] = sorted.get(i).serialize();
        }
        CatalystLog.log("IO/Pins", rows);
    }
}
