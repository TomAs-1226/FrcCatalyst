package frc.lib.catalyst.hardware;

import frc.lib.catalyst.identity.RobotIdentity;
import frc.lib.catalyst.logging.CatalystLog;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Process-wide registry of every CAN device claimed in this robot program.
 *
 * <p>Two ways to populate it:
 * <ol>
 *   <li><b>Up front</b>, from a generated {@code CANIds.java} produced by the
 *       <a href="https://tomas-1226.github.io/FrcCatalyst/tools/canids/">CAN
 *       ID Planner</a>. A static block in that file pre-registers every
 *       planned device so the wiring plan is enforced at robot boot.</li>
 *   <li><b>Lazily</b>, by the mechanism builders themselves. When
 *       {@link CatalystMotor.Builder#build()} runs, the primary motor and
 *       every follower automatically claim their {@code (bus, canId)}. Any
 *       attached CANcoder (fused / sync / remote) does the same.</li>
 * </ol>
 *
 * <p>Either way, a duplicate {@code (bus, canId)} for two <em>different</em>
 * device names throws {@link CANConflictException} with a message that
 * names both sides of the collision. Identical re-registrations (same
 * name, same id, same bus, same type) are silently idempotent, so you can
 * reference {@code CANIds.X} from the planner output and the same id from
 * the subsystem code without tripping yourself up.
 *
 * <p>The full plan is published through {@link CatalystLog} as a string array
 * at {@code /Catalyst/CAN/Devices} so the Health Dashboard and any other NT
 * viewer can see exactly what's wired where.
 */
public final class CANRegistry {

    /** Thrown when two devices claim the same {@code (bus, canId)} pair. */
    public static class CANConflictException extends RuntimeException {
        public CANConflictException(String message) { super(message); }
    }

    /** One entry in the registry. */
    public record Entry(String name, int canId, String bus, String type) {
        /** Pipe-delimited line used for NT serialization. */
        String serialize() {
            return String.format("%s|%d|%s|%s", bus, canId, type, name);
        }
    }

    private static final Map<String, Entry> byKey = new LinkedHashMap<>();   // "bus/id" → entry

    private CANRegistry() {}

    /**
     * Claim a CAN ID. Throws {@link CANConflictException} if a device with
     * a different name has already claimed this {@code (bus, canId)}.
     * Re-registering the same {@code (name, canId, bus, type)} is a no-op.
     *
     * @param name human-readable name (e.g. {@code "FrontLeftDrive"})
     * @param canId CAN bus id (0–62)
     * @param bus   Phoenix bus name (e.g. {@code "canivore"} or {@code ""} for the rio bus)
     * @param type  device type label (e.g. {@code "Kraken X60"}, {@code "CANcoder"})
     */
    public static synchronized void register(String name, int canId, String bus, String type) {
        if (canId < 0 || canId > 62) {
            throw new IllegalArgumentException("CAN id must be 0–62, got " + canId);
        }
        // Normalised, not raw. "", "0" and "can_s0" are three spellings of one physical bus, and
        // keying on the raw string aliased them apart - so an elevator built with the default bus
        // and an intake built with .canBus("can_s0"), both at id 12 and both on the same wire,
        // registered under different keys and never collided. The team met that as a Phoenix
        // failure at an event instead of a named conflict at robotInit.
        //
        // The same aliasing split byBus(), so contention warnings and utilisation each saw two
        // half-loaded buses where there was one overloaded one.
        String b = CatalystCANBus.of(bus).name();
        String key = b + "/" + canId;
        Entry existing = byKey.get(key);
        Entry candidate = new Entry(name, canId, b, type == null ? "" : type);
        if (existing != null) {
            if (existing.name.equals(candidate.name)
                    && existing.type.equals(candidate.type)) {
                // Idempotent re-registration — same device. Silent OK.
                return;
            }
            throw new CANConflictException(
                    "CAN conflict on bus \"" + (b.isEmpty() ? "rio" : b) + "\" id " + canId + ":\n"
                            + "  was: " + existing.name + " (" + existing.type + ")\n"
                            + "  now: " + candidate.name + " (" + candidate.type + ")");
        }
        byKey.put(key, candidate);
        republish();
    }

    /**
     * Convenience overload for callers that don't have a sensible type
     * label. Stores {@code "Unknown"} as the type.
     */
    public static void register(String name, int canId, String bus) {
        register(name, canId, bus, "Unknown");
    }

    /**
     * Like {@link #register(String, int, String, String)} but returns
     * {@code false} on conflict instead of throwing. Useful for test code
     * that wants to recover.
     */
    public static synchronized boolean tryRegister(String name, int canId, String bus, String type) {
        try {
            register(name, canId, bus, type);
            return true;
        } catch (CANConflictException e) {
            return false;
        }
    }

    /** Look up which device owns a given {@code (canId, bus)} pair. */
    public static synchronized Optional<Entry> lookup(int canId, String bus) {
        String b = bus == null ? "" : bus;
        return Optional.ofNullable(byKey.get(b + "/" + canId));
    }

    /** Snapshot of every registered device, ordered by bus then id. */
    public static synchronized List<Entry> all() {
        List<Entry> out = new ArrayList<>(byKey.values());
        out.sort((a, b) -> {
            int c = a.bus.compareTo(b.bus);
            return c != 0 ? c : Integer.compare(a.canId, b.canId);
        });
        return Collections.unmodifiableList(out);
    }

    /** Names grouped by bus, sorted by id. Convenient for status displays. */
    public static synchronized Map<String, List<Entry>> byBus() {
        Map<String, List<Entry>> out = new LinkedHashMap<>();
        for (Entry e : all()) {
            out.computeIfAbsent(e.bus, k -> new ArrayList<>()).add(e);
        }
        return out;
    }

    /** Number of registered devices. */
    public static synchronized int size() {
        return byKey.size();
    }

    /**
     * Claim a CAN ID on a typed bus. Equivalent to the string overload, and clearer at the call
     * site now that there are five buses to choose between.
     */
    public static void register(String name, int canId, CatalystCANBus bus, String type) {
        register(name, canId, bus == null ? "" : bus.name(), type);
    }

    /**
     * Wiring problems the registry can see from the plan alone, before the robot is ever enabled.
     *
     * <p>Two kinds, both specific to Systemcore:
     *
     * <ul>
     *   <li><b>Everything on one bus.</b> Five buses exist; putting every device on {@code can_s0}
     *       wastes four of them and is the single easiest way to run out of bandwidth. Reported
     *       once the count passes what a 1 Mbit/s CAN 2.0 bus comfortably carries.</li>
     *   <li><b>Loaded buses that share an SPI controller.</b> {@code can_s0}/{@code can_s1} and
     *       {@code can_s3}/{@code can_s4} are paired in hardware, so splitting a heavy load across
     *       a pair buys much less than splitting it across unpaired buses. This is the failure that
     *       looks like a mystery at an event, because on paper the devices are on "different buses".</li>
     * </ul>
     *
     * <p>Advisory only — nothing here throws. It is meant for {@code SystemCheck} and the health
     * dashboard to surface in the pit, where it can still be acted on.
     *
     * @return human-readable warnings, empty when the plan looks sound
     */
    public static synchronized List<String> contentionWarnings() {
        // Devices per bus is a rough proxy for load. A precise figure needs each device's status
        // signal rates, which is the next step for the planner; this catches the obvious cases.
        final int busyThreshold = 12;

        Map<String, List<Entry>> buses = byBus();
        List<String> warnings = new ArrayList<>();

        for (var e : buses.entrySet()) {
            if (e.getValue().size() > busyThreshold) {
                warnings.add("Bus " + displayName(e.getKey()) + " carries " + e.getValue().size()
                        + " devices. Systemcore has " + CatalystCANBus.SYSTEMCORE_BUS_COUNT
                        + " buses - spreading these out will cut utilisation.");
            }
        }

        List<String> names = new ArrayList<>(buses.keySet());
        for (int i = 0; i < names.size(); i++) {
            for (int j = i + 1; j < names.size(); j++) {
                CatalystCANBus a = safeBus(names.get(i));
                CatalystCANBus b = safeBus(names.get(j));
                if (a == null || b == null || !a.sharesControllerWith(b)) continue;

                int load = buses.get(names.get(i)).size() + buses.get(names.get(j)).size();
                if (load > busyThreshold) {
                    warnings.add("Buses " + a.name() + " and " + b.name()
                            + " share an SPI controller and together carry " + load
                            + " devices. Moving some onto an unpaired bus will help more than "
                            + "splitting them across this pair.");
                }
            }
        }
        return warnings;
    }

    private static String displayName(String bus) {
        return bus == null || bus.isEmpty() ? CatalystCANBus.DEFAULT.name() : bus;
    }

    /** Resolve a stored bus string, tolerating anything unparseable rather than throwing. */
    private static CatalystCANBus safeBus(String bus) {
        try {
            return CatalystCANBus.of(bus);
        } catch (RuntimeException ignored) {
            return null;
        }
    }

    /**
     * Drop every registration. Mostly useful in unit tests — production
     * code shouldn't need this.
     */
    /**
     * The wiring plan as a DBC file, for Systemcore's CAN Bus Monitor.
     *
     * <p>Systemcore's web UI decodes live CAN traffic if you upload a DBC or REV JSON spec. Without
     * one it shows numbers: an id and some bytes. Catalyst already knows every device's name, id,
     * bus and type, so it can emit the file and turn that display into names.
     *
     * <p>Scope, stated plainly: this maps <b>identity</b>, not payloads. Each device becomes a DBC
     * message named the way robot code names it, so the monitor shows {@code FrontLeftDrive} rather
     * than an id. It does not describe Phoenix's signal layout inside those frames - that is CTRE's
     * to publish, it is firmware-dependent, and guessing at it would produce a file that decodes
     * confidently and wrongly, which is worse than one that decodes nothing.
     *
     * <pre>{@code
     * java.nio.file.Files.writeString(java.nio.file.Path.of("catalyst.dbc"), CANRegistry.toDbc());
     * }</pre>
     *
     * @return DBC file content, one message per registered device
     */
    public static synchronized String toDbc() {
        StringBuilder out = new StringBuilder();
        out.append("VERSION \"\"\n\n");
        out.append("NS_ :\n\n");
        out.append("BS_:\n\n");

        // One node per bus, so the monitor groups devices the way the robot is actually wired.
        out.append("BU_:");
        for (String bus : byBus().keySet()) {
            out.append(' ').append(nodeName(bus));
        }
        out.append("\n\n");

        for (Entry e : all()) {
            // Phoenix device ids are not CAN arbitration ids, and that mapping is CTRE's business.
            // The id is used here only to keep messages distinct; the name is the part teams read.
            String message = dbcSafe(e.name());
            String node = nodeName(e.bus());
            out.append(String.format("BO_ %d %s: 8 %s%n", e.canId(), message, node));
            out.append(String.format("  SG_ %s_raw : 0|64@1+ (1,0) [0|0] \"\" %s%n%n", message, node));
        }
        return out.toString();
    }

    /** DBC node name for a bus. */
    private static String nodeName(String bus) {
        return dbcSafe(bus == null || bus.isEmpty()
                ? CatalystCANBus.DEFAULT.name()
                : CatalystCANBus.of(bus).name());
    }

    /** DBC identifiers allow letters, digits and underscores, and cannot start with a digit. */
    private static String dbcSafe(String raw) {
        String cleaned = raw == null ? "" : raw.replaceAll("[^A-Za-z0-9_]", "_");
        if (cleaned.isEmpty()) {
            return "Unnamed";
        }
        return Character.isDigit(cleaned.charAt(0)) ? "_" + cleaned : cleaned;
    }

    public static synchronized void clear() {
        byKey.clear();
        republish();
    }

    private static void republish() {
        List<Entry> sorted = all();
        String[] out = new String[sorted.size()];
        for (int i = 0; i < sorted.size(); i++) out[i] = sorted.get(i).serialize();
        CatalystLog.log("CAN/Devices", out);

        // The spec sheet counts devices by type, and mechanisms claim their ids as they are built —
        // which is usually after the robot has declared itself. No-op until it has.
        RobotIdentity.refresh();
    }

    /** Test hook. Returns the internal map view for assertions. Not for production code. */
    @SuppressWarnings("unused")
    static synchronized Map<String, Entry> internalSnapshot() {
        return new HashMap<>(byKey);
    }
}
