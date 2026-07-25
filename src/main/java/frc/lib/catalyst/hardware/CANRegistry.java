package frc.lib.catalyst.hardware;

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
        String b = bus == null ? "" : bus;
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
     * Drop every registration. Mostly useful in unit tests — production
     * code shouldn't need this.
     */
    public static synchronized void clear() {
        byKey.clear();
        republish();
    }

    private static void republish() {
        List<Entry> sorted = all();
        String[] out = new String[sorted.size()];
        for (int i = 0; i < sorted.size(); i++) out[i] = sorted.get(i).serialize();
        CatalystLog.log("CAN/Devices", out);
    }

    /** Test hook. Returns the internal map view for assertions. Not for production code. */
    @SuppressWarnings("unused")
    static synchronized Map<String, Entry> internalSnapshot() {
        return new HashMap<>(byKey);
    }
}
