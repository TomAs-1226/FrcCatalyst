package frc.lib.catalyst.util;

import frc.lib.catalyst.logging.CatalystLog;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Says what each tunable <em>is</em>, so a dashboard can draw it properly.
 *
 * <h2>The problem</h2>
 *
 * <p>{@link TunableNumber} carries a key and a value, and nothing else. That is all a robot needs —
 * it reads the number and applies it — but it is not enough to draw a control. A dashboard that finds
 * {@code /Catalyst/Tuning/Elevator/kG} knows only that some number lives there: not what it means, not
 * what units it is in, not what a sensible range would be, and not that it belongs beside
 * {@code kP}, {@code kI} and {@code kD} rather than beside the shooter's gains.
 *
 * <p>So every gain on every 2.x robot — including the ones the six built-in mechanism types get for
 * free — showed up as an unbounded, ungrouped, unitless numeric field. Not broken: a technician could
 * still type a number. But nothing could offer a slider, nothing could group a mechanism's gains
 * together, and nothing could stop a mis-typed {@code kP} of 500 where 5 was meant.
 *
 * <p>This publishes the missing half, once, as JSON at {@code /Catalyst/Tunables/.manifest}:
 *
 * <pre>
 * [{"key":"Elevator/kP","name":"kP","group":"Elevator","unit":"","min":0,"max":50,"step":0.5}, ...]
 * </pre>
 *
 * <h2>Registering</h2>
 *
 * <p>{@link TunableGains} registers everything it creates, so a robot using the built-in mechanisms
 * gets a complete manifest with no code. A hand-rolled {@link TunableNumber} registers itself with
 * {@link #register}:
 *
 * <pre>{@code
 * TunableNumber shotAngle = new TunableNumber("Shooter/HoodDeg", 38.0);
 * TunablesManifest.register("Shooter/HoodDeg", "Hood angle", "Shooter", "deg", 20, 60, 0.5);
 * }</pre>
 *
 * <h2>Published once, and why that is enough</h2>
 *
 * <p>NetworkTables retains the last value of a topic and sends it to anyone who subscribes later, so a
 * manifest written at boot is still there for a dashboard that connects in the third match. It is
 * republished whenever a new entry is registered, which in practice means a handful of times during
 * construction and then never — the same pattern {@code RobotIdentity} uses for the spec sheet.
 *
 * <p>Ranges are advisory. A dashboard should let a technician go outside them, because the whole point
 * of a tunable is to find out what the right value is, and a range is a guess about where to look.
 *
 * @since 2.0.0
 */
public final class TunablesManifest {

    private TunablesManifest() {}

    /** One tunable's metadata. */
    public record Entry(String key, String name, String group, String unit,
                        double min, double max, double step) {}

    /** Insertion-ordered so a mechanism's gains stay together in the manifest as well as in the UI. */
    private static final Map<String, Entry> ENTRIES = new LinkedHashMap<>();

    /** The topic, without the {@code /Catalyst/} root. */
    public static final String KEY = "Tunables/.manifest";

    /**
     * Describe a tunable.
     *
     * <p>Idempotent on the key: registering the same key twice replaces the description rather than
     * duplicating it, so a mechanism constructed twice in a test does not produce a manifest with two
     * of everything.
     *
     * @param key the tunable's key, exactly as it was given to {@link TunableNumber}, without the
     *     {@code Catalyst/Tuning/} prefix
     * @param name what to call it in the UI
     * @param group what to put it beside — the mechanism name, usually
     * @param unit free text; empty for a dimensionless gain
     * @param min advisory low end, or {@link Double#NaN} for no opinion
     * @param max advisory high end, or {@link Double#NaN}
     * @param step a sensible increment, or {@link Double#NaN}
     */
    public static synchronized void register(String key, String name, String group, String unit,
                                             double min, double max, double step) {
        ENTRIES.put(key, new Entry(key, name, group, unit, min, max, step));
        publish();
    }

    /** Describe a dimensionless tunable with no declared range. */
    public static void register(String key, String name, String group) {
        register(key, name, group, "", Double.NaN, Double.NaN, Double.NaN);
    }

    /** Everything registered so far, in registration order. */
    public static synchronized List<Entry> entries() {
        return List.copyOf(ENTRIES.values());
    }

    /** Forget everything. For tests; a robot has no reason to call it. */
    public static synchronized void clear() {
        ENTRIES.clear();
        publish();
    }

    /** Write the manifest. */
    private static void publish() {
        CatalystLog.log(KEY, toJson(ENTRIES.values()));
    }

    /**
     * The manifest as JSON.
     *
     * <p>Hand-rolled rather than through {@code MiniJson}, because a {@code NaN} has no JSON
     * representation and must be omitted: a dashboard reading {@code "min": NaN} would either fail to
     * parse the whole manifest or take NaN as a real bound. An absent field means "no opinion", which
     * is what NaN was standing for.
     */
    static String toJson(Iterable<Entry> entries) {
        StringBuilder sb = new StringBuilder("[");
        boolean first = true;
        for (Entry e : entries) {
            if (!first) {
                sb.append(',');
            }
            first = false;
            sb.append("{\"key\":").append(quote(e.key()))
                    .append(",\"name\":").append(quote(e.name()))
                    .append(",\"group\":").append(quote(e.group()));
            if (!e.unit().isEmpty()) {
                sb.append(",\"unit\":").append(quote(e.unit()));
            }
            appendIfFinite(sb, "min", e.min());
            appendIfFinite(sb, "max", e.max());
            appendIfFinite(sb, "step", e.step());
            sb.append('}');
        }
        return sb.append(']').toString();
    }

    private static void appendIfFinite(StringBuilder sb, String field, double value) {
        if (Double.isFinite(value)) {
            sb.append(",\"").append(field).append("\":").append(trim(value));
        }
    }

    /** {@code 5.0} prints as {@code 5}, so the manifest does not read as though it were measured. */
    private static String trim(double v) {
        if (v == Math.rint(v) && Math.abs(v) < 1e15) {
            return Long.toString((long) v);
        }
        return Double.toString(v);
    }

    private static String quote(String s) {
        StringBuilder sb = new StringBuilder(s.length() + 2).append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"' -> sb.append("\\\"");
                case '\\' -> sb.append("\\\\");
                case '\n' -> sb.append("\\n");
                case '\r' -> sb.append("\\r");
                case '\t' -> sb.append("\\t");
                default -> {
                    if (c < 0x20) {
                        sb.append(String.format("\\u%04x", (int) c));
                    } else {
                        sb.append(c);
                    }
                }
            }
        }
        return sb.append('"').toString();
    }

    /** A gain's conventional advisory range, so {@link TunableGains} does not have to invent one. */
    static List<Entry> describeGains(String prefix, String group, boolean motionMagic) {
        List<Entry> out = new ArrayList<>();
        // These bounds are where to start looking, not limits. kP spans orders of magnitude between a
        // duty-cycle loop and a torque-current one, so it gets the widest range of the three.
        out.add(new Entry(prefix + "/kP", "kP", group, "", 0, 200, 0.5));
        out.add(new Entry(prefix + "/kI", "kI", group, "", 0, 10, 0.01));
        out.add(new Entry(prefix + "/kD", "kD", group, "", 0, 20, 0.01));
        out.add(new Entry(prefix + "/kS", "kS", group, "V or A", 0, 20, 0.01));
        out.add(new Entry(prefix + "/kV", "kV", group, "per unit/s", 0, 5, 0.001));
        out.add(new Entry(prefix + "/kA", "kA", group, "per unit/s²", 0, 5, 0.001));
        out.add(new Entry(prefix + "/kG", "kG", group, "V or A", -20, 20, 0.01));
        if (motionMagic) {
            out.add(new Entry(prefix + "/MM/CruiseVelocity", "Cruise velocity", group, "units/s", 0, Double.NaN, 0.5));
            out.add(new Entry(prefix + "/MM/Acceleration", "Acceleration", group, "units/s²", 0, Double.NaN, 1));
            out.add(new Entry(prefix + "/MM/Jerk", "Jerk", group, "units/s³", 0, Double.NaN, 10));
        }
        return out;
    }

    /** Register a whole gain bundle. Called by {@link TunableGains}; teams do not need this. */
    static synchronized void registerGains(String prefix, boolean motionMagic) {
        // The group is the prefix's first segment: "Elevator/Slot0" groups under "Elevator".
        int slash = prefix.indexOf('/');
        String group = slash < 0 ? prefix : prefix.substring(0, slash);
        for (Entry e : describeGains(prefix, group, motionMagic)) {
            ENTRIES.put(e.key(), e);
        }
        publish();
    }
}
