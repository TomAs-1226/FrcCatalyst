package frc.lib.catalyst.driver;

import frc.lib.catalyst.logging.CatalystLog;

import java.util.ArrayList;
import java.util.List;

/**
 * What each control does, published so a pit dashboard can show the driver's map.
 *
 * <h2>Why</h2>
 *
 * <p>Catalyst Tab has a controls screen. On a stock Catalyst robot it is empty, and its empty state
 * prints the topic it wanted and an example of the JSON that would fill it — because no library class
 * ever published one. The only robot that filled that screen was Catalyst X1, which hand-rolled a
 * publisher of its own.
 *
 * <p>The screen is worth filling. The question it answers — "which button does the thing I need right
 * now" — gets asked in a pit, by whoever is standing there when the usual driver is not, thirty
 * seconds before a match. The answer normally lives in someone's head or on a printout that is one
 * revision behind the code. It is in the code already; it was just never said out loud.
 *
 * <h2>Use</h2>
 *
 * <p>One line beside each binding, in whatever method configures them:
 *
 * <pre>{@code
 * driver.a().onTrue(superstructure.score());
 * ControlsManifest.bind("A", "Score", "driver");
 *
 * driver.leftBumper().and(driver.rightBumper()).whileTrue(climber.deploy());
 * ControlsManifest.combo("LB + RB", "Deploy climber", "driver");
 * }</pre>
 *
 * <p>Deliberately not automatic. Catalyst could have wrapped the binding helpers and captured this
 * for free, and the reason not to is that the useful field is the description — "Score", "Deploy
 * climber" — and no wrapper can infer it. A generated manifest reading {@code "A" -> "ScoreCommand"}
 * would be worse than none, because it looks complete while telling a pit nothing it could not guess.
 *
 * <p>Published as JSON at {@code /Catalyst/Controls/.manifest}, the shape the tablet already parses:
 *
 * <pre>
 * [{"control":"A","action":"Score","controller":"driver"},
 *  {"control":"LB + RB","action":"Deploy climber","controller":"driver","combo":true}]
 * </pre>
 *
 * <p>Written once per {@code bind}, which is boot-time only in practice, and retained by
 * NetworkTables for any dashboard that connects later.
 *
 * @since 2.0.0
 */
public final class ControlsManifest {

    private ControlsManifest() {}

    /** One binding. */
    public record Binding(String control, String action, String controller, boolean combo) {}

    private static final List<Binding> BINDINGS = new ArrayList<>();

    /** The topic, without the {@code /Catalyst/} root. */
    public static final String KEY = "Controls/.manifest";

    /**
     * Record a binding.
     *
     * @param control what the driver presses, as they would say it: {@code "A"}, {@code "Left stick"},
     *     {@code "POV up"}
     * @param action what it does, in the words someone in a pit would use
     * @param controller which controller it is on: {@code "driver"}, {@code "operator"}, or a name
     */
    public static synchronized void bind(String control, String action, String controller) {
        add(control, action, controller, false);
    }

    /** Record a binding on the driver's controller. */
    public static void bind(String control, String action) {
        bind(control, action, "driver");
    }

    /** Record a binding that needs more than one button held at once. */
    public static synchronized void combo(String control, String action, String controller) {
        add(control, action, controller, true);
    }

    private static void add(String control, String action, String controller, boolean combo) {
        // Replacing rather than appending on a repeated control, so a robot whose bindings are
        // configured twice - which happens in tests, and on a robot that rebuilds its container -
        // does not publish a manifest listing every button twice.
        BINDINGS.removeIf(b -> b.control().equals(control) && b.controller().equals(controller));
        BINDINGS.add(new Binding(control, action, controller, combo));
        publish();
    }

    /** Everything recorded so far. */
    public static synchronized List<Binding> bindings() {
        return List.copyOf(BINDINGS);
    }

    /** Forget everything. For tests. */
    public static synchronized void clear() {
        BINDINGS.clear();
        publish();
    }

    private static void publish() {
        CatalystLog.log(KEY, toJson(BINDINGS));
    }

    static String toJson(List<Binding> bindings) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < bindings.size(); i++) {
            Binding b = bindings.get(i);
            if (i > 0) {
                sb.append(',');
            }
            sb.append("{\"control\":").append(quote(b.control()))
                    .append(",\"action\":").append(quote(b.action()));
            if (!b.controller().isEmpty()) {
                sb.append(",\"controller\":").append(quote(b.controller()));
            }
            if (b.combo()) {
                sb.append(",\"combo\":true");
            }
            sb.append('}');
        }
        return sb.append(']').toString();
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
}
