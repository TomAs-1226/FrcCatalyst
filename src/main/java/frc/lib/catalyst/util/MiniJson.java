package frc.lib.catalyst.util;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The smallest JSON that is still correct: maps, lists, strings, numbers, booleans and null.
 *
 * <p>Catalyst keeps a few small files on the robot - the motor history is one - and reads them
 * back on the next boot. That wants JSON, because everything else that will ever read those files
 * (the Console, the App, a person with a text editor) speaks it. It does not want a JSON library:
 * the library's dependencies are declared flat for GradleRIO, so every jar it needs is one more
 * line in every team's vendordep, and a parser is a page of code.
 *
 * <p>Writes {@code Map} (in insertion order), {@code List}, {@code String}, {@code Number},
 * {@code Boolean}, {@code null}, and arrays of primitives. Reads back into {@code LinkedHashMap},
 * {@code ArrayList}, {@code String}, {@code Double} (every number - a caller that stored an integer
 * casts it back), {@code Boolean} and {@code null}. Whitespace is accepted anywhere JSON allows it;
 * comments are not JSON and are rejected.
 *
 * @since 2.0.0
 */
public final class MiniJson {
    private MiniJson() {}

    // ------------------------------------------------------------------ writing

    /** Serialise a value. Maps and lists nest; anything else goes through {@link Object#toString}. */
    public static String write(Object value) {
        StringBuilder sb = new StringBuilder();
        write(sb, value, 0, false);
        return sb.toString();
    }

    /** Serialise with newlines and two-space indentation, for a file a person may open. */
    public static String pretty(Object value) {
        StringBuilder sb = new StringBuilder();
        write(sb, value, 0, true);
        sb.append('\n');
        return sb.toString();
    }

    private static void write(StringBuilder sb, Object v, int depth, boolean pretty) {
        if (v == null) {
            sb.append("null");
        } else if (v instanceof String s) {
            quote(sb, s);
        } else if (v instanceof Boolean b) {
            sb.append(b ? "true" : "false");
        } else if (v instanceof Number n) {
            number(sb, n);
        } else if (v instanceof Map<?, ?> m) {
            sb.append('{');
            boolean first = true;
            for (Map.Entry<?, ?> e : m.entrySet()) {
                if (!first) {
                    sb.append(',');
                }
                first = false;
                newline(sb, depth + 1, pretty);
                quote(sb, String.valueOf(e.getKey()));
                sb.append(pretty ? ": " : ":");
                write(sb, e.getValue(), depth + 1, pretty);
            }
            if (!first) {
                newline(sb, depth, pretty);
            }
            sb.append('}');
        } else if (v instanceof Iterable<?> it) {
            sb.append('[');
            boolean first = true;
            for (Object o : it) {
                if (!first) {
                    sb.append(',');
                }
                first = false;
                newline(sb, depth + 1, pretty);
                write(sb, o, depth + 1, pretty);
            }
            if (!first) {
                newline(sb, depth, pretty);
            }
            sb.append(']');
        } else if (v instanceof double[] a) {
            sb.append('[');
            for (int i = 0; i < a.length; i++) {
                if (i > 0) {
                    sb.append(',');
                }
                number(sb, a[i]);
            }
            sb.append(']');
        } else if (v instanceof long[] a) {
            sb.append('[');
            for (int i = 0; i < a.length; i++) {
                if (i > 0) {
                    sb.append(',');
                }
                sb.append(a[i]);
            }
            sb.append(']');
        } else if (v instanceof int[] a) {
            sb.append('[');
            for (int i = 0; i < a.length; i++) {
                if (i > 0) {
                    sb.append(',');
                }
                sb.append(a[i]);
            }
            sb.append(']');
        } else if (v instanceof Object[] a) {
            write(sb, List.of(a), depth, pretty);
        } else {
            quote(sb, v.toString());
        }
    }

    private static void number(StringBuilder sb, Number n) {
        double d = n.doubleValue();
        if (n instanceof Integer || n instanceof Long || n instanceof Short || n instanceof Byte) {
            sb.append(n.longValue());
        } else if (Double.isNaN(d) || Double.isInfinite(d)) {
            sb.append("null");   // JSON has no NaN; null is the honest value
        } else if (d == Math.rint(d) && Math.abs(d) < 1e15) {
            sb.append((long) d);
        } else {
            sb.append(d);
        }
    }

    private static void newline(StringBuilder sb, int depth, boolean pretty) {
        if (!pretty) {
            return;
        }
        sb.append('\n');
        for (int i = 0; i < depth; i++) {
            sb.append("  ");
        }
    }

    private static void quote(StringBuilder sb, String s) {
        sb.append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"' -> sb.append("\\\"");
                case '\\' -> sb.append("\\\\");
                case '\n' -> sb.append("\\n");
                case '\r' -> sb.append("\\r");
                case '\t' -> sb.append("\\t");
                case '\b' -> sb.append("\\b");
                case '\f' -> sb.append("\\f");
                default -> {
                    if (c < 0x20) {
                        sb.append(String.format("\\u%04x", (int) c));
                    } else {
                        sb.append(c);
                    }
                }
            }
        }
        sb.append('"');
    }

    // ------------------------------------------------------------------ reading

    /**
     * Parse a document.
     *
     * @throws IllegalArgumentException with the offset of the problem, for anything that is not JSON
     */
    public static Object read(String text) {
        Parser p = new Parser(text);
        p.ws();
        Object v = p.value();
        p.ws();
        if (p.i != text.length()) {
            throw p.error("trailing content");
        }
        return v;
    }

    /** {@link #read} that must be an object, for the common case of a file with named fields. */
    @SuppressWarnings("unchecked")
    public static Map<String, Object> readObject(String text) {
        Object v = read(text);
        if (!(v instanceof Map)) {
            throw new IllegalArgumentException("expected a JSON object at the top level");
        }
        return (Map<String, Object>) v;
    }

    private static final class Parser {
        private final String s;
        private int i = 0;

        Parser(String s) {
            this.s = s;
        }

        IllegalArgumentException error(String what) {
            return new IllegalArgumentException("JSON: " + what + " at offset " + i);
        }

        void ws() {
            while (i < s.length()) {
                char c = s.charAt(i);
                if (c == ' ' || c == '\n' || c == '\r' || c == '\t') {
                    i++;
                } else {
                    break;
                }
            }
        }

        Object value() {
            if (i >= s.length()) {
                throw error("unexpected end");
            }
            char c = s.charAt(i);
            switch (c) {
                case '{': return object();
                case '[': return array();
                case '"': return string();
                case 't': literal("true"); return Boolean.TRUE;
                case 'f': literal("false"); return Boolean.FALSE;
                case 'n': literal("null"); return null;
                default:
                    if (c == '-' || (c >= '0' && c <= '9')) {
                        return number();
                    }
                    throw error("unexpected character '" + c + "'");
            }
        }

        private void literal(String word) {
            if (!s.startsWith(word, i)) {
                throw error("expected " + word);
            }
            i += word.length();
        }

        private Map<String, Object> object() {
            Map<String, Object> m = new LinkedHashMap<>();
            i++;   // {
            ws();
            if (peek() == '}') {
                i++;
                return m;
            }
            while (true) {
                ws();
                if (peek() != '"') {
                    throw error("expected a key");
                }
                String k = string();
                ws();
                if (peek() != ':') {
                    throw error("expected ':'");
                }
                i++;
                ws();
                m.put(k, value());
                ws();
                char c = peek();
                if (c == ',') {
                    i++;
                } else if (c == '}') {
                    i++;
                    return m;
                } else {
                    throw error("expected ',' or '}'");
                }
            }
        }

        private List<Object> array() {
            List<Object> l = new ArrayList<>();
            i++;   // [
            ws();
            if (peek() == ']') {
                i++;
                return l;
            }
            while (true) {
                ws();
                l.add(value());
                ws();
                char c = peek();
                if (c == ',') {
                    i++;
                } else if (c == ']') {
                    i++;
                    return l;
                } else {
                    throw error("expected ',' or ']'");
                }
            }
        }

        private String string() {
            i++;   // opening quote
            StringBuilder sb = new StringBuilder();
            while (true) {
                if (i >= s.length()) {
                    throw error("unterminated string");
                }
                char c = s.charAt(i++);
                if (c == '"') {
                    return sb.toString();
                }
                if (c != '\\') {
                    sb.append(c);
                    continue;
                }
                if (i >= s.length()) {
                    throw error("unterminated escape");
                }
                char e = s.charAt(i++);
                switch (e) {
                    case '"' -> sb.append('"');
                    case '\\' -> sb.append('\\');
                    case '/' -> sb.append('/');
                    case 'n' -> sb.append('\n');
                    case 'r' -> sb.append('\r');
                    case 't' -> sb.append('\t');
                    case 'b' -> sb.append('\b');
                    case 'f' -> sb.append('\f');
                    case 'u' -> {
                        if (i + 4 > s.length()) {
                            throw error("short \\u escape");
                        }
                        sb.append((char) Integer.parseInt(s.substring(i, i + 4), 16));
                        i += 4;
                    }
                    default -> throw error("bad escape '\\" + e + "'");
                }
            }
        }

        private Double number() {
            int start = i;
            if (peek() == '-') {
                i++;
            }
            while (i < s.length()) {
                char c = s.charAt(i);
                if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') {
                    i++;
                } else {
                    break;
                }
            }
            try {
                return Double.parseDouble(s.substring(start, i));
            } catch (NumberFormatException ex) {
                throw error("bad number '" + s.substring(start, i) + "'");
            }
        }

        private char peek() {
            if (i >= s.length()) {
                throw error("unexpected end");
            }
            return s.charAt(i);
        }
    }
}
