package frc.lib.catalyst.util;

import edu.wpi.first.wpilibj.Timer;

import frc.lib.catalyst.logging.CatalystLog;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

/**
 * Fixed-capacity ring buffer of recent {@link HealthCheck} fire / clear
 * edges. The most recent {@code N} (default 100) events are kept in memory
 * and published as a string array through CatalystLog for the Health
 * Dashboard's timeline view.
 *
 * <p>{@link HealthMonitor} feeds events in automatically — teams don't
 * normally interact with this class directly. To query history from the
 * robot code:
 *
 * <pre>{@code
 * for (HealthHistory.Event e : HealthHistory.snapshot()) {
 *     System.out.println(e);
 * }
 * }</pre>
 *
 * <p>Snapshots are returned newest-first.
 */
public final class HealthHistory {

    /** A single transition recorded in the history buffer. */
    public record Event(double timestamp, String subsystem, String id,
                        HealthCheck.Severity severity, Kind kind, String detail) {
        @Override
        public String toString() {
            return String.format("[%7.2fs] %s %-12s %s/%s %s",
                    timestamp, kind, severity, subsystem, id,
                    detail == null ? "" : "— " + detail);
        }

        /** Pipe-delimited line used for NT serialization. */
        String serialize() {
            return String.format("%.3f|%s|%s|%s|%s|%s",
                    timestamp, kind, severity, subsystem, id,
                    detail == null ? "" : detail.replace('|', '_'));
        }
    }

    public enum Kind { FIRED, CLEARED }

    private static final int DEFAULT_CAPACITY = 100;

    private static int capacity = DEFAULT_CAPACITY;
    private static final Deque<Event> events = new ArrayDeque<>();

    private HealthHistory() {}

    /** Resize the buffer. Older events past the new cap are dropped. Default is 100. */
    public static synchronized void setCapacity(int n) {
        if (n < 1) throw new IllegalArgumentException("capacity must be >= 1");
        capacity = n;
        while (events.size() > capacity) events.removeLast();
        republish();
    }

    /** Record an event. Called by {@link HealthMonitor} on every FIRED / CLEARED transition. */
    static synchronized void record(HealthCheck check, Kind kind) {
        Event e = new Event(
                Timer.getFPGATimestamp(),
                check.subsystem(),
                check.id(),
                check.severity(),
                kind,
                check.currentDetail());
        events.addFirst(e);
        while (events.size() > capacity) events.removeLast();
        republish();
    }

    /** Newest-first snapshot of all recorded events. Safe to iterate. */
    public static synchronized List<Event> snapshot() {
        return List.copyOf(events);
    }

    /** Drop every recorded event. Mostly for tests. */
    public static synchronized void clear() {
        events.clear();
        republish();
    }

    private static void republish() {
        String[] out = new String[events.size()];
        int i = 0;
        for (Event e : events) out[i++] = e.serialize();
        CatalystLog.log("Health/History", out);
    }
}
