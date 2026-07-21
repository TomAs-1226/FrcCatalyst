package frc.lib.catalyst.statemachine;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

/**
 * A bounded, newest-first ring of {@link TransitionRecord}s.
 *
 * <p>Deliberately an <em>instance</em> rather than the static-state style used by
 * {@code HealthHistory}: two state machines on the same robot — a superstructure and a climber,
 * say — must not be able to corrupt each other's history.
 *
 * <p>Contains no clock of its own. Timestamps arrive inside the records, which is what keeps this
 * class usable in a unit test with a fake clock.
 *
 * @param <S> the machine's state enum
 * @since 1.2.0
 */
public final class TransitionHistory<S extends Enum<S>> {

    private final Deque<TransitionRecord<S>> records = new ArrayDeque<>();
    private int capacity;

    /**
     * @param capacity maximum entries retained; values below 1 are clamped to 1
     */
    public TransitionHistory(int capacity) {
        this.capacity = Math.max(1, capacity);
    }

    /** Add a record as the newest entry, evicting the oldest if the ring is full. */
    public synchronized void record(TransitionRecord<S> record) {
        if (record == null) return;
        records.addFirst(record);
        trim();
    }

    /** An immutable newest-first copy. Safe to iterate while the machine keeps running. */
    public synchronized List<TransitionRecord<S>> snapshot() {
        return List.copyOf(records);
    }

    /** The newest record, or {@code null} when nothing has happened yet. */
    public synchronized TransitionRecord<S> newest() {
        return records.peekFirst();
    }

    /** Newest-first serialized lines, ready to publish as a {@code String[]}. */
    public synchronized String[] serialized() {
        List<String> out = new ArrayList<>(records.size());
        for (TransitionRecord<S> r : records) out.add(r.serialize());
        return out.toArray(new String[0]);
    }

    /** Drop everything. */
    public synchronized void clear() {
        records.clear();
    }

    /** Resize the ring, evicting oldest entries immediately if it shrank. */
    public synchronized void setCapacity(int capacity) {
        this.capacity = Math.max(1, capacity);
        trim();
    }

    /** Current number of retained records. */
    public synchronized int size() {
        return records.size();
    }

    /** Configured maximum. */
    public synchronized int capacity() {
        return capacity;
    }

    private void trim() {
        while (records.size() > capacity) records.removeLast();
    }
}
