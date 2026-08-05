package frc.lib.catalyst.physics.data;

import java.util.OptionalDouble;

/**
 * A fixed-size ring buffer of timestamped numbers that can be asked "what was this value at time T?".
 *
 * <p>Physics Core needs this because measurements do not arrive together. A camera reports a pose
 * that was true 40 ms ago; the gyro reported 2 ms ago; the modules were sampled somewhere in
 * between. Comparing them means being able to look each signal up at a common instant, which is
 * what {@link #sampleAt(double)} does — linear interpolation between the two straddling samples,
 * and an empty result when the request falls outside what is still in the buffer.
 *
 * <p>The buffer never allocates after construction: it is two {@code double[]} arrays and an index.
 * That matters when several of these run every 20 ms loop.
 *
 * <pre>{@code
 * SignalBuffer yawRate = new SignalBuffer(50);   // ~1 second at 50 Hz
 *
 * yawRate.add(Timer.getFPGATimestamp(), gyro.getRate());
 *
 * OptionalDouble atCapture = yawRate.sampleAt(cameraTimestamp);
 * }</pre>
 *
 * <p>Not thread-safe; like the rest of Catalyst it expects to be driven from the main robot loop.
 *
 * @since 1.5.0
 */
public final class SignalBuffer {

    private final double[] timestamps;
    private final double[] values;
    /** Index the next sample will be written to. */
    private int head = 0;
    private int size = 0;

    /**
     * @param capacity how many samples to keep; at a 50 Hz loop, 50 is about one second of history
     *                 (must be at least 2, since interpolation needs two points to work with)
     */
    public SignalBuffer(int capacity) {
        if (capacity < 2) throw new IllegalArgumentException("capacity must be >= 2 (got " + capacity + ")");
        this.timestamps = new double[capacity];
        this.values = new double[capacity];
    }

    /**
     * Record a sample. Samples must arrive in non-decreasing timestamp order; one that goes backwards
     * is dropped rather than corrupting the ordering that {@link #sampleAt(double)} relies on. If your
     * clock genuinely restarted, call {@link #clear()}.
     *
     * @return true if the sample was stored
     */
    public boolean add(double timestampSeconds, double value) {
        if (size > 0 && timestampSeconds < latestTimestamp()) return false;
        timestamps[head] = timestampSeconds;
        values[head] = value;
        head = (head + 1) % timestamps.length;
        if (size < timestamps.length) size++;
        return true;
    }

    /**
     * The value at {@code timestampSeconds}, linearly interpolated between the samples on either
     * side. Empty when the buffer is empty or the time is outside the range it still holds — an
     * honest "I do not know" rather than an extrapolated guess.
     */
    public OptionalDouble sampleAt(double timestampSeconds) {
        if (size == 0) return OptionalDouble.empty();
        if (timestampSeconds < oldestTimestamp() || timestampSeconds > latestTimestamp()) {
            return OptionalDouble.empty();
        }
        if (size == 1) return OptionalDouble.of(values[indexOf(0)]);

        // Ordered by construction, so walk back from the newest until we straddle the request. In a
        // robot loop the answer is almost always in the last few samples, so this beats a binary
        // search on both simplicity and typical cost.
        for (int i = size - 1; i > 0; i--) {
            double newer = timestamps[indexOf(i)];
            double older = timestamps[indexOf(i - 1)];
            if (timestampSeconds >= older && timestampSeconds <= newer) {
                double span = newer - older;
                if (span <= 0) return OptionalDouble.of(values[indexOf(i)]);
                double t = (timestampSeconds - older) / span;
                double a = values[indexOf(i - 1)];
                double b = values[indexOf(i)];
                return OptionalDouble.of(a + t * (b - a));
            }
        }
        return OptionalDouble.of(values[indexOf(0)]);
    }

    /**
     * Average rate of change across the whole buffer, in units per second. Empty until there are two
     * samples separated in time. This is a chord slope, not a filtered derivative — it is deliberately
     * coarse, meant for "how fast has this been drifting", not for a control loop.
     */
    public OptionalDouble averageRate() {
        if (size < 2) return OptionalDouble.empty();
        double span = latestTimestamp() - oldestTimestamp();
        if (span <= 0) return OptionalDouble.empty();
        return OptionalDouble.of((latest() - values[indexOf(0)]) / span);
    }

    /** The most recent value. Zero if nothing has been recorded yet. */
    public double latest() {
        return size == 0 ? 0.0 : values[indexOf(size - 1)];
    }

    /** Timestamp of the most recent sample. Zero if nothing has been recorded yet. */
    public double latestTimestamp() {
        return size == 0 ? 0.0 : timestamps[indexOf(size - 1)];
    }

    /** Timestamp of the oldest sample still held. Zero if nothing has been recorded yet. */
    public double oldestTimestamp() {
        return size == 0 ? 0.0 : timestamps[indexOf(0)];
    }

    /** How many samples are currently held, up to the capacity. */
    public int size() {
        return size;
    }

    /** True when nothing has been recorded, or everything was cleared. */
    public boolean isEmpty() {
        return size == 0;
    }

    /** The number of samples this buffer can hold. */
    public int capacity() {
        return timestamps.length;
    }

    /** Drop every sample. Use this after a clock reset or a robot-code restart. */
    public void clear() {
        head = 0;
        size = 0;
    }

    /** Maps a logical index (0 = oldest) to its slot in the ring. */
    private int indexOf(int logical) {
        int start = (head - size + timestamps.length) % timestamps.length;
        return (start + logical) % timestamps.length;
    }
}
