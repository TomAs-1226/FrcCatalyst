package frc.lib.catalyst.physics.data;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.OptionalDouble;

/**
 * Lines several signals up onto one clock so their residuals mean something.
 *
 * <p>Register each signal once, telling the synchronizer how late that source typically reports.
 * From then on {@link #record(String, double, double)} stamps the sample at <em>capture</em> time — the
 * moment the sensor actually measured it — rather than the moment your code got around to reading
 * it. {@link #snapshotAt(double)} then interpolates every signal to a single instant.
 *
 * <pre>{@code
 * TimestampSynchronizer sync = new TimestampSynchronizer(50);
 * sync.register("yawRate", 0.002);        // gyro reports ~2 ms late
 * sync.register("moduleSpeed", 0.010);    // CAN status frame, ~10 ms late
 *
 * // each loop, at the time you read them:
 * double now = Timer.getFPGATimestamp();
 * sync.record("yawRate", gyro.getRate(), now);
 * sync.record("moduleSpeed", module.getVelocity(), now);
 *
 * // compare them honestly:
 * SignalSnapshot at = sync.snapshotAt(sync.latestCommonTimestamp());
 * }</pre>
 *
 * <p>{@link #latestCommonTimestamp()} is the newest instant every registered signal can answer for —
 * which is set by the <em>slowest</em> source. Asking for anything newer than that means at least one
 * signal has to be extrapolated, and this class will not do that; it reports the value as missing
 * instead.
 *
 * @since 1.5.0
 */
public final class TimestampSynchronizer {

    private final int capacityPerSignal;
    private final Map<String, SignalBuffer> buffers = new LinkedHashMap<>();
    private final Map<String, Double> latencies = new LinkedHashMap<>();

    /**
     * @param capacityPerSignal samples of history kept per signal; at 50 Hz, 50 is about one second
     */
    public TimestampSynchronizer(int capacityPerSignal) {
        this.capacityPerSignal = capacityPerSignal;
    }

    /** A synchronizer holding about one second of history at a 50 Hz loop. */
    public TimestampSynchronizer() {
        this(50);
    }

    /**
     * Declare a signal and how stale its readings are when you get them.
     *
     * @param name             the key used by {@link #record(String, double, double)} and
     *                         {@link SignalSnapshot#get(String)}
     * @param latencySeconds   how long before you read it the value was actually true; {@code 0} for
     *                         a direct read, larger for a value that crossed a CAN or network hop
     * @return this, for chaining
     */
    public TimestampSynchronizer register(String name, double latencySeconds) {
        if (latencySeconds < 0) {
            throw new IllegalArgumentException("latencySeconds must be >= 0 for '" + name + "'");
        }
        buffers.put(name, new SignalBuffer(capacityPerSignal));
        latencies.put(name, latencySeconds);
        return this;
    }

    /** Declare a signal with no measurable latency. */
    public TimestampSynchronizer register(String name) {
        return register(name, 0.0);
    }

    /**
     * Record a reading taken now, back-dated by the signal's registered latency.
     *
     * @param name      a signal passed to {@link #register}
     * @param value     the reading
     * @param readAtSeconds when your code read it, on the FPGA clock
     * @throws IllegalArgumentException if {@code name} was never registered
     */
    public void record(String name, double value, double readAtSeconds) {
        SignalBuffer buffer = requireBuffer(name);
        buffer.add(readAtSeconds - latencies.get(name), value);
    }

    /**
     * Record a reading whose capture time you already know exactly — a vision frame that carries its
     * own timestamp, for instance. The registered latency is <b>not</b> applied, because the source
     * already accounted for it.
     */
    public void recordAtCaptureTime(String name, double value, double captureTimeSeconds) {
        requireBuffer(name).add(captureTimeSeconds, value);
    }

    /**
     * The newest instant at which every registered signal has data — the limit of what
     * {@link #snapshotAt(double)} can answer completely. Zero when any signal is still empty.
     */
    public double latestCommonTimestamp() {
        if (buffers.isEmpty()) return 0.0;
        double earliestLatest = Double.POSITIVE_INFINITY;
        for (SignalBuffer buffer : buffers.values()) {
            if (buffer.isEmpty()) return 0.0;
            earliestLatest = Math.min(earliestLatest, buffer.latestTimestamp());
        }
        return earliestLatest;
    }

    /**
     * Every signal interpolated to {@code timestampSeconds}. Signals that cannot cover that instant
     * are simply absent from the result — check with {@link SignalSnapshot#has(String)}.
     */
    public SignalSnapshot snapshotAt(double timestampSeconds) {
        Map<String, Double> resolved = new LinkedHashMap<>();
        for (Map.Entry<String, SignalBuffer> entry : buffers.entrySet()) {
            OptionalDouble value = entry.getValue().sampleAt(timestampSeconds);
            if (value.isPresent()) resolved.put(entry.getKey(), value.getAsDouble());
        }
        return new SignalSnapshot(timestampSeconds, resolved);
    }

    /** Convenience for {@code snapshotAt(latestCommonTimestamp())}. */
    public SignalSnapshot latestSnapshot() {
        return snapshotAt(latestCommonTimestamp());
    }

    /** The raw history of one signal, for a caller that wants more than a single instant. */
    public SignalBuffer buffer(String name) {
        return requireBuffer(name);
    }

    /** Drop all history for every signal, keeping the registrations. */
    public void clear() {
        buffers.values().forEach(SignalBuffer::clear);
    }

    private SignalBuffer requireBuffer(String name) {
        SignalBuffer buffer = buffers.get(name);
        if (buffer == null) {
            throw new IllegalArgumentException("signal '" + name + "' was never registered");
        }
        return buffer;
    }
}
