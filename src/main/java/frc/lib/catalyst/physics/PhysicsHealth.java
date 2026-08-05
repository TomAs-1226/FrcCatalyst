package frc.lib.catalyst.physics;

import java.util.List;
import java.util.Locale;

/**
 * Whether Physics Core itself is working — as distinct from whether the robot is.
 *
 * <p>The most common way an advisory layer fails is quietly: a supplier was never wired up, the
 * accelerometer returns zeros, {@code update()} stopped being called when a command took over. In all
 * three cases the estimates keep coming out, they are just wrong. This makes that visible.
 *
 * <pre>{@code
 * SystemCheck.builder()
 *     .test("Physics Core running", () -> physics.health().isHealthy())
 *     .build();
 * }</pre>
 *
 * @param running               true once at least one update has been folded in
 * @param sampleAgeSeconds      how long since the last update; grows without bound if updates stop
 * @param profile               the compute profile in effect
 * @param degradedInputs        inputs that were expected but are missing, e.g. {@code "accelerometer"}
 * @since 1.5.0
 */
public record PhysicsHealth(
        boolean running,
        double sampleAgeSeconds,
        PhysicsProfile profile,
        List<String> degradedInputs) {

    /** Compact constructor: defensively copies the degraded-input list. */
    public PhysicsHealth {
        degradedInputs = degradedInputs == null ? List.of() : List.copyOf(degradedInputs);
    }

    /**
     * True when Physics Core is running, was updated recently, and has every input it expected.
     * A degraded but running core reports {@code false} here and explains itself in {@link #describe()}
     * — an honest signal is more useful than a green light that means "probably fine".
     */
    public boolean isHealthy() {
        return running && sampleAgeSeconds < 0.5 && degradedInputs.isEmpty();
    }

    /** One line for a dashboard or a self-test result. */
    public String describe() {
        if (!running) return "not running - update() has never been called";
        if (sampleAgeSeconds >= 0.5) {
            return String.format(Locale.ROOT, "stale - last update %.1f s ago", sampleAgeSeconds);
        }
        if (!degradedInputs.isEmpty()) {
            return "degraded - missing " + String.join(", ", degradedInputs);
        }
        return "healthy (" + profile + ")";
    }
}
