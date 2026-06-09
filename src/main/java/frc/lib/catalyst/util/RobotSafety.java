package frc.lib.catalyst.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Cross-mechanism safety watchdog driven by {@link HealthMonitor}.
 *
 * <p>When too many {@code ERROR}-severity (or, optionally, {@code WARN})
 * health checks fire simultaneously, the safety layer "trips" — a single
 * boolean that team code can read to bail out of teleop / auto cleanly.
 * The library never forcibly disables motors itself; the trip signal is
 * advisory so each team decides exactly what "all-stop" means for their
 * robot.
 *
 * <p>The watchdog also de-bounces transient bursts: a few ERRORs that
 * clear within a half-second won't trip, but a sustained group will.
 *
 * <p>The state is published to NetworkTables under
 * {@code /Catalyst/Safety/...} so dashboards (including the Health
 * Dashboard at {@code docs/tools/health/}) light up the same way teams'
 * driver station does.
 *
 * <p>Example usage in {@code Robot.robotInit()}:
 * <pre>{@code
 * RobotSafety.configure(
 *     RobotSafety.Config.builder()
 *         .maxConcurrentErrors(2)
 *         .debounce(0.25)
 *         .onTrip(() -> {
 *             drive.stop();
 *             superstructure.stow();
 *             leds.fire();
 *         })
 *         .build());
 *
 * // ...in command bindings...
 * Trigger tripped = new Trigger(RobotSafety::isTripped);
 * tripped.onTrue(Commands.runOnce(drive::stop, drive));
 * }</pre>
 *
 * <p>If {@link #configure(Config)} is never called, the watchdog stays
 * disabled and adds zero overhead — the {@link HealthMonitor} just calls
 * a single empty {@code tick()} per loop.
 */
public final class RobotSafety {

    private static Config config = null;
    private static boolean tripped = false;
    private static double overThresholdSince = -1;
    private static double underThresholdSince = -1;
    private static String lastReason = "";

    private static BooleanPublisher trippedPub;
    private static StringPublisher reasonPub;
    private static IntegerPublisher errorPub;
    private static IntegerPublisher warnPub;

    private RobotSafety() {}

    /** Install (or replace) the safety policy. */
    public static synchronized void configure(Config c) {
        config = c;
        if (trippedPub == null) {
            NetworkTable table = NetworkTableInstance.getDefault()
                    .getTable("Catalyst").getSubTable("Safety");
            trippedPub = table.getBooleanTopic("Tripped").publish();
            reasonPub = table.getStringTopic("Reason").publish();
            errorPub = table.getIntegerTopic("ErrorCount").publish();
            warnPub = table.getIntegerTopic("WarnCount").publish();
        }
        trippedPub.set(tripped);
        reasonPub.set(lastReason);
    }

    /** True if the watchdog has tripped (sustained over-threshold faults). */
    public static boolean isTripped() {
        return tripped;
    }

    /**
     * A WPILib {@link Trigger} that fires whenever the watchdog is tripped.
     * Convenient for binding emergency-stop commands directly in
     * {@code RobotContainer.configureBindings()}:
     *
     * <pre>{@code
     * RobotSafety.trippedTrigger().onTrue(drive.stopCommand());
     * }</pre>
     */
    public static Trigger trippedTrigger() {
        return new Trigger(RobotSafety::isTripped);
    }

    /**
     * Human-readable explanation of why the watchdog tripped, or empty when
     * the system is healthy. Useful to display on a driver-station widget.
     */
    public static String reason() {
        return lastReason;
    }

    /**
     * Manually trip the watchdog with a reason. Use for conditions outside
     * the health-check system — e.g. a brownout-prediction or a low-battery
     * trigger. Runs the configured {@code onTrip} callback. No-op if already
     * tripped.
     */
    public static synchronized void trip(String reason) {
        if (tripped) return;
        tripped = true;
        lastReason = reason == null ? "manual trip" : reason;
        if (trippedPub != null) {
            trippedPub.set(true);
            reasonPub.set(lastReason);
        }
        if (config != null && config.onTrip != null) {
            try { config.onTrip.run(); } catch (Throwable ignored) {}
        }
    }

    /** Command form of {@link #trip(String)} for use in bindings. */
    public static Command tripCommand(String reason) {
        return Commands.runOnce(() -> trip(reason)).ignoringDisable(true);
    }

    /**
     * Manually clear the trip. The watchdog will re-trip if the underlying
     * faults haven't actually cleared, so it's safe to wire this to a driver
     * button — at worst, you waste one button press.
     */
    public static synchronized void reset() {
        tripped = false;
        lastReason = "";
        if (trippedPub != null) trippedPub.set(false);
        if (reasonPub != null) reasonPub.set("");
        if (config != null && config.onReset != null) {
            try { config.onReset.run(); } catch (Throwable ignored) {}
        }
    }

    /**
     * Called by {@link HealthMonitor#update()} every loop. Cheap when no
     * config has been installed.
     */
    public static synchronized void tick(int errorCount, int warnCount) {
        if (config == null) return;

        double now = Timer.getFPGATimestamp();
        boolean over =
                errorCount >= config.maxConcurrentErrors
                        || (config.maxConcurrentWarns > 0 && warnCount >= config.maxConcurrentWarns);

        if (over) {
            if (overThresholdSince < 0) overThresholdSince = now;
            underThresholdSince = -1;

            if (!tripped && now - overThresholdSince >= config.debounceSeconds) {
                tripped = true;
                lastReason = buildReason(errorCount, warnCount);
                if (trippedPub != null) {
                    trippedPub.set(true);
                    reasonPub.set(lastReason);
                }
                if (config.onTrip != null) {
                    try { config.onTrip.run(); } catch (Throwable ignored) {}
                }
            }
        } else {
            if (underThresholdSince < 0) underThresholdSince = now;
            overThresholdSince = -1;

            if (tripped && config.autoReset && now - underThresholdSince >= config.autoResetSeconds) {
                tripped = false;
                lastReason = "";
                if (trippedPub != null) {
                    trippedPub.set(false);
                    reasonPub.set("");
                }
                if (config.onReset != null) {
                    try { config.onReset.run(); } catch (Throwable ignored) {}
                }
            }
        }

        if (errorPub != null) errorPub.set(errorCount);
        if (warnPub != null) warnPub.set(warnCount);
    }

    private static String buildReason(int errorCount, int warnCount) {
        if (errorCount >= config.maxConcurrentErrors) {
            return errorCount + " error-level health checks firing (limit "
                    + config.maxConcurrentErrors + ")";
        }
        return warnCount + " warn-level health checks firing (limit "
                + config.maxConcurrentWarns + ")";
    }

    // ===========================================
    //                  CONFIG
    // ===========================================

    public static class Config {
        final int maxConcurrentErrors;
        final int maxConcurrentWarns;
        final double debounceSeconds;
        final boolean autoReset;
        final double autoResetSeconds;
        final Runnable onTrip;
        final Runnable onReset;

        private Config(Builder b) {
            this.maxConcurrentErrors = b.maxConcurrentErrors;
            this.maxConcurrentWarns = b.maxConcurrentWarns;
            this.debounceSeconds = b.debounceSeconds;
            this.autoReset = b.autoReset;
            this.autoResetSeconds = b.autoResetSeconds;
            this.onTrip = b.onTrip;
            this.onReset = b.onReset;
        }

        public static Builder builder() { return new Builder(); }

        public static class Builder {
            private int maxConcurrentErrors = 1;
            private int maxConcurrentWarns = 0; // disabled by default
            private double debounceSeconds = 0.25;
            private boolean autoReset = false;
            private double autoResetSeconds = 2.0;
            private Runnable onTrip;
            private Runnable onReset;

            /** Trip after this many ERROR-severity checks fire simultaneously. Default 1. */
            public Builder maxConcurrentErrors(int n) { this.maxConcurrentErrors = n; return this; }

            /** Also trip when this many WARNs fire at once. Default 0 = disabled. */
            public Builder maxConcurrentWarns(int n) { this.maxConcurrentWarns = n; return this; }

            /** Faults must persist for this long before tripping. Default 0.25 s. */
            public Builder debounce(double seconds) { this.debounceSeconds = seconds; return this; }

            /**
             * Auto-clear the trip once the system has been healthy for
             * {@code clearAfter} seconds. Off by default — usually you want
             * a human-in-the-loop reset.
             */
            public Builder autoReset(double clearAfter) {
                this.autoReset = true;
                this.autoResetSeconds = clearAfter;
                return this;
            }

            /** Run once when the watchdog trips. */
            public Builder onTrip(Runnable r) { this.onTrip = r; return this; }

            /** Run once when the watchdog clears (manual {@link #reset()} or auto-reset). */
            public Builder onReset(Runnable r) { this.onReset = r; return this; }

            public Config build() { return new Config(this); }
        }
    }
}
