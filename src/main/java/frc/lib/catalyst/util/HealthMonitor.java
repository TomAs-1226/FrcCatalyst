package frc.lib.catalyst.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.catalyst.hardware.CatalystMotor;

import java.util.ArrayList;
import java.util.List;

/**
 * Singleton registry + per-loop evaluator for {@link HealthCheck}s.
 *
 * <p>Each built-in Catalyst mechanism registers a small set of checks for its
 * own motors (over-current, over-temperature, stall, …). Teams add their own
 * via {@link HealthCheck#builder(String, String)}.
 *
 * <p>{@link #update()} is called automatically from every Catalyst mechanism's
 * periodic loop, so for normal usage teams never need to call it themselves.
 * For unit tests or custom integrations, the method is public.
 *
 * <p>State is published to NetworkTables under
 * {@code /Catalyst/Health/<subsystem>/<id>/{firing,severity,description,detail,firedAt}}.
 * Firing checks are also forwarded to the legacy {@link AlertManager} so any
 * existing dashboard / driver-station integration keeps working.
 */
public final class HealthMonitor {

    private static HealthMonitor instance;

    private final List<HealthCheck> checks = new ArrayList<>();
    private final NetworkTable healthTable;
    private double lastUpdateTs = -1;

    private HealthMonitor() {
        this.healthTable = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("Health");
    }

    public static synchronized HealthMonitor getInstance() {
        if (instance == null) instance = new HealthMonitor();
        return instance;
    }

    /** Register a check. Normal users go through {@link HealthCheck.Builder#register()}. */
    public synchronized void register(HealthCheck check) {
        checks.add(check);
        publishStatic(check);
    }

    /** Currently-registered checks. Snapshot copy — safe to iterate from callers. */
    public synchronized List<HealthCheck> checks() {
        return List.copyOf(checks);
    }

    /**
     * Evaluate every registered check once. Cheap — a check that is stable
     * (consistently false, or consistently firing) does no I/O.
     *
     * <p>This is throttled to at most once every 5&nbsp;ms regardless of how
     * many mechanisms call into it per loop, so a robot with twelve
     * mechanisms only pays the cost once per scheduler tick.
     */
    public synchronized void update() {
        double now = Timer.getFPGATimestamp();
        if (now - lastUpdateTs < 0.005) return;
        lastUpdateTs = now;

        int errorCount = 0, warnCount = 0, infoCount = 0;
        for (HealthCheck c : checks) {
            HealthCheck.Transition t = c.evaluate(now);
            if (t == HealthCheck.Transition.FIRED) {
                publishState(c);
                relayToAlertManager(c, true);
            } else if (t == HealthCheck.Transition.CLEARED) {
                publishState(c);
                relayToAlertManager(c, false);
            } else if (c.isFiring()) {
                // Live-update detail string for still-firing checks (e.g., a
                // temperature alert that updates "92°C" → "95°C" without
                // re-firing).
                healthTable.getSubTable(c.subsystem()).getSubTable(c.id())
                        .getEntry("detail").setString(c.currentDetail());
            }

            if (c.isFiring()) {
                switch (c.severity()) {
                    case ERROR -> errorCount++;
                    case WARN  -> warnCount++;
                    case INFO  -> infoCount++;
                }
            }
        }

        healthTable.getEntry("ErrorCount").setInteger(errorCount);
        healthTable.getEntry("WarnCount").setInteger(warnCount);
        healthTable.getEntry("InfoCount").setInteger(infoCount);
        healthTable.getEntry("Healthy").setBoolean(errorCount == 0 && warnCount == 0);

        // Forward to the optional cross-mechanism safety watchdog. Cheap no-op
        // when teams haven't called RobotSafety.configure(...).
        RobotSafety.tick(errorCount, warnCount);
    }

    /**
     * Register the three checks every motor-driven mechanism wants: stator
     * current near limit (warn), temperature high (warn), and temperature
     * cutoff (error, triggers {@code motor.stop()}).
     *
     * <p>Each Catalyst mechanism calls this in its constructor so teams get
     * sensible monitoring for free. Override thresholds via the mechanism's
     * Config if needed.
     *
     * @param subsystem       subsystem name to label alerts with
     * @param motor           the motor to monitor
     * @param statorLimitAmps configured stator current limit; warn fires at 90% of this
     * @param tempWarnC       configured warn temperature (°C); cutoff is +10 °C above
     */
    public static void standardMotorChecks(String subsystem, CatalystMotor motor,
                                           double statorLimitAmps, double tempWarnC) {
        standardMotorChecks(subsystem, "", motor, statorLimitAmps, tempWarnC);
    }

    /**
     * Variant of {@link #standardMotorChecks(String, CatalystMotor, double, double)}
     * for mechanisms with more than one motor — pass a non-empty {@code idSuffix}
     * (e.g., {@code "Left"}, {@code "Right"}, {@code "Sec"}) so the registered
     * checks don't collide on the {@code OverCurrent} / {@code HighTemp} /
     * {@code OverTemp} ids.
     */
    public static void standardMotorChecks(String subsystem, String idSuffix, CatalystMotor motor,
                                           double statorLimitAmps, double tempWarnC) {
        final double warnAmps = statorLimitAmps * 0.9;
        final double cutoffC  = tempWarnC + 10;
        final String suffix = idSuffix == null ? "" : idSuffix;
        final String detailSuffix = suffix.isEmpty() ? "" : " (" + suffix + ")";

        HealthCheck.builder(subsystem, "OverCurrent" + suffix)
                .severity(HealthCheck.Severity.WARN)
                .description("Stator current near limit" + detailSuffix)
                .when(() -> motor.getStatorCurrent() > warnAmps)
                .detail(() -> String.format("%.1f A", motor.getStatorCurrent()))
                .debounce(0.5)
                .clearAfter(1.0)
                .register();

        HealthCheck.builder(subsystem, "HighTemp" + suffix)
                .severity(HealthCheck.Severity.WARN)
                .description("Motor temperature high" + detailSuffix)
                .when(() -> motor.getTemperature() > tempWarnC)
                .detail(() -> String.format("%.0f C", motor.getTemperature()))
                .debounce(1.0)
                .clearAfter(5.0)
                .register();

        HealthCheck.builder(subsystem, "OverTemp" + suffix)
                .severity(HealthCheck.Severity.ERROR)
                .description("Motor over-temperature cutoff" + detailSuffix)
                .when(() -> motor.getTemperature() > cutoffC)
                .detail(() -> String.format("%.0f C", motor.getTemperature()))
                .debounce(0.0)
                .clearAfter(5.0)
                .onFire(motor::stop)
                .register();
    }

    /** Reset all checks. Useful for tests; not generally needed at runtime. */
    public synchronized void clear() {
        checks.clear();
        // Note: we don't unpublish individual NT entries; they'll be stale but
        // harmless. Real-world callers don't deregister checks.
    }

    // -------------------------------------------------------------

    private void publishStatic(HealthCheck c) {
        NetworkTable t = healthTable.getSubTable(c.subsystem()).getSubTable(c.id());
        t.getEntry("description").setString(c.description());
        t.getEntry("severity").setString(c.severity().name());
        t.getEntry("firing").setBoolean(false);
        t.getEntry("detail").setString("");
        t.getEntry("firedAt").setDouble(0);
    }

    private void publishState(HealthCheck c) {
        NetworkTable t = healthTable.getSubTable(c.subsystem()).getSubTable(c.id());
        t.getEntry("firing").setBoolean(c.isFiring());
        t.getEntry("detail").setString(c.currentDetail());
        t.getEntry("firedAt").setDouble(c.firedAt());
    }

    private void relayToAlertManager(HealthCheck c, boolean firing) {
        AlertManager alerts = AlertManager.getInstance();
        // Use only the static description for the AlertManager key so the
        // fire/clear messages match exactly. The live detail string is in NT
        // for dashboards that want it.
        String msg = c.description();
        if (firing) {
            switch (c.severity()) {
                case ERROR -> alerts.error(c.subsystem(), msg);
                case WARN  -> alerts.warning(c.subsystem(), msg);
                case INFO  -> alerts.info(c.subsystem(), msg);
            }
        } else {
            // AlertManager doesn't have a generic clear by id, but we can
            // remove via the exact same text we used to add.
            switch (c.severity()) {
                case ERROR -> alerts.clearError(c.subsystem(), msg);
                case WARN  -> alerts.clearWarning(c.subsystem(), msg);
                case INFO  -> { /* AlertManager has no clearInfo — info auto-stale */ }
            }
        }
    }
}
