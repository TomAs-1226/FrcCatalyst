package frc.lib.catalyst.util;

import frc.lib.catalyst.system.SystemCoreStatus;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;
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

    // --- Who calls update(), and why it is not the scheduler ---------------------
    //
    // Nothing used to, on a large class of robot. update() is the only evaluator of every check,
    // the only writer of the health topics, the only feeder of HealthHistory and the only caller of
    // RobotSafety.tick() - and its only callers were the nine updateTelemetry() bodies inside the
    // mechanism classes. A robot built from SwerveSubsystem, a vision subsystem and the team's own
    // CatalystSubsystems - which the state machine explicitly invites, since "a team's own subsystem
    // is exactly as first-class as LinearMechanism" - owns no mechanism, so nothing called it.
    //
    // Measured: with a CAN bus taken off-line for five seconds, a mechanism-less robot left the
    // ERROR check un-fired, HealthHistory empty, the health topics never written at all, and a
    // configured RobotSafety all-stop never invoked. Adding one flywheel to the same robot, changing
    // nothing else, brought every one of them back. The dashboard showed a healthy robot with a dead
    // CAN bus, and the watchdog a team had configured to stop the robot sat there never asked.
    //
    // TRIED AND REJECTED: Scheduler.getDefault().addPeriodic(this::update) on first registration.
    // It is the obvious fix and it is quietly unsafe. addPeriodic delegates to sideload, which
    // stamps the callback with BindingScope.createNarrowestScope(this) - an opmode-scoped binding
    // when an opmode is selected - and runPeriodicSideloads removes any callback whose scope has
    // gone inactive, without running it. So a monitor installed during autonomous stops ticking the
    // moment the Driver Station switches to teleop, silently, which is the same failure this was
    // meant to fix and harder to find the second time.
    //
    // Driven from CatalystOpMode.periodic()/disabledPeriodic() instead, next to the
    // SystemCoreStatus.publish() call that was already there. That runs in both robot states, cannot
    // be reaped by a scope, and needs no registration. update() is throttled and idempotent, so it
    // composes with the nine mechanism call sites and none of them had to be removed.
    //
    // A robot that does not use CatalystOpMode - a bare OpMode, or a test - still has to call
    // update() itself. That is a real limit and is why this comment names it rather than leaving it
    // to be rediscovered.

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
        double now = Timer.getTimestamp();
        if (now - lastUpdateTs < 0.005) return;
        lastUpdateTs = now;

        int errorCount = 0, warnCount = 0, infoCount = 0;
        for (HealthCheck c : checks) {
            HealthCheck.Transition t = c.evaluate(now);
            if (t == HealthCheck.Transition.FIRED) {
                publishState(c);
                relayToAlertManager(c, true);
                HealthHistory.record(c, HealthHistory.Kind.FIRED);
            } else if (t == HealthCheck.Transition.CLEARED) {
                publishState(c);
                relayToAlertManager(c, false);
                HealthHistory.record(c, HealthHistory.Kind.CLEARED);
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

        // The device roster rides on this tick because it is the one Catalyst already drives
        // whether the robot is enabled or not. Rate-limited inside; this is called at loop rate.
        frc.lib.catalyst.identity.DeviceRoster.publish(now);
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

    /**
     * Register health checks for the Systemcore itself.
     *
     * <p>Everything Catalyst monitored before this was about the robot — motor temperature, current,
     * loop time. The machine underneath was invisible, largely because on a roboRIO there was very
     * little to see. Systemcore measures and publishes its own CPU, memory, storage and power, which
     * turns a class of mystery into a reading: a robot that browns out because logs filled the disk,
     * or misses loop deadlines because something else is pinning a core, now says so in the pit
     * rather than on the field.
     *
     * <p>No-ops off Systemcore. In simulation or a desktop build there is no system server, nothing
     * is registered and nothing fires — call it unconditionally from robot init.
     *
     * <p>Thresholds are deliberately loose. These catch a machine in trouble, they do not grade it:
     * a Systemcore at 70% RAM is fine, one at 95% is about to have a bad match.
     */
    public static void systemCoreChecks() {
        SystemCoreStatus status = SystemCoreStatus.getInstance();
        if (!status.isAvailable()) {
            return;
        }
        final String subsystem = "Systemcore";

        HealthCheck.builder(subsystem, "BrownedOut")
                .severity(HealthCheck.Severity.ERROR)
                .description("Systemcore reports a brownout")
                .when(status::isBrownedOut)
                .detail(() -> String.format("battery %.2f V",
                        status.batteryVolts().orElse(Double.NaN)))
                .register();

        HealthCheck.builder(subsystem, "HighCpu")
                .severity(HealthCheck.Severity.WARN)
                .description("Systemcore CPU sustained above 90%")
                .when(() -> status.cpuUtilization().orElse(0) > 90.0)
                // Debounced: a spike at startup or on a log flush is normal, a sustained pin is not.
                .debounce(5.0)
                .detail(() -> String.format("cpu %.0f%%", status.cpuUtilization().orElse(Double.NaN)))
                .register();

        HealthCheck.builder(subsystem, "LowMemory")
                .severity(HealthCheck.Severity.WARN)
                .description("Systemcore RAM above 90% used")
                .when(() -> status.ramFraction().orElse(0) > 0.90)
                .debounce(5.0)
                .detail(() -> String.format("ram %.0f%%",
                        status.ramFraction().orElse(Double.NaN) * 100))
                .register();

        // Storage fills up silently and takes the robot with it: a full disk stops logs, then stops
        // the robot program, and nothing about the failure points back at the disk.
        HealthCheck.builder(subsystem, "LowStorage")
                .severity(HealthCheck.Severity.ERROR)
                .description("Systemcore storage above 90% used")
                .when(() -> status.storageFraction().orElse(0) > 0.90)
                .detail(() -> String.format("storage %.0f%%",
                        status.storageFraction().orElse(Double.NaN) * 100))
                .register();

        // A team number set on the device that disagrees with the code is an afternoon of
        // "why won't the Driver Station connect".
        HealthCheck.builder(subsystem, "TeamNumberMismatch")
                .severity(HealthCheck.Severity.WARN)
                .description("Systemcore team number differs from the robot program's")
                .when(() -> {
                    var device = status.teamNumber();
                    if (device.isEmpty()) {
                        return false;
                    }
                    try {
                        return device.getAsInt() != org.wpilib.system.RobotController.getTeamNumber();
                    } catch (Throwable ignored) {
                        return false;
                    }
                })
                .detail(() -> "device reports team " + status.teamNumber().orElse(-1))
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
