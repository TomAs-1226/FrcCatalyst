package frc.lib.catalyst.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

/**
 * Centralized alert system for mechanism health monitoring and fault detection.
 * Publishes alerts to NetworkTables for dashboard display and reports to DriverStation.
 *
 * <p>Example usage:
 * <pre>{@code
 * AlertManager alerts = AlertManager.getInstance();
 *
 * // In periodic:
 * if (motor.getTemperature() > 70) {
 *     alerts.warning("Elevator", "Motor temperature high: " + motor.getTemperature() + "C");
 * }
 * if (motor.getStatorCurrent() > 100) {
 *     alerts.error("Elevator", "Stator current exceeded 100A!");
 * }
 * }</pre>
 */
public class AlertManager {

    private static AlertManager instance;

    private final NetworkTable alertTable;
    private final List<String> errors = new ArrayList<>();
    private final List<String> warnings = new ArrayList<>();
    private final List<String> infos = new ArrayList<>();

    private AlertManager() {
        alertTable = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("Alerts");
    }

    /** Get the singleton AlertManager instance. */
    public static synchronized AlertManager getInstance() {
        if (instance == null) {
            instance = new AlertManager();
        }
        return instance;
    }

    /**
     * Report an error alert. Errors indicate failures that prevent correct operation.
     * @param subsystem the subsystem/mechanism name
     * @param message the alert message
     */
    public void error(String subsystem, String message) {
        String full = "[" + subsystem + "] " + message;
        if (!errors.contains(full)) {
            errors.add(full);
            DriverStation.reportError("Catalyst ERROR: " + full, false);
            publishAlerts();
        }
    }

    /**
     * Report a warning alert. Warnings indicate degraded but functional operation.
     * @param subsystem the subsystem/mechanism name
     * @param message the alert message
     */
    public void warning(String subsystem, String message) {
        String full = "[" + subsystem + "] " + message;
        if (!warnings.contains(full)) {
            warnings.add(full);
            DriverStation.reportWarning("Catalyst WARNING: " + full, false);
            publishAlerts();
        }
    }

    /**
     * Report an info alert. Info alerts are for notable but non-critical events.
     * @param subsystem the subsystem/mechanism name
     * @param message the alert message
     */
    public void info(String subsystem, String message) {
        String full = "[" + subsystem + "] " + message;
        if (!infos.contains(full)) {
            infos.add(full);
            publishAlerts();
        }
    }

    /** Clear a specific error by message. */
    public void clearError(String subsystem, String message) {
        errors.remove("[" + subsystem + "] " + message);
        publishAlerts();
    }

    /** Clear a specific warning by message. */
    public void clearWarning(String subsystem, String message) {
        warnings.remove("[" + subsystem + "] " + message);
        publishAlerts();
    }

    /** Clear all alerts for a given subsystem. */
    public void clearSubsystem(String subsystem) {
        String prefix = "[" + subsystem + "]";
        errors.removeIf(s -> s.startsWith(prefix));
        warnings.removeIf(s -> s.startsWith(prefix));
        infos.removeIf(s -> s.startsWith(prefix));
        publishAlerts();
    }

    /** Clear all alerts. */
    public void clearAll() {
        errors.clear();
        warnings.clear();
        infos.clear();
        publishAlerts();
    }

    /** Check if there are any active errors. */
    public boolean hasErrors() {
        return !errors.isEmpty();
    }

    /** Check if there are any active warnings. */
    public boolean hasWarnings() {
        return !warnings.isEmpty();
    }

    /** Get all active error messages. */
    public List<String> getErrors() {
        return List.copyOf(errors);
    }

    /** Get all active warning messages. */
    public List<String> getWarnings() {
        return List.copyOf(warnings);
    }

    private void publishAlerts() {
        alertTable.getEntry("Errors").setStringArray(errors.toArray(new String[0]));
        alertTable.getEntry("Warnings").setStringArray(warnings.toArray(new String[0]));
        alertTable.getEntry("Info").setStringArray(infos.toArray(new String[0]));
        alertTable.getEntry("ErrorCount").setInteger(errors.size());
        alertTable.getEntry("WarningCount").setInteger(warnings.size());
        alertTable.getEntry("HasFaults").setBoolean(!errors.isEmpty());
    }
}
