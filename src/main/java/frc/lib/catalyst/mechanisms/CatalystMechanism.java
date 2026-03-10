package frc.lib.catalyst.mechanisms;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base class for all Catalyst mechanisms.
 * Provides automatic telemetry, subsystem registration, and common command factories.
 * Each mechanism IS a SubsystemBase so it integrates with WPILib's command scheduler
 * for mutual exclusion and default commands.
 */
public abstract class CatalystMechanism extends SubsystemBase {

    protected final String name;
    protected final NetworkTable telemetryTable;
    private final StringPublisher statePub;

    protected CatalystMechanism(String name) {
        super(name);
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable(name);
        this.statePub = telemetryTable.getStringTopic("State").publish();
    }

    /** Log a double value to NetworkTables under this mechanism's table. */
    protected void log(String key, double value) {
        telemetryTable.getEntry(key).setDouble(value);
    }

    /** Log a boolean value to NetworkTables under this mechanism's table. */
    protected void log(String key, boolean value) {
        telemetryTable.getEntry(key).setBoolean(value);
    }

    /** Log a string value to NetworkTables under this mechanism's table. */
    protected void log(String key, String value) {
        telemetryTable.getEntry(key).setString(value);
    }

    /** Set the mechanism's displayed state. */
    protected void setState(String state) {
        statePub.set(state);
    }

    /** Get the mechanism name. */
    public String getMechanismName() {
        return name;
    }

    /** Command to stop the mechanism. */
    public Command stopCommand() {
        return runOnce(this::stop).withName(name + ".Stop");
    }

    /** Stop all outputs. Subclasses must implement. */
    protected abstract void stop();

    /** Update telemetry. Called from periodic(). Subclasses should override. */
    protected void updateTelemetry() {}

    @Override
    public void periodic() {
        updateTelemetry();
    }
}
