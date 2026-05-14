package frc.lib.catalyst.mechanisms;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.CatalystLog;

/**
 * Base class for all Catalyst mechanisms. Provides automatic telemetry,
 * subsystem registration, and shared command factories.
 *
 * <p>Each mechanism extends {@link SubsystemBase} so it participates in
 * WPILib's command scheduler for mutual exclusion and default commands.
 *
 * <p>Telemetry is published through {@link CatalystLog} — by default a
 * {@link frc.lib.catalyst.logging.NetworkTablesSink} routes everything to
 * {@code /Catalyst/&lt;mechanismName&gt;/...} on NT, so v0.2 dashboards keep
 * working unchanged. Teams that want AdvantageKit (or any other framework)
 * just swap the sink at robot init; no mechanism code changes.
 */
public abstract class CatalystMechanism extends SubsystemBase {

    protected final String name;
    /**
     * NetworkTable handle for this mechanism. Kept for backwards
     * compatibility with v0.2 user code that may have grabbed it directly.
     * New code should call {@link #log(String, double)} (and overloads) which
     * route through {@link CatalystLog}.
     */
    protected final NetworkTable telemetryTable;
    /** Fully-qualified telemetry prefix used by {@link CatalystLog} calls. */
    protected final String logPrefix;

    protected CatalystMechanism(String name) {
        super(name);
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable(name);
        // CatalystLog already prefixes everything with "Catalyst/" via the
        // NetworkTablesSink root table, so the prefix here is just the
        // mechanism name (no leading "Catalyst/").
        this.logPrefix = name;
    }

    /** Publish a {@code double} under this mechanism's prefix. */
    protected void log(String key, double value) {
        CatalystLog.log(logPrefix + "/" + key, value);
    }

    /** Publish a {@code boolean} under this mechanism's prefix. */
    protected void log(String key, boolean value) {
        CatalystLog.log(logPrefix + "/" + key, value);
    }

    /** Publish a {@code long} under this mechanism's prefix. */
    protected void log(String key, long value) {
        CatalystLog.log(logPrefix + "/" + key, value);
    }

    /** Publish a {@code String} under this mechanism's prefix. */
    protected void log(String key, String value) {
        CatalystLog.log(logPrefix + "/" + key, value);
    }

    /**
     * Forward an {@link CatalystInputs} snapshot to the active sink.
     * Useful for mechanisms backed by an IO layer that produces an Inputs POJO
     * each loop; lets replay tooling rehydrate the mechanism from recorded data.
     */
    protected void processInputs(CatalystInputs inputs) {
        CatalystLog.processInputs(logPrefix, inputs);
    }

    /** Set the mechanism's displayed state. */
    protected void setState(String state) {
        CatalystLog.log(logPrefix + "/State", state);
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
