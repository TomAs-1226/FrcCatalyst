package frc.lib.catalyst.mechanisms;

import frc.lib.catalyst.command.CatalystCommand;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import frc.lib.catalyst.sysid.SysIdRoutine.Direction;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.util.RumbleEvents;

/**
 * Base class for all Catalyst mechanisms. Provides automatic telemetry,
 * subsystem registration, and shared command factories.
 *
 * <p>Each mechanism extends {@link Mechanism} so it participates in
 * WPILib's command scheduler for mutual exclusion and default commands.
 *
 * <p>Telemetry is published through {@link CatalystLog} — by default a
 * {@link frc.lib.catalyst.logging.NetworkTablesSink} routes everything to
 * {@code /Catalyst/&lt;mechanismName&gt;/...} on NT, so v0.2 dashboards keep
 * working unchanged. Teams that want AdvantageKit (or any other framework)
 * just swap the sink at robot init; no mechanism code changes.
 */
public abstract class CatalystMechanism extends frc.lib.catalyst.command.CatalystSubsystem {

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
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable(name);
        // CatalystLog already prefixes everything with "Catalyst/" via the
        // NetworkTablesSink root table, so the prefix here is just the
        // mechanism name (no leading "Catalyst/").
        this.logPrefix = name;
        // Commands v3's Mechanism has no constructor to hook, so periodic() is registered
        // explicitly. Without this the method compiles and is simply never called - see
        // CatalystSubsystem#registerPeriodic.
        registerPeriodic();
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

    /**
     * A uniform snapshot of this mechanism's live state for the generic
     * simulation dashboard ({@code frc.lib.catalyst.sim.SimDashboard}).
     * Subclasses override this to report their value, setpoint, range, etc.
     * with the right units. The base default reports a "generic" widget — a
     * user's custom mechanism still shows up, it just won't get a type-specific
     * gauge until it overrides this.
     */
    public MechanismView describe() {
        return MechanismView.of(name, "generic").build();
    }

    /** Command to stop the mechanism. */
    public CatalystCommand stopCommand() {
        return runOnce(this::stop).withName(name + ".Stop");
    }

    /** Stop all outputs. Subclasses must implement. */
    protected abstract void stop();

    /**
     * Quasistatic SysId command — slow ramp characterising kS / kV. By
     * default this runs against {@link #primaryMotorForSysId()} (the
     * primary motor of the mechanism). Subclasses with multi-motor
     * layouts can override.
     *
     * <p>Teams must call {@code SignalLogger.start()} from {@code robotInit()}
     * for the WPILib SysId tooling to find the captured data.
     */
    public CatalystCommand sysIdQuasistatic(Direction dir) {
        CatalystMotor m = primaryMotorForSysId();
        if (m == null) throw new UnsupportedOperationException(
                name + " does not expose a motor for SysId. Override sysIdQuasistatic(...).");
        return m.sysIdQuasistatic(this, dir).withName(name + ".SysIdQuasistatic." + dir);
    }

    /** Dynamic SysId command — step input characterising kA. */
    public CatalystCommand sysIdDynamic(Direction dir) {
        CatalystMotor m = primaryMotorForSysId();
        if (m == null) throw new UnsupportedOperationException(
                name + " does not expose a motor for SysId. Override sysIdDynamic(...).");
        return m.sysIdDynamic(this, dir).withName(name + ".SysIdDynamic." + dir);
    }

    /**
     * Override in mechanisms that have a clear "primary motor" to run
     * SysId against. Default returns {@code null} so the SysId helpers
     * complain loudly instead of silently doing nothing. Most mechanisms
     * just need to return their leader motor.
     */
    protected CatalystMotor primaryMotorForSysId() {
        return null;
    }

    /**
     * Bind {@code trigger} to a {@link RumbleEvents} pattern. Helper so
     * teams don't have to import the {@link RumbleEvents.Channel} /
     * {@link RumbleEvents.Pattern} enums in every binding file.
     *
     * <p>Per-mechanism subclasses override the no-trigger short form
     * {@link #bindRumble(RumbleEvents, RumbleEvents.Pattern, RumbleEvents.Channel)}
     * to pre-select the "obvious" trigger (e.g. {@code hasPieceTrigger()}
     * for grippers, {@code atSpeedTrigger()} for flywheels).
     */
    public void bindRumble(RumbleEvents events, Trigger trigger,
                           RumbleEvents.Pattern pattern, RumbleEvents.Channel channel) {
        events.onTrigger(trigger, pattern, channel);
    }

    /**
     * Bind the mechanism's "obvious" event to a rumble pattern. Default
     * implementation throws — subclasses with a natural event override.
     *
     * <p>For mechanisms that already provide it ({@link ClawMechanism},
     * {@link RollerMechanism}, {@link FlywheelMechanism},
     * {@link DifferentialWristMechanism}, {@link LinearMechanism},
     * {@link RotationalMechanism}), this is the one-line ergonomic path.
     */
    public void bindRumble(RumbleEvents events,
                           RumbleEvents.Pattern pattern, RumbleEvents.Channel channel) {
        throw new UnsupportedOperationException(
                name + " has no default rumble event — use the four-arg overload with an explicit Trigger.");
    }

    /** Update telemetry. Called from periodic(). Subclasses should override. */
    protected void updateTelemetry() {}

    @Override
    public void periodic() {
        updateTelemetry();
    }
}
