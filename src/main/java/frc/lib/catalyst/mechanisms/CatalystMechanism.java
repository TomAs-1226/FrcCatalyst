package frc.lib.catalyst.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
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

    protected CatalystMechanism(String name) {
        super(name);
        this.name = name;
    }

    /** Log a double value under this mechanism's namespace. */
    protected void log(String key, double value) {
        Logger.recordOutput("Catalyst/" + name + "/" + key, value);
    }

    /** Log a boolean value under this mechanism's namespace. */
    protected void log(String key, boolean value) {
        Logger.recordOutput("Catalyst/" + name + "/" + key, value);
    }

    /** Log a string value under this mechanism's namespace. */
    protected void log(String key, String value) {
        Logger.recordOutput("Catalyst/" + name + "/" + key, value);
    }

    /** Set the mechanism's displayed state. */
    protected void setState(String state) {
        Logger.recordOutput("Catalyst/" + name + "/State", state);
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
