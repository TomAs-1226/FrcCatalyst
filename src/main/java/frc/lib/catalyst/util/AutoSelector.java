package frc.lib.catalyst.util;

import frc.lib.catalyst.command.LegacyCommands;
import frc.lib.catalyst.command.CatalystCommand;
import org.wpilib.driverstation.DriverStationErrors;
import com.pathplanner.lib.auto.AutoBuilder;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;
import org.wpilib.command3.Command;
import frc.lib.catalyst.command.Commands;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Auto routine selector with PathPlanner integration and safe defaults.
 *
 * <p>Wraps {@link Selectable} to provide a clean interface for
 * selecting autonomous routines from the dashboard, with built-in
 * error handling and a "Do Nothing" fallback.
 *
 * <p>Supports both PathPlanner named autos and custom command-based autos.
 *
 * <p>Example:
 * <pre>{@code
 * AutoSelector autos = new AutoSelector()
 *     .addPathPlannerAuto("3 Piece Center")
 *     .addPathPlannerAuto("2 Piece Amp Side")
 *     .addPathPlannerAuto("1 Piece + Taxi")
 *     .addCustom("Shoot and Sit", () -> shooter.shootSpeaker())
 *     .addCustom("Just Taxi", () -> drive.driveForward(2.0));
 *
 * // In autonomousInit:
 * Command auto = autos.getSelected();
 * auto.schedule();
 * }</pre>
 */
public class AutoSelector {

    private final Selectable<String> chooser = new Selectable<>();
    private final Map<String, java.util.function.Supplier<Command>> autos = new LinkedHashMap<>();
    private final String dashboardKey;
    private boolean firstAdded = false;

    /** Create an auto selector published to "Auto Selector" on the dashboard. */
    public AutoSelector() {
        this("Auto Selector");
    }

    /**
     * Create an auto selector with a custom dashboard key.
     *
     * @param dashboardKey dashboard key name
     */
    public AutoSelector(String dashboardKey) {
        this.dashboardKey = dashboardKey;

        // Always have a safe "do nothing" option
        chooser.addDefault("Do Nothing", "Do Nothing");
        autos.put("Do Nothing", Commands::none);

        Tunables.publish(dashboardKey, chooser);
    }

    /**
     * Add a PathPlanner named auto.
     * The name must match a PathPlanner auto file in deploy/pathplanner/autos/.
     *
     * @param autoName PathPlanner auto name (as saved in PathPlanner UI)
     * @return this (for chaining)
     */
    public AutoSelector addPathPlannerAuto(String autoName) {
        autos.put(autoName, () -> {
            try {
                return LegacyCommands.fromV2(AutoBuilder.buildAuto(autoName));
            } catch (Exception e) {
                DriverStationErrors.reportError(
                        "AutoSelector: Failed to build '" + autoName + "' - " + e.getMessage(), false);
                return Commands.none();
            }
        });

        if (!firstAdded) {
            chooser.addDefault(autoName, autoName);
            firstAdded = true;
        } else {
            chooser.add(autoName, autoName);
        }
        return this;
    }

    /**
     * Add a custom command-based auto routine.
     *
     * @param name display name on the dashboard
     * @param commandSupplier supplier that creates the auto command
     * @return this (for chaining)
     */
    public AutoSelector addCustom(String name, java.util.function.Supplier<Command> commandSupplier) {
        autos.put(name, commandSupplier);

        if (!firstAdded) {
            chooser.addDefault(name, name);
            firstAdded = true;
        } else {
            chooser.add(name, name);
        }
        return this;
    }

    /**
     * Get the currently selected auto command.
     * Returns a "Do Nothing" command if the selection fails.
     */
    public CatalystCommand getSelected() {
        String selected = chooser.getSelected();
        if (selected == null) {
            DriverStationErrors.reportWarning("AutoSelector: No auto selected, using Do Nothing", false);
            return Commands.none();
        }

        var supplier = autos.get(selected);
        if (supplier == null) {
            DriverStationErrors.reportWarning(
                    "AutoSelector: Unknown auto '" + selected + "', using Do Nothing", false);
            return Commands.none();
        }

        return CatalystCommand.of(supplier.get()).withName("Auto:" + selected);
    }

    /** Get the name of the currently selected auto. */
    public String getSelectedName() {
        String selected = chooser.getSelected();
        return selected != null ? selected : "Do Nothing";
    }

    /**
     * Get the underlying selector.
     *
     * <p><b>Changed in 2.0.0.</b> WPILib 2027 removed {@code SendableChooser}; this now returns
     * {@link Selectable}, its replacement. Every other method on {@code AutoSelector} is unchanged,
     * so this is the only call site a migrating team has to touch — and only if they used it.
     */
    public Selectable<String> getChooser() {
        return chooser;
    }
}
