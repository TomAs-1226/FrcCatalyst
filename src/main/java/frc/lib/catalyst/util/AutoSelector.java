package frc.lib.catalyst.util;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Auto routine selector with PathPlanner integration and safe defaults.
 *
 * <p>Wraps {@link SendableChooser} to provide a clean interface for
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

    private final SendableChooser<String> chooser = new SendableChooser<>();
    private final Map<String, java.util.function.Supplier<Command>> autos = new LinkedHashMap<>();
    private final String dashboardKey;
    private boolean firstAdded = false;

    /** Create an auto selector published to "Auto Selector" on SmartDashboard. */
    public AutoSelector() {
        this("Auto Selector");
    }

    /**
     * Create an auto selector with a custom dashboard key.
     *
     * @param dashboardKey SmartDashboard key name
     */
    public AutoSelector(String dashboardKey) {
        this.dashboardKey = dashboardKey;

        // Always have a safe "do nothing" option
        chooser.setDefaultOption("Do Nothing", "Do Nothing");
        autos.put("Do Nothing", Commands::none);

        SmartDashboard.putData(dashboardKey, chooser);
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
                return AutoBuilder.buildAuto(autoName);
            } catch (Exception e) {
                DriverStation.reportError(
                        "AutoSelector: Failed to build '" + autoName + "' - " + e.getMessage(), false);
                return Commands.none();
            }
        });

        if (!firstAdded) {
            chooser.setDefaultOption(autoName, autoName);
            firstAdded = true;
        } else {
            chooser.addOption(autoName, autoName);
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
            chooser.setDefaultOption(name, name);
            firstAdded = true;
        } else {
            chooser.addOption(name, name);
        }
        return this;
    }

    /**
     * Get the currently selected auto command.
     * Returns a "Do Nothing" command if the selection fails.
     */
    public Command getSelected() {
        String selected = chooser.getSelected();
        if (selected == null) {
            DriverStation.reportWarning("AutoSelector: No auto selected, using Do Nothing", false);
            return Commands.none();
        }

        var supplier = autos.get(selected);
        if (supplier == null) {
            DriverStation.reportWarning(
                    "AutoSelector: Unknown auto '" + selected + "', using Do Nothing", false);
            return Commands.none();
        }

        return supplier.get().withName("Auto:" + selected);
    }

    /** Get the name of the currently selected auto. */
    public String getSelectedName() {
        String selected = chooser.getSelected();
        return selected != null ? selected : "Do Nothing";
    }

    /** Get the underlying SendableChooser. */
    public SendableChooser<String> getChooser() {
        return chooser;
    }
}
