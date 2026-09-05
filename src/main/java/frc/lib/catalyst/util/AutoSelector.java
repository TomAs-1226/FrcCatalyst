package frc.lib.catalyst.util;

import frc.lib.catalyst.command.LegacyCommands;
import frc.lib.catalyst.command.CatalystCommand;
import org.wpilib.driverstation.DriverStationErrors;
import com.pathplanner.lib.auto.AutoBuilder;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command3.Command;
import frc.lib.catalyst.command.Commands;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Auto routine selector with PathPlanner integration and safe defaults.
 *
 * <p>Wraps {@link SendableChooser} to provide a clean interface for selecting autonomous routines
 * from the dashboard, with built-in error handling and a "Do Nothing" fallback.
 *
 * <p><b>This class was excluded from the 2027 build for a while, and it should not have been.</b>
 * It had been ported to {@code org.wpilib.tunable.Selectable} on the belief that
 * {@code SendableChooser} was removed in 2027, and then dropped from the source set when that
 * package turned out to be missing from the released alpha-6. Only the second half was true.
 * Checked against the shipped jars: {@code org.wpilib.tunables} is genuinely absent, but
 * {@code SendableChooser}, {@code SmartDashboard}, {@code Sendable} and {@code SendableBuilder} are
 * all present in {@code wpilibj-java}. The removal claim came from the development snapshot, where
 * {@code org.wpilib.telemetry} replaced them - a different build that happens to share the alpha-6
 * name. The cost of the mistake was that teams on this branch had no auto chooser at all.
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

    /** Whether the autos have been offered to the Driver Station as op modes. */
    private boolean opModesPublished = false;

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
        pathPlannerAutos.add(autoName);
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
     *
     * <p>Resolves through the same path as {@link #getSelectedName()}: the Driver Station's op mode
     * when one is published and recognised, otherwise the dashboard selector. The two must never
     * disagree — a robot that reports one auto and runs another is worse than one that runs the
     * wrong auto openly.
     */
    public CatalystCommand getSelected() {
        String selected = resolveSelection();
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
        String selected = resolveSelection();
        return selected != null ? selected : "Do Nothing";
    }

    /**
     * The single place the selection is decided, so every caller agrees.
     *
     * @return the selected auto's name, or null when nothing is selected anywhere
     */
    private String resolveSelection() {
        String fromDriverStation = opModesPublished ? selectedOpModeName() : null;
        return fromDriverStation != null ? fromDriverStation : chooser.getSelected();
    }

    /**
     * Offer every auto to the Driver Station as an autonomous op mode.
     *
     * <p>New in 2027. The FIRST Driver Station can select an op mode directly, which means the auto
     * is chosen on the same screen the drive team already has open rather than on a dashboard widget
     * that may or may not be running. Call once, after every auto has been added.
     *
     * <p>Once this has been called the Driver Station's choice takes priority over the dashboard
     * selector, and only when it names an auto this selector knows. Two things follow from that,
     * both deliberate: a Driver Station that has selected nothing falls back to the dashboard, so
     * this is safe to call unconditionally; and an op mode named something Catalyst has never heard
     * of is ignored rather than silently running "Do Nothing", because that would look identical to
     * a working robot right up until autonomous started.
     *
     * <p>No-op off hardware — publishing op modes needs the HAL.
     */
    public AutoSelector publishAsOpModes() {
        try {
            for (String name : autos.keySet()) {
                org.wpilib.driverstation.RobotState.addOpMode(
                        org.wpilib.hardware.hal.RobotMode.AUTONOMOUS, name);
            }
            org.wpilib.driverstation.RobotState.publishOpModes();
            opModesPublished = true;
        } catch (Throwable ignored) {
            // Simulation, a unit test, or a desktop build. The dashboard selector still works.
            opModesPublished = false;
        }
        return this;
    }

    /** Whether the Driver Station's op modes are in use, and it has picked one this selector knows. */
    public boolean isUsingDriverStationSelection() {
        return opModesPublished && selectedOpModeName() != null;
    }

    /**
     * The Driver Station's chosen op mode, if it names an auto this selector knows about.
     *
     * @return the name, or null to fall back to the dashboard selector
     */
    private String selectedOpModeName() {
        try {
            String name = org.wpilib.driverstation.RobotState.getOpMode();
            return name != null && autos.containsKey(name) ? name : null;
        } catch (Throwable ignored) {
            return null;
        }
    }

    /**
     * The underlying chooser, for anything this class does not wrap.
     *
     * <p>Back to {@link SendableChooser}, the type it always returned. The 2027 notes recorded this
     * as the one signature Catalyst was forced to break; that turned out to be wrong, so there is no
     * break and nothing for a migrating team to change here.
     */
    public SendableChooser<String> getChooser() {
        return chooser;
    }

    /** Which entries are PathPlanner autos, and so have a starting pose in their file. */
    private final java.util.Set<String> pathPlannerAutos = new java.util.HashSet<>();

    /** Built once per auto; the pose is asked for fresh each time, because it flips with alliance. */
    private final java.util.Map<String, com.pathplanner.lib.commands.PathPlannerAuto> startPoseSources =
            new java.util.HashMap<>();

    /**
     * Where the currently selected auto expects the robot to start, if it can say.
     *
     * <p>PathPlanner autos carry their starting pose in the file, already on the alliance the
     * Driver Station reports. A custom auto has no file, and an auto whose file cannot be loaded
     * answers empty rather than with a pose nobody chose. Meant for {@link AutoStartCheck}, which
     * compares it with where the robot actually is while the robot is still disabled.
     */
    public java.util.Optional<org.wpilib.math.geometry.Pose2d> selectedStartingPose() {
        String selected = resolveSelection();
        if (selected == null || !pathPlannerAutos.contains(selected)) {
            return java.util.Optional.empty();
        }
        try {
            com.pathplanner.lib.commands.PathPlannerAuto auto = startPoseSources.get(selected);
            if (auto == null) {
                auto = new com.pathplanner.lib.commands.PathPlannerAuto(selected);
                startPoseSources.put(selected, auto);
            }
            return java.util.Optional.ofNullable(auto.getStartingPose());
        } catch (RuntimeException e) {
            return java.util.Optional.empty();
        }
    }

    /**
     * A start check wired to this selector: compares {@code currentPose} against the selected
     * auto's starting pose. Call its {@code update()} from a disabled periodic.
     */
    public AutoStartCheck startCheck(java.util.function.Supplier<org.wpilib.math.geometry.Pose2d> currentPose) {
        return new AutoStartCheck(currentPose, this::selectedStartingPose);
    }
}
