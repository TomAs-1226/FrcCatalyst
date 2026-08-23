package frc.robot;

import frc.lib.catalyst.command.Commands;
import frc.lib.catalyst.driverstation.DriverBoard;
import frc.lib.catalyst.opmode.CatalystOpMode;
import frc.lib.catalyst.opmode.CommandOpMode;

import org.wpilib.framework.OpModeRobot;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.Teleop;

/**
 * The robot, as an {@link OpModeRobot}.
 *
 * <p>2027 replaced "autonomous is whatever {@code autonomousInit} does" with named modes the Driver
 * Station lists and selects. A routine is a class with an annotation on it, and it appears on the
 * Driver Station under its group, ready to pick — no chooser to publish, nothing to remember when a
 * routine is added.
 *
 * <p>{@code TimedRobot} still exists and still works, and a team porting from 1.x can keep it. This
 * example uses OpModes because they are the way autonomous is selected on this control system, and
 * an example that shows the old way teaches the old way.
 *
 * <p>All the wiring is still in {@link RobotContainer}. This class only says which modes exist.
 */
public class Robot extends OpModeRobot {

    /**
     * Built once, shared by every mode.
     *
     * <p>Not per-mode: the mechanisms own hardware, and constructing them twice would mean two
     * objects claiming the same CAN ids — which {@code CANRegistry} would reject at the second
     * claim, correctly and confusingly.
     */
    static final RobotContainer CONTAINER = new RobotContainer();

    public Robot() {
        // Finds every annotated OpMode in this package, including the nested ones below.
        addAnnotatedOpModeClasses(getClass().getPackage());

        // What is wrong and the battery, on the Driver Station itself. Updates itself from here.
        DriverBoard.standard().start();
    }

    /**
     * The competition autonomous.
     *
     * <p>{@link CommandOpMode} takes a <em>supplier</em>, not a command, and that is not a style
     * choice. An OpMode is constructed when the Driver Station lists the modes — at startup — so a
     * command built then captures the pose the robot had at boot and the alliance before the FMS
     * said. It runs, it looks entirely plausible, and it drives the wrong way.
     */
    @Autonomous(name = "Showcase Auto", group = "Competition",
                description = "Runs the showcase superstructure through its scoring sequence")
    public static class ShowcaseAuto extends CommandOpMode {
        public ShowcaseAuto() {
            super(CONTAINER::getAutonomousCommand);
        }
    }

    /**
     * A do-nothing autonomous.
     *
     * <p>Worth having as a real, selectable mode rather than as "just do not enable": a robot that
     * has to sit still because a mechanism is broken should be a thing the driver picks, not a thing
     * they achieve by remembering not to do something.
     */
    @Autonomous(name = "Do Nothing", group = "Competition",
                description = "Sits still. For when something is broken.")
    public static class DoNothing extends CommandOpMode {
        public DoNothing() {
            super(() -> Commands.none().withName("do nothing"));
        }
    }

    /**
     * Teleop.
     *
     * <p>{@link CatalystOpMode} rather than {@code CommandOpMode}, because teleop is not one command
     * — the bindings in {@link RobotContainer} schedule commands as buttons are pressed, and this
     * only has to keep the scheduler running, which the base class does.
     */
    @Teleop(name = "Driver", group = "Competition")
    public static class DriverControl extends CatalystOpMode {
        @Override
        protected void onPeriodic() {
            // Services the browser Sim Cockpit (localhost:5805) and auto-enables in simulation.
            // A no-op on a real robot.
            CONTAINER.simPeriodic();
        }
    }
}
