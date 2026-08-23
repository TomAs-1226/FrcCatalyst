package frc.robot;

import org.wpilib.framework.TimedRobot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

/**
 * TimedRobot shell. All wiring lives in {@link RobotContainer}; this class just
 * runs the {@link CommandScheduler} and hands off auto/teleop.
 */
public class Robot extends TimedRobot {

    private Command autonomousCommand;
    private final RobotContainer container = new RobotContainer();

    @Override
    public void robotPeriodic() {
        // Runs every mechanism's periodic() and simulationPeriodic(), plus the
        // SuperstructureCoordinator + GoalDirector commands.
        Scheduler.getDefault().run();
    }

    @Override
    public void simulationPeriodic() {
        // Services the browser Sim Cockpit (localhost:5805) and auto-enables.
        container.simPeriodic();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = container.getAutonomousCommand();
        if (autonomousCommand != null) {
            Scheduler.getDefault().schedule(autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            Scheduler.getDefault().cancel(autonomousCommand);
        }
    }
}
