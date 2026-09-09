package frc.robot;

import org.wpilib.framework.RobotBase;

/**
 * Entry point. Do not put robot logic here — it lives in {@link Robot}.
 */
public final class Main {
    private Main() {}

    public static void main(String... args) {
        // alpha-7 takes a supplier rather than a Class: startRobot(Supplier<T extends RobotBase>).
        // Constructing it here rather than reflectively means a constructor that throws fails at
        // the call site with a real stack trace, instead of inside reflection.
        RobotBase.startRobot(Robot::new);
    }
}
