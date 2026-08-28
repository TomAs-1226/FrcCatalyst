package frc.robot;

import org.wpilib.framework.RobotBase;

/**
 * Entry point. Do not put robot logic here — it lives in {@link Robot}.
 */
public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot.class);
    }
}
