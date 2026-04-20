package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for common telemetry operations.
 */
public final class TelemetryUtil {

    private static final Field2d field = new Field2d();
    private static boolean fieldInitialized = false;

    private TelemetryUtil() {}

    /**
     * Get a shared Field2d widget for displaying the robot on the field.
     * Automatically initializes on first call.
     */
    public static Field2d getField() {
        if (!fieldInitialized) {
            SmartDashboard.putData("Catalyst/Field", field);
            fieldInitialized = true;
        }
        return field;
    }

    /** Update the field widget with the robot's current pose. */
    public static void updateFieldPose(Pose2d pose) {
        getField().setRobotPose(pose);
        Logger.recordOutput("Catalyst/Field/RobotPose", pose);
    }

    /** Log a value via AdvantageKit under the Catalyst namespace. */
    public static void log(String key, double value) {
        Logger.recordOutput("Catalyst/" + key, value);
    }

    /** Log a value via AdvantageKit under the Catalyst namespace. */
    public static void log(String key, boolean value) {
        Logger.recordOutput("Catalyst/" + key, value);
    }

    /** Log a value via AdvantageKit under the Catalyst namespace. */
    public static void log(String key, String value) {
        Logger.recordOutput("Catalyst/" + key, value);
    }
}
