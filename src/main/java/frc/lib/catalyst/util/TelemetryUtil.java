package frc.lib.catalyst.util;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.Field2d;
import frc.lib.catalyst.logging.CatalystLog;
import org.wpilib.telemetry.Telemetry;

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
            // Field2d implements TelemetryLoggable in 2027, so it logs itself.
            Telemetry.getTable("Catalyst").log("Field", field);
            fieldInitialized = true;
        }
        return field;
    }

    /** Update the field widget with the robot's current pose. */
    public static void updateFieldPose(Pose2d pose) {
        getField().setRobotPose(pose);
    }

    /** Log a value to SmartDashboard under the Catalyst namespace. */
    public static void log(String key, double value) {
        CatalystLog.log(key, value);
    }

    /** Log a value to SmartDashboard under the Catalyst namespace. */
    public static void log(String key, boolean value) {
        CatalystLog.log(key, value);
    }

    /** Log a value to SmartDashboard under the Catalyst namespace. */
    public static void log(String key, String value) {
        CatalystLog.log(key, value);
    }
}
