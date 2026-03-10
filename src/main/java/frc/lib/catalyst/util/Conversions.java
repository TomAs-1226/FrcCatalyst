package frc.lib.catalyst.util;

/**
 * Common unit conversions for FRC.
 */
public final class Conversions {

    private Conversions() {}

    /** Convert inches to meters. */
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    /** Convert meters to inches. */
    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    /** Convert degrees to radians. */
    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    /** Convert radians to degrees. */
    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    /** Convert RPM to rotations per second. */
    public static double rpmToRPS(double rpm) {
        return rpm / 60.0;
    }

    /** Convert rotations per second to RPM. */
    public static double rpsToRPM(double rps) {
        return rps * 60.0;
    }

    /**
     * Convert motor rotations to linear distance (meters).
     * @param rotations motor rotations
     * @param gearRatio motor rotations per mechanism rotation
     * @param drumRadius drum/pulley radius in meters
     */
    public static double rotationsToLinear(double rotations, double gearRatio, double drumRadius) {
        return (rotations / gearRatio) * (2.0 * Math.PI * drumRadius);
    }

    /**
     * Convert linear distance (meters) to motor rotations.
     * @param meters distance in meters
     * @param gearRatio motor rotations per mechanism rotation
     * @param drumRadius drum/pulley radius in meters
     */
    public static double linearToRotations(double meters, double gearRatio, double drumRadius) {
        return (meters / (2.0 * Math.PI * drumRadius)) * gearRatio;
    }

    /**
     * Convert motor rotations to mechanism angle in degrees.
     * @param rotations motor rotations
     * @param gearRatio motor rotations per mechanism rotation
     */
    public static double rotationsToDegrees(double rotations, double gearRatio) {
        return (rotations / gearRatio) * 360.0;
    }

    /**
     * Convert mechanism angle in degrees to motor rotations.
     * @param degrees angle in degrees
     * @param gearRatio motor rotations per mechanism rotation
     */
    public static double degreesToRotations(double degrees, double gearRatio) {
        return (degrees / 360.0) * gearRatio;
    }

    /** Clamp a value between min and max. */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Deadband a joystick value. */
    public static double deadband(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0;
        return (value - Math.copySign(deadband, value)) / (1.0 - deadband);
    }
}
