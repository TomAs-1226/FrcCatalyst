package frc.lib.catalyst.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Math utilities for FRC robot programming.
 * Includes joystick response curves, geometry helpers,
 * angle math, and common calculations.
 */
public final class CatalystMath {

    private CatalystMath() {}

    // ==========================================
    //           JOYSTICK RESPONSE CURVES
    // ==========================================

    /**
     * Apply a deadband and then rescale so the output starts from 0.
     * @param input raw joystick input [-1, 1]
     * @param deadband threshold below which output is 0 (e.g., 0.05)
     * @return processed value [-1, 1] with deadband applied
     */
    public static double deadband(double input, double deadband) {
        if (Math.abs(input) < deadband) return 0;
        return Math.signum(input) * ((Math.abs(input) - deadband) / (1.0 - deadband));
    }

    /**
     * Square the input while preserving sign. Gives more precision at low speeds.
     * @param input joystick input [-1, 1]
     * @return squared input [-1, 1]
     */
    public static double squareInput(double input) {
        return Math.copySign(input * input, input);
    }

    /**
     * Cube the input while preserving sign. Even more precision at low speeds.
     * @param input joystick input [-1, 1]
     * @return cubed input [-1, 1]
     */
    public static double cubeInput(double input) {
        return input * input * input;
    }

    /**
     * Apply an exponential curve to joystick input.
     * Higher exponent = more precision at low speeds, less at high.
     * @param input joystick input [-1, 1]
     * @param exponent curve exponent (1 = linear, 2 = square, 3 = cube)
     * @return curved input [-1, 1]
     */
    public static double curveInput(double input, double exponent) {
        return Math.copySign(Math.pow(Math.abs(input), exponent), input);
    }

    /**
     * Complete joystick processing: deadband + curve + optional scaling.
     * @param input raw joystick value [-1, 1]
     * @param deadband deadband threshold (e.g., 0.05)
     * @param exponent curve exponent (1 = linear, 2 = square)
     * @param maxOutput maximum output magnitude (e.g., 1.0 or 0.5 for slow mode)
     * @return processed value [-maxOutput, maxOutput]
     */
    public static double processJoystick(double input, double deadband, double exponent, double maxOutput) {
        double processed = deadband(input, deadband);
        processed = curveInput(processed, exponent);
        return processed * maxOutput;
    }

    // ==========================================
    //                ANGLE MATH
    // ==========================================

    /**
     * Normalize an angle to [-180, 180] degrees.
     */
    public static double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) degrees -= 360;
        if (degrees < -180) degrees += 360;
        return degrees;
    }

    /**
     * Get the shortest angular distance between two angles (degrees).
     * Result is signed: positive = counterclockwise.
     */
    public static double angleDifference(double fromDegrees, double toDegrees) {
        return normalizeAngle(toDegrees - fromDegrees);
    }

    /**
     * Check if an angle is within a tolerance of a target (degrees).
     */
    public static boolean angleWithinTolerance(double currentDeg, double targetDeg, double toleranceDeg) {
        return Math.abs(angleDifference(currentDeg, targetDeg)) < toleranceDeg;
    }

    /**
     * Wrap a continuous encoder value to [0, 360) degrees.
     */
    public static double wrapTo360(double degrees) {
        degrees = degrees % 360;
        if (degrees < 0) degrees += 360;
        return degrees;
    }

    // ==========================================
    //              GEOMETRY HELPERS
    // ==========================================

    /**
     * Calculate distance between two poses in meters.
     */
    public static double distanceBetween(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }

    /**
     * Calculate the angle from one pose to another (in the field frame).
     * @return angle in degrees
     */
    public static double angleTo(Pose2d from, Pose2d to) {
        Translation2d delta = to.getTranslation().minus(from.getTranslation());
        return Math.toDegrees(Math.atan2(delta.getY(), delta.getX()));
    }

    /**
     * Check if a pose is within a circular region.
     * @param pose the pose to check
     * @param center center of the region
     * @param radiusMeters radius of the region
     */
    public static boolean isWithinRadius(Pose2d pose, Translation2d center, double radiusMeters) {
        return pose.getTranslation().getDistance(center) <= radiusMeters;
    }

    /**
     * Check if a pose is within a rectangular region (axis-aligned).
     */
    public static boolean isWithinRectangle(Pose2d pose, double minX, double minY,
                                            double maxX, double maxY) {
        double x = pose.getX();
        double y = pose.getY();
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    /**
     * Mirror a pose across the field centerline (for red/blue alliance symmetry).
     * Assumes standard FRC field where the field is 16.54m long.
     * @param pose blue-alliance pose
     * @param fieldLengthMeters field length (default 16.54 for 2025+)
     * @return mirrored (red-alliance) pose
     */
    public static Pose2d mirrorPose(Pose2d pose, double fieldLengthMeters) {
        return new Pose2d(
                fieldLengthMeters - pose.getX(),
                pose.getY(),
                new Rotation2d(Math.PI).minus(pose.getRotation()));
    }

    /** Mirror pose for a standard FRC field (16.54m). */
    public static Pose2d mirrorPose(Pose2d pose) {
        return mirrorPose(pose, 16.54);
    }

    // ==========================================
    //             PHYSICS HELPERS
    // ==========================================

    /**
     * Calculate the feedforward voltage for an elevator to hold against gravity.
     * @param massKg mass being lifted
     * @param drumRadiusMeters radius of the spool/drum
     * @param gearRatio motor-to-mechanism gear ratio
     * @param motorStallTorqueNm stall torque of the motor (e.g., 7.09 for Kraken X60)
     * @return approximate voltage needed to hold position
     */
    public static double elevatorGravityFF(double massKg, double drumRadiusMeters,
                                           double gearRatio, double motorStallTorqueNm) {
        double force = massKg * 9.81; // N
        double torqueAtDrum = force * drumRadiusMeters; // Nm
        double torqueAtMotor = torqueAtDrum / gearRatio; // Nm
        return (torqueAtMotor / motorStallTorqueNm) * 12.0; // volts
    }

    /**
     * Calculate the feedforward voltage for an arm at a given angle.
     * @param massKg mass of the arm (center of mass)
     * @param lengthMeters distance from pivot to center of mass
     * @param angleDegrees current angle from horizontal
     * @param gearRatio motor-to-mechanism gear ratio
     * @param motorStallTorqueNm stall torque of the motor
     * @return approximate voltage needed to hold position
     */
    public static double armGravityFF(double massKg, double lengthMeters, double angleDegrees,
                                      double gearRatio, double motorStallTorqueNm) {
        double torqueAtJoint = massKg * 9.81 * lengthMeters * Math.cos(Math.toRadians(angleDegrees));
        double torqueAtMotor = torqueAtJoint / gearRatio;
        return (torqueAtMotor / motorStallTorqueNm) * 12.0;
    }

    /**
     * Estimate time for a flywheel to spin up to target speed.
     * @param targetRPS target speed in rotations per second
     * @param moiKgM2 moment of inertia in kg*m^2
     * @param motorCount number of motors
     * @param gearRatio motor-to-flywheel gear ratio
     * @param motorStallTorqueNm stall torque per motor
     * @return estimated spin-up time in seconds
     */
    public static double flywheelSpinUpTime(double targetRPS, double moiKgM2, int motorCount,
                                            double gearRatio, double motorStallTorqueNm) {
        double targetRadPerSec = targetRPS * 2 * Math.PI;
        double availableTorque = motorCount * motorStallTorqueNm * gearRatio * 0.5; // ~50% avg
        double angularAccel = availableTorque / moiKgM2;
        return targetRadPerSec / angularAccel;
    }

    // ==========================================
    //              COMMON FRC CONSTANTS
    // ==========================================

    /** Kraken X60 stall torque in Nm. */
    public static final double KRAKEN_STALL_TORQUE = 7.09;

    /** Kraken X60 free speed in RPM. */
    public static final double KRAKEN_FREE_SPEED_RPM = 6000;

    /** Falcon 500 stall torque in Nm. */
    public static final double FALCON_STALL_TORQUE = 4.69;

    /** Falcon 500 free speed in RPM. */
    public static final double FALCON_FREE_SPEED_RPM = 6380;

    /** Standard FRC field length in meters (2025+). */
    public static final double FIELD_LENGTH = 16.54;

    /** Standard FRC field width in meters. */
    public static final double FIELD_WIDTH = 8.21;

    /** Gravity acceleration constant in m/s^2. */
    public static final double G = 9.81;
}
