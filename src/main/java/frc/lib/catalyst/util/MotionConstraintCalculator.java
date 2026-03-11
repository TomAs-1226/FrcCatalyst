package frc.lib.catalyst.util;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.catalyst.hardware.MotorType;

/**
 * Physics-based motion constraint calculator for FRC mechanisms.
 * Computes physically realistic max velocities, accelerations, and torques
 * based on motor specifications and mechanism geometry.
 *
 * <p>Useful for setting Motion Magic / trajectory constraints that
 * match actual motor capabilities instead of arbitrary tuning values.
 *
 * <p>Example:
 * <pre>{@code
 * // Calculate elevator constraints
 * var constraints = MotionConstraintCalculator.elevator(
 *     MotorType.KRAKEN_X60, 2,      // motor type, motor count
 *     10.0,                          // gear ratio
 *     0.0254,                        // drum radius (1 inch)
 *     5.0,                           // mass (kg)
 *     40.0                           // current limit (amps)
 * );
 * System.out.println("Max velocity: " + constraints.maxVelocity + " m/s");
 * System.out.println("Max accel: " + constraints.maxAcceleration + " m/s^2");
 * System.out.println("Gravity FF: " + constraints.gravityFF + " V");
 *
 * // Use the constraints for Motion Magic
 * double cruiseVelocity = constraints.maxVelocityRotations * 0.8; // 80% of max
 * double acceleration = constraints.maxAccelerationRotations * 0.6;
 * }</pre>
 */
public final class MotionConstraintCalculator {

    private MotionConstraintCalculator() {}

    /** Computed constraints for a linear mechanism. */
    public static class LinearConstraints {
        /** Max linear velocity in m/s at free speed. */
        public final double maxVelocity;
        /** Max linear acceleration in m/s^2 (current-limited). */
        public final double maxAcceleration;
        /** Max linear acceleration in m/s^2 going UP (against gravity). */
        public final double maxAccelerationUp;
        /** Max linear acceleration in m/s^2 going DOWN (with gravity). */
        public final double maxAccelerationDown;
        /** Voltage to hold position against gravity. */
        public final double gravityFF;
        /** Max velocity in mechanism rotations/s (for Motion Magic). */
        public final double maxVelocityRotations;
        /** Max acceleration in mechanism rotations/s^2 (for Motion Magic). */
        public final double maxAccelerationRotations;
        /** Max continuous force in Newtons (current-limited). */
        public final double maxForce;
        /** Time to travel full range at max speed in seconds. */
        public final double fullTravelTime;

        LinearConstraints(double maxVel, double maxAccel, double maxAccelUp, double maxAccelDown,
                           double gravityFF, double maxVelRot, double maxAccelRot,
                           double maxForce, double fullTravelTime) {
            this.maxVelocity = maxVel;
            this.maxAcceleration = maxAccel;
            this.maxAccelerationUp = maxAccelUp;
            this.maxAccelerationDown = maxAccelDown;
            this.gravityFF = gravityFF;
            this.maxVelocityRotations = maxVelRot;
            this.maxAccelerationRotations = maxAccelRot;
            this.maxForce = maxForce;
            this.fullTravelTime = fullTravelTime;
        }
    }

    /** Computed constraints for a rotational mechanism. */
    public static class RotationalConstraints {
        /** Max angular velocity in degrees/s at free speed. */
        public final double maxVelocityDPS;
        /** Max angular acceleration in degrees/s^2 (current-limited). */
        public final double maxAccelerationDPS2;
        /** Peak gravity FF voltage (at horizontal). */
        public final double peakGravityFF;
        /** Max velocity in mechanism rotations/s (for Motion Magic). */
        public final double maxVelocityRotations;
        /** Max acceleration in mechanism rotations/s^2 (for Motion Magic). */
        public final double maxAccelerationRotations;
        /** Max continuous torque at the mechanism output in Nm. */
        public final double maxTorque;
        /** Time to travel full range at max speed in seconds. */
        public final double fullTravelTime;

        RotationalConstraints(double maxVelDPS, double maxAccelDPS2, double peakGravityFF,
                               double maxVelRot, double maxAccelRot, double maxTorque,
                               double fullTravelTime) {
            this.maxVelocityDPS = maxVelDPS;
            this.maxAccelerationDPS2 = maxAccelDPS2;
            this.peakGravityFF = peakGravityFF;
            this.maxVelocityRotations = maxVelRot;
            this.maxAccelerationRotations = maxAccelRot;
            this.maxTorque = maxTorque;
            this.fullTravelTime = fullTravelTime;
        }
    }

    /** Computed constraints for a flywheel. */
    public static class FlywheelConstraints {
        /** Max velocity in RPS (rotations per second) at free speed. */
        public final double maxVelocityRPS;
        /** Estimated spin-up time to 90% of max speed in seconds. */
        public final double spinUpTime90Percent;
        /** Max continuous torque at the flywheel in Nm. */
        public final double maxTorque;
        /** Surface speed in m/s (if wheel radius provided). */
        public final double surfaceSpeedMPS;

        FlywheelConstraints(double maxVelRPS, double spinUpTime, double maxTorque,
                             double surfaceSpeed) {
            this.maxVelocityRPS = maxVelRPS;
            this.spinUpTime90Percent = spinUpTime;
            this.maxTorque = maxTorque;
            this.surfaceSpeedMPS = surfaceSpeed;
        }
    }

    /**
     * Calculate constraints for an elevator mechanism.
     *
     * @param motorType motor type
     * @param motorCount number of motors driving the mechanism
     * @param gearRatio motor-to-mechanism gear ratio (motor rotations per mechanism rotation)
     * @param drumRadiusMeters spool/drum radius in meters
     * @param massKg mass of the carriage/load in kg
     * @param currentLimitAmps per-motor current limit in amps
     */
    public static LinearConstraints elevator(MotorType motorType, int motorCount,
                                              double gearRatio, double drumRadiusMeters,
                                              double massKg, double currentLimitAmps) {
        return elevator(motorType, motorCount, gearRatio, drumRadiusMeters,
                massKg, currentLimitAmps, 1, 1.0);
    }

    /**
     * Calculate constraints for an elevator with travel range.
     */
    public static LinearConstraints elevator(MotorType motorType, int motorCount,
                                              double gearRatio, double drumRadiusMeters,
                                              double massKg, double currentLimitAmps,
                                              int stages, double travelMeters) {
        DCMotor dcMotor = motorType.getDCMotor(motorCount);

        // Max velocity: free speed / gear ratio * drum circumference * stages
        double maxMotorRPS = motorType.freeSpeedRPS();
        double maxMechRPS = maxMotorRPS / gearRatio;
        double maxVelocity = maxMechRPS * 2.0 * Math.PI * drumRadiusMeters * stages;

        // Current-limited torque per motor
        double torquePerAmp = motorType.stallTorqueNm / motorType.stallCurrentAmps;
        double limitedTorquePerMotor = currentLimitAmps * torquePerAmp;
        double totalMotorTorque = limitedTorquePerMotor * motorCount;

        // Force at the carriage
        double mechanismTorque = totalMotorTorque * gearRatio;
        double maxForce = mechanismTorque / drumRadiusMeters * stages;

        // Gravity force
        double gravityForce = massKg * 9.81;

        // Accelerations
        double maxAccelUp = (maxForce - gravityForce) / massKg;
        double maxAccelDown = (maxForce + gravityForce) / massKg;
        double maxAccel = Math.min(maxAccelUp, maxAccelDown);

        // Gravity feedforward
        double gravityTorqueAtDrum = gravityForce * drumRadiusMeters / stages;
        double gravityFF = motorType.holdingVoltage(gravityTorqueAtDrum, gearRatio);

        // Rotational equivalents for Motion Magic
        double maxVelRot = maxMechRPS;
        double maxAccelRot = maxAccel / (2.0 * Math.PI * drumRadiusMeters * stages);

        // Travel time estimate (trapezoidal profile)
        double fullTravelTime = estimateTrapezoidTime(travelMeters, maxVelocity * 0.8, maxAccel * 0.6);

        return new LinearConstraints(maxVelocity, maxAccel, maxAccelUp, maxAccelDown,
                gravityFF, maxVelRot, maxAccelRot, maxForce, fullTravelTime);
    }

    /**
     * Calculate constraints for a rotational mechanism (arm, wrist).
     *
     * @param motorType motor type
     * @param motorCount number of motors
     * @param gearRatio motor-to-mechanism gear ratio
     * @param armLengthMeters pivot to center of mass distance
     * @param massKg arm mass in kg
     * @param currentLimitAmps per-motor current limit
     * @param rangeDegrees total range of motion in degrees
     */
    public static RotationalConstraints arm(MotorType motorType, int motorCount,
                                             double gearRatio, double armLengthMeters,
                                             double massKg, double currentLimitAmps,
                                             double rangeDegrees) {
        DCMotor dcMotor = motorType.getDCMotor(motorCount);

        // Max angular velocity
        double maxMotorRPS = motorType.freeSpeedRPS();
        double maxMechRPS = maxMotorRPS / gearRatio;
        double maxVelocityDPS = maxMechRPS * 360.0;

        // Current-limited torque
        double torquePerAmp = motorType.stallTorqueNm / motorType.stallCurrentAmps;
        double limitedTorquePerMotor = currentLimitAmps * torquePerAmp;
        double totalMotorTorque = limitedTorquePerMotor * motorCount;
        double maxMechTorque = totalMotorTorque * gearRatio;

        // MOI estimate (point mass at end of arm)
        double moi = massKg * armLengthMeters * armLengthMeters;

        // Max angular acceleration
        double maxAngAccelRadS2 = maxMechTorque / moi;
        double maxAccelDPS2 = Math.toDegrees(maxAngAccelRadS2);

        // Peak gravity FF (at horizontal)
        double peakGravityTorque = massKg * 9.81 * armLengthMeters;
        double peakGravityFF = motorType.holdingVoltage(peakGravityTorque, gearRatio);

        // Rotational equivalents for Motion Magic
        double maxVelRot = maxMechRPS;
        double maxAccelRot = maxAngAccelRadS2 / (2.0 * Math.PI);

        // Travel time
        double fullTravelTime = estimateTrapezoidTime(
                Math.toRadians(rangeDegrees),
                maxMechRPS * 2.0 * Math.PI * 0.8,
                maxAngAccelRadS2 * 0.6);

        return new RotationalConstraints(maxVelocityDPS, maxAccelDPS2, peakGravityFF,
                maxVelRot, maxAccelRot, maxMechTorque, fullTravelTime);
    }

    /**
     * Calculate constraints for a flywheel mechanism.
     *
     * @param motorType motor type
     * @param motorCount number of motors
     * @param gearRatio motor-to-flywheel gear ratio
     * @param moiKgM2 moment of inertia in kg*m^2
     * @param currentLimitAmps per-motor current limit
     * @param wheelRadiusMeters flywheel radius (for surface speed, 0 to skip)
     */
    public static FlywheelConstraints flywheel(MotorType motorType, int motorCount,
                                                double gearRatio, double moiKgM2,
                                                double currentLimitAmps,
                                                double wheelRadiusMeters) {
        // Max velocity
        double maxMotorRPS = motorType.freeSpeedRPS();
        double maxFlywheelRPS = maxMotorRPS / gearRatio;

        // Current-limited torque
        double torquePerAmp = motorType.stallTorqueNm / motorType.stallCurrentAmps;
        double limitedTorquePerMotor = currentLimitAmps * torquePerAmp;
        double totalMotorTorque = limitedTorquePerMotor * motorCount;
        double flywheelTorque = totalMotorTorque * gearRatio;

        // Spin-up time to 90% (using average torque approximation)
        double targetRadPerSec = maxFlywheelRPS * 0.9 * 2.0 * Math.PI;
        double angAccel = flywheelTorque / moiKgM2;
        double spinUpTime = targetRadPerSec / angAccel;

        // Surface speed
        double surfaceSpeed = wheelRadiusMeters > 0
                ? maxFlywheelRPS * 2.0 * Math.PI * wheelRadiusMeters
                : 0;

        return new FlywheelConstraints(maxFlywheelRPS, spinUpTime, flywheelTorque, surfaceSpeed);
    }

    /**
     * Estimate time for a trapezoidal profile to cover a distance.
     */
    private static double estimateTrapezoidTime(double distance, double maxVelocity, double maxAcceleration) {
        if (maxVelocity <= 0 || maxAcceleration <= 0) return Double.MAX_VALUE;

        double accelTime = maxVelocity / maxAcceleration;
        double accelDistance = 0.5 * maxAcceleration * accelTime * accelTime;

        if (2 * accelDistance >= distance) {
            // Triangle profile (never reaches max velocity)
            return 2.0 * Math.sqrt(distance / maxAcceleration);
        } else {
            // Trapezoid profile
            double cruiseDistance = distance - 2 * accelDistance;
            double cruiseTime = cruiseDistance / maxVelocity;
            return 2 * accelTime + cruiseTime;
        }
    }
}
