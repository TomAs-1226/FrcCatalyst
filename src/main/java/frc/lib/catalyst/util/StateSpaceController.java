package frc.lib.catalyst.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * State-space controller wrapper for FRC mechanisms using LQR + Kalman filter.
 *
 * <p>State-space control provides optimal gain calculation via LQR and
 * noise-filtered state estimation via Kalman filter. This is the approach
 * used by top teams (6328, 254) for precise mechanism control.
 *
 * <p>Advantages over PID:
 * <ul>
 *   <li>LQR automatically computes optimal gains given error and effort constraints</li>
 *   <li>Kalman filter provides lag-free noise rejection (vs moving average which adds lag)</li>
 *   <li>Model-based: changes in gear ratio or motor count auto-adjust gains</li>
 * </ul>
 *
 * <p>Example: Flywheel with state-space control
 * <pre>{@code
 * StateSpaceController.Velocity controller = StateSpaceController.createFlywheel(
 *     DCMotor.getKrakenX60(1), 0.01, 1.5,  // motor, MOI, gearing
 *     3.0,    // model std dev (how much we trust the model)
 *     0.01,   // encoder std dev (how much we trust the encoder)
 *     8.0,    // max acceptable velocity error (rad/s)
 *     12.0    // max voltage
 * );
 *
 * // In periodic:
 * controller.setReference(targetVelocityRadPerSec);
 * controller.correct(encoderVelocityRadPerSec);
 * controller.predict(0.020);
 * motor.setVoltage(controller.getVoltage());
 * }</pre>
 *
 * <p>Example: Elevator with state-space control
 * <pre>{@code
 * StateSpaceController.Position controller = StateSpaceController.createElevator(
 *     DCMotor.getKrakenX60(2), 5.0, 0.0254, 10.0,  // motor, mass, drumRadius, gearing
 *     0.05,   // position model std dev
 *     3.0,    // velocity model std dev
 *     0.001,  // encoder position std dev
 *     0.01,   // encoder velocity std dev
 *     0.02,   // max acceptable position error (m)
 *     0.4,    // max acceptable velocity error (m/s)
 *     12.0    // max voltage
 * );
 * }</pre>
 */
public final class StateSpaceController {

    private StateSpaceController() {}

    /**
     * Velocity-only state-space controller (1 state: velocity).
     * Use for flywheels and other velocity-controlled mechanisms.
     */
    public static class Velocity {
        private final LinearSystemLoop<N1, N1, N1> loop;
        private final LinearSystem<N1, N1, N1> plant;

        public Velocity(LinearSystem<N1, N1, N1> plant,
                         double modelStdDev, double encoderStdDev,
                         double maxVelocityError, double maxVoltage,
                         double dtSeconds) {
            this.plant = plant;

            KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
                    Nat.N1(), Nat.N1(), plant,
                    VecBuilder.fill(modelStdDev),
                    VecBuilder.fill(encoderStdDev),
                    dtSeconds);

            LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(
                    plant,
                    VecBuilder.fill(maxVelocityError),
                    VecBuilder.fill(maxVoltage),
                    dtSeconds);

            this.loop = new LinearSystemLoop<>(plant, controller, observer, maxVoltage, dtSeconds);
        }

        /** Set the desired velocity reference (in the same units as the plant model). */
        public void setReference(double velocity) {
            loop.setNextR(VecBuilder.fill(velocity));
        }

        /** Correct the state estimate with an encoder measurement. */
        public void correct(double measuredVelocity) {
            loop.correct(VecBuilder.fill(measuredVelocity));
        }

        /** Predict the next state (call after correct). */
        public void predict(double dtSeconds) {
            loop.predict(dtSeconds);
        }

        /** Get the calculated control voltage. */
        public double getVoltage() {
            return loop.getU(0);
        }

        /** Get the estimated velocity from the Kalman filter. */
        public double getEstimatedVelocity() {
            return loop.getXHat(0);
        }

        /** Reset the controller state. */
        public void reset(double currentVelocity) {
            loop.reset(VecBuilder.fill(currentVelocity));
        }

        /** Get the underlying plant model. */
        public LinearSystem<N1, N1, N1> getPlant() {
            return plant;
        }
    }

    /**
     * Position + velocity state-space controller (2 states: position, velocity).
     * Use for elevators, arms, and other position-controlled mechanisms.
     *
     * <p>The plant has 2 outputs (position and velocity), matching WPILib's
     * createElevatorSystem and createSingleJointedArmSystem factory methods.
     */
    public static class Position {
        private final LinearSystemLoop<N2, N1, N2> loop;
        private final LinearSystem<N2, N1, N2> plant;

        public Position(LinearSystem<N2, N1, N2> plant,
                          double posModelStdDev, double velModelStdDev,
                          double encoderPosStdDev, double encoderVelStdDev,
                          double maxPosError, double maxVelError, double maxVoltage,
                          double dtSeconds) {
            this.plant = plant;

            KalmanFilter<N2, N1, N2> observer = new KalmanFilter<>(
                    Nat.N2(), Nat.N2(), plant,
                    VecBuilder.fill(posModelStdDev, velModelStdDev),
                    VecBuilder.fill(encoderPosStdDev, encoderVelStdDev),
                    dtSeconds);

            LinearQuadraticRegulator<N2, N1, N2> controller = new LinearQuadraticRegulator<>(
                    plant,
                    VecBuilder.fill(maxPosError, maxVelError),
                    VecBuilder.fill(maxVoltage),
                    dtSeconds);

            this.loop = new LinearSystemLoop<>(plant, controller, observer, maxVoltage, dtSeconds);
        }

        /** Set the desired position reference. */
        public void setReference(double position) {
            loop.setNextR(VecBuilder.fill(position, 0));
        }

        /** Set position and velocity reference. */
        public void setReference(double position, double velocity) {
            loop.setNextR(VecBuilder.fill(position, velocity));
        }

        /** Correct the state estimate with position and velocity measurements. */
        public void correct(double measuredPosition, double measuredVelocity) {
            loop.correct(VecBuilder.fill(measuredPosition, measuredVelocity));
        }

        /**
         * Correct the state estimate with position only.
         * Uses the Kalman filter's current velocity estimate for the velocity measurement.
         */
        public void correct(double measuredPosition) {
            correct(measuredPosition, getEstimatedVelocity());
        }

        /** Predict the next state (call after correct). */
        public void predict(double dtSeconds) {
            loop.predict(dtSeconds);
        }

        /** Get the calculated control voltage. */
        public double getVoltage() {
            return loop.getU(0);
        }

        /** Get estimated position from Kalman filter. */
        public double getEstimatedPosition() {
            return loop.getXHat(0);
        }

        /** Get estimated velocity from Kalman filter. */
        public double getEstimatedVelocity() {
            return loop.getXHat(1);
        }

        /** Reset the controller. */
        public void reset(double currentPosition, double currentVelocity) {
            loop.reset(VecBuilder.fill(currentPosition, currentVelocity));
        }

        /** Get the underlying plant model. */
        public LinearSystem<N2, N1, N2> getPlant() {
            return plant;
        }
    }

    // --- Factory Methods ---

    /**
     * Create a flywheel velocity controller from motor model.
     *
     * @param motor DCMotor model (e.g., DCMotor.getKrakenX60(1))
     * @param moiKgM2 flywheel moment of inertia in kg*m^2
     * @param gearing motor-to-flywheel gear ratio (>1 means reduction)
     * @param modelStdDev how much to trust the model (higher = less trust)
     * @param encoderStdDev how much to trust the encoder (lower = more trust)
     * @param maxVelocityError max acceptable velocity error in rad/s for LQR tuning
     * @param maxVoltage max voltage (usually 12.0)
     */
    public static Velocity createFlywheel(DCMotor motor, double moiKgM2, double gearing,
                                           double modelStdDev, double encoderStdDev,
                                           double maxVelocityError, double maxVoltage) {
        LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(motor, moiKgM2, gearing);
        return new Velocity(plant, modelStdDev, encoderStdDev, maxVelocityError, maxVoltage, 0.020);
    }

    /**
     * Create a flywheel velocity controller from SysId gains.
     *
     * @param kV velocity gain (volts per rad/s)
     * @param kA acceleration gain (volts per rad/s^2)
     */
    public static Velocity createFlywheelFromGains(double kV, double kA,
                                                     double modelStdDev, double encoderStdDev,
                                                     double maxVelocityError, double maxVoltage) {
        LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(kV, kA);
        return new Velocity(plant, modelStdDev, encoderStdDev, maxVelocityError, maxVoltage, 0.020);
    }

    /**
     * Create an elevator position controller from motor model.
     *
     * @param motor DCMotor model
     * @param massKg carriage mass in kg
     * @param drumRadiusMeters spool/drum radius in meters
     * @param gearing motor-to-drum gear ratio
     * @param posModelStdDev position model trust (higher = less trust)
     * @param velModelStdDev velocity model trust
     * @param encoderPosStdDev position encoder trust (lower = more trust)
     * @param encoderVelStdDev velocity encoder trust
     * @param maxPosError max acceptable position error in meters
     * @param maxVelError max acceptable velocity error in m/s
     * @param maxVoltage max voltage
     */
    public static Position createElevator(DCMotor motor, double massKg, double drumRadiusMeters,
                                           double gearing,
                                           double posModelStdDev, double velModelStdDev,
                                           double encoderPosStdDev, double encoderVelStdDev,
                                           double maxPosError, double maxVelError,
                                           double maxVoltage) {
        LinearSystem<N2, N1, N2> plant = LinearSystemId.createElevatorSystem(
                motor, massKg, drumRadiusMeters, gearing);
        return new Position(plant, posModelStdDev, velModelStdDev, encoderPosStdDev, encoderVelStdDev,
                maxPosError, maxVelError, maxVoltage, 0.020);
    }

    /**
     * Create a single-jointed arm position controller from motor model.
     *
     * @param motor DCMotor model
     * @param moiKgM2 arm moment of inertia in kg*m^2
     * @param gearing motor-to-arm gear ratio
     * @param posModelStdDev position model trust
     * @param velModelStdDev velocity model trust
     * @param encoderPosStdDev position encoder trust
     * @param encoderVelStdDev velocity encoder trust
     * @param maxPosError max acceptable angle error in radians
     * @param maxVelError max acceptable angular velocity error in rad/s
     * @param maxVoltage max voltage
     */
    public static Position createArm(DCMotor motor, double moiKgM2, double gearing,
                                      double posModelStdDev, double velModelStdDev,
                                      double encoderPosStdDev, double encoderVelStdDev,
                                      double maxPosError, double maxVelError,
                                      double maxVoltage) {
        LinearSystem<N2, N1, N2> plant = LinearSystemId.createSingleJointedArmSystem(
                motor, moiKgM2, gearing);
        return new Position(plant, posModelStdDev, velModelStdDev, encoderPosStdDev, encoderVelStdDev,
                maxPosError, maxVelError, maxVoltage, 0.020);
    }

    /**
     * Create a position controller from SysId gains.
     *
     * @param kV velocity gain
     * @param kA acceleration gain
     */
    public static Position createPositionFromGains(double kV, double kA,
                                                     double posModelStdDev, double velModelStdDev,
                                                     double encoderPosStdDev, double encoderVelStdDev,
                                                     double maxPosError, double maxVelError,
                                                     double maxVoltage) {
        LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(kV, kA);
        return new Position(plant, posModelStdDev, velModelStdDev, encoderPosStdDev, encoderVelStdDev,
                maxPosError, maxVelError, maxVoltage, 0.020);
    }
}
