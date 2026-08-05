package frc.lib.catalyst.physics.model;

import java.util.Locale;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * What the carpet and gravity will actually let your drivetrain do.
 *
 * <p>Two limits matter, and a robot is bound by whichever comes first:
 *
 * <ul>
 *   <li><b>Traction</b> — push harder than {@code mu * g} and the wheels break loose. Beyond this
 *       point the encoders keep reporting acceleration the robot is not getting, which is exactly
 *       the failure mode {@link frc.lib.catalyst.physics.estimation.SlipEstimator} looks for.</li>
 *   <li><b>Tipping</b> — accelerate harder than {@code g * halfWidth / comHeight} and the robot
 *       rotates about its outside wheels. A low robot never reaches this; a robot with its elevator
 *       up can reach it well before it reaches the traction limit.</li>
 * </ul>
 *
 * <p>Everything here is a closed-form function of {@link RobotModel}. There is no state and no
 * hardware, so it is cheap to call inside a loop and trivial to unit test.
 *
 * <pre>{@code
 * DrivetrainModel drivetrain = new DrivetrainModel(model);
 *
 * double limit = drivetrain.maxSafeAccelerationMpsSq();       // min(traction, tipping)
 * double stop  = drivetrain.stoppingDistanceMeters(4.2);      // metres, from 4.2 m/s
 * double used  = drivetrain.tractionUsage(state.fieldAcceleration());  // 0..1+, >1 means slipping
 * }</pre>
 *
 * @since 1.5.0
 */
public final class DrivetrainModel {

    private final RobotModel robot;

    /** @param robot the physical model this drivetrain belongs to */
    public DrivetrainModel(RobotModel robot) {
        if (robot == null) throw new IllegalArgumentException("robot model must not be null");
        this.robot = robot;
    }

    /** The model these limits were derived from. */
    public RobotModel robot() {
        return robot;
    }

    /**
     * Hardest the robot can accelerate before the wheels slip, in m/s^2 — {@code mu * g}.
     *
     * <p>Note what this does <em>not</em> depend on: mass. A heavier robot needs more force to
     * accelerate and gets proportionally more grip to do it with, and the two cancel.
     */
    public double maxTractionAccelerationMpsSq() {
        return robot.coefficientOfFriction() * RobotModel.GRAVITY;
    }

    /** Total force the carpet can transmit before slipping, in newtons — {@code mu * m * g}. */
    public double maxTractionForceNewtons() {
        return maxTractionAccelerationMpsSq() * robot.massKg();
    }

    /**
     * Hardest the robot can accelerate before it starts to tip, in m/s^2 —
     * {@code g * halfWidth / comHeight}.
     *
     * <p>This is the static limit: the acceleration at which the load on the inside wheels reaches
     * zero. Real tipping needs a little more than this because the robot has to rotate through some
     * angle first, so treating this as the ceiling leaves margin rather than eating into it.
     */
    public double maxTippingAccelerationMpsSq() {
        return RobotModel.GRAVITY * robot.tippingHalfWidthMeters() / robot.centerOfMassHeightMeters();
    }

    /** The binding limit: the lower of {@link #maxTractionAccelerationMpsSq} and {@link #maxTippingAccelerationMpsSq}. */
    public double maxSafeAccelerationMpsSq() {
        return Math.min(maxTractionAccelerationMpsSq(), maxTippingAccelerationMpsSq());
    }

    /** True when tipping binds before traction does — the case where a tall robot is the problem. */
    public boolean isTippingLimited() {
        return maxTippingAccelerationMpsSq() < maxTractionAccelerationMpsSq();
    }

    /**
     * The safe acceleration limit with a margin applied, in m/s^2.
     *
     * @param usableFraction how much of the physical limit to actually use, {@code (0, 1]};
     *                       0.8 is a reasonable competition value
     */
    public double maxSafeAccelerationMpsSq(double usableFraction) {
        double fraction = Math.max(0.0, Math.min(1.0, usableFraction));
        return maxSafeAccelerationMpsSq() * fraction;
    }

    /**
     * How far the robot travels before stopping from {@code speedMetersPerSecond}, braking at the
     * safe limit — {@code v^2 / 2a}. Zero for a stationary robot.
     */
    public double stoppingDistanceMeters(double speedMetersPerSecond) {
        return stoppingDistanceMeters(speedMetersPerSecond, 1.0);
    }

    /**
     * Stopping distance while using only {@code usableFraction} of the safe limit. Braking gently
     * takes proportionally more room.
     */
    public double stoppingDistanceMeters(double speedMetersPerSecond, double usableFraction) {
        double speed = Math.abs(speedMetersPerSecond);
        double decel = maxSafeAccelerationMpsSq(usableFraction);
        if (decel <= 0) return Double.POSITIVE_INFINITY;
        return (speed * speed) / (2.0 * decel);
    }

    /**
     * The fastest the robot may travel and still stop within {@code distanceMeters}. The inverse of
     * {@link #stoppingDistanceMeters(double)}, and the useful form when approaching something solid.
     */
    public double maxSpeedForStoppingDistance(double distanceMeters) {
        if (distanceMeters <= 0) return 0.0;
        return Math.sqrt(2.0 * maxSafeAccelerationMpsSq() * distanceMeters);
    }

    /**
     * What fraction of available grip an acceleration is using: {@code 0} coasting, {@code 1} right
     * at the traction limit, above {@code 1} more than the carpet can deliver — which means either
     * the wheels are slipping or something outside the robot is pushing it.
     */
    public double tractionUsage(Translation2d fieldAcceleration) {
        if (fieldAcceleration == null) return 0.0;
        return fieldAcceleration.getNorm() / maxTractionAccelerationMpsSq();
    }

    /**
     * How much of the tipping margin an acceleration is using, on the same {@code 0..1+} scale.
     * Only meaningful for a robot whose centre of mass is high enough to matter.
     */
    public double tippingUsage(Translation2d fieldAcceleration) {
        if (fieldAcceleration == null) return 0.0;
        return fieldAcceleration.getNorm() / maxTippingAccelerationMpsSq();
    }

    /**
     * Acceleration the drivetrain would produce from a total wheel force, in m/s^2 — {@code F / m},
     * clipped at the traction limit because the carpet cannot transmit more than that no matter what
     * the motors are doing.
     */
    public double accelerationFromForce(double totalWheelForceNewtons) {
        double unclipped = totalWheelForceNewtons / robot.massKg();
        double limit = maxTractionAccelerationMpsSq();
        return Math.max(-limit, Math.min(limit, unclipped));
    }

    /** One line naming both limits and which one binds. */
    public String describe() {
        return String.format(Locale.ROOT,
                "DrivetrainModel[traction %.1f m/s^2, tipping %.1f m/s^2, limited by %s]",
                maxTractionAccelerationMpsSq(), maxTippingAccelerationMpsSq(),
                isTippingLimited() ? "tipping" : "traction");
    }
}
