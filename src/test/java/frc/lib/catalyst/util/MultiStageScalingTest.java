package frc.lib.catalyst.util;

import frc.lib.catalyst.hardware.MotorType;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Which way {@code stages} goes, derived rather than hard-coded.
 *
 * <p>A cascade elevator trades force for travel: the carriage moves {@code stages} times further
 * than the drum pays out cable, so by power balance the cable tension is {@code stages} times the
 * carriage force. Both consequences were inverted, in both classes that compute them — the holding
 * torque was <em>divided</em> by stages and the available force <em>multiplied</em> by it.
 *
 * <p>Neither failed. A three-stage elevator asked for a third of the gravity feedforward it needed
 * and sagged, while its acceleration limit claimed three times the force the motors could deliver,
 * so it also fell behind its own profile. Both read as tuning problems, and the usual response —
 * add kG by hand until it stops drooping — hides the bug and leaves everything derived from it
 * wrong.
 *
 * <p>The assertions below derive the expected ratios from the kinematics rather than pinning
 * measured constants, so they stay true if the motor model or the units change. Pinning numbers
 * would have locked in whatever the code did on the day, which is how this survived.
 */
class MultiStageScalingTest {

    private static final MotorType MOTOR = MotorType.KRAKEN_X60;
    private static final double GEAR = 10.0;
    private static final double DRUM = 0.02;
    private static final double MASS = 15.0;
    private static final double LIMIT = 40.0;

    private static MotionConstraintCalculator.LinearConstraints at(int stages) {
        return MotionConstraintCalculator.elevator(MOTOR, 2, GEAR, DRUM, MASS, LIMIT, stages, 1.5);
    }

    @Test
    void moreStagesNeedMoreHoldingVoltageNotLess() {
        double one = at(1).gravityFF;
        double three = at(3).gravityFF;

        assertTrue(three > one,
                "a 3-stage elevator holds a longer lever and needs MORE feedforward, not less "
                        + "(1 stage " + one + " V, 3 stages " + three + " V)");
        // holdingVoltage is linear in torque, and torque scales with stages.
        assertEquals(3.0, three / one, 1e-6, "holding voltage should scale linearly with stages");
    }

    @Test
    void moreStagesLeaveLessForceAtTheCarriage() {
        // Same motors, same current limit: spreading the travel over more stages cannot create
        // force. Acceleration going down is force-limited and gravity-assisted, so it is the
        // cleanest place to read the force scaling without the gravity term flipping sign.
        double one = at(1).maxAccelerationDown;
        double three = at(3).maxAccelerationDown;

        assertTrue(three < one,
                "more stages must mean less force at the carriage, not more "
                        + "(1 stage " + one + " m/s^2, 3 stages " + three + " m/s^2)");
    }

    @Test
    void aSingleStageIsUnaffectedEitherWay() {
        // stages == 1 is the case that cannot distinguish multiply from divide, which is why the
        // inversion survived: the default path is identical under both.
        var s = at(1);
        assertTrue(s.gravityFF > 0);
        assertTrue(s.maxVelocity > 0);
    }

    @Test
    void travelAndForceTradeAgainstEachOther() {
        // The invariant behind both: stages buys travel at the cost of force, so their product is
        // what the motors actually supply and must not change with stages.
        var one = at(1);
        var three = at(3);

        // Isolate the motor force from gravity first. accelUp = (F - mg)/m and
        // accelDown = (F + mg)/m, so their mean is F/m and the gravity term drops out. Using
        // accelDown alone leaves +g in the product, which does not scale with stages - that was
        // wrong in the first draft of this test, and it failed, which is the test working.
        double forceOne = MASS * (one.maxAccelerationUp + one.maxAccelerationDown) / 2.0;
        double forceThree = MASS * (three.maxAccelerationUp + three.maxAccelerationDown) / 2.0;

        assertEquals(1.0, (three.maxVelocity * forceThree) / (one.maxVelocity * forceOne), 1e-6,
                "speed x force is set by the motors and must be independent of staging");
    }
}
