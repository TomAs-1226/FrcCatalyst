package frc.lib.catalyst.physics.estimation;

import org.junit.jupiter.api.Test;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * {@code reset()} has to clear the slip budget.
 *
 * <p>The budget is how long the estimator has been dead-reckoning through reported wheel slip. When
 * it is exhausted the estimator stops trusting the wheels, which is the entire point of it — and it
 * was the one mutable field {@code reset()} did not clear.
 *
 * <p>The consequence is quiet and badly timed. A team calling {@code physics.reset()} at the
 * auto-to-teleop boundary, after a long shove in auto, carried an exhausted budget into teleop: the
 * wheels were over-trusted by roughly two hundred times through the very next slip, which is exactly
 * the case the fusion exists for, and it stayed that way for about a second of clean driving.
 *
 * <p>The existing estimator tests exercise the budget draining and refilling. None of them called
 * {@code reset()} in between.
 */
class SlipBudgetResetTest {

    private static final double DT = 0.02;

    private static PhysicalStateEstimator estimator() {
        return PhysicalStateEstimator.builder().build();
    }

    /** One loop, at the given slip factor. */
    private static void step(PhysicalStateEstimator est, double t, double slipFactor) {
        est.update(
                t,
                new Pose2d(0, 0, Rotation2d.ZERO),
                new ChassisVelocities(1.0, 0, 0),
                new Translation2d(0, 0),
                0.0,
                slipFactor);
    }

    /** Slip hard for long enough to spend the whole budget. */
    private static void exhaust(PhysicalStateEstimator est) {
        for (int i = 0; i < 150; i++) {
            step(est, i * DT, 1.0);
        }
    }

    @Test
    void aLongSlipActuallySpendsTheBudget() {
        // The premise. If this stopped being true the test below would pass for the wrong reason.
        PhysicalStateEstimator est = estimator();
        exhaust(est);

        assertTrue(est.slipBudgetExhaustion() > 0.9,
                "expected the budget to be spent, was " + est.slipBudgetExhaustion());
    }

    @Test
    void resetClearsIt() {
        PhysicalStateEstimator est = estimator();
        exhaust(est);
        est.reset();

        assertEquals(0.0, est.slipSeconds(), 1e-9);
        assertEquals(0.0, est.slipBudgetExhaustion(), 1e-9);
    }

    @Test
    void afterAResetTheWheelsAreTrustedAgain() {
        // Stated as the behaviour a team would notice rather than as a field value. An exhausted
        // budget means the estimator has stopped believing the wheels; a reset has to give that
        // back, or the fusion is switched off through the next slip.
        PhysicalStateEstimator exhausted = estimator();
        exhaust(exhausted);
        double whileExhausted = exhausted.kinematicWeight(1.0);

        exhausted.reset();
        double afterReset = exhausted.kinematicWeight(1.0);

        PhysicalStateEstimator fresh = estimator();
        assertEquals(fresh.kinematicWeight(1.0), afterReset, 1e-9,
                "a reset estimator should behave exactly like a new one");
        assertTrue(afterReset < whileExhausted,
                "an exhausted budget over-trusts the wheels; the reset must undo that");
    }
}
