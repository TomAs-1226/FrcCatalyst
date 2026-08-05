package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.lib.catalyst.physics.model.ArticulatedRobotModel;
import frc.lib.catalyst.physics.model.DrivetrainModel;
import frc.lib.catalyst.physics.model.MechanismModel;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.model.StabilityModel;

/** Pure-Java tests for {@link StabilityModel}, cross-checked against the closed-form limit. */
class StabilityModelTest {

    /** A square 0.8 x 0.8 m robot, 60 kg, centre of mass 0.25 m up and centred. */
    private static RobotModel squareChassis() {
        return RobotModel.builder()
                .massKg(60.0)
                .footprintMeters(0.8, 0.8)
                .centerOfMassHeightMeters(0.25)
                .coefficientOfFriction(1.0)
                .build();
    }

    private static StabilityModel centred() {
        return StabilityModel.ofChassisOnly(squareChassis());
    }

    @Test
    void aStationaryBalancedRobotSitsInTheMiddleOfItsFootprint() {
        StabilityModel stability = centred();

        Translation2d zmp = stability.zeroMomentPoint(Translation2d.kZero);
        assertEquals(0.0, zmp.getX(), 1e-9);
        assertEquals(0.0, zmp.getY(), 1e-9);
        // Half the 0.8 m footprint: 0.4 m of margin in every direction.
        assertEquals(0.4, stability.tipMarginMeters(Translation2d.kZero), 1e-9);
        assertFalse(stability.isTipping(Translation2d.kZero));
    }

    @Test
    void theZeroMomentPointMovesOppositeTheAcceleration() {
        StabilityModel stability = centred();

        // Accelerating forward tips the robot back, so the ground reaction moves backward.
        Translation2d zmp = stability.zeroMomentPoint(new Translation2d(RobotModel.GRAVITY, 0.0));
        // lever = h/g = 0.25/9.80665, times a = g, so the shift is exactly h.
        assertEquals(-0.25, zmp.getX(), 1e-9);
        assertEquals(0.0, zmp.getY(), 1e-9);
        assertEquals(0.4 - 0.25, stability.tipMarginMeters(new Translation2d(RobotModel.GRAVITY, 0.0)), 1e-9);
    }

    @Test
    void theTippingLimitAgreesWithTheClosedFormWhenTheMassIsCentred() {
        // Independent derivation: a = g · halfWidth / comHeight. The full ZMP calculation must
        // reproduce it exactly in the symmetric case, or one of the two is wrong.
        StabilityModel stability = centred();
        DrivetrainModel simple = new DrivetrainModel(squareChassis());

        assertEquals(simple.maxTippingAccelerationMpsSq(),
                stability.maxAccelerationMpsSq(new Translation2d(1, 0)), 1e-9);
        assertEquals(simple.maxTippingAccelerationMpsSq(),
                stability.maxAccelerationMpsSq(new Translation2d(0, 1)), 1e-9);
        assertEquals(simple.maxTippingAccelerationMpsSq(), stability.worstCaseAccelerationMpsSq(), 1e-9);
    }

    @Test
    void acceleratingRightAtTheLimitLeavesExactlyNoMargin() {
        StabilityModel stability = centred();
        double limit = stability.maxAccelerationMpsSq(new Translation2d(1, 0));

        assertEquals(0.0, stability.tipMarginMeters(new Translation2d(limit, 0)), 1e-9);
        assertTrue(stability.isTipping(new Translation2d(limit * 1.01, 0)));
        assertFalse(stability.isTipping(new Translation2d(limit * 0.99, 0)));
    }

    @Test
    void raisingAnElevatorTightensTheTippingLimit() {
        double[] height = {0.0};
        ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
                .chassis(squareChassis())
                .add(MechanismModel.linear("Elevator", 12.0)
                        .mountedAt(new Translation3d(0, 0, 0.2))
                        .position(() -> height[0]).build())
                .build();
        StabilityModel stability = new StabilityModel(robot);

        double stowedLimit = stability.worstCaseAccelerationMpsSq();
        height[0] = 1.4;
        double raisedLimit = stability.worstCaseAccelerationMpsSq();

        assertTrue(raisedLimit < stowedLimit,
                "raising 12 kg by 1.4 m must reduce the limit, got " + raisedLimit + " vs " + stowedLimit);
        // The limit is g·halfWidth/h, so it falls in proportion to the centre-of-mass height.
        assertEquals(RobotModel.GRAVITY * 0.4 / robot.centerOfMassHeightMeters(), raisedLimit, 1e-9);
    }

    @Test
    void anOffsetCenterOfMassMakesTheRobotHarderToTipOneWayThanTheOther() {
        // Put 20 kg well forward, which moves the CoM toward the front wheels.
        ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
                .chassis(squareChassis())
                .add(MechanismModel.fixed("Nose", 20.0, new Translation3d(0.30, 0.0, 0.25)))
                .build();
        StabilityModel stability = new StabilityModel(robot);

        double forward = stability.maxAccelerationMpsSq(new Translation2d(1, 0));
        double backward = stability.maxAccelerationMpsSq(new Translation2d(-1, 0));

        // Accelerating forward pushes the ZMP backward, away from the already-forward CoM, so there
        // is more room that way and less the other.
        assertTrue(forward > backward, "forward " + forward + " should exceed backward " + backward);
        assertEquals(stability.worstCaseAccelerationMpsSq(), backward, 1e-9);
    }

    @Test
    void wheelLoadsSumToTheRobotsWeightAndShiftUnderAcceleration() {
        StabilityModel stability = centred();
        double weight = 60.0 * RobotModel.GRAVITY;

        double[] resting = stability.wheelLoadsNewtons(Translation2d.kZero);
        assertEquals(weight, resting[0] + resting[1] + resting[2] + resting[3], 1e-6);
        for (double load : resting) assertEquals(weight / 4.0, load, 1e-6);

        // Accelerating forward transfers load onto the back wheels.
        double[] accelerating = stability.wheelLoadsNewtons(new Translation2d(5.0, 0.0));
        assertEquals(weight, accelerating[0] + accelerating[1] + accelerating[2] + accelerating[3], 1e-6);
        assertTrue(accelerating[StabilityModel.BACK_LEFT] > accelerating[StabilityModel.FRONT_LEFT]);
        assertEquals(accelerating[StabilityModel.BACK_LEFT], accelerating[StabilityModel.BACK_RIGHT], 1e-9);
    }

    @Test
    void aWheelAtTheTippingLimitCarriesNoLoad() {
        StabilityModel stability = centred();
        double limit = stability.maxAccelerationMpsSq(new Translation2d(1, 0));

        double[] loads = stability.wheelLoadsNewtons(new Translation2d(limit, 0));
        assertEquals(0.0, loads[StabilityModel.FRONT_LEFT], 1e-6);
        assertEquals(0.0, loads[StabilityModel.FRONT_RIGHT], 1e-6);
        assertTrue(loads[StabilityModel.BACK_LEFT] > 0);
        assertEquals(0.0, stability.loadBalance(new Translation2d(limit, 0)), 1e-6);
    }

    @Test
    void loadBalanceIsOneWhenPlantedAndFallsAsItLeans() {
        StabilityModel stability = centred();

        assertEquals(1.0, stability.loadBalance(Translation2d.kZero), 1e-9);
        double leaning = stability.loadBalance(new Translation2d(5.0, 0.0));
        assertTrue(leaning > 0 && leaning < 1.0);
    }

    @Test
    void fieldRelativeAccelerationIsRotatedIntoTheRobotFrame() {
        StabilityModel stability = centred();
        Translation2d fieldAccel = new Translation2d(0.0, 6.0);   // straight down the field's +Y

        // Facing +Y, that same push is straight ahead in the robot frame.
        double asRobotForward = stability.tipMarginMeters(new Translation2d(6.0, 0.0));
        assertEquals(asRobotForward, stability.tipMarginMeters(fieldAccel, Rotation2d.fromDegrees(90)), 1e-9);
    }

    @Test
    void aDegenerateOrNullInputIsHandledRatherThanThrowing() {
        StabilityModel stability = centred();

        assertEquals(0.4, stability.tipMarginMeters(null), 1e-9);
        assertEquals(0.0, stability.maxAccelerationMpsSq(null), 1e-9);
        assertEquals(0.0, stability.maxAccelerationMpsSq(Translation2d.kZero), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> new StabilityModel(null));
    }

    @Test
    void aRobotWithItsMassOnTheCarpetHasNoTippingLimit() {
        RobotModel flat = RobotModel.builder()
                .massKg(60).footprintMeters(0.8, 0.8).centerOfMassHeightMeters(1e-12).build();
        StabilityModel stability = StabilityModel.ofChassisOnly(flat);

        assertTrue(Double.isInfinite(stability.maxAccelerationMpsSq(new Translation2d(1, 0))));
    }
}
