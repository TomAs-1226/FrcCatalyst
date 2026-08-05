package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

import frc.lib.catalyst.physics.model.DrivetrainModel;
import frc.lib.catalyst.physics.model.RobotModel;

/** Pure-Java tests for {@link RobotModel} and {@link DrivetrainModel}. */
class PhysicsModelTest {

    /** A 60 kg, 0.7 x 0.7 m robot with a low centre of mass — traction-limited. */
    private static RobotModel lowRobot() {
        return RobotModel.builder()
                .massKg(60.0)
                .footprintMeters(0.7, 0.7)
                .centerOfMassHeightMeters(0.20)
                .coefficientOfFriction(1.0)
                .build();
    }

    @Test
    void tractionLimitIsMuTimesGravityAndDoesNotDependOnMass() {
        DrivetrainModel light = new DrivetrainModel(
                RobotModel.builder().massKg(30.0).coefficientOfFriction(1.0).build());
        DrivetrainModel heavy = new DrivetrainModel(
                RobotModel.builder().massKg(70.0).coefficientOfFriction(1.0).build());

        assertEquals(RobotModel.GRAVITY, light.maxTractionAccelerationMpsSq(), 1e-9);
        assertEquals(light.maxTractionAccelerationMpsSq(), heavy.maxTractionAccelerationMpsSq(), 1e-9);

        // Force does scale with mass, even though acceleration does not.
        assertEquals(30.0 * RobotModel.GRAVITY, light.maxTractionForceNewtons(), 1e-6);
        assertEquals(70.0 * RobotModel.GRAVITY, heavy.maxTractionForceNewtons(), 1e-6);
    }

    @Test
    void aLowRobotIsTractionLimitedAndATallOneIsTippingLimited() {
        DrivetrainModel low = new DrivetrainModel(lowRobot());
        assertFalse(low.isTippingLimited());
        assertEquals(low.maxTractionAccelerationMpsSq(), low.maxSafeAccelerationMpsSq(), 1e-9);

        // Same robot with the elevator up: centre of mass at 0.9 m, half-width still 0.35 m.
        DrivetrainModel tall = new DrivetrainModel(RobotModel.builder()
                .massKg(60.0)
                .footprintMeters(0.7, 0.7)
                .centerOfMassHeightMeters(0.9)
                .coefficientOfFriction(1.0)
                .build());

        assertTrue(tall.isTippingLimited());
        assertEquals(RobotModel.GRAVITY * 0.35 / 0.9, tall.maxTippingAccelerationMpsSq(), 1e-9);
        assertEquals(tall.maxTippingAccelerationMpsSq(), tall.maxSafeAccelerationMpsSq(), 1e-9);
        assertTrue(tall.maxSafeAccelerationMpsSq() < low.maxSafeAccelerationMpsSq());
    }

    @Test
    void stoppingDistanceAndMaxSpeedAreInverses() {
        DrivetrainModel model = new DrivetrainModel(lowRobot());

        double distance = model.stoppingDistanceMeters(4.0);
        assertEquals(16.0 / (2.0 * RobotModel.GRAVITY), distance, 1e-9);
        assertEquals(4.0, model.maxSpeedForStoppingDistance(distance), 1e-9);

        assertEquals(0.0, model.stoppingDistanceMeters(0.0), 1e-9);
        assertEquals(0.0, model.maxSpeedForStoppingDistance(0.0), 1e-9);
    }

    @Test
    void brakingGentlyTakesProportionallyMoreRoom() {
        DrivetrainModel model = new DrivetrainModel(lowRobot());

        double full = model.stoppingDistanceMeters(4.0, 1.0);
        double half = model.stoppingDistanceMeters(4.0, 0.5);
        assertEquals(2.0 * full, half, 1e-9);
    }

    @Test
    void tractionUsageReadsOneAtTheLimitAndAboveOneBeyondIt() {
        DrivetrainModel model = new DrivetrainModel(lowRobot());
        double limit = model.maxTractionAccelerationMpsSq();

        assertEquals(0.0, model.tractionUsage(Translation2d.kZero), 1e-9);
        assertEquals(1.0, model.tractionUsage(new Translation2d(limit, 0.0)), 1e-9);
        assertTrue(model.tractionUsage(new Translation2d(0.0, limit * 1.5)) > 1.0);
    }

    @Test
    void accelerationFromForceIsClippedAtWhatTheCarpetCanTransmit() {
        DrivetrainModel model = new DrivetrainModel(lowRobot());
        double limit = model.maxTractionAccelerationMpsSq();

        assertEquals(2.0, model.accelerationFromForce(120.0), 1e-9);          // 120 N / 60 kg
        assertEquals(limit, model.accelerationFromForce(100_000.0), 1e-9);    // more than grip allows
        assertEquals(-limit, model.accelerationFromForce(-100_000.0), 1e-9);
    }

    @Test
    void momentOfInertiaIsEstimatedFromTheFootprintUnlessMeasured() {
        RobotModel estimated = lowRobot();
        assertEquals(RobotModel.estimateMomentOfInertia(60.0, 0.7, 0.7),
                estimated.momentOfInertiaKgM2(), 1e-9);

        RobotModel measured = RobotModel.builder().massKg(60.0).momentOfInertiaKgM2(4.5).build();
        assertEquals(4.5, measured.momentOfInertiaKgM2(), 1e-9);
    }

    @Test
    void tippingLeverArmUsesTheShorterFootprintDimension() {
        RobotModel oblong = RobotModel.builder().massKg(60.0).footprintMeters(0.9, 0.5).build();
        assertEquals(0.25, oblong.tippingHalfWidthMeters(), 1e-9);
    }

    @Test
    void massInPoundsConvertsToKilograms() {
        RobotModel model = RobotModel.builder().massPounds(120.0).build();
        assertEquals(54.43, model.massKg(), 0.01);
    }

    @Test
    void missingOrNonsensicalModelValuesAreRejected() {
        assertThrows(IllegalStateException.class, () -> RobotModel.builder().build());
        assertThrows(IllegalStateException.class, () -> RobotModel.builder().massKg(0).build());
        assertThrows(IllegalStateException.class, () -> RobotModel.builder().massKg(-5).build());
        assertThrows(IllegalStateException.class,
                () -> RobotModel.builder().massKg(60).footprintMeters(0.0, 0.7).build());
        assertThrows(IllegalStateException.class,
                () -> RobotModel.builder().massKg(60).centerOfMassHeightMeters(0.0).build());
        assertThrows(IllegalArgumentException.class, () -> new DrivetrainModel(null));
    }
}
