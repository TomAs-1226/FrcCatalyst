package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.lib.catalyst.physics.model.DrivetrainModel;
import frc.lib.catalyst.physics.model.ModelUncertainty;
import frc.lib.catalyst.physics.model.RobotModel;

/** Pure-Java tests for {@link ModelUncertainty} and the model accessors it reasons about. */
class ModelUncertaintyTest {

    /** A low, traction-limited robot. */
    private static RobotModel lowRobot() {
        return RobotModel.builder()
                .massKg(60.0).footprintMeters(0.8, 0.8)
                .centerOfMassHeightMeters(0.20).coefficientOfFriction(1.0).build();
    }

    /** The same robot with its elevator up — tipping-limited. */
    private static RobotModel tallRobot() {
        return RobotModel.builder()
                .massKg(60.0).footprintMeters(0.8, 0.8)
                .centerOfMassHeightMeters(0.90).coefficientOfFriction(1.0).build();
    }

    @Test
    void massUncertaintyDoesNotAffectTheTractionLimitAtAll() {
        var precise = ModelUncertainty.builder(lowRobot()).massKg(0.1).coefficientOfFriction(0.1).build();
        var vague = ModelUncertainty.builder(lowRobot()).massKg(20.0).coefficientOfFriction(0.1).build();

        // a = mu*g. Mass cancels between "needs more force" and "gets more grip".
        assertEquals(precise.tractionLimitSigma(), vague.tractionLimitSigma(), 1e-12);
    }

    @Test
    void frictionUncertaintyMapsOneForOneOntoTheTractionLimit() {
        var uncertainty = ModelUncertainty.builder(lowRobot()).coefficientOfFriction(0.15).build();
        assertEquals(0.15 * RobotModel.GRAVITY, uncertainty.tractionLimitSigma(), 1e-12);

        // 15% uncertainty on mu = 1.0 is 15% uncertainty on the limit.
        double limit = new DrivetrainModel(lowRobot()).maxTractionAccelerationMpsSq();
        assertEquals(0.15, uncertainty.tractionLimitSigma() / limit, 1e-12);
    }

    @Test
    void massUncertaintyDoesAffectTheTractionForce() {
        var precise = ModelUncertainty.builder(lowRobot())
                .massKg(0.1).coefficientOfFriction(0.0).build();
        var vague = ModelUncertainty.builder(lowRobot())
                .massKg(20.0).coefficientOfFriction(0.0).build();

        // F = mu*m*g, so here mass does not cancel.
        assertTrue(vague.tractionForceSigma() > precise.tractionForceSigma() * 10);
    }

    @Test
    void centerOfMassHeightDominatesTheTippingLimit() {
        var uncertainty = ModelUncertainty.builder(tallRobot())
                .centerOfMassHeightMeters(0.09)     // 10% of 0.90
                .footprintMeters(0.004)             // 1% of the 0.40 half-width
                .build();

        double limit = new DrivetrainModel(tallRobot()).maxTippingAccelerationMpsSq();
        // 10% and 1% in quadrature is 10.05%.
        assertEquals(0.1005, uncertainty.tippingLimitSigma() / limit, 1e-3);

        var dominant = uncertainty.dominantSource();
        assertEquals("centre-of-mass height", dominant.measurement());
        assertTrue(dominant.varianceShare() > 0.95);
    }

    @Test
    void aTractionLimitedRobotIsLimitedByFrictionInstead() {
        var uncertainty = ModelUncertainty.builder(lowRobot()).build();

        assertTrue(new DrivetrainModel(lowRobot()).isTippingLimited() == false);
        assertEquals("coefficient of friction", uncertainty.dominantSource().measurement());
        assertEquals(uncertainty.tractionLimitSigma(), uncertainty.safeAccelerationSigma(), 1e-12);
    }

    @Test
    void aTippingLimitedRobotTakesItsUncertaintyFromTheTippingLimit() {
        var uncertainty = ModelUncertainty.builder(tallRobot()).build();

        assertTrue(new DrivetrainModel(tallRobot()).isTippingLimited());
        assertEquals(uncertainty.tippingLimitSigma(), uncertainty.safeAccelerationSigma(), 1e-12);
    }

    @Test
    void stoppingDistanceInheritsTheRelativeErrorOfTheAccelerationLimit() {
        var uncertainty = ModelUncertainty.builder(lowRobot()).coefficientOfFriction(0.10).build();
        DrivetrainModel model = new DrivetrainModel(lowRobot());

        double relativeLimit = uncertainty.safeAccelerationSigma() / model.maxSafeAccelerationMpsSq();
        double relativeDistance = uncertainty.stoppingDistanceSigma(4.0) / model.stoppingDistanceMeters(4.0);
        assertEquals(relativeLimit, relativeDistance, 1e-12);
    }

    @Test
    void perfectMeasurementsGivePerfectLimits() {
        var uncertainty = ModelUncertainty.builder(lowRobot())
                .massKg(0).centerOfMassHeightMeters(0).coefficientOfFriction(0).footprintMeters(0)
                .build();

        assertEquals(0.0, uncertainty.tractionLimitSigma(), 1e-12);
        assertEquals(0.0, uncertainty.tippingLimitSigma(), 1e-12);
        assertEquals(0.0, uncertainty.tractionForceSigma(), 1e-12);
    }

    @Test
    void theReportNamesEveryLimitAndWhatIsDrivingIt() {
        String report = ModelUncertainty.builder(tallRobot()).build().describe();

        assertTrue(report.contains("traction limit"));
        assertTrue(report.contains("tipping limit"));
        assertTrue(report.contains("limited by: centre-of-mass height"));
    }

    // ===========================================
    //   The model accessors the guide tells teams to measure
    // ===========================================

    @Test
    void angularAccelerationLimitUsesTheMomentOfInertia() {
        DrivetrainModel model = new DrivetrainModel(lowRobot());

        // tau = mu*m*g*r with r the module radius; alpha = tau / I.
        double moduleRadius = Math.hypot(0.4, 0.4);
        double expected = model.maxTractionForceNewtons() * moduleRadius
                / lowRobot().momentOfInertiaKgM2();
        assertEquals(expected, model.maxAngularAccelerationRadPerSecSq(), 1e-9);
        assertTrue(model.maxAngularAccelerationRadPerSecSq() > 0);
    }

    @Test
    void aRobotWithMoreInertiaSpinsUpMoreSlowly() {
        DrivetrainModel light = new DrivetrainModel(
                RobotModel.builder().massKg(60).footprintMeters(0.8, 0.8)
                        .momentOfInertiaKgM2(4.0).build());
        DrivetrainModel heavy = new DrivetrainModel(
                RobotModel.builder().massKg(60).footprintMeters(0.8, 0.8)
                        .momentOfInertiaKgM2(8.0).build());

        assertEquals(2.0, light.maxAngularAccelerationRadPerSecSq()
                / heavy.maxAngularAccelerationRadPerSecSq(), 1e-9);
    }

    @Test
    void wheelForceFromTorqueUsesTheWheelRadius() {
        DrivetrainModel model = new DrivetrainModel(
                RobotModel.builder().massKg(60).wheelRadiusMeters(0.05).build());

        // 10 Nm at each of four wheels through a 0.05 m radius is 200 N per wheel, 800 N total.
        assertEquals(800.0, model.wheelForceFromTorque(10.0, 4), 1e-9);
        assertEquals(0.0, model.wheelForceFromTorque(10.0, 0), 1e-9);
    }

    @Test
    void torqueBeyondGripStillOnlyDeliversTheTractionLimit() {
        DrivetrainModel model = new DrivetrainModel(lowRobot());

        double absurdForce = model.wheelForceFromTorque(500.0, 4);
        assertEquals(model.maxTractionAccelerationMpsSq(),
                model.accelerationFromForce(absurdForce), 1e-9);
    }

    @Test
    void negativeUncertaintiesAreRejected() {
        assertThrows(IllegalStateException.class,
                () -> ModelUncertainty.builder(lowRobot()).massKg(-1).build());
        assertThrows(IllegalArgumentException.class, () -> ModelUncertainty.builder(null));
    }
}
