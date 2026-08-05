package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.lib.catalyst.physics.model.ArticulatedRobotModel;
import frc.lib.catalyst.physics.model.DrivetrainModel;
import frc.lib.catalyst.physics.model.MechanismModel;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.model.StabilityModel;
import frc.lib.catalyst.physics.prediction.CapabilityEvaluator;
import frc.lib.catalyst.physics.prediction.PowerPredictor;

/** Pure-Java tests for {@link PowerPredictor} and {@link CapabilityEvaluator}. */
class PhysicsPredictionTest {

    private static RobotModel chassis() {
        return RobotModel.builder()
                .massKg(60.0)
                .footprintMeters(0.8, 0.8)
                .centerOfMassHeightMeters(0.25)
                .coefficientOfFriction(1.0)
                .build();
    }

    /** A robot state at the origin, stationary, with a confident estimate. */
    private static PhysicalRobotState atRest() {
        return moving(0.0);
    }

    private static PhysicalRobotState moving(double vx) {
        return new PhysicalRobotState(0.0, Pose2d.kZero,
                new ChassisSpeeds(vx, 0.0, 0.0), Translation2d.kZero, 0.0,
                new LocalizationQuality(0.95, 0.03, 0.01, 0.06, 0.2, "nominal"));
    }

    /** A predictor at 12.0 V and 30 A, R = 0.02, floor 7.5 V. */
    private static PowerPredictor power() {
        return PowerPredictor.builder()
                .presentVoltage(() -> 12.0)
                .presentCurrent(() -> 30.0)
                .internalResistance(0.02)
                .minimumVoltage(7.5)
                .build();
    }

    // ===========================================
    //               PowerPredictor
    // ===========================================

    @Test
    void openCircuitVoltageIsInferredFromThePresentLoad() {
        // 12.0 V measured while pulling 30 A through 0.02 ohm means the battery is really at 12.6.
        assertEquals(12.6, power().openCircuitVolts(), 1e-9);
    }

    @Test
    void addingLoadSagsTheBusByCurrentTimesResistance() {
        PowerPredictor power = power();

        assertEquals(12.0, power.predictedVoltage(0.0), 1e-9);
        assertEquals(12.0 - 50 * 0.02, power.predictedVoltage(50.0), 1e-9);
        assertTrue(power.canSustain(50.0));
    }

    @Test
    void headroomIsWhatIsLeftBeforeTheFloor() {
        PowerPredictor power = power();

        // (12.6 - 7.5) / 0.02 = 255 A total, minus the 30 already flowing.
        assertEquals(225.0, power.headroomAmps(), 1e-9);
        assertTrue(power.canSustain(224.0));
        assertFalse(power.canSustain(226.0));
        assertEquals(7.5, power.predictedVoltage(power.headroomAmps()), 1e-9);
    }

    @Test
    void headroomGoesNegativeRatherThanClampingWhenAlreadyOverdrawn() {
        PowerPredictor drained = PowerPredictor.builder()
                .presentVoltage(() -> 7.0).presentCurrent(() -> 200.0)
                .internalResistance(0.02).minimumVoltage(7.5).build();

        assertTrue(drained.headroomAmps() < 0, "an overdrawn bus should report how much to shed");
    }

    @Test
    void demandsThatFitTogetherStayInOneWave() {
        var plan = power().plan(
                new PowerPredictor.PowerDemand("Elevator", 60),
                new PowerPredictor.PowerDemand("Shooter", 45));

        assertTrue(plan.fitsInOneWave());
        assertEquals(1, plan.waveCount());
        assertTrue(plan.describe().contains("Elevator + Shooter"));
    }

    @Test
    void demandsThatDoNotFitAreSequencedInTheOrderGiven() {
        PowerPredictor tight = PowerPredictor.builder()
                .presentVoltage(() -> 11.0).presentCurrent(() -> 60.0)
                .internalResistance(0.04).minimumVoltage(9.0).build();
        // Open circuit 13.4, floor 9.0, R 0.04 -> 110 A total, minus 60 flowing = 50 A of headroom.
        assertEquals(50.0, tight.headroomAmps(), 1e-9);

        var plan = tight.plan(
                new PowerPredictor.PowerDemand("Elevator", 30),
                new PowerPredictor.PowerDemand("Shooter", 30),
                new PowerPredictor.PowerDemand("Intake", 15));

        assertFalse(plan.fitsInOneWave());
        assertEquals(2, plan.waveCount());
        assertEquals("Elevator", plan.waves().get(0).get(0).name());
        assertTrue(plan.describe().contains("then"));
    }

    @Test
    void aDemandBiggerThanTheWholeBudgetIsFlaggedRatherThanSequencedAway() {
        PowerPredictor tight = PowerPredictor.builder()
                .presentVoltage(() -> 11.0).presentCurrent(() -> 60.0)
                .internalResistance(0.04).minimumVoltage(9.0).build();

        var plan = tight.plan(new PowerPredictor.PowerDemand("Everything", 500));

        assertTrue(plan.oversized().contains("Everything"));
        assertTrue(plan.describe().contains("exceeds the 50 A headroom alone"));
    }

    // ===========================================
    //            Motion profile timing
    // ===========================================

    @Test
    void aTriangularProfileMatchesTheHandCalculation() {
        // From rest, 2 m at 4 m/s^2, never reaching a 10 m/s cap.
        // Peak = sqrt(2·4·2 / 2) = 2.828 m/s; time = 2 · 2.828/4 = 1.414 s.
        double t = CapabilityEvaluator.timeToTravel(2.0, 0.0, 10.0, 4.0).orElseThrow();
        double peak = Math.sqrt(2.0 * 4.0 * 2.0 / 2.0);
        assertEquals(2.0 * peak / 4.0, t, 1e-12);
        assertEquals(1.4142, t, 1e-4);
    }

    @Test
    void aTrapezoidalProfileMatchesTheHandCalculation() {
        // From rest, 20 m at 4 m/s^2 with a 4 m/s cap.
        // Accelerate 1 s over 2 m, decelerate 1 s over 2 m, cruise 16 m at 4 m/s = 4 s. Total 6 s.
        assertEquals(6.0, CapabilityEvaluator.timeToTravel(20.0, 0.0, 4.0, 4.0).orElseThrow(), 1e-12);
    }

    @Test
    void startingWithSpeedAlreadyOnBoardArrivesSooner() {
        double fromRest = CapabilityEvaluator.timeToTravel(10.0, 0.0, 5.0, 3.0).orElseThrow();
        double rolling = CapabilityEvaluator.timeToTravel(10.0, 2.0, 5.0, 3.0).orElseThrow();
        assertTrue(rolling < fromRest);
    }

    @Test
    void aRobotTooFastToStopInTheDistanceHasNoProfile() {
        // Stopping from 5 m/s at 2 m/s^2 needs 6.25 m; only 3 m is available.
        assertTrue(CapabilityEvaluator.timeToTravel(3.0, 5.0, 10.0, 2.0).isEmpty());
        assertTrue(CapabilityEvaluator.timeToTravel(7.0, 5.0, 10.0, 2.0).isPresent());
    }

    @Test
    void degenerateProfileInputsAreHandled() {
        assertEquals(0.0, CapabilityEvaluator.timeToTravel(0.0, 0.0, 4.0, 4.0).orElseThrow(), 1e-12);
        assertEquals(OptionalDouble.empty(), CapabilityEvaluator.timeToTravel(5.0, 0.0, 4.0, 0.0));
        assertEquals(OptionalDouble.empty(), CapabilityEvaluator.timeToTravel(5.0, 0.0, 0.0, 4.0));
    }

    // ===========================================
    //            CapabilityEvaluator
    // ===========================================

    private static CapabilityEvaluator evaluator(StabilityModel stability) {
        return CapabilityEvaluator.builder()
                .drivetrain(new DrivetrainModel(chassis()))
                .stability(stability)
                .power(power())
                .maxSpeedMps(4.0)
                .usableTractionFraction(0.8)
                .build();
    }

    @Test
    void aComfortableMoveIsReliableAndReportsEveryFigure() {
        var capability = evaluator(StabilityModel.ofChassisOnly(chassis()))
                .evaluateDriveTo(atRest(), new Translation2d(3.0, 0.0), 40.0);

        assertTrue(capability.feasible());
        assertTrue(capability.isReliable());
        assertEquals(CapabilityEvaluator.Risk.LOW, capability.risk());
        assertTrue(capability.seconds().orElseThrow() > 0);
        assertTrue(capability.minimumVolts().isPresent());
        assertTrue(capability.tipMarginMeters() > 0);

        String report = capability.describe();
        assertTrue(report.contains("Feasible: yes"));
        assertTrue(report.contains("Predicted completion"));
        assertTrue(report.contains("Tip margin"));
    }

    @Test
    void aTargetTooCloseToStopInIsInfeasibleAndSaysWhy() {
        var capability = evaluator(StabilityModel.ofChassisOnly(chassis()))
                .evaluateDriveTo(moving(4.0), new Translation2d(0.5, 0.0), 40.0);

        assertFalse(capability.feasible());
        assertEquals(CapabilityEvaluator.Risk.INFEASIBLE, capability.risk());
        assertTrue(capability.limitingFactor().contains("cannot stop"));
    }

    @Test
    void aTallRobotGetsATighterAccelerationLimitAndTakesLonger() {
        double[] height = {0.0};
        ArticulatedRobotModel articulated = ArticulatedRobotModel.builder()
                .chassis(chassis())
                .add(MechanismModel.linear("Elevator", 15.0)
                        .mountedAt(new Translation3d(0, 0, 0.2)).position(() -> height[0]).build())
                .build();
        var evaluator = evaluator(new StabilityModel(articulated));

        double stowed = evaluator.evaluateDriveTo(atRest(), new Translation2d(3, 0), 40)
                .seconds().orElseThrow();
        height[0] = 1.5;
        double raised = evaluator.evaluateDriveTo(atRest(), new Translation2d(3, 0), 40)
                .seconds().orElseThrow();

        assertTrue(raised > stowed,
                "with the elevator up the move must be predicted slower, got " + raised + " vs " + stowed);
    }

    @Test
    void anActionThatWouldBrownOutIsRefused() {
        var evaluator = CapabilityEvaluator.builder()
                .drivetrain(new DrivetrainModel(chassis()))
                .power(PowerPredictor.builder()
                        .presentVoltage(() -> 8.0).presentCurrent(() -> 100.0)
                        .internalResistance(0.02).minimumVoltage(7.5).build())
                .build();

        var capability = evaluator.evaluateAction(atRest(), 1.0, 300.0);

        assertFalse(capability.feasible());
        assertTrue(capability.limitingFactor().contains("brown out"));
    }

    @Test
    void aLongMoveWithAnUncertainEstimateIsOnlyModeratelyRisky() {
        PhysicalRobotState unsure = new PhysicalRobotState(0.0, Pose2d.kZero,
                new ChassisSpeeds(), Translation2d.kZero, 0.0,
                new LocalizationQuality(0.5, 0.10, 0.05, 0.5, 3.0, "vision stale 3.0 s"));

        var capability = evaluator(StabilityModel.ofChassisOnly(chassis()))
                .evaluateDriveTo(unsure, new Translation2d(5.0, 0.0), 40.0);

        assertTrue(capability.feasible());
        assertEquals(CapabilityEvaluator.Risk.MODERATE, capability.risk());
        assertTrue(capability.limitingFactor().contains("position uncertain"));
        assertTrue(capability.isAcceptable());
        assertFalse(capability.isReliable());
    }

    @Test
    void anAlreadyReachedTargetCostsNothing() {
        var capability = evaluator(StabilityModel.ofChassisOnly(chassis()))
                .evaluateDriveTo(atRest(), Translation2d.kZero, 10.0);

        assertTrue(capability.feasible());
        assertEquals(0.0, capability.seconds().orElseThrow(), 1e-12);
    }

    @Test
    void withoutOptionalModelsTheMissingFiguresAreAbsentRatherThanInvented() {
        var bare = CapabilityEvaluator.builder()
                .drivetrain(new DrivetrainModel(chassis())).build();

        var capability = bare.evaluateDriveTo(atRest(), new Translation2d(2, 0), 40);

        assertTrue(capability.feasible());
        assertTrue(capability.minimumVolts().isEmpty());
        assertTrue(Double.isNaN(capability.tipMarginMeters()));
        assertFalse(capability.describe().contains("Tip margin"));
    }

    @Test
    void badConfigurationIsRejected() {
        assertThrows(IllegalStateException.class, () -> CapabilityEvaluator.builder().build());
        assertThrows(IllegalStateException.class, () -> CapabilityEvaluator.builder()
                .drivetrain(new DrivetrainModel(chassis())).maxSpeedMps(0).build());
        assertThrows(IllegalStateException.class, () -> CapabilityEvaluator.builder()
                .drivetrain(new DrivetrainModel(chassis())).usableTractionFraction(1.5).build());
        assertThrows(IllegalStateException.class, () -> PowerPredictor.builder().build());
        assertThrows(IllegalArgumentException.class,
                () -> new PowerPredictor.PowerDemand("bad", -1));
    }
}
