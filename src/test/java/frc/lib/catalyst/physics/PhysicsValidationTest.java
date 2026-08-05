package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.sim.PhysicsValidator;
import frc.lib.catalyst.physics.sim.SimulatedRobot;

/**
 * Runs Physics Core against a simulation that knows the truth, and asserts the RFC's acceptance
 * criteria.
 *
 * <p>These are the only tests in the suite that check whether Physics Core <em>works</em>, as opposed
 * to whether its arithmetic is right. Everything else verifies a formula in isolation; this drives a
 * robot through slip, impacts, and vision dropout with deliberately imperfect sensors and asks whether
 * the estimates came out closer to the truth than the raw measurements did.
 *
 * <p>Deterministic — the simulator's noise is seeded — so a failure here is reproducible rather than
 * flaky.
 */
class PhysicsValidationTest {

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(0.35, 0.35), new Translation2d(0.35, -0.35),
            new Translation2d(-0.35, 0.35), new Translation2d(-0.35, -0.35));

    private static RobotModel robot() {
        return RobotModel.builder()
                .massKg(55.0)
                .footprintMeters(0.7, 0.7)
                .wheelRadiusInches(2.0)
                .centerOfMassHeightMeters(0.22)
                .coefficientOfFriction(1.0)
                .build();
    }

    private static PhysicsValidator validator() {
        return PhysicsValidator.builder().robotModel(robot()).kinematics(KINEMATICS).build();
    }

    @Test
    void physicsCoreMeetsEveryAcceptanceCriterion() {
        var report = validator().runAll();

        // Print the whole thing: when this fails, the metrics are what tell you why.
        System.out.println(report.describe());

        assertTrue(report.allPassed(), () -> "failed scenarios:\n"
                + String.join("\n", report.failures().stream().map(f -> "  " + f.describe()).toList()));
    }

    @Test
    void theValidationIsDeterministic() {
        var first = validator().runAll();
        var second = validator().runAll();

        for (int i = 0; i < first.results().size(); i++) {
            assertEquals(first.results().get(i).passed(), second.results().get(i).passed());
            assertEquals(first.results().get(i).metrics(), second.results().get(i).metrics(),
                    "scenario '" + first.results().get(i).name() + "' is not reproducible");
        }
    }

    @Test
    void aDifferentSeedStillPasses() {
        // A single lucky seed proves nothing. The criteria should hold across noise realisations.
        for (long seed : new long[]{1L, 7L, 4242L}) {
            var report = PhysicsValidator.builder()
                    .robotModel(robot()).kinematics(KINEMATICS).seed(seed).build().runAll();
            assertTrue(report.allPassed(), () -> "seed " + seed + " failed:\n"
                    + String.join("\n", report.failures().stream().map(f -> "  " + f.describe()).toList()));
        }
    }

    // ===========================================
    //   The simulator itself has to be right too
    // ===========================================

    @Test
    void theSimulatorRespectsItsOwnTractionLimit() {
        SimulatedRobot sim = SimulatedRobot.builder()
                .robotModel(robot()).kinematics(KINEMATICS).build();

        sim.command(new ChassisSpeeds(10.0, 0, 0));   // far more than grip allows
        double peakAcceleration = 0.0;
        for (int i = 0; i < 30; i++) {
            sim.step();
            peakAcceleration = Math.max(peakAcceleration, sim.trueAcceleration().getNorm());
        }

        // mu = 1.0, so the ceiling is one g. Allow a hair for the discrete step.
        assertTrue(peakAcceleration <= RobotModel.GRAVITY * 1.001,
                "simulated acceleration exceeded the traction limit: " + peakAcceleration);
        assertTrue(sim.isSlipping(), "commanding 10 m/s at once should slip");
    }

    @Test
    void slipMakesTheWheelsOverReportAndDriftsTheOdometry() {
        SimulatedRobot sim = SimulatedRobot.builder()
                .robotModel(robot()).kinematics(KINEMATICS).build();

        sim.command(new ChassisSpeeds(2.0, 0, 0));
        sim.step(40);
        // Not exactly zero: the wheels re-sync with ground speed over a few loops rather than
        // snapping to it, so accelerating up to speed leaves a small permanent offset - which is
        // what a real drivetrain's odometry does too.
        assertTrue(sim.odometryErrorMeters() < 0.03,
                "clean driving should barely drift, got " + sim.odometryErrorMeters());

        sim.setFrictionScale(0.2);
        sim.command(new ChassisSpeeds(6.0, 0, 0));
        sim.step(30);

        assertTrue(sim.wheelVelocity().getNorm() > sim.trueSpeed() + 0.5,
                "slipping wheels must read faster than the robot is moving");
        assertTrue(sim.odometryErrorMeters() > 0.05,
                "a slip must displace the odometry pose, got " + sim.odometryErrorMeters());
    }

    @Test
    void aPerfectVisionCorrectionRemovesTheDrift() {
        SimulatedRobot sim = SimulatedRobot.builder()
                .robotModel(robot()).kinematics(KINEMATICS).build();
        sim.setFrictionScale(0.2);
        sim.command(new ChassisSpeeds(6.0, 0, 0));
        sim.step(30);
        assertTrue(sim.odometryErrorMeters() > 0.05);

        sim.applyPerfectVisionCorrection();
        assertEquals(0.0, sim.odometryErrorMeters(), 1e-9);
    }

    @Test
    void theWheelRadiusErrorShowsUpInTheReportedSpeed() {
        SimulatedRobot sim = SimulatedRobot.builder()
                .robotModel(robot()).kinematics(KINEMATICS).wheelRadiusError(1.05).build();

        sim.command(new ChassisSpeeds(3.0, 0, 0));
        sim.step(100);

        // The robot really travels at 3.0; the encoders insist it is 5% faster.
        assertEquals(3.0, sim.trueSpeed(), 0.05);
        assertEquals(3.0 * 1.05, sim.wheelVelocity().getNorm(), 0.06);
    }

    @Test
    void perModuleBiasIsIndependentOfUniformGripLoss() {
        SimulatedRobot sim = SimulatedRobot.builder()
                .robotModel(robot()).kinematics(KINEMATICS).build();
        sim.command(new ChassisSpeeds(3.0, 0, 0));
        sim.step(40);

        assertFalse(sim.hasModuleSlip());
        var clean = sim.sample();

        sim.setModuleSlipBias(1, 1.5);
        sim.step();
        var biased = sim.sample();

        assertTrue(sim.hasModuleSlip());
        assertEquals(1.5,
                biased.moduleStates()[1].speedMetersPerSecond
                        - clean.moduleStates()[1].speedMetersPerSecond, 0.15);
        assertEquals(0.0,
                biased.moduleStates()[0].speedMetersPerSecond
                        - clean.moduleStates()[0].speedMetersPerSecond, 0.15);
    }

    @Test
    void anExternalPushIsNotLimitedByTraction() {
        SimulatedRobot sim = SimulatedRobot.builder()
                .robotModel(robot()).kinematics(KINEMATICS).build();

        sim.applyExternalAcceleration(new Translation2d(0.0, -20.0));
        sim.step();

        // Being hit is not something the carpet gets a say in.
        assertTrue(sim.trueAcceleration().getNorm() > RobotModel.GRAVITY);
        // And it lasts one step unless reapplied.
        sim.step();
        assertTrue(sim.trueAcceleration().getNorm() < RobotModel.GRAVITY);
    }

    @Test
    void resetReturnsTheSimulationToTheOrigin() {
        SimulatedRobot sim = SimulatedRobot.builder()
                .robotModel(robot()).kinematics(KINEMATICS).build();
        sim.command(new ChassisSpeeds(3.0, 1.0, 0.5));
        sim.step(50);
        assertTrue(sim.truePose().getTranslation().getNorm() > 1.0);

        sim.reset();
        assertEquals(0.0, sim.truePose().getTranslation().getNorm(), 1e-12);
        assertEquals(0.0, sim.trueSpeed(), 1e-12);
        assertEquals(0.0, sim.timestamp(), 1e-12);
        assertFalse(sim.hasModuleSlip());
    }
}
