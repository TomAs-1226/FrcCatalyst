package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import java.util.ArrayList;
import java.util.List;

import frc.lib.catalyst.physics.contact.ContactMaterial;
import frc.lib.catalyst.physics.contact.SimulatedGamePiece;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.sim.SimulatedRobot;
import org.junit.jupiter.api.Test;

/**
 * A test autonomous routine, driven through the simulator, checked against ground truth.
 *
 * <p>This is the thing a team actually wants before trusting an auto: not "does the code compile" but
 * "if I run this path, where does the robot end up, does Physics Core agree with reality along the
 * way, and does it stay inside the field". The simulator knows the true pose, so every claim here is
 * checked against something rather than eyeballed.
 *
 * <p>The routine is a three-leg path — drive out, arc across, come back — with a game piece dropped in
 * the way so the contact model gets exercised by a robot that is actually moving.
 *
 * <p>Pure Java: no HAL, no NetworkTables, no command scheduler. It runs in CI.
 */
class AutoPathPhysicsTest {

    private static final double DT = 0.02;

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(0.29, 0.29), new Translation2d(0.29, -0.29),
            new Translation2d(-0.29, 0.29), new Translation2d(-0.29, -0.29));

    private static RobotModel chassis() {
        return RobotModel.builder()
                .massKg(55.0)
                .footprintMeters(0.86, 0.86)
                .centerOfMassHeightMeters(0.22)
                .coefficientOfFriction(1.05)
                .build();
    }

    /** One leg of the routine: hold these robot-relative speeds for this long. */
    private record Leg(String name, ChassisSpeeds speeds, double seconds) {}

    /**
     * The routine under test.
     *
     * <p>Deliberately not a trivial straight line: it accelerates, turns while translating, and stops.
     * A path that only ever drives forward will not catch a sign error in the rotation handling, and
     * that is the mistake that actually ruins autos.
     */
    private static List<Leg> testAuto() {
        return List.of(
                new Leg("leave the line", new ChassisSpeeds(2.4, 0.0, 0.0), 1.20),
                new Leg("arc across", new ChassisSpeeds(2.0, 1.1, 0.9), 1.40),
                new Leg("square up", new ChassisSpeeds(1.4, 0.0, -0.9), 0.80),
                new Leg("stop", new ChassisSpeeds(0.0, 0.0, 0.0), 0.60));
    }

    private record Run(
            SimulatedRobot sim,
            PhysicsCore physics,
            List<Pose2d> truePath,
            List<Pose2d> estimatedPath,
            double worstVelocityError,
            double worstSlip) {}

    private static Run runAuto(SimulatedGamePiece piece) {
        double[] clock = {0.0};

        SimulatedRobot sim = SimulatedRobot.builder()
                .robotModel(chassis())
                .kinematics(KINEMATICS)
                .loopPeriod(DT)
                .withRealisticSensors()
                .seed(20260805L)
                .build();

        PhysicsCore physics = PhysicsCore.builder()
                .robotModel(chassis())
                .kinematics(KINEMATICS)
                .clock(() -> clock[0])
                .withLogging(false)
                .withAlerts(false)
                .build();

        List<Pose2d> truePath = new ArrayList<>();
        List<Pose2d> estimatedPath = new ArrayList<>();
        double worstVelocityError = 0.0;
        double worstSlip = 0.0;

        for (Leg leg : testAuto()) {
            int steps = (int) Math.round(leg.seconds() / DT);
            for (int i = 0; i < steps; i++) {
                sim.command(leg.speeds());
                sim.step();
                clock[0] += DT;

                var state = physics.update(sim.sample());
                truePath.add(sim.truePose());
                estimatedPath.add(state.pose());

                double trueSpeed = sim.trueSpeed();
                double estimated = state.speedMetersPerSecond();
                worstVelocityError = Math.max(worstVelocityError, Math.abs(trueSpeed - estimated));
                worstSlip = Math.max(worstSlip, physics.analyze().slipFactor());

                if (piece != null) {
                    piece.step(DT);
                    piece.interactWithRobot(
                            sim.truePose(), sim.trueVelocityVector(), 0.43, 0.43);
                }
            }
        }

        return new Run(sim, physics, truePath, estimatedPath, worstVelocityError, worstSlip);
    }

    // ------------------------------------------------------------------ the path

    @Test
    void theAutoDrivesTheDistanceItIsSupposedTo() {
        Run run = runAuto(null);
        Pose2d end = run.sim().truePose();

        // Every leg has a positive forward component, so it has to finish well down the field.
        assertTrue(end.getX() > 4.0,
                "expected the routine to cover ground; it ended at x=" + end.getX());
        assertTrue(run.truePath().size() > 190, "the whole routine should have been stepped");
    }

    @Test
    void theRobotTurnsInTheDirectionItIsCommandedTo() {
        Run run = runAuto(null);
        // Leg 2 turns +0.9 rad/s for 1.4 s, leg 3 turns -0.9 for 0.8 s: net positive.
        double net = run.sim().truePose().getRotation().getRadians();
        assertTrue(net > 0.15,
                "net rotation should follow the commanded sign, got " + net + " rad");
    }

    @Test
    void theRoutineIsRepeatable() {
        // Same seed, same path — otherwise this whole test file proves nothing.
        Pose2d first = runAuto(null).sim().truePose();
        Pose2d second = runAuto(null).sim().truePose();
        assertEquals(first.getX(), second.getX(), 1e-9);
        assertEquals(first.getY(), second.getY(), 1e-9);
    }

    // ------------------------------------------------------- physics vs ground truth

    @Test
    void physicsCoreTracksTheRealSpeedThroughTheWholeRoutine() {
        Run run = runAuto(null);
        assertTrue(run.worstVelocityError() < 0.75,
                "worst speed error over the auto was " + run.worstVelocityError()
                        + " m/s, which is too far from ground truth to be useful");
    }

    @Test
    void theEstimatedPathStaysNearTheRealOne() {
        Run run = runAuto(null);
        double worst = 0.0;
        for (int i = 0; i < run.truePath().size(); i++) {
            worst = Math.max(worst,
                    run.truePath().get(i).getTranslation()
                            .getDistance(run.estimatedPath().get(i).getTranslation()));
        }
        // Odometry-only estimation drifts; this asserts it drifts slowly, not that it is perfect.
        assertTrue(worst < 1.5,
                "estimated path wandered " + worst + " m from the true one");
    }

    @Test
    void physicsCoreStaysHealthyAndConfidentWhileDriving() {
        Run run = runAuto(null);
        assertTrue(run.physics().health().running(), "it should be running after 200 updates");
        assertTrue(run.physics().state().quality().confidence() > 0.25,
                "confidence collapsed to " + run.physics().state().quality().confidence());
    }

    @Test
    void aClipInAndOutOfTheThrottleDoesNotReportPermanentSlip() {
        Run run = runAuto(null);
        assertTrue(run.worstSlip() <= 1.0, "slip factor is a fraction and must stay one");
        assertEquals(0.0, run.physics().analyze().slipFactor(), 0.35,
                "after the stop leg the wheels should agree with the body again");
    }

    // --------------------------------------------------------------- with contact

    @Test
    void theAutoShovesAGamePieceOutOfItsWay() {
        // Dropped a couple of metres down the first leg, right on the robot's line.
        SimulatedGamePiece fuel = SimulatedGamePiece.builder()
                .position(new Translation3d(2.2, 0.0, 0.12))
                .material(ContactMaterial.FOAM_GAME_PIECE)
                .build();

        Translation2d before = fuel.groundPosition();
        runAuto(fuel);

        assertTrue(fuel.contactCount() > 0, "the robot drove straight through where it was sitting");
        assertTrue(fuel.groundPosition().getDistance(before) > 0.1,
                "a robot hitting a foam ball at 2 m/s should move it");
    }

    @Test
    void theGamePieceEndsUpSomewhereLegal() {
        SimulatedGamePiece fuel = SimulatedGamePiece.builder()
                .position(new Translation3d(2.2, 0.0, 0.12))
                .field(16.54, 8.07)
                .build();
        runAuto(fuel);

        Translation3d at = fuel.position();
        assertTrue(at.getX() >= 0 && at.getX() <= 16.54, "left the field lengthwise at " + at.getX());
        assertTrue(at.getY() >= 0 && at.getY() <= 8.07, "left the field widthwise at " + at.getY());
        assertTrue(at.getZ() >= 0, "fell through the carpet to z=" + at.getZ());
    }

    @Test
    void drivingThroughAPieceDoesNotDisturbTheRobotsOwnEstimate() {
        // A 270 g ball must not visibly perturb a 55 kg robot's state estimate. If this ever fails,
        // the contact model has started feeding force back into the drivetrain, which it must not.
        Pose2d withoutPiece = runAuto(null).sim().truePose();
        Pose2d withPiece = runAuto(SimulatedGamePiece.builder()
                .position(new Translation3d(2.2, 0.0, 0.12)).build()).sim().truePose();

        assertEquals(withoutPiece.getX(), withPiece.getX(), 1e-9);
        assertEquals(withoutPiece.getY(), withPiece.getY(), 1e-9);
    }
}
