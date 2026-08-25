package frc.lib.catalyst.physics.sim;

import org.junit.jupiter.api.Test;

import frc.lib.catalyst.physics.model.RobotModel;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.ChassisVelocities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The simulator's reported velocity, at headings other than zero.
 *
 * <p>This exists because of a bug that 319 physics tests did not catch, and the reason they did not
 * is the point: the sample's frame conversion was inverted, and at heading zero
 * {@code toFieldRelative} and {@code toRobotRelative} are the same function. Every validator
 * scenario drove at heading zero.
 *
 * <p>The one existing test that <em>did</em> turn while translating — and whose comment says it
 * exists to "catch a sign error in the rotation handling" — asserted on speed, which is a magnitude
 * and rotation-invariant, and on pose, which passes through from odometry untouched. Both were
 * blind to a rotation by exactly the wrong angle.
 *
 * <p>So every assertion here is on a <b>vector at a non-zero heading</b>. That is the only shape of
 * test that can see this class of error.
 */
class SimulatedRobotFrameTest {

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(0.35, 0.35), new Translation2d(0.35, -0.35),
            new Translation2d(-0.35, 0.35), new Translation2d(-0.35, -0.35));

    /** The same chassis the other physics tests use, so numbers here are comparable to theirs. */
    private static RobotModel model() {
        return RobotModel.builder()
                .massKg(55.0)
                .footprintMeters(0.7, 0.7)
                .wheelRadiusInches(2.0)
                .centerOfMassHeightMeters(0.22)
                .coefficientOfFriction(1.0)
                .build();
    }

    private static SimulatedRobot robot() {
        // Perfect sensors deliberately. This is about the frame the reading is expressed in, and
        // noise would only make a reversed vector harder to see.
        return SimulatedRobot.builder().robotModel(model()).kinematics(KINEMATICS).build();
    }

    /** Drive the robot forward in its own frame until it is up to speed, then read the sample. */
    private static SimulatedRobot drivenAtHeading(double headingRadians, double forwardMps) {
        SimulatedRobot robot = robot();
        // Turn in place to the heading first, then drive straight ahead in the robot's own frame.
        robot.command(new ChassisVelocities(0, 0, 4.0));
        while (robot.truePose().getRotation().getRadians() < headingRadians) {
            robot.step();
        }
        robot.command(new ChassisVelocities(forwardMps, 0, 0));
        robot.step(60);
        return robot;
    }

    @Test
    void aSampleTakenSidewaysAgreesWithTheTruth() {
        // The case the bug reversed outright. Driving forward at 90 degrees means moving along
        // field +Y; an inverted conversion reports -Y, an error of twice the speed, and every
        // downstream consumer believes the robot is going backwards.
        SimulatedRobot robot = drivenAtHeading(Math.PI / 2, 2.0);

        Translation2d truth = robot.trueVelocityVector();
        ChassisVelocities reported = robot.sample().robotRelativeSpeeds();

        // The sample is robot-relative, so rotating it by the heading has to give back the truth.
        Translation2d asField = new Translation2d(reported.vx, reported.vy)
                .rotateBy(robot.truePose().getRotation());

        assertEquals(truth.getX(), asField.getX(), 0.25, "field X");
        assertEquals(truth.getY(), asField.getY(), 0.25, "field Y");
    }

    @Test
    void theReportedVectorIsNeverTheReverseOfTheTruth() {
        // Stated as the thing that actually went wrong, at several headings. A rotation by the
        // wrong sign is indistinguishable from the right one by magnitude alone, so this compares
        // direction.
        for (double heading : new double[] {Math.PI / 6, Math.PI / 3, Math.PI / 2, 2.0}) {
            SimulatedRobot robot = drivenAtHeading(heading, 2.0);

            Translation2d truth = robot.trueVelocityVector();
            ChassisVelocities reported = robot.sample().robotRelativeSpeeds();
            Translation2d asField = new Translation2d(reported.vx, reported.vy)
                    .rotateBy(robot.truePose().getRotation());

            if (truth.getNorm() < 0.5) {
                continue;   // not up to speed; nothing to compare directions with
            }
            double dot = truth.getX() * asField.getX() + truth.getY() * asField.getY();
            assertTrue(dot > 0,
                    String.format("at heading %.2f rad the reported velocity points against the "
                            + "truth: truth=(%.2f, %.2f) reported-as-field=(%.2f, %.2f)",
                            heading, truth.getX(), truth.getY(), asField.getX(), asField.getY()));
        }
    }

    @Test
    void headingZeroStillWorks() {
        // The case that always passed, kept so a future fix cannot break it while chasing the
        // others.
        SimulatedRobot robot = robot();
        robot.command(new ChassisVelocities(2.0, 0, 0));
        robot.step(60);

        ChassisVelocities reported = robot.sample().robotRelativeSpeeds();
        assertEquals(robot.trueVelocityVector().getX(), reported.vx, 0.25);
    }

    @Test
    void turningWhileDrivingDoesNotInventADisturbance() {
        // The downstream symptom, asserted directly. With the conversion inverted, a robot turning
        // while translating showed a phantom disturbance over half the traction limit, so
        // isDisturbed() read true on every turn - in the simulator teams are told to validate
        // against before trusting Physics Core on carpet.
        SimulatedRobot robot = robot();
        robot.command(new ChassisVelocities(2.0, 0, 0.9));
        robot.step(120);

        Translation2d truth = robot.trueVelocityVector();
        ChassisVelocities reported = robot.sample().robotRelativeSpeeds();
        Translation2d asField = new Translation2d(reported.vx, reported.vy)
                .rotateBy(robot.truePose().getRotation());

        double error = truth.minus(asField).getNorm();
        assertTrue(error < 0.5,
                String.format("reported velocity is %.3f m/s from the truth while turning "
                        + "(truth %.2f,%.2f vs %.2f,%.2f)", error,
                        truth.getX(), truth.getY(), asField.getX(), asField.getY()));
    }
}
