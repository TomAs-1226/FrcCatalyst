package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;

import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.observation.PoseObservation;

/**
 * A robot that launches hard, cruises and stops hard, with every wheel rolling and an IMU that agrees
 * with the wheels exactly. Nothing is wrong with this robot, so Physics Core must say so: no slip, no
 * impact, full confidence.
 *
 * <p>It did not. The disturbance residual compared the wheels' acceleration, smoothed, against the
 * IMU's, raw; the smoothed one lags, so at the end of every launch the wheels still "claimed"
 * acceleration the IMU no longer saw, which is the signature of wheel slip, and at the start of every
 * launch the IMU saw acceleration the wheels had not yet claimed, which is the signature of an
 * impact. On Catalyst X1 (2026-09-19) that is the 151 confidence dips on hard stick pushes.
 */
class CleanAccelerationTest {

    private static final double DT = 0.02;
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
            new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3));

    private static PhysicsCore core() {
        return PhysicsCore.builder()
                .robotModel(RobotModel.builder().massKg(60.0).footprintMeters(0.6, 0.6)
                        .centerOfMassHeightMeters(0.25).coefficientOfFriction(1.1).build())
                .kinematics(KINEMATICS)
                .clock(() -> 0.0)
                .withLogging(false)
                .withAlerts(false)
                .build();
    }

    /** The robot's acceleration at {@code t}, m/s^2 along its heading: launch, cruise, stop, launch again. */
    private static double profile(double t) {
        if (t < 0.5) return 0.0;
        if (t < 1.0) return 7.0;      // launch to 3.5 m/s
        if (t < 2.0) return 0.0;
        if (t < 2.5) return -7.0;     // stop
        if (t < 3.0) return 0.0;
        if (t < 3.3) return 9.0;      // a harder launch, 2.7 m/s
        if (t < 3.6) return -9.0;     // straight into a hard stop: the driver who corrects by reversing
        return 0.0;
    }

    private static void run(Rotation2d heading) {
        PhysicsCore physics = core();
        double v = 0.0;
        double worstConfidence = 1.0;
        double worstSlipTerm = 0.0;
        String worstReason = "nominal";
        Pose2d pose = new Pose2d(2.0, 2.0, heading);
        for (int i = 0; i < 250; i++) {
            double t = i * DT;
            double a = profile(t - DT / 2);   // the acceleration over the loop that just ended
            v += a * DT;
            ChassisVelocities speeds = new ChassisVelocities(v, 0.0, 0.0);
            physics.observe(PoseObservation.of(pose, t, "vision"));
            PhysicalRobotState state = physics.update(new PhysicsSample(
                    t, pose, speeds, KINEMATICS.toSwerveModuleVelocities(speeds), new Translation2d(a, 0.0), 0.0));
            if (i > 2 && state.quality().confidence() < worstConfidence) {
                worstConfidence = state.quality().confidence();
                worstReason = state.quality().reason();
            }
            worstSlipTerm = Math.max(worstSlipTerm, physics.analyze().disturbanceFraction());
            assertTrue(physics.analyze().lastCollision().isEmpty(),
                    "nothing hit this robot, at t = " + t + ": " + physics.analyze().lastCollision());
            // The fused velocity is the robot's: the IMU and the wheels say the same thing.
            assertEquals(v * heading.getCos(), state.fieldVelocity().vx, 0.05, "fused vx at t = " + t);
            assertEquals(v * heading.getSin(), state.fieldVelocity().vy, 0.05, "fused vy at t = " + t);
        }
        System.out.printf("[clean accel] heading %.0f deg: worst confidence %.3f (%s), worst residual %.3f of traction%n",
                heading.getDegrees(), worstConfidence, worstReason, worstSlipTerm);
        assertTrue(worstConfidence > 0.95,
                "a robot rolling cleanly keeps full confidence; it fell to " + worstConfidence + ": " + worstReason);
        assertTrue(worstSlipTerm < 0.05,
                "and the wheels and the IMU never disagree by more than noise: " + worstSlipTerm);
    }

    @Test
    void hardLaunchesAndStopsOnGoodTractionAreNotSlipOrImpact() {
        run(Rotation2d.ZERO);
    }

    @Test
    void theSameFacingAcrossTheField() {
        run(Rotation2d.fromDegrees(127.0));
    }
}
