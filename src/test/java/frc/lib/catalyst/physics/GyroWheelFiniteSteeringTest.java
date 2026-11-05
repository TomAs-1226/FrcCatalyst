package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import frc.lib.catalyst.physics.estimation.GyroWheelOdometry;
import frc.lib.catalyst.physics.estimation.GyroWheelOdometry.Estimate;

/** Original physical/reference cases: no external implementation or native perfect-fix adapter. */
class GyroWheelFiniteSteeringTest {
    private static final Translation2d[] LOCATIONS = {
        new Translation2d(.3, .3), new Translation2d(.3, -.3),
        new Translation2d(-.3, .3), new Translation2d(-.3, -.3)
    };

    private static SwerveModulePosition[] parallel(double distance, double angle) {
        SwerveModulePosition[] modules = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) modules[i] = new SwerveModulePosition(distance, new Rotation2d(angle));
        return modules;
    }

    private static Estimate update(GyroWheelOdometry odometry, double time, double rawYaw,
            SwerveModulePosition[] positions) {
        // After the initial native anchor, a poisoned native position must not provide the answer.
        return odometry.update(time, time == 0 ? Pose2d.ZERO
                : new Pose2d(99, 99, Rotation2d.ZERO), new Rotation2d(rawYaw), positions);
    }

    @Test void quarterTurnSteeringIntegratesItsArcRatherThanTheEndingWheelDirection() {
        var odometry = new GyroWheelOdometry(LOCATIONS);
        update(odometry, 0, 0, parallel(0, 0));
        Estimate result = update(odometry, .02, 0, parallel(.10, Math.PI / 2));
        // Integral of a uniformly rotating unit vector over a quarter-circle: each component 2/pi.
        double component = .10 * 2 / Math.PI;
        assertTrue(result.valid());
        assertEquals(4, result.inliers());
        assertEquals(component, result.pose().getX(), 1e-10);
        assertEquals(component, result.pose().getY(), 1e-10);
        assertTrue(Math.abs(result.pose().getX()) > .06, "ending-angle-only integration loses all x travel");
    }

    @Test void signedDistanceAndNegativeDriveOptimizationPreservePhysicalDirections() {
        var reverse = new GyroWheelOdometry(LOCATIONS);
        update(reverse, 0, 0, parallel(0, 0));
        Estimate arc = update(reverse, .02, 0, parallel(-.10, Math.PI / 2));
        assertEquals(-.10 * 2 / Math.PI, arc.pose().getX(), 1e-10);
        assertEquals(-.10 * 2 / Math.PI, arc.pose().getY(), 1e-10);
        // An optimized target can command negative drive with physical azimuth pi, moving forward.
        // The measured physical azimuth is already pi at baseline; do not synthesize a target flip.
        var optimized = new GyroWheelOdometry(LOCATIONS);
        update(optimized, 0, 0, parallel(0, Math.PI));
        Estimate forward = update(optimized, .02, 0, parallel(-.05, Math.PI));
        assertTrue(forward.valid());
        assertEquals(.05, forward.pose().getX(), 1e-10);
        assertEquals(0, forward.pose().getY(), 1e-10);
        Estimate reversing = update(optimized, .04, 0, parallel(-.02, Math.PI));
        assertEquals(.02, reversing.pose().getX(), 1e-10,
                "a signed drive reversal at constant physical azimuth is ordinary signed travel");
    }

    @Test void steeringWrapUsesTheShortContinuousSweepWithoutAnArtificialFullTurn() {
        var odometry = new GyroWheelOdometry(LOCATIONS);
        update(odometry, 0, 0, parallel(0, Math.toRadians(179)));
        Estimate result = update(odometry, .02, 0, parallel(.10, Math.toRadians(-179)));
        double half = Math.toRadians(1);
        assertTrue(result.valid());
        assertEquals(-.10 * Math.sin(half) / half, result.pose().getX(), 1e-10);
        assertEquals(0, result.pose().getY(), 1e-10);
    }

    @Test void finiteGyroRotationSubtractsEachModulesExactCircularTravel() {
        var odometry = new GyroWheelOdometry(LOCATIONS);
        SwerveModulePosition[] modules = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) modules[i] = new SwerveModulePosition(0,
                new Rotation2d(-LOCATIONS[i].getY(), LOCATIONS[i].getX()));
        update(odometry, 0, 0, modules);
        double turn = .8;
        for (int i = 0; i < 4; i++) modules[i] = new SwerveModulePosition(
                LOCATIONS[i].getNorm() * turn, modules[i].angle);
        Estimate result = update(odometry, .08, turn, modules);
        assertTrue(result.valid());
        assertEquals(4, result.inliers());
        assertEquals(0, result.pose().getTranslation().getNorm(), 1e-10);
        assertEquals(turn, result.pose().getRotation().getRadians(), 1e-10);
    }

    /**
     * Independently generates measurements by integrating each MODULE POINT's physical velocity.
     * Chassis translation is fixed in the field while yaw changes, so measured body azimuths and
     * wheel speeds vary nonlinearly. Midpoint substeps approximate the underlying path, not the
     * estimator's endpoint-chord formula. Reversed wheels have physical azimuth pi and signed speed.
     */
    private static final class PhysicalRig {
        final GyroWheelOdometry odometry = new GyroWheelOdometry(LOCATIONS);
        final double vx, vy, omega;
        final boolean reversed;
        final double[] distance = new double[4];
        final double[] sensorError = new double[4];
        double time;
        PhysicalRig(double vx, double vy, double omega, boolean reversed) {
            this.vx = vx; this.vy = vy; this.omega = omega; this.reversed = reversed;
            update(odometry, 0, 0, measured());
        }
        Translation2d bodyVelocity(int index, double at) {
            double theta = omega * at;
            Translation2d r = LOCATIONS[index];
            return new Translation2d(Math.cos(theta) * vx + Math.sin(theta) * vy - omega * r.getY(),
                    -Math.sin(theta) * vx + Math.cos(theta) * vy + omega * r.getX());
        }
        SwerveModulePosition[] measured() {
            SwerveModulePosition[] positions = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++) {
                Translation2d v = bodyVelocity(i, time);
                double angle = Math.atan2(v.getY(), v.getX()) + (reversed ? Math.PI : 0);
                positions[i] = new SwerveModulePosition(distance[i] + sensorError[i], new Rotation2d(angle));
            }
            return positions;
        }
        Estimate step(double dt, int badModule, double extraDistance) {
            final int slices = 400;
            for (int i = 0; i < 4; i++) {
                double travel = 0;
                for (int n = 0; n < slices; n++) {
                    travel += bodyVelocity(i, time + (n + .5) * dt / slices).getNorm() * dt / slices;
                }
                distance[i] += reversed ? -travel : travel;
            }
            if (badModule >= 0) sensorError[badModule] += extraDistance;
            time += dt;
            return update(odometry, time, omega * time, measured());
        }
        void assertReference(Estimate result, double tolerance) {
            assertEquals(vx * time, result.pose().getX(), tolerance);
            assertEquals(vy * time, result.pose().getY(), tolerance);
            assertEquals(0, new Rotation2d(omega * time).minus(result.pose().getRotation()).getRadians(), 1e-9);
        }
    }

    @Test void independentlyIntegratedChangingSteeringRemainsCoherentAt100And250Hz() {
        for (double dt : new double[] {.01, .004}) {
            for (boolean reversed : new boolean[] {false, true}) {
                var rig = new PhysicalRig(1.5, .7, 5, reversed);
                for (int i = 0; i < Math.round(.8 / dt); i++) {
                    Estimate result = rig.step(dt, -1, 0);
                    assertTrue(result.valid(), "dt=" + dt + " frame=" + i + " " + result.reason());
                    assertEquals(4, result.inliers());
                    rig.assertReference(result, .002);
                }
            }
        }
    }

    @Test void oneBadEncoderDuringChangingSteerIsExcludedWithoutLosingTheReferencePath() {
        var rig = new PhysicalRig(1.5, .7, 5, false);
        for (int i = 0; i < 50; i++) {
            Estimate result = rig.step(.004, 1, .02);
            assertTrue(result.valid());
            assertEquals(3, result.inliers());
            assertEquals(1, result.rejectedModule());
            rig.assertReference(result, .001);
        }
        Estimate recovered = rig.step(.004, -1, 0);
        assertTrue(recovered.valid());
        assertEquals(4, recovered.inliers());
        rig.assertReference(recovered, .001);
    }

    @Test void withinIntervalReversalRequiresPreservedSamplesNotInventedEndpointTravel() {
        var sampled = new GyroWheelOdometry(LOCATIONS);
        update(sampled, 0, 0, parallel(0, 0));
        update(sampled, .01, 0, parallel(.05, Math.PI / 4));
        Estimate resolved = update(sampled, .02, 0, parallel(0, Math.PI / 2));
        // Positive travel during the first eighth-circle, negative during the second: net ds=0
        // does not mean net POSITION=0 when physical wheel direction changes during the reversal.
        assertTrue(resolved.valid());
        assertTrue(resolved.pose().getX() > .02);
        assertEquals(-resolved.pose().getX(), resolved.pose().getY(), 1e-10);
        var endpointsOnly = new GyroWheelOdometry(LOCATIONS);
        update(endpointsOnly, 0, 0, parallel(0, 0));
        Estimate aliased = update(endpointsOnly, .02, 0, parallel(0, Math.PI / 2));
        assertEquals(0, aliased.pose().getTranslation().getNorm(), 1e-10,
                "the lost positive/negative travel is absent from these identical encoder endpoints");
        assertTrue(aliased.reason().contains("ground motion unverified"));
    }

    @Test void exactHalfTurnsFailClosedAndTheirTravelIsNotReplayed() {
        var steering = new GyroWheelOdometry(LOCATIONS);
        update(steering, 0, 0, parallel(0, 0));
        Estimate ambiguous = update(steering, .02, 0, parallel(.02, Math.PI));
        assertFalse(ambiguous.valid());
        assertTrue(ambiguous.reason().contains("ambiguous half-turn steering"));
        assertEquals(0, ambiguous.pose().getTranslation().getNorm(), 1e-10);
        Estimate recovered = update(steering, .04, 0, parallel(.04, Math.PI));
        assertTrue(recovered.valid());
        assertEquals(-.02, recovered.pose().getX(), 1e-10);
        var gyro = new GyroWheelOdometry(LOCATIONS);
        update(gyro, 0, 0, parallel(0, 0));
        assertFalse(update(gyro, .02, Math.PI, parallel(.02, 0)).valid());
    }
}
