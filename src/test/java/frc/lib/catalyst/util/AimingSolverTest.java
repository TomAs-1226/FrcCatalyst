package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Airtight verification of the Shoot-On-The-Fly aiming math.
 */
class AimingSolverSOTMTest {

    private static InterpolatingTable shotTimeTable() {
        return new InterpolatingTable()
                .add(1.0, 0.18).add(3.0, 0.38).add(6.0, 0.62)
                .add(9.0, 0.88).add(13.0, 1.20);
    }

    private static AimingSolverTOF solver(Translation2d goal) {
        return AimingSolverTOF.builder().target(goal).shotTime(shotTimeTable()).iterations(5).build();
    }

    /**
     * The core guarantee: a perfect shot built straight from the solution lands
     * on the real goal for every position and velocity. This is SOTF correctness
     * independent of any mechanism lag.
     */
    @Test
    void idealShotAlwaysLandsOnGoal() {
        Translation2d goal = new Translation2d(0.6, 4.10);
        AimingSolverTOF solver = solver(goal);

        double[] vs = {0.0, 1.0, 2.5, 4.0, -2.5, -4.0, 3.5};
        double maxErr = 0.0;
        int cases = 0;

        for (double px = 2.0; px <= 14.0; px += 1.0) {
            for (double py = 1.0; py <= 7.0; py += 1.0) {
                for (double vx : vs) {
                    for (double vy : vs) {
                        AimingSolverTOF.Solution s = solver.solve(
                                new Pose2d(px, py, new Rotation2d()), new ChassisSpeeds(vx, vy, 0));
                        if (!s.feasible() || s.shotTimeSeconds() <= 0) continue;

                        double tof = s.shotTimeSeconds();
                        double dVirt = s.virtualGoal().getDistance(new Translation2d(px, py));
                        double ang = Math.toRadians(s.turretFieldAngleDeg());
                        double speed = dVirt / tof;
                        double lx = px + (Math.cos(ang) * speed + vx) * tof;
                        double ly = py + (Math.sin(ang) * speed + vy) * tof;
                        maxErr = Math.max(maxErr, Math.hypot(lx - goal.getX(), ly - goal.getY()));
                        cases++;
                    }
                }
            }
        }
        System.out.printf("closed-loop: %d cases, max ideal landing error = %.3e m%n", cases, maxErr);
        assertTrue(maxErr < 1e-6, "max ideal landing error = " + maxErr + " m");
    }

    /** The reported solution is a true fixed point: tof == table(distance to virtualGoal). */
    @Test
    void solutionIsSelfConsistent() {
        Translation2d goal = new Translation2d(0.6, 4.10);
        AimingSolverTOF solver = solver(goal);
        InterpolatingTable tbl = shotTimeTable();
        double maxTofErr = 0, maxPinErr = 0;

        for (double px = 2; px <= 14; px += 1.7) {
            for (double py = 1; py <= 7; py += 1.3) {
                for (double v : new double[]{-4, -1.5, 0, 2, 4}) {
                    Translation2d r = new Translation2d(px, py);
                    AimingSolverTOF.Solution s = solver.solve(
                            new Pose2d(px, py, new Rotation2d()), new ChassisSpeeds(v, v * 0.5, 0));
                    if (!s.feasible()) continue;
                    double dVirt = s.virtualGoal().getDistance(r);
                    maxTofErr = Math.max(maxTofErr, Math.abs(s.shotTimeSeconds() - tbl.get(dVirt)));
                    // virtualGoal == goal - v*tof
                    double pinX = goal.getX() - v * s.shotTimeSeconds();
                    double pinY = goal.getY() - v * 0.5 * s.shotTimeSeconds();
                    maxPinErr = Math.max(maxPinErr,
                            Math.hypot(s.virtualGoal().getX() - pinX, s.virtualGoal().getY() - pinY));
                }
            }
        }
        assertTrue(maxTofErr < 1e-6, "tof not consistent with distance: " + maxTofErr);
        assertTrue(maxPinErr < 1e-9, "virtualGoal not consistent with tof: " + maxPinErr);
    }

    @Test
    void stationaryAimsStraightAtGoal() {
        Translation2d goal = new Translation2d(0.0, 0.0);
        AimingSolverTOF.Solution s = solver(goal).solve(
                new Pose2d(3.0, 0.0, new Rotation2d()), new ChassisSpeeds());
        assertEquals(180.0, Math.abs(s.turretFieldAngleDeg()), 1e-9);
        assertEquals(0.0, s.virtualGoal().getDistance(goal), 1e-9);
        assertEquals(0.0, s.turretFieldRateDps(), 1e-9);
    }

    /** Moving straight at/away from the goal changes distance, not bearing, and rate is ~0. */
    @Test
    void radialMotionPreservesBearing() {
        Translation2d goal = new Translation2d(0.0, 0.0);
        AimingSolverTOF solver = solver(goal);
        Pose2d pose = new Pose2d(5.0, 0.0, new Rotation2d());
        double stationary = solver.solve(pose, new ChassisSpeeds()).turretFieldAngleDeg();
        AimingSolverTOF.Solution moving = solver.solve(pose, new ChassisSpeeds(-3.0, 0.0, 0)); // toward goal
        assertEquals(stationary, moving.turretFieldAngleDeg(), 1e-9);
        assertEquals(0.0, moving.turretFieldRateDps(), 1e-9);
    }

    /** Moving tangentially produces a bearing rate of about v/d with the right sign. */
    @Test
    void tangentialMotionLeadsCorrectly() {
        Translation2d goal = new Translation2d(0.0, 0.0);
        AimingSolverTOF solver = solver(goal);
        Pose2d pose = new Pose2d(0.0, 5.0, new Rotation2d()); // straight above the goal
        AimingSolverTOF.Solution s = solver.solve(pose, new ChassisSpeeds(2.0, 0.0, 0)); // +x tangential
        double d = 5.0;
        double expected = Math.toDegrees(-2.0 / d); // moving +x above goal -> bearing decreases
        assertTrue(s.turretFieldRateDps() < 0, "rate should be negative; got " + s.turretFieldRateDps());
        assertEquals(expected, s.turretFieldRateDps(), Math.abs(expected) * 0.15);
        // and the lead must be opposite to motion: virtual goal shifts -x
        assertTrue(s.virtualGoal().getX() < 0, "virtual goal should lead opposite motion");
    }

    @Test
    void nonFiniteVelocityFallsBackToStationary() {
        Translation2d goal = new Translation2d(1.0, 2.0);
        AimingSolverTOF solver = solver(goal);
        Pose2d pose = new Pose2d(5.0, 6.0, new Rotation2d());
        AimingSolverTOF.Solution stat = solver.solve(pose, new ChassisSpeeds());
        AimingSolverTOF.Solution nan = solver.solve(pose,
                new ChassisSpeeds(Double.NaN, Double.POSITIVE_INFINITY, 0));
        assertEquals(stat.turretFieldAngleDeg(), nan.turretFieldAngleDeg(), 1e-9);
        assertTrue(Double.isFinite(nan.turretFieldAngleDeg()));
        assertTrue(Double.isFinite(nan.virtualGoal().getX()));
    }

    @Test
    void beyondMaxRangeIsInfeasible() {
        Translation2d goal = new Translation2d(0.0, 0.0);
        AimingSolverTOF solver = AimingSolverTOF.builder()
                .target(goal).shotTime(shotTimeTable()).maxRange(5.0).build();
        assertFalse(solver.solve(new Pose2d(10.0, 0.0, new Rotation2d()), new ChassisSpeeds()).feasible());
        assertTrue(solver.solve(new Pose2d(3.0, 0.0, new Rotation2d()), new ChassisSpeeds()).feasible());
    }

    @Test
    void robotOnTargetIsInfeasibleNotNaN() {
        Translation2d goal = new Translation2d(2.0, 2.0);
        AimingSolverTOF.Solution s = solver(goal).solve(
                new Pose2d(2.0, 2.0, new Rotation2d()), new ChassisSpeeds());
        assertFalse(s.feasible());
        assertEquals(0.0, s.turretFieldAngleDeg(), 1e-9);
        assertTrue(Double.isFinite(s.turretFieldRateDps()));
    }
}
