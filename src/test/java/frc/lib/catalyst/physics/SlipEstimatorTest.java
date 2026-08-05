package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.catalyst.physics.estimation.SlipEstimator;

/** Pure-Java tests for {@link SlipEstimator}. Module states are built by hand; no drivetrain needed. */
class SlipEstimatorTest {

    /** A square 0.7 x 0.7 m four-module swerve, in the usual FL/FR/BL/BR order. */
    private static SwerveDriveKinematics kinematics() {
        return new SwerveDriveKinematics(
                new Translation2d(0.35, 0.35),
                new Translation2d(0.35, -0.35),
                new Translation2d(-0.35, 0.35),
                new Translation2d(-0.35, -0.35));
    }

    private static SlipEstimator estimator() {
        // Unfiltered, so a single update is enough to assert on.
        return SlipEstimator.builder().kinematics(kinematics()).smoothing(1.0).build();
    }

    /** Exactly what the kinematics predicts for these chassis speeds — a perfectly rolling robot. */
    private static SwerveModuleState[] rolling(ChassisSpeeds speeds) {
        return kinematics().toSwerveModuleStates(speeds);
    }

    @Test
    void aCleanlyRollingRobotScoresZero() {
        SlipEstimator slip = estimator();
        ChassisSpeeds speeds = new ChassisSpeeds(3.0, 0.0, 0.0);

        assertEquals(0.0, slip.update(rolling(speeds), speeds), 1e-9);
        assertEquals(0.0, slip.peakSlip(), 1e-9);
        assertEquals(-1, slip.worstModule());
    }

    @Test
    void oneSpinningWheelShowsUpAsPeakButBarelyMovesTheMean() {
        SlipEstimator slip = estimator();
        ChassisSpeeds speeds = new ChassisSpeeds(3.0, 0.0, 0.0);

        SwerveModuleState[] measured = rolling(speeds);
        measured[2] = new SwerveModuleState(3.0 + 2.0, measured[2].angle);   // module 2 is spinning

        double mean = slip.update(measured, speeds);

        assertEquals(1.0, slip.peakSlip(), 1e-9);       // saturated: 2.0 m/s over a 0.5 threshold
        assertEquals(2, slip.worstModule());
        assertEquals(0.25, mean, 1e-9);                  // one module of four
        assertTrue(slip.peakSlip() > mean);
    }

    @Test
    void allFourWheelsSpinningSaturatesTheMeanToo() {
        SlipEstimator slip = estimator();
        ChassisSpeeds speeds = new ChassisSpeeds(2.0, 0.0, 0.0);

        SwerveModuleState[] measured = rolling(speeds);
        for (int i = 0; i < measured.length; i++) {
            measured[i] = new SwerveModuleState(measured[i].speedMetersPerSecond + 1.5, measured[i].angle);
        }

        assertEquals(1.0, slip.update(measured, speeds), 1e-9);
    }

    @Test
    void slipScalesLinearlyUpToTheThreshold() {
        SlipEstimator slip = estimator();
        ChassisSpeeds speeds = new ChassisSpeeds(2.0, 0.0, 0.0);

        SwerveModuleState[] measured = rolling(speeds);
        measured[0] = new SwerveModuleState(2.0 + 0.25, measured[0].angle);   // half of the 0.5 threshold

        slip.update(measured, speeds);
        assertEquals(0.5, slip.moduleScores()[0], 1e-9);
    }

    @Test
    void detectionIsSuppressedBelowTheMinimumSpeed() {
        SlipEstimator slip = estimator();
        ChassisSpeeds crawling = new ChassisSpeeds(0.1, 0.0, 0.0);   // under the 0.25 m/s floor

        SwerveModuleState[] measured = rolling(crawling);
        measured[1] = new SwerveModuleState(5.0, measured[1].angle);   // wildly wrong, but standing still

        assertEquals(0.0, slip.update(measured, crawling), 1e-9);
    }

    @Test
    void aModuleStillRotatingTowardItsSetpointIsNotMistakenForSlip() {
        SlipEstimator slip = estimator();
        ChassisSpeeds speeds = new ChassisSpeeds(3.0, 0.0, 0.0);

        SwerveModuleState[] measured = rolling(speeds);
        // Module 0 is 60 degrees off; only its along-track component counts, and the wheel has sped
        // up so that component still matches the prediction.
        measured[0] = new SwerveModuleState(3.0 / Math.cos(Math.toRadians(60.0)),
                measured[0].angle.plus(Rotation2d.fromDegrees(60.0)));

        slip.update(measured, speeds);
        assertEquals(0.0, slip.moduleScores()[0], 1e-6);
    }

    @Test
    void smoothingMakesASingleNoisyFrameLessThanConclusive() {
        SlipEstimator slip = SlipEstimator.builder().kinematics(kinematics()).smoothing(0.3).build();
        ChassisSpeeds speeds = new ChassisSpeeds(3.0, 0.0, 0.0);

        slip.update(rolling(speeds), speeds);   // settle at zero first

        SwerveModuleState[] spike = rolling(speeds);
        spike[0] = new SwerveModuleState(3.0 + 2.0, spike[0].angle);
        slip.update(spike, speeds);

        assertEquals(0.3, slip.moduleScores()[0], 1e-6);   // one frame gets 30% of the way there
        assertTrue(slip.peakSlip() < 0.5);
    }

    @Test
    void resetClearsScores() {
        SlipEstimator slip = estimator();
        ChassisSpeeds speeds = new ChassisSpeeds(3.0, 0.0, 0.0);
        SwerveModuleState[] measured = rolling(speeds);
        measured[0] = new SwerveModuleState(6.0, measured[0].angle);
        slip.update(measured, speeds);
        assertTrue(slip.peakSlip() > 0);

        slip.reset();
        assertEquals(0.0, slip.peakSlip(), 1e-9);
        assertEquals(-1, slip.worstModule());
    }

    @Test
    void badConfigurationAndMismatchedModuleCountsAreRejected() {
        assertThrows(IllegalStateException.class, () -> SlipEstimator.builder().build());
        assertThrows(IllegalStateException.class,
                () -> SlipEstimator.builder().kinematics(kinematics()).slipThreshold(0).build());
        assertThrows(IllegalStateException.class,
                () -> SlipEstimator.builder().kinematics(kinematics()).minSpeedForDetection(-1).build());

        SlipEstimator slip = estimator();
        assertThrows(IllegalArgumentException.class,
                () -> slip.update(new SwerveModuleState[2], new ChassisSpeeds()));
        assertThrows(IllegalArgumentException.class, () -> slip.update(null, new ChassisSpeeds()));
    }
}
