package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.lib.catalyst.physics.diagnostics.CollisionDetector;
import frc.lib.catalyst.physics.diagnostics.CollisionEvent;
import frc.lib.catalyst.physics.estimation.DisturbanceEstimator;
import frc.lib.catalyst.physics.model.DrivetrainModel;
import frc.lib.catalyst.physics.model.RobotModel;

/** Pure-Java tests for {@link DisturbanceEstimator} and {@link CollisionDetector}. */
class DisturbanceAndCollisionTest {

    /** 60 kg, mu = 1.0, so the traction limit is exactly one g. */
    private static DrivetrainModel drivetrain() {
        return new DrivetrainModel(RobotModel.builder()
                .massKg(60.0)
                .footprintMeters(0.7, 0.7)
                .centerOfMassHeightMeters(0.2)
                .coefficientOfFriction(1.0)
                .build());
    }

    /** Unfiltered, so a single update is enough to assert on. */
    private static DisturbanceEstimator unfiltered() {
        return new DisturbanceEstimator(drivetrain(), 1.0);
    }

    @Test
    void aRobotDoingWhatItsWheelsSayHasNoDisturbance() {
        DisturbanceEstimator disturbance = unfiltered();

        disturbance.update(new Translation2d(3.0, 0.0), new Translation2d(3.0, 0.0));

        assertEquals(0.0, disturbance.magnitudeMpsSq(), 1e-9);
        assertEquals(0.0, disturbance.normalizedMagnitude(), 1e-9);
        assertEquals(0.0, disturbance.externalForceNewtons(), 1e-9);
        assertEquals(Rotation2d.kZero, disturbance.direction());   // too small to have a direction
    }

    @Test
    void spinningWheelsShowUpAsAResidualAgainstTheDirectionOfTravel() {
        DisturbanceEstimator disturbance = unfiltered();

        // Wheels claim 8 m/s^2 forward; the IMU says the robot barely accelerated.
        disturbance.update(new Translation2d(8.0, 0.0), new Translation2d(1.0, 0.0));

        assertEquals(7.0, disturbance.magnitudeMpsSq(), 1e-9);
        assertEquals(180.0, Math.abs(disturbance.direction().getDegrees()), 1e-6);
        assertEquals(7.0 * 60.0, disturbance.externalForceNewtons(), 1e-6);
    }

    @Test
    void anImpactShowsUpAsAccelerationTheWheelsNeverCommanded() {
        DisturbanceEstimator disturbance = unfiltered();

        // Wheels coasting; the IMU sees a sideways slam.
        disturbance.update(Translation2d.kZero, new Translation2d(0.0, -12.0));

        assertEquals(12.0, disturbance.magnitudeMpsSq(), 1e-9);
        assertTrue(disturbance.normalizedMagnitude() > 1.0);   // more than the carpet could deliver
        assertEquals(-90.0, disturbance.direction().getDegrees(), 1e-6);
    }

    @Test
    void nullAccelerationsAreTreatedAsZeroRatherThanThrowing() {
        DisturbanceEstimator disturbance = unfiltered();

        disturbance.update(null, new Translation2d(4.0, 0.0));
        assertEquals(4.0, disturbance.magnitudeMpsSq(), 1e-9);

        disturbance.update(new Translation2d(4.0, 0.0), null);
        assertEquals(4.0, disturbance.magnitudeMpsSq(), 1e-9);
    }

    @Test
    void smoothingStopsOneNoisyFrameFromDominating() {
        DisturbanceEstimator disturbance = new DisturbanceEstimator(drivetrain(), 0.4);

        disturbance.update(Translation2d.kZero, Translation2d.kZero);          // settle at zero
        disturbance.update(Translation2d.kZero, new Translation2d(10.0, 0.0)); // one spike

        assertEquals(4.0, disturbance.magnitudeMpsSq(), 1e-6);   // 40% of the way there
    }

    @Test
    void aCollisionNeedsToPersistBeforeItFires() {
        DisturbanceEstimator disturbance = unfiltered();
        CollisionDetector detector = CollisionDetector.builder().disturbance(disturbance).build();

        disturbance.update(Translation2d.kZero, new Translation2d(12.0, 0.0));
        assertTrue(detector.update(0.00).isEmpty());    // one loop over threshold is not enough
        assertTrue(detector.isImpactInProgress());

        Optional<CollisionEvent> event = detector.update(0.02);
        assertTrue(event.isPresent());
        assertEquals(12.0, event.get().magnitudeMpsSq(), 1e-9);
        assertEquals(12.0 * 60.0, event.get().peakForceNewtons(), 1e-6);
        assertEquals(0.02, event.get().timestampSeconds(), 1e-9);
    }

    @Test
    void aSingleNoisyFrameNeverFires() {
        DisturbanceEstimator disturbance = unfiltered();
        CollisionDetector detector = CollisionDetector.builder().disturbance(disturbance).build();

        disturbance.update(Translation2d.kZero, new Translation2d(12.0, 0.0));
        assertTrue(detector.update(0.00).isEmpty());

        disturbance.update(Translation2d.kZero, Translation2d.kZero);   // back to normal
        assertTrue(detector.update(0.02).isEmpty());
        assertFalse(detector.isImpactInProgress());

        // The streak was broken, so a later spike has to build up again from scratch.
        disturbance.update(Translation2d.kZero, new Translation2d(12.0, 0.0));
        assertTrue(detector.update(0.04).isEmpty());
    }

    @Test
    void theEventReportsThePeakOfTheImpactNotTheFirstFrameOverThreshold() {
        DisturbanceEstimator disturbance = unfiltered();
        CollisionDetector detector = CollisionDetector.builder()
                .disturbance(disturbance).requiredLoops(3).build();

        disturbance.update(Translation2d.kZero, new Translation2d(8.0, 0.0));
        detector.update(0.00);
        disturbance.update(Translation2d.kZero, new Translation2d(20.0, 0.0));   // the real hit
        detector.update(0.02);
        disturbance.update(Translation2d.kZero, new Translation2d(9.0, 0.0));
        Optional<CollisionEvent> event = detector.update(0.04);

        assertTrue(event.isPresent());
        assertEquals(20.0, event.get().magnitudeMpsSq(), 1e-9);
    }

    @Test
    void oneCollisionIsReportedOnceNotEveryLoopItRingsFor() {
        DisturbanceEstimator disturbance = unfiltered();
        CollisionDetector detector = CollisionDetector.builder().disturbance(disturbance).build();

        int fired = 0;
        for (int i = 0; i < 20; i++) {   // 400 ms of sustained over-threshold acceleration
            disturbance.update(Translation2d.kZero, new Translation2d(12.0, 0.0));
            if (detector.update(i * 0.02).isPresent()) fired++;
        }

        assertEquals(1, fired);   // the 0.5 s refractory period suppresses the rest
    }

    @Test
    void secondsSinceLastEventIsInfiniteUntilOneHappens() {
        DisturbanceEstimator disturbance = unfiltered();
        CollisionDetector detector = CollisionDetector.builder().disturbance(disturbance).build();

        assertTrue(Double.isInfinite(detector.secondsSinceLastEvent(1.0)));
        assertTrue(detector.lastEvent().isEmpty());

        disturbance.update(Translation2d.kZero, new Translation2d(12.0, 0.0));
        detector.update(0.00);
        detector.update(0.02);

        assertTrue(detector.lastEvent().isPresent());
        assertEquals(0.98, detector.secondsSinceLastEvent(1.0), 1e-9);
    }

    @Test
    void hardButLegalBrakingDoesNotCountAsACollision() {
        DisturbanceEstimator disturbance = unfiltered();
        CollisionDetector detector = CollisionDetector.builder().disturbance(disturbance).build();

        // Braking hard, and the IMU agrees with the wheels: no residual, no event.
        for (int i = 0; i < 10; i++) {
            disturbance.update(new Translation2d(-9.0, 0.0), new Translation2d(-9.0, 0.0));
            assertTrue(detector.update(i * 0.02).isEmpty());
        }
    }

    @Test
    void resetClearsTheDetectorAndTheResidual() {
        DisturbanceEstimator disturbance = unfiltered();
        CollisionDetector detector = CollisionDetector.builder().disturbance(disturbance).build();

        disturbance.update(Translation2d.kZero, new Translation2d(12.0, 0.0));
        detector.update(0.00);
        detector.update(0.02);
        assertTrue(detector.lastEvent().isPresent());

        detector.reset();
        disturbance.reset();

        assertTrue(detector.lastEvent().isEmpty());
        assertFalse(detector.isImpactInProgress());
        assertEquals(0.0, disturbance.magnitudeMpsSq(), 1e-9);
    }

    @Test
    void badConfigurationIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> new DisturbanceEstimator(null));
        assertThrows(IllegalStateException.class, () -> CollisionDetector.builder().build());
        assertThrows(IllegalStateException.class,
                () -> CollisionDetector.builder().disturbance(unfiltered()).threshold(0).build());
        assertThrows(IllegalStateException.class,
                () -> CollisionDetector.builder().disturbance(unfiltered()).requiredLoops(0).build());
        assertThrows(IllegalStateException.class,
                () -> CollisionDetector.builder().disturbance(unfiltered()).refractoryPeriod(-1).build());
    }
}
