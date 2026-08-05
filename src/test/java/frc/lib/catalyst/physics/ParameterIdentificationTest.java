package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.lib.catalyst.physics.estimation.BatteryResistanceIdentifier;
import frc.lib.catalyst.physics.estimation.FeedforwardIdentifier;
import frc.lib.catalyst.physics.estimation.RecursiveLeastSquares;

/**
 * Pure-Java tests for online parameter identification. Every one generates data from coefficients the
 * test picked, then checks the fit recovers them — the only way to know a least-squares implementation
 * is right rather than merely plausible.
 *
 * <p>Deterministic throughout: the "noise" is a fixed sinusoid, not a random number, so a failure is
 * always reproducible.
 */
class ParameterIdentificationTest {

    @Test
    void itRecoversKnownCoefficientsFromCleanData() {
        RecursiveLeastSquares fit = RecursiveLeastSquares.forConstant(3);
        double[] truth = {2.5, -1.25, 0.75};

        for (int i = 0; i < 200; i++) {
            double x1 = Math.sin(i * 0.37);
            double x2 = Math.cos(i * 0.21);
            double x3 = Math.sin(i * 0.13 + 1.0);
            double y = truth[0] * x1 + truth[1] * x2 + truth[2] * x3;
            fit.add(new double[]{x1, x2, x3}, y);
        }

        assertEquals(truth[0], fit.parameter(0), 1e-3);
        assertEquals(truth[1], fit.parameter(1), 1e-3);
        assertEquals(truth[2], fit.parameter(2), 1e-3);
        assertTrue(fit.isConverged());
    }

    @Test
    void itStillRecoversThemThroughNoise() {
        RecursiveLeastSquares fit = RecursiveLeastSquares.forConstant(2);

        for (int i = 0; i < 2000; i++) {
            double x1 = Math.sin(i * 0.11);
            double x2 = Math.cos(i * 0.29);
            // A deterministic disturbance that averages to zero over the run.
            double noise = 0.02 * Math.sin(i * 2.7);
            fit.add(new double[]{x1, x2}, 3.0 * x1 + 0.5 * x2 + noise);
        }

        assertEquals(3.0, fit.parameter(0), 0.01);
        assertEquals(0.5, fit.parameter(1), 0.01);
    }

    @Test
    void aConstantRegressorLeavesTheFitUnconvinced() {
        // Every sample identical: infinitely many parameter pairs explain it equally well, and the
        // covariance must keep saying so no matter how many samples arrive.
        RecursiveLeastSquares fit = RecursiveLeastSquares.forConstant(2);
        for (int i = 0; i < 500; i++) fit.add(new double[]{1.0, 0.0}, 5.0);

        assertFalse(fit.isConverged());
    }

    @Test
    void forgettingLetsTheFitFollowAParameterThatChanges() {
        RecursiveLeastSquares fit = new RecursiveLeastSquares(1, 0.98, 100.0, 0.5);

        for (int i = 0; i < 500; i++) fit.add(new double[]{1.0}, 2.0);
        assertEquals(2.0, fit.parameter(0), 1e-3);

        for (int i = 0; i < 500; i++) fit.add(new double[]{1.0}, 5.0);
        assertEquals(5.0, fit.parameter(0), 1e-3);
    }

    @Test
    void aNeverForgettingFitResistsALateChange() {
        RecursiveLeastSquares fit = RecursiveLeastSquares.forConstant(1);

        for (int i = 0; i < 500; i++) fit.add(new double[]{1.0}, 2.0);
        for (int i = 0; i < 50; i++) fit.add(new double[]{1.0}, 5.0);

        // 50 new samples against 500 old ones barely move it - which is the point of not forgetting.
        assertTrue(fit.parameter(0) < 2.5, "expected the old data to dominate, got " + fit.parameter(0));
    }

    @Test
    void seedingSetsTheStartingPointWithoutPinningIt() {
        RecursiveLeastSquares fit = RecursiveLeastSquares.forConstant(1);
        fit.seed(9.0);
        assertEquals(9.0, fit.parameter(0), 1e-12);

        for (int i = 0; i < 300; i++) fit.add(new double[]{1.0}, 1.0);
        assertEquals(1.0, fit.parameter(0), 1e-3);
    }

    @Test
    void feedforwardGainsAreRecoveredFromSimulatedMotion() {
        double kS = 0.18, kV = 2.35, kA = 0.07;
        FeedforwardIdentifier identifier = FeedforwardIdentifier.builder("Flywheel").build();

        for (int i = 0; i < 400; i++) {
            // Sweep through a range of speeds and accelerations in both directions, which is what
            // makes the three terms separable.
            double velocity = 3.0 * Math.sin(i * 0.05);
            double acceleration = 8.0 * Math.cos(i * 0.05);
            if (Math.abs(velocity) < 0.1) continue;
            double volts = kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
            identifier.addSample(volts, velocity, acceleration);
        }

        assertEquals(kS, identifier.kS(), 1e-3);
        assertEquals(kV, identifier.kV(), 1e-3);
        assertEquals(kA, identifier.kA(), 1e-3);
        assertEquals(0.0, identifier.kG(), 1e-12);
    }

    @Test
    void anElevatorsGravityTermIsRecoveredToo() {
        double kS = 0.12, kV = 3.1, kA = 0.05, kG = 0.42;
        FeedforwardIdentifier identifier = FeedforwardIdentifier.builder("Elevator")
                .withElevatorGravity().build();

        for (int i = 0; i < 400; i++) {
            double velocity = 1.5 * Math.sin(i * 0.07);
            double acceleration = 4.0 * Math.cos(i * 0.11);
            if (Math.abs(velocity) < 0.1) continue;
            double volts = kS * Math.signum(velocity) + kV * velocity + kA * acceleration + kG;
            identifier.addSample(volts, velocity, acceleration);
        }

        assertEquals(kS, identifier.kS(), 1e-3);
        assertEquals(kV, identifier.kV(), 1e-3);
        assertEquals(kA, identifier.kA(), 1e-3);
        assertEquals(kG, identifier.kG(), 1e-3);
    }

    @Test
    void anArmsCosineGravityTermIsRecovered() {
        double kS = 0.10, kV = 1.8, kA = 0.03, kG = 0.65;
        double[] angle = {0.0};
        FeedforwardIdentifier identifier = FeedforwardIdentifier.builder("Arm")
                .withArmGravity(() -> angle[0]).build();

        for (int i = 0; i < 500; i++) {
            angle[0] = Math.sin(i * 0.09) * 1.2;
            double velocity = 2.0 * Math.cos(i * 0.05);
            double acceleration = 3.0 * Math.sin(i * 0.13);
            if (Math.abs(velocity) < 0.1) continue;
            double volts = kS * Math.signum(velocity) + kV * velocity + kA * acceleration
                    + kG * Math.cos(angle[0]);
            identifier.addSample(volts, velocity, acceleration);
        }

        assertEquals(kG, identifier.kG(), 1e-3);
        assertEquals(kV, identifier.kV(), 1e-3);
    }

    @Test
    void samplesTooSlowToSignAreRejected() {
        FeedforwardIdentifier identifier = FeedforwardIdentifier.builder("Roller")
                .velocityDeadband(0.5).build();

        assertFalse(identifier.addSample(1.0, 0.1, 0.0));
        assertFalse(identifier.addSample(1.0, -0.4, 0.0));
        assertTrue(identifier.addSample(1.0, 0.6, 0.0));

        assertEquals(2, identifier.rejectedSampleCount());
        assertEquals(1, identifier.sampleCount());
    }

    @Test
    void noRecommendationIsOfferedUntilTheFitHasSettled() {
        FeedforwardIdentifier identifier = FeedforwardIdentifier.builder("Flywheel").build();
        assertTrue(identifier.recommendation().isEmpty());

        // A mechanism run at one steady speed forever cannot separate kS from kV.
        for (int i = 0; i < 500; i++) identifier.addSample(2.0, 1.0, 0.0);
        assertTrue(identifier.recommendation().isEmpty(),
                "a single operating point must not produce a recommendation");
    }

    @Test
    void aConvergedRecommendationDescribesItselfAndTheChange() {
        FeedforwardIdentifier identifier = FeedforwardIdentifier.builder("Flywheel").build();
        for (int i = 0; i < 400; i++) {
            double v = 3.0 * Math.sin(i * 0.05);
            double a = 8.0 * Math.cos(i * 0.05);
            if (Math.abs(v) < 0.1) continue;
            identifier.addSample(0.2 * Math.signum(v) + 2.0 * v + 0.1 * a, v, a);
        }

        var recommendation = identifier.recommendation().orElseThrow();
        assertTrue(recommendation.describe().contains("Flywheel"));
        // Against gains that are 10% off, the reported relative change should be around 0.1.
        assertEquals(0.1, recommendation.relativeChangeFrom(0.2 / 1.1, 2.0 / 1.1, 0.1 / 1.1), 0.02);
    }

    @Test
    void batteryResistanceIsRecoveredFromVaryingLoad() {
        double openCircuit = 12.8;
        double resistance = 0.0185;
        BatteryResistanceIdentifier battery = new BatteryResistanceIdentifier();

        for (int i = 0; i < 600; i++) {
            double current = 30.0 + 25.0 * Math.sin(i * 0.08);
            battery.addSample(openCircuit - current * resistance, current);
        }

        assertEquals(resistance, battery.resistanceOhms(), 1e-4);
        assertEquals(openCircuit, battery.openCircuitVolts(), 1e-3);
        assertTrue(battery.isConverged());
        assertTrue(battery.recommendation().orElseThrow().isWorseThan(0.010));
        assertFalse(battery.recommendation().orElseThrow().isWorseThan(0.030));
    }

    @Test
    void aSteadyLoadCannotIdentifyResistanceNoMatterHowLongItRuns() {
        BatteryResistanceIdentifier battery = new BatteryResistanceIdentifier();
        for (int i = 0; i < 2000; i++) battery.addSample(12.3, 25.0);

        assertFalse(battery.isConverged());
        assertTrue(battery.recommendation().isEmpty());
        assertEquals(0.0, battery.currentSpreadAmps(), 1e-9);
    }

    @Test
    void thePredictedVoltageMatchesTheFittedLine() {
        BatteryResistanceIdentifier battery = new BatteryResistanceIdentifier();
        for (int i = 0; i < 400; i++) {
            double current = 20.0 + 20.0 * Math.sin(i * 0.1);
            battery.addSample(12.5 - current * 0.02, current);
        }

        assertEquals(12.5 - 60 * 0.02, battery.predictedVoltageAt(60), 1e-3);
    }

    @Test
    void resettingClearsEverything() {
        BatteryResistanceIdentifier battery = new BatteryResistanceIdentifier();
        for (int i = 0; i < 300; i++) battery.addSample(12.5 - i % 40 * 0.02, i % 40);
        assertTrue(battery.sampleCount() > 0);

        battery.reset();
        assertEquals(0, battery.sampleCount());
        assertEquals(0.0, battery.currentSpreadAmps(), 1e-9);
        assertFalse(battery.isConverged());
    }

    @Test
    void badConfigurationIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> new RecursiveLeastSquares(0, 1.0, 10, 0.1));
        assertThrows(IllegalArgumentException.class, () -> new RecursiveLeastSquares(2, 1.5, 10, 0.1));
        assertThrows(IllegalArgumentException.class, () -> new RecursiveLeastSquares(2, 1.0, 0, 0.1));

        RecursiveLeastSquares fit = RecursiveLeastSquares.forConstant(2);
        assertThrows(IllegalArgumentException.class, () -> fit.add(new double[]{1.0}, 1.0));
        assertThrows(IllegalArgumentException.class, () -> fit.seed(1.0));

        assertThrows(IllegalStateException.class,
                () -> FeedforwardIdentifier.builder("Arm").withArmGravity(null).build());
        assertThrows(IllegalStateException.class,
                () -> FeedforwardIdentifier.builder("X").seed(1.0, 2.0).build());
        assertThrows(IllegalStateException.class,
                () -> FeedforwardIdentifier.builder("X").velocityDeadband(-1).build());
    }
}
