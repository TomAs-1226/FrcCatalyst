package frc.lib.catalyst.physics.estimation;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

/** Invalid telemetry must not destroy an identification fit or masquerade as new evidence. */
class ParameterIdentificationFaultTest {
    private static void unchanged(RecursiveLeastSquares fit, double[] parameters, double trace,
                                  long samples, double residual) {
        assertArrayEquals(parameters, fit.parameters(), 0);
        assertEquals(trace, fit.covarianceTrace(), 0);
        assertEquals(samples, fit.sampleCount());
        assertEquals(residual, fit.lastResidual(), 0);
    }

    @Test void nonfiniteMeasurementsAndRegressorsLeaveEveryStateFieldUntouched() {
        for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            for (boolean badMeasurement : new boolean[] {true, false}) {
                var fit = RecursiveLeastSquares.forConstant(2);
                fit.seed(2, -1);
                fit.add(new double[] {1, 2}, 0.1);
                double[] before = fit.parameters();
                double trace = fit.covarianceTrace(), residual = fit.lastResidual();
                long count = fit.sampleCount();
                fit.add(badMeasurement ? new double[] {1, 2} : new double[] {1, invalid},
                        badMeasurement ? invalid : 3);
                unchanged(fit, before, trace, count, residual);
            }
        }
    }

    @Test void finiteInputOverflowCannotPartiallyUpdateParametersCovarianceOrResidual() {
        var denominatorOverflow = RecursiveLeastSquares.forConstant(2);
        denominatorOverflow.seed(2, -1);
        denominatorOverflow.add(new double[] {1, 2}, 0.1);
        double[] before = denominatorOverflow.parameters();
        double trace = denominatorOverflow.covarianceTrace(), residual = denominatorOverflow.lastResidual();
        long count = denominatorOverflow.sampleCount();
        denominatorOverflow.add(new double[] {1e200, 1}, 5);
        unchanged(denominatorOverflow, before, trace, count, residual);

        var covarianceOverflow = new RecursiveLeastSquares(1, 1, 1e200, 0.01);
        covarianceOverflow.seed(3);
        covarianceOverflow.add(new double[] {1}, 4);
        unchanged(covarianceOverflow, new double[] {3}, 1e200, 0, 0);

        var predictionOverflow = RecursiveLeastSquares.forConstant(2);
        predictionOverflow.seed(1, Double.MAX_VALUE);
        predictionOverflow.add(new double[] {1, 2}, 0);
        unchanged(predictionOverflow, new double[] {1, Double.MAX_VALUE}, 2000, 0, 0);
    }

    @Test void invalidSeedIsRejectedBeforeAnyCoefficientChanges() {
        var fit = RecursiveLeastSquares.forConstant(2);
        fit.seed(2, -1);
        for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class, () -> fit.seed(5, invalid));
            assertArrayEquals(new double[] {2, -1}, fit.parameters(), 0);
        }
    }

    @Test void rejectedFaultsDoNotPreventLaterValidDataFromConverging() {
        var fit = RecursiveLeastSquares.forConstant(2);
        fit.add(new double[] {1, 2}, Double.NaN);
        fit.add(new double[] {1e200, 1}, 0);
        fit.add(new double[] {1, Double.POSITIVE_INFINITY}, 0);
        assertEquals(0, fit.sampleCount());
        for (int i = 0; i < 1000; i++) {
            double x = Math.sin(i * 0.13), y = Math.cos(i * 0.31);
            fit.add(new double[] {x, y}, 2 * x - y);
        }
        assertTrue(fit.isConverged());
        assertEquals(2, fit.parameter(0), 1e-4);
        assertEquals(-1, fit.parameter(1), 1e-4);
        assertTrue(Double.isFinite(fit.covarianceTrace()));
    }

    @Test void feedforwardReportsRejectedTelemetryAndRetainsFiniteRecommendation() {
        var id = FeedforwardIdentifier.builder("fault-test").build();
        assertFalse(id.addSample(Double.NaN, 1, 1));
        assertFalse(id.addSample(Double.POSITIVE_INFINITY, 1, 1));
        assertFalse(id.addSample(1, Double.NaN, 1));
        assertFalse(id.addSample(1, 1, Double.NEGATIVE_INFINITY));
        assertFalse(id.addSample(1, 1e200, 1));
        assertEquals(0, id.sampleCount());
        assertEquals(5, id.rejectedSampleCount());
        for (int i = 0; i < 1200; i++) {
            double v = 3 * Math.sin(i * 0.13), a = 4 * Math.cos(i * 0.31);
            id.addSample(0.2 * Math.signum(v) + 2 * v + 0.1 * a, v, a);
        }
        assertTrue(id.isConverged());
        assertTrue(id.recommendation().isPresent());
        assertEquals(0.2, id.kS(), 1e-3);
        assertEquals(2, id.kV(), 1e-3);
        assertEquals(0.1, id.kA(), 1e-3);
    }

    @Test void invalidArmAngleIsARejectedRegressorRatherThanSuccessfulSample() {
        var id = FeedforwardIdentifier.builder("arm-fault")
                .withArmGravity(() -> Double.NaN).build();
        assertFalse(id.addSample(2, 1, 1));
        assertEquals(0, id.sampleCount());
        assertEquals(1, id.rejectedSampleCount());
        assertFalse(id.recommendation().isPresent());
    }
}