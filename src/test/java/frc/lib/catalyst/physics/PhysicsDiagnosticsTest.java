package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.lib.catalyst.physics.diagnostics.FaultIsolator;
import frc.lib.catalyst.physics.diagnostics.JamDetector;
import frc.lib.catalyst.physics.diagnostics.ModelResidualMonitor;

/** Pure-Java tests for the diagnostics layer. No HAL, no NetworkTables, no robot. */
class PhysicsDiagnosticsTest {

    // ===========================================
    //          ModelResidualMonitor
    // ===========================================

    @Test
    void aResidualThatSwingsBothWaysIsCalledNoise() {
        ModelResidualMonitor monitor = new ModelResidualMonitor("wheel speed", 0.05);

        // Large scatter, zero mean: noisy sensor, correct model.
        for (int i = 0; i < 200; i++) monitor.record(0.4 * Math.sin(i * 1.1));

        assertFalse(monitor.isBiased(), "zero-mean scatter must not be reported as a fault");
        assertEquals(0.0, monitor.mean(), 0.02);
        assertTrue(monitor.standardDeviation() > 0.2);
    }

    @Test
    void aSmallConsistentOffsetIsCalledBiasEvenThoughItIsSmallerThanTheNoise() {
        ModelResidualMonitor monitor = new ModelResidualMonitor("wheel speed", 0.05);

        // A 0.12 offset buried in 0.4 of scatter. Peak-to-peak this looks like noise; the mean does
        // not, which is the whole reason to track both.
        for (int i = 0; i < 400; i++) monitor.record(0.12 + 0.4 * Math.sin(i * 1.1));

        assertTrue(monitor.isBiased());
        assertEquals(0.12, monitor.mean(), 0.03);
        assertTrue(monitor.biasDirection() > 0);
        assertTrue(monitor.normalizedBias() > 1.0);
    }

    @Test
    void anOffsetSmallerThanTheToleranceIsRealButNotWorthReporting() {
        ModelResidualMonitor monitor = new ModelResidualMonitor("wheel speed", 0.20);

        for (int i = 0; i < 400; i++) monitor.record(0.05);   // consistent, but well under tolerance

        assertFalse(monitor.isBiased());
        assertEquals(0.05, monitor.mean(), 1e-9);
    }

    @Test
    void noVerdictIsGivenFromAHandfulOfSamples() {
        ModelResidualMonitor monitor = new ModelResidualMonitor("wheel speed", 0.01);

        for (int i = 0; i < 5; i++) monitor.record(1.0);   // huge, consistent, and far too few
        assertFalse(monitor.isBiased());
        assertTrue(monitor.describe().contains("no verdict yet"));
    }

    @Test
    void theRecentMeanFollowsAModelThatStartsBeingWrongPartWayThrough() {
        ModelResidualMonitor monitor = new ModelResidualMonitor("wheel speed", 0.05);

        for (int i = 0; i < 200; i++) monitor.record(0.0);
        assertFalse(monitor.isBiased());

        for (int i = 0; i < 200; i++) monitor.record(0.3);   // something changed
        assertTrue(monitor.isBiased());
        assertEquals(0.3, monitor.recentMean(), 0.01);
        // The long-run mean is still dragged down by the clean half - which is why isBiased() uses
        // the recent one.
        assertTrue(monitor.mean() < monitor.recentMean());
    }

    @Test
    void nonFiniteResidualsAreIgnoredRatherThanPoisoningTheStatistics() {
        ModelResidualMonitor monitor = new ModelResidualMonitor("x", 0.01);
        for (int i = 0; i < 100; i++) monitor.record(0.1);
        double before = monitor.mean();

        monitor.record(Double.NaN);
        monitor.record(Double.POSITIVE_INFINITY);

        assertEquals(before, monitor.mean(), 1e-12);
        assertEquals(100, monitor.sampleCount());
    }

    @Test
    void resettingClearsTheMonitor() {
        ModelResidualMonitor monitor = new ModelResidualMonitor("x", 0.01);
        for (int i = 0; i < 100; i++) monitor.record(0.5);
        assertTrue(monitor.isBiased());

        monitor.reset();
        assertEquals(0, monitor.sampleCount());
        assertFalse(monitor.isBiased());
    }

    // ===========================================
    //                JamDetector
    // ===========================================

    /** A detector that needs 30 A, under 1.0 velocity, for 0.2 s. */
    private static JamDetector.Builder detector() {
        return JamDetector.builder("Intake").stallCurrentAmps(30).stallVelocity(1.0).holdSeconds(0.2);
    }

    @Test
    void aFreelyRunningMechanismIsFine() {
        JamDetector jam = detector().build();
        assertEquals(JamDetector.State.OK, jam.update(0.0, 12.0, 40.0, true));
    }

    @Test
    void aStartupCurrentSpikeDoesNotTripIt() {
        JamDetector jam = detector().build();

        // High current, not yet moving - exactly how every mechanism starts.
        assertEquals(JamDetector.State.OK, jam.update(0.00, 45.0, 0.0, true));
        assertEquals(JamDetector.State.OK, jam.update(0.10, 45.0, 0.0, true));
        // Then it breaks free before the hold time elapses.
        assertEquals(JamDetector.State.OK, jam.update(0.15, 30.0, 20.0, true));
    }

    @Test
    void aSustainedStallWithNoPieceIsAJam() {
        JamDetector jam = detector().pieceHeld(() -> false).build();

        jam.update(0.00, 45.0, 0.0, true);
        assertEquals(JamDetector.State.JAMMED, jam.update(0.25, 45.0, 0.0, true));
        assertTrue(jam.isJammed());
        assertEquals(0.25, jam.stallDurationSeconds(), 1e-9);
        assertTrue(jam.describe().contains("JAMMED"));
    }

    @Test
    void theSameSignalsWithAPieceHeldAreASuccessfulIntake() {
        JamDetector jam = detector().pieceHeld(() -> true).build();

        jam.update(0.00, 45.0, 0.0, true);
        assertEquals(JamDetector.State.ACQUIRED, jam.update(0.25, 45.0, 0.0, true));
        assertFalse(jam.isJammed(), "a successful intake must never be reported as a jam");
        assertTrue(jam.hasAcquired());
    }

    @Test
    void withNoPieceSensorItReportsTheStallWithoutGuessingWhy() {
        JamDetector jam = detector().build();

        jam.update(0.00, 45.0, 0.0, true);
        assertEquals(JamDetector.State.STALLED, jam.update(0.25, 45.0, 0.0, true));
        assertFalse(jam.isJammed());
        assertTrue(jam.describe().contains("cannot tell jam from acquisition"));
    }

    @Test
    void aMechanismNotBeingCommandedIsNeverStalled() {
        JamDetector jam = detector().pieceHeld(() -> false).build();

        // Resting: no velocity, and a current reading that would otherwise qualify.
        for (double t = 0; t < 2.0; t += 0.1) {
            assertEquals(JamDetector.State.OK, jam.update(t, 45.0, 0.0, false));
        }
    }

    @Test
    void clearingTheStallReturnsItToOk() {
        JamDetector jam = detector().pieceHeld(() -> false).build();

        jam.update(0.00, 45.0, 0.0, true);
        assertEquals(JamDetector.State.JAMMED, jam.update(0.25, 45.0, 0.0, true));
        assertEquals(JamDetector.State.OK, jam.update(0.30, 20.0, 15.0, true));
        assertEquals(0.0, jam.stallDurationSeconds(), 1e-9);
    }

    // ===========================================
    //               FaultIsolator
    // ===========================================

    @Test
    void thePatternOfResidualsSeparatesTwoCausesThatLookAlike() {
        ModelResidualMonitor wheelSpeed = new ModelResidualMonitor("FL wheel speed", 0.05);
        ModelResidualMonitor wheelsVsImu = new ModelResidualMonitor("wheels vs IMU", 0.10);

        FaultIsolator isolator = FaultIsolator.builder()
                .monitor(wheelSpeed)
                .monitor(wheelsVsImu)
                .candidate("Front-left wheel slip", c -> c
                        .expects("FL wheel speed", 1.0)
                        .expects("wheels vs IMU", 0.8))
                .candidate("Front-left wheel radius calibration", c -> c
                        .expects("FL wheel speed", 1.0)
                        .expectsQuiet("wheels vs IMU", 1.0))
                .build();

        // A radius error: that module reads wrong, but the robot's acceleration still agrees with
        // the wheels, so the IMU comparison stays clean.
        for (int i = 0; i < 200; i++) {
            wheelSpeed.record(0.15);
            wheelsVsImu.record(0.0);
        }
        assertEquals("Front-left wheel radius calibration", isolator.mostLikely().orElseThrow().cause());

        // Now genuine slip: both go off together.
        wheelsVsImu.reset();
        for (int i = 0; i < 200; i++) wheelsVsImu.record(0.4);
        assertEquals("Front-left wheel slip", isolator.mostLikely().orElseThrow().cause());
    }

    @Test
    void aContradictedCauseIsPenalisedAndSaysWhy() {
        ModelResidualMonitor a = new ModelResidualMonitor("a", 0.05);
        ModelResidualMonitor b = new ModelResidualMonitor("b", 0.05);

        FaultIsolator isolator = FaultIsolator.builder()
                .monitor(a).monitor(b)
                .candidate("Only A", c -> c.expects("a", 1.0).expectsQuiet("b", 1.0))
                .build();

        for (int i = 0; i < 200; i++) {
            a.record(0.2);
            b.record(0.2);   // this cause said b should have stayed quiet
        }

        var finding = isolator.mostLikely();
        // Fully contradicted: the penalty cancels the support, so it is not offered at all.
        assertTrue(finding.isEmpty() || !finding.get().isStrong());
        assertTrue(isolator.hasAnomaly());
    }

    @Test
    void nothingIsReportedWhenEveryResidualLooksLikeNoise() {
        ModelResidualMonitor a = new ModelResidualMonitor("a", 0.05);
        FaultIsolator isolator = FaultIsolator.builder()
                .monitor(a)
                .candidate("Something", c -> c.expects("a"))
                .build();

        for (int i = 0; i < 200; i++) a.record(0.3 * Math.sin(i * 1.3));

        assertTrue(isolator.rank().isEmpty());
        assertFalse(isolator.hasAnomaly());
        assertTrue(isolator.describe().contains("nothing to isolate"));
    }

    @Test
    void aCauseWhoseEvidenceIsAllPresentOutranksOneOnlyHalfSupported() {
        ModelResidualMonitor first = new ModelResidualMonitor("first", 0.05);
        ModelResidualMonitor second = new ModelResidualMonitor("second", 0.05);

        FaultIsolator isolator = FaultIsolator.builder()
                .monitor(first).monitor(second)
                .candidate("Explains both", c -> c.expects("first", 1.0).expects("second", 1.0))
                .candidate("Explains only the first", c -> c.expects("first", 1.0))
                .candidate("Needs the second too", c -> c.expects("second", 1.0).expects("first", 1.0))
                .build();

        for (int i = 0; i < 200; i++) {
            first.record(0.5);
            second.record(0.0);     // stays clean, so half of "Explains both" goes unsupported
        }

        var ranked = isolator.rank();
        assertEquals("Explains only the first", ranked.get(0).cause());
        assertEquals(1.0, ranked.get(0).score(), 1e-9);
        // The two-residual causes only got half their evidence, so they score half as well.
        assertEquals(0.5, ranked.get(1).score(), 1e-9);
        assertTrue(ranked.get(0).score() > ranked.get(1).score());
        assertTrue(ranked.get(0).describe().contains("supported by first"));
        assertTrue(ranked.get(0).isStrong());
    }

    @Test
    void severityDoesNotInflateTheScoreOnceAResidualIsClearlyBiased() {
        // The score answers "how well does this cause explain the pattern", not "how bad is it".
        // A residual at 10x tolerance and one at 1.2x are both unambiguously biased, so a cause that
        // predicts either explains its evidence equally well. Conflating the two would dress a
        // severity measure up as a confidence one.
        ModelResidualMonitor huge = new ModelResidualMonitor("huge", 0.05);
        ModelResidualMonitor slight = new ModelResidualMonitor("slight", 0.05);

        FaultIsolator isolator = FaultIsolator.builder()
                .monitor(huge).monitor(slight)
                .candidate("Explains the huge one", c -> c.expects("huge"))
                .candidate("Explains the slight one", c -> c.expects("slight"))
                .build();

        for (int i = 0; i < 200; i++) {
            huge.record(0.5);
            slight.record(0.06);
        }

        var ranked = isolator.rank();
        assertEquals(2, ranked.size());
        assertEquals(ranked.get(0).score(), ranked.get(1).score(), 1e-9);
        // Severity is still visible - just on the monitor, where it belongs.
        assertTrue(huge.normalizedBias() > slight.normalizedBias());
    }

    @Test
    void badConfigurationIsRejected() {
        ModelResidualMonitor a = new ModelResidualMonitor("a", 0.05);

        assertThrows(IllegalStateException.class, () -> FaultIsolator.builder()
                .monitor(a).candidate("No expectations", c -> { }).build());
        assertThrows(IllegalStateException.class, () -> FaultIsolator.builder()
                .monitor(a).candidate("Unknown residual", c -> c.expects("nope")).build());
        assertThrows(IllegalArgumentException.class, () -> new ModelResidualMonitor("x", 0.0));
        assertThrows(IllegalArgumentException.class, () -> new ModelResidualMonitor("x", 1.0, 3.0, 1, 0.5));
        assertThrows(IllegalStateException.class, () -> JamDetector.builder("x").stallCurrentAmps(0).build());
    }
}
