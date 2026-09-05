package frc.lib.catalyst.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import java.util.Optional;

class AutoStartCheckTest {

    private static final Pose2d START = new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(180));

    @Test
    void atTheStartIsReady() {
        var r = AutoStartCheck.evaluate(new Pose2d(1.6, 5.4, Rotation2d.fromDegrees(176)), START, 0.30, 10);
        assertTrue(r.ready());
        assertTrue(r.available());
        assertEquals(0.141, r.distanceMeters(), 0.001);
        assertEquals(-4.0, r.headingErrorDegrees(), 1e-9);
    }

    @Test
    void halfAMetreOffIsNotReady() {
        var r = AutoStartCheck.evaluate(new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(180)), START, 0.30, 10);
        assertFalse(r.ready());
        assertTrue(r.detail().contains("0.50 m"), r.detail());
    }

    @Test
    void headingErrorWrapsAcrossTheSeam() {
        // -179 vs 180 is one degree apart, not 359.
        var r = AutoStartCheck.evaluate(new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(-179)), START, 0.30, 10);
        assertEquals(1.0, r.headingErrorDegrees(), 1e-9);
        assertTrue(r.ready());
        assertEquals(-170.0, AutoStartCheck.wrapDegrees(190), 1e-9);
        assertEquals(180.0, AutoStartCheck.wrapDegrees(-180), 1e-9);
    }

    @Test
    void headingAloneCanFailIt() {
        var r = AutoStartCheck.evaluate(new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(150)), START, 0.30, 10);
        assertFalse(r.ready());
        assertEquals(-30.0, r.headingErrorDegrees(), 1e-9);
    }

    @Test
    void noSelectedStartStandsDown() {
        AutoStartCheck check = new AutoStartCheck(() -> START, Optional::empty);
        var r = check.update();
        assertFalse(r.available());
        assertFalse(r.ready());
        assertTrue(Double.isNaN(r.distanceMeters()));
        assertFalse(AlertManager.getInstance().getWarnings().stream().anyMatch(w -> w.contains("starting pose")));
    }

    @Test
    void warningRaisesWhileOffAndClearsWhenBack() {
        Pose2d[] where = {new Pose2d(3.0, 5.5, Rotation2d.fromDegrees(180))};
        AutoStartCheck check = new AutoStartCheck(() -> where[0], () -> Optional.of(START));
        check.update();
        assertTrue(AlertManager.getInstance().getWarnings().stream()
                .anyMatch(w -> w.equals("[Auto] Robot is not at the selected auto's starting pose")));
        where[0] = START;
        var r = check.update();
        assertTrue(r.ready());
        assertFalse(AlertManager.getInstance().getWarnings().stream().anyMatch(w -> w.contains("starting pose")));
    }

    @Test
    void aSupplierThatThrowsStandsDownRatherThanCrashingDisabledPeriodic() {
        AutoStartCheck check = new AutoStartCheck(() -> { throw new IllegalStateException("no pose"); },
                () -> Optional.of(START));
        assertFalse(check.update().available());
    }
}
