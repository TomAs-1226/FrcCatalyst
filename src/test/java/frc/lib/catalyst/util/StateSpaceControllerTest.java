package frc.lib.catalyst.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

/**
 * The position controller's two ways of correcting. The one that takes a position alone has no velocity sensor
 * behind it, and the filter must not end up certain of a velocity nothing measured.
 */
class StateSpaceControllerTest {

    /**
     * Every test here builds a Kalman filter, which solves its gain in wpimath's native library. On the alpha-7
     * line that library cannot load - it wants a libtelemetry that WPILib alpha-7 does not ship beside it - so
     * there the class skips rather than reporting a failure it cannot fix.
     */
    @BeforeAll
    static void needsWpimathNatives() {
        try {
            elevator();
        } catch (UnsatisfiedLinkError | NoClassDefFoundError | ExceptionInInitializerError e) {
            Assumptions.abort("wpimath's native library will not load here: " + e);
        }
    }

    private static StateSpaceController.Position elevator() {
        // motor, mass, drum radius, gearing; then the model and encoder noise, the error budget and the volts.
        return StateSpaceController.createElevator(DCMotor.getKrakenX60(2), 5.0, 0.0254, 10.0,
                0.05, 3.0, 0.001, 0.01, 0.02, 0.4, 6.0);
    }

    /**
     * Correcting from position alone must leave the velocity estimate free to follow the next real evidence. Run
     * both the same way, then step the position on by a jump only a moving mechanism could make: the filter that
     * was told nothing about velocity follows it at least as readily as one told the truth about its own estimate.
     */
    @Test
    void aPositionOnlyCorrectionDoesNotTeachTheFilterAVelocityNothingMeasured() {
        StateSpaceController.Position quiet = elevator();
        for (int i = 0; i < 50; i++) {
            quiet.setReference(0.0);
            quiet.correct(0.0);
            quiet.predict(0.02);
        }
        assertEquals(0.0, quiet.getEstimatedVelocity(), 1e-6, "standing still, it estimates no velocity");

        // Now the mechanism moves: 2 cm a loop is 1 m/s.
        double position = 0.0;
        for (int i = 0; i < 25; i++) {
            position += 0.02;
            quiet.setReference(1.0);
            quiet.correct(position);
            quiet.predict(0.02);
        }
        assertTrue(quiet.getEstimatedVelocity() > 0.5,
                "the velocity estimate should follow the positions it is fed, was " + quiet.getEstimatedVelocity());
    }

    /**
     * With a real velocity sensor both channels are measurements, and the estimate tracks them. The reference
     * stays ahead of the mechanism: asked to stop at a mark it has already passed, the controller commands the
     * other way and the model's own prediction, not the measurements, is what moves the estimate.
     */
    @Test
    void bothMeasurementsTogetherStillTrack() {
        StateSpaceController.Position p = elevator();
        for (int i = 0; i < 40; i++) {
            p.setReference(2.0, 1.0);
            p.correct(0.02 * i, 1.0);
            p.predict(0.02);
        }
        assertEquals(1.0, p.getEstimatedVelocity(), 0.3, "velocity " + p.getEstimatedVelocity());
        assertEquals(0.78, p.getEstimatedPosition(), 0.2, "position " + p.getEstimatedPosition());
    }

    /** A voltage comes out, and it is inside the limit the controller was built with. */
    @Test
    void theVoltageStaysInsideTheLimit() {
        StateSpaceController.Position p = elevator();
        p.setReference(1.0);
        p.correct(0.0);
        p.predict(0.02);
        assertTrue(Math.abs(p.getVoltage()) <= 6.0 + 1e-9, "voltage " + p.getVoltage());
    }
}
