package frc.lib.catalyst.identity;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;

import org.junit.jupiter.api.Test;

/**
 * Track width and wheelbase are the one place on the spec sheet where the arithmetic will happily
 * produce two convincing numbers for a robot that does not exist.
 *
 * <p>{@code 2*max|y|} by {@code 2*max|x|} describes a rectangle. Hand it a diamond layout and it
 * still answers, and the answer is a rectangle the modules do not sit on. So the layout is tested
 * before the span is published, and these are the cases that test decides.
 */
class RobotIdentityGeometryTest {

    /** The usual 4-module square: 0.6 m between contact patches on both axes. */
    private static Translation2d[] square() {
        return new Translation2d[] {
            new Translation2d(0.3, 0.3),
            new Translation2d(0.3, -0.3),
            new Translation2d(-0.3, 0.3),
            new Translation2d(-0.3, -0.3),
        };
    }

    @Test
    void rectangularLayoutGivesTrackWidthAndWheelBase() {
        Optional<double[]> span = RobotIdentity.rectangularSpan(square());
        assertTrue(span.isPresent());
        assertEquals(0.6, span.get()[0], 1e-9, "track width is side to side");
        assertEquals(0.6, span.get()[1], 1e-9, "wheel base is front to back");
    }

    @Test
    void nonSquareRectangleIsStillARectangle() {
        Translation2d[] locations = {
            new Translation2d(0.35, 0.28),
            new Translation2d(0.35, -0.28),
            new Translation2d(-0.35, 0.28),
            new Translation2d(-0.35, -0.28),
        };
        Optional<double[]> span = RobotIdentity.rectangularSpan(locations);
        assertTrue(span.isPresent());
        assertEquals(0.56, span.get()[0], 1e-9);
        assertEquals(0.70, span.get()[1], 1e-9);
    }

    @Test
    void diamondLayoutHasNoSpanToPublish() {
        Translation2d[] diamond = {
            new Translation2d(0.4, 0.0),
            new Translation2d(0.0, 0.4),
            new Translation2d(-0.4, 0.0),
            new Translation2d(0.0, -0.4),
        };
        assertTrue(RobotIdentity.rectangularSpan(diamond).isEmpty(),
                "a diamond has no track width, and 0.8 x 0.8 would describe a robot that isn't there");
    }

    @Test
    void oneModuleOutOfLineDisqualifiesTheWholeLayout() {
        Translation2d[] skewed = square();
        skewed[2] = new Translation2d(-0.25, 0.3);
        assertTrue(RobotIdentity.rectangularSpan(skewed).isEmpty());
    }

    @Test
    void buildToleranceIsAccepted() {
        // Half a millimetre of asymmetry is a measurement, not a different chassis.
        Translation2d[] locations = square();
        locations[0] = new Translation2d(0.3005, 0.3);
        assertTrue(RobotIdentity.rectangularSpan(locations).isPresent());
    }

    @Test
    void oddAndUndersizedLayoutsAreRejected() {
        assertTrue(RobotIdentity.rectangularSpan(null).isEmpty());
        assertTrue(RobotIdentity.rectangularSpan(new Translation2d[0]).isEmpty());
        assertTrue(RobotIdentity.rectangularSpan(new Translation2d[] {
            new Translation2d(0.3, 0.3), new Translation2d(-0.3, -0.3),
        }).isEmpty(), "two modules cannot describe a rectangle");
        assertTrue(RobotIdentity.rectangularSpan(new Translation2d[] {
            new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
            new Translation2d(-0.3, 0.3),
        }).isEmpty(), "three modules cannot either");
    }

    @Test
    void modulesOnTheCentrelineHaveNoSpan() {
        Translation2d[] inLine = {
            new Translation2d(0.3, 0.0),
            new Translation2d(0.3, 0.0),
            new Translation2d(-0.3, 0.0),
            new Translation2d(-0.3, 0.0),
        };
        assertTrue(RobotIdentity.rectangularSpan(inLine).isEmpty(),
                "a zero track width is not a track width");
    }
}
