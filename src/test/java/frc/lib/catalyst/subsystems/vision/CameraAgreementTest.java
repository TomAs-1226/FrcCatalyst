package frc.lib.catalyst.subsystems.vision;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Catching a mis-mounted camera by asking the other cameras.
 *
 * <p>The thing being tested is the one capability a second camera adds that no amount of work on a
 * single camera can: a {@code robotToCamera} transform has nothing to be checked against on its own,
 * so a camera mounted 12 cm from where the code believes produces confidently wrong poses that read
 * as noise. Two cameras seeing tags in the same loop are describing the same robot, so a persistent
 * disagreement is a wrong transform.
 */
class CameraAgreementTest {

    private static CameraAgreement.Sighting at(String name, double x, double y) {
        return new CameraAgreement.Sighting(name, new Pose2d(x, y, Rotation2d.kZero));
    }

    /** Feed the same geometry n times at a fixed heading. */
    private static void feed(CameraAgreement a, int n, Rotation2d heading,
                             List<CameraAgreement.Sighting> sightings) {
        for (int i = 0; i < n; i++) {
            a.observe(sightings, heading);
        }
    }

    @Test
    void camerasThatAgreeShowNoOffset() {
        CameraAgreement a = new CameraAgreement();
        feed(a, 60, Rotation2d.kZero, List.of(
                at("front", 4.0, 2.0),
                at("left", 4.0, 2.0),
                at("right", 4.0, 2.0)));

        assertEquals(0.0, a.medianOffset("front").orElseThrow().getNorm(), 1e-9);
        assertEquals(0.0, a.medianOffset("right").orElseThrow().getNorm(), 1e-9);
    }

    @Test
    void aCameraOffsetInTheRobotFrameIsMeasured() {
        // "front" thinks the robot is 0.4 m further along +X than the other two do. With the robot
        // pointing along +X, that is 0.4 m forward in the robot frame.
        CameraAgreement a = new CameraAgreement();
        feed(a, 60, Rotation2d.kZero, List.of(
                at("front", 4.4, 2.0),
                at("left", 4.0, 2.0),
                at("right", 4.0, 2.0)));

        Translation2d off = a.medianOffset("front").orElseThrow();
        assertEquals(0.4, off.getX(), 1e-9, "0.4 m forward");
        assertEquals(0.0, off.getY(), 1e-9);

        // And the honest majority reads zero, because the median is the consensus.
        assertEquals(0.0, a.medianOffset("left").orElseThrow().getNorm(), 1e-9);
    }

    @Test
    void theOffsetIsHeldInTheRobotFrameAsTheRobotTurns() {
        // The heart of it. A mounting error is fixed ON THE ROBOT, so it must survive the robot
        // turning. Measured in the field frame the same error would point a different way at every
        // heading and average toward nothing over a match - which is exactly how a real bias hides.
        CameraAgreement a = new CameraAgreement();

        // Facing +X: the bad camera reads 0.4 m along field +X.
        feed(a, 30, Rotation2d.kZero, List.of(
                at("front", 4.4, 2.0), at("left", 4.0, 2.0), at("right", 4.0, 2.0)));
        // Now facing +Y: the same physical error now points along field +Y.
        feed(a, 30, Rotation2d.fromDegrees(90), List.of(
                at("front", 4.0, 2.4), at("left", 4.0, 2.0), at("right", 4.0, 2.0)));

        Translation2d off = a.medianOffset("front").orElseThrow();
        assertEquals(0.4, off.getX(), 1e-6,
                "the same 0.4 m forward at both headings - that is what makes it a mount error");
        assertEquals(0.0, off.getY(), 1e-6);
    }

    @Test
    void aFieldFrameAccumulationWouldHaveCancelledOut() {
        // The control that proves the rotation above is load-bearing rather than decorative. Summed
        // in the FIELD frame the two headings cancel: +0.4 X then +0.4 Y averages to a direction
        // that is neither, and half the magnitude. Anything that reported that number would call a
        // real 0.4 m mount error a 0.28 m one pointing diagonally into nothing.
        double fieldX = (0.4 + 0.0) / 2.0;
        double fieldY = (0.0 + 0.4) / 2.0;
        double fieldNorm = Math.hypot(fieldX, fieldY);

        assertTrue(fieldNorm < 0.4,
                "field-frame averaging understates the error, which is why robot-frame is used");
        assertEquals(0.283, fieldNorm, 0.01);
    }

    @Test
    void oneCameraAloneIsNeverJudged() {
        // The single-camera case is exactly the one that cannot be checked. It must record nothing
        // rather than invent a consensus from one opinion.
        CameraAgreement a = new CameraAgreement();
        feed(a, 60, Rotation2d.kZero, List.of(at("only", 4.0, 2.0)));

        assertTrue(a.medianOffset("only").isEmpty(),
                "a lone camera has nothing to be compared against and must not be scored");
    }

    @Test
    void aVerdictNeedsEnoughSamples() {
        // A handful of bad frames while the robot is moving must not accuse a camera.
        CameraAgreement a = new CameraAgreement();
        feed(a, 5, Rotation2d.kZero, List.of(
                at("front", 9.0, 2.0), at("left", 4.0, 2.0), at("right", 4.0, 2.0)));

        assertTrue(a.medianOffset("front").isEmpty(), "five samples is not a verdict");
    }

    @Test
    void aBriefGlitchDoesNotMoveTheMedian() {
        // Median rather than mean, on purpose: one frame where a camera reads the far end of the
        // field must not drag the number. A mean over these samples would sit around 0.1 m.
        CameraAgreement a = new CameraAgreement();
        feed(a, 59, Rotation2d.kZero, List.of(
                at("front", 4.0, 2.0), at("left", 4.0, 2.0), at("right", 4.0, 2.0)));
        feed(a, 1, Rotation2d.kZero, List.of(
                at("front", 12.0, 2.0), at("left", 4.0, 2.0), at("right", 4.0, 2.0)));

        assertEquals(0.0, a.medianOffset("front").orElseThrow().getNorm(), 1e-9,
                "one glitched frame in sixty must not register as a mounting error");
    }
}
