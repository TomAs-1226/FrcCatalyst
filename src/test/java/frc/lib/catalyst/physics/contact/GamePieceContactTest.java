package frc.lib.catalyst.physics.contact;

import org.junit.jupiter.api.Test;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

/**
 * Contact between a game piece and a robot, in the geometry the old tests avoided.
 *
 * <p>The push-out direction came from {@code Math.signum} of an offset component, and signum of an
 * exact zero is zero — which built a contact normal with no length and threw straight out of the
 * simulation loop.
 *
 * <p>Exact zeros are not exotic here. A piece on the robot's centreline has an offset of exactly
 * zero on one axis whenever the two share a coordinate at heading zero, and a piece sitting exactly
 * at the robot's centre does it on both at once. The existing tests all used a square robot with the
 * X offset larger than the Y offset, which never reaches the branch that broke.
 */
class GamePieceContactTest {

    /** A robot that is not moving: the contact geometry is what is under test, not the impulse. */
    private static final Translation2d STATIONARY = new Translation2d(0, 0);

    private static SimulatedGamePiece pieceAt(double x, double y, double z) {
        return SimulatedGamePiece.builder()
                .position(new Translation3d(x, y, z))
                .radius(0.12)
                .massKg(0.27)
                .build();
    }

    /** A robot longer than it is wide, which is what makes one branch reachable. */
    private static Pose2d robot() {
        return new Pose2d(1.0, 4.0, Rotation2d.ZERO);
    }

    @Test
    void aPieceOnTheRobotsCentrelineDoesNotThrow() {
        // offset.y is exactly 0.0: same y as the robot, heading zero. This threw for every x from
        // 0.98 to 1.09 - the whole width of the robot.
        for (double x = 0.95; x <= 1.10; x += 0.01) {
            SimulatedGamePiece piece = pieceAt(x, 4.0, 0.12);
            double at = x;
            assertDoesNotThrow(() -> piece.interactWithRobot(robot(), STATIONARY, 0.50, 0.40),
                    String.format("a piece at x=%.2f on the centreline", at));
        }
    }

    @Test
    void aPieceExactlyAtTheRobotsCentreDoesNotThrow() {
        // Both offsets exactly zero, which no shape of robot escapes.
        SimulatedGamePiece piece = pieceAt(1.0, 4.0, 0.12);
        assertDoesNotThrow(() -> piece.interactWithRobot(robot(), STATIONARY, 0.50, 0.40));
    }

    @Test
    void aPieceOnTheOtherAxisDoesNotThrowEither() {
        // The mirror case, reachable when the robot is wider than it is long.
        SimulatedGamePiece piece = pieceAt(1.0, 4.05, 0.12);
        assertDoesNotThrow(() -> piece.interactWithRobot(robot(), STATIONARY, 0.40, 0.50));
    }

    @Test
    void ordinaryOffCentreContactStillWorks() {
        // The case that always passed, kept so a fix cannot break it while chasing the others.
        SimulatedGamePiece piece = pieceAt(1.30, 4.20, 0.12);
        assertDoesNotThrow(() -> piece.interactWithRobot(robot(), STATIONARY, 0.50, 0.40));
    }
}
