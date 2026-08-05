package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.catalyst.physics.contact.ContactMaterial;
import frc.lib.catalyst.physics.contact.ContactResolver;
import frc.lib.catalyst.physics.contact.SimulatedGamePiece;
import org.junit.jupiter.api.Test;

/** Pure-Java tests for contact physics. No HAL, no NetworkTables, no robot. */
class ContactTest {

    private static final double EPS = 1e-9;

    // ---------------------------------------------------------------- materials

    @Test
    void combiningMaterialsSitsBetweenTheTwo() {
        double e = ContactMaterial.FOAM_GAME_PIECE.restitutionWith(ContactMaterial.POLYCARBONATE);
        assertTrue(e > ContactMaterial.FOAM_GAME_PIECE.restitution());
        assertTrue(e < ContactMaterial.POLYCARBONATE.restitution());
    }

    @Test
    void aDeadSurfaceKillsTheBounceOfWhateverHitsIt() {
        ContactMaterial deadened = ContactMaterial.CARPET.withRestitution(0.0);
        assertEquals(0.0, ContactMaterial.POLYCARBONATE.restitutionWith(deadened), EPS);
    }

    @Test
    void theSamePieceBouncesFurtherOffTheWallThanOffTheCarpet() {
        ContactMaterial piece = ContactMaterial.FOAM_GAME_PIECE;
        assertTrue(
                piece.restitutionWith(ContactMaterial.POLYCARBONATE)
                        > piece.restitutionWith(ContactMaterial.CARPET),
                "the whole point of per-material contact is that these two differ");
    }

    @Test
    void impossibleMaterialsAreRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> new ContactMaterial("perpetual motion", 1.4, 0.5, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> new ContactMaterial("negative", -0.1, 0.5, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> new ContactMaterial(" ", 0.5, 0.5, 0.0));
    }

    // ---------------------------------------------------------------- resolver

    @Test
    void aPerfectlyElasticHeadOnBounceReversesTheVelocity() {
        ContactMaterial superball = new ContactMaterial("superball", 1.0, 0.0, 0.0);
        ContactMaterial wall = new ContactMaterial("hard wall", 1.0, 0.0, 0.0);

        var after = ContactResolver.resolveAgainstStatic(
                new Translation3d(0, 0, -4.0), new Translation3d(0, 0, 1), 0.3, superball, wall);

        assertEquals(4.0, after.velocityA().getZ(), 1e-6);
        assertTrue(after.resolved());
    }

    @Test
    void aDeadBounceJustStopsTheApproach() {
        ContactMaterial clay = new ContactMaterial("clay", 0.0, 0.0, 0.0);
        var after = ContactResolver.resolveAgainstStatic(
                new Translation3d(0, 0, -4.0), new Translation3d(0, 0, 1), 0.3, clay, clay);
        assertEquals(0.0, after.velocityA().getZ(), 1e-6);
    }

    @Test
    void aSeparatingContactIsLeftAlone() {
        // Already moving away from the floor: resolving would yank it back down.
        var after = ContactResolver.resolveAgainstStatic(
                new Translation3d(0, 0, 2.0), new Translation3d(0, 0, 1), 0.3,
                ContactMaterial.FOAM_GAME_PIECE, ContactMaterial.CARPET);
        assertFalse(after.resolved());
        assertEquals(2.0, after.velocityA().getZ(), EPS);
    }

    @Test
    void aSlowContactDoesNotBounceSoThingsCanComeToRest() {
        double crawling = ContactResolver.RESTING_SPEED * 0.5;
        var after = ContactResolver.resolveAgainstStatic(
                new Translation3d(0, 0, -crawling), new Translation3d(0, 0, 1), 0.3,
                ContactMaterial.FOAM_GAME_PIECE, ContactMaterial.CARPET);
        assertEquals(0.0, after.velocityA().getZ(), 1e-6);
    }

    @Test
    void frictionScrubsSidewaysSpeedAndGrippySurfacesScrubMore() {
        Translation3d approach = new Translation3d(3.0, 0, -2.0);
        Translation3d up = new Translation3d(0, 0, 1);

        var onCarpet = ContactResolver.resolveAgainstStatic(
                approach, up, 0.3, ContactMaterial.FOAM_GAME_PIECE, ContactMaterial.CARPET);
        var onSlick = ContactResolver.resolveAgainstStatic(
                approach, up, 0.3, ContactMaterial.FOAM_GAME_PIECE,
                ContactMaterial.CARPET.withFriction(0.05));

        assertTrue(onCarpet.velocityA().getX() < approach.getX(), "carpet should slow the slide");
        assertTrue(onCarpet.velocityA().getX() < onSlick.velocityA().getX(),
                "a grippier floor must remove more sideways speed than a slick one");
    }

    @Test
    void frictionNeverReversesTheSlide() {
        // A huge friction coefficient must stop the slide, not send it backwards.
        var after = ContactResolver.resolveAgainstStatic(
                new Translation3d(1.0, 0, -0.5), new Translation3d(0, 0, 1), 0.3,
                ContactMaterial.FOAM_GAME_PIECE.withFriction(50.0),
                ContactMaterial.CARPET.withFriction(50.0));
        assertTrue(after.velocityA().getX() >= -1e-9,
                "clamping to the Coulomb cone should stop it, never push it back");
        assertFalse(after.sliding(), "friction this large is enough to hold, so the contact sticks");
    }

    @Test
    void twoMovingBodiesConserveMomentum() {
        double massA = 2.0, massB = 5.0;
        Translation3d va = new Translation3d(4.0, 0, 0);
        Translation3d vb = new Translation3d(-1.0, 0, 0);

        var after = ContactResolver.resolve(
                va, vb, new Translation3d(-1, 0, 0), massA, massB,
                new ContactMaterial("a", 0.5, 0.0, 0.0), new ContactMaterial("b", 0.5, 0.0, 0.0));

        double before = massA * va.getX() + massB * vb.getX();
        double now = massA * after.velocityA().getX() + massB * after.velocityB().getX();
        assertEquals(before, now, 1e-9, "an internal impulse cannot change total momentum");
    }

    @Test
    void anImmovableSurfaceDoesNotMove() {
        var after = ContactResolver.resolveAgainstStatic(
                new Translation3d(0, 0, -3.0), new Translation3d(0, 0, 1), 0.3,
                ContactMaterial.FOAM_GAME_PIECE, ContactMaterial.CARPET);
        assertEquals(0.0, after.velocityB().getNorm(), EPS);
    }

    @Test
    void aNormalWithNoDirectionIsRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> ContactResolver.resolveAgainstStatic(
                        new Translation3d(0, 0, -1), Translation3d.kZero, 1.0,
                        ContactMaterial.CARPET, ContactMaterial.CARPET));
    }

    // ---------------------------------------------------------------- game piece

    private static SimulatedGamePiece dropFrom(double height, ContactMaterial floor) {
        return SimulatedGamePiece.builder()
                .position(new Translation3d(8.0, 4.0, height))
                .floorMaterial(floor)
                .dragCoefficient(0.0)
                .build();
    }

    @Test
    void aDroppedPieceSettlesOnTheFloor() {
        SimulatedGamePiece piece = dropFrom(2.0, ContactMaterial.CARPET);
        for (int i = 0; i < 2000 && !piece.isAtRest(); i++) {
            piece.step(0.002);
        }
        assertTrue(piece.isAtRest(), "it should stop bouncing rather than jitter forever");
        assertEquals(0.12, piece.position().getZ(), 1e-3, "at rest it sits one radius off the carpet");
        assertTrue(piece.contactCount() > 0);
    }

    @Test
    void aPieceBouncesHigherOnAHarderFloor() {
        double carpetPeak = peakAfterFirstBounce(ContactMaterial.CARPET);
        double woodPeak = peakAfterFirstBounce(ContactMaterial.POLYCARBONATE);
        assertTrue(woodPeak > carpetPeak,
                "same drop, different floor, different bounce — carpet " + carpetPeak
                        + " vs polycarbonate " + woodPeak);
    }

    private static double peakAfterFirstBounce(ContactMaterial floor) {
        SimulatedGamePiece piece = dropFrom(2.0, floor);
        while (piece.contactCount() == 0) {
            piece.step(0.001);
        }
        double peak = 0;
        for (int i = 0; i < 3000 && piece.velocity().getZ() >= 0; i++) {
            piece.step(0.001);
            peak = Math.max(peak, piece.position().getZ());
        }
        return peak;
    }

    @Test
    void aPieceStaysInsideTheField() {
        SimulatedGamePiece piece = SimulatedGamePiece.builder()
                .position(new Translation3d(1.0, 1.0, 0.5))
                .velocity(new Translation3d(-9.0, -9.0, 0))
                .field(16.54, 8.07)
                .build();
        for (int i = 0; i < 3000; i++) {
            piece.step(0.002);
            assertTrue(piece.position().getX() >= 0.0, "left the field at x=" + piece.position().getX());
            assertTrue(piece.position().getY() >= 0.0, "left the field at y=" + piece.position().getY());
            assertTrue(piece.position().getX() <= 16.54);
            assertTrue(piece.position().getY() <= 8.07);
        }
    }

    @Test
    void rollingResistanceBringsItToAStop() {
        SimulatedGamePiece piece = SimulatedGamePiece.builder()
                .position(new Translation3d(4.0, 4.0, 0.12))
                .velocity(new Translation3d(2.5, 0, 0))
                .build();
        for (int i = 0; i < 5000 && !piece.isAtRest(); i++) {
            piece.step(0.002);
        }
        assertTrue(piece.isAtRest(), "a ball rolling on carpet has to stop");
        assertTrue(piece.groundPosition().getX() > 4.0, "but not before it has gone somewhere");
    }

    @Test
    void aRobotDrivingIntoAPieceShovesItAway() {
        SimulatedGamePiece piece = SimulatedGamePiece.builder()
                .position(new Translation3d(4.5, 4.0, 0.12))
                .build();

        // Robot just behind it, driving forward in +x.
        Pose2d robot = new Pose2d(4.0, 4.0, Rotation2d.kZero);
        boolean touched = piece.interactWithRobot(robot, new Translation2d(2.0, 0), 0.45, 0.45);

        assertTrue(touched, "the bumper is overlapping the ball, so this is a contact");
        assertTrue(piece.velocity().getX() > 0.5, "it should be pushed along, not merely nudged");
    }

    @Test
    void aRobotThatIsNowhereNearDoesNothing() {
        SimulatedGamePiece piece = SimulatedGamePiece.builder()
                .position(new Translation3d(10.0, 4.0, 0.12))
                .build();
        boolean touched = piece.interactWithRobot(
                new Pose2d(4.0, 4.0, Rotation2d.kZero), new Translation2d(2.0, 0), 0.45, 0.45);
        assertFalse(touched);
        assertEquals(0.0, piece.velocity().getNorm(), EPS);
    }

    @Test
    void aStationaryRobotDoesNotSuckPiecesIn() {
        SimulatedGamePiece piece = SimulatedGamePiece.builder()
                .position(new Translation3d(4.5, 4.0, 0.12))
                .build();
        piece.interactWithRobot(
                new Pose2d(4.0, 4.0, Rotation2d.kZero), Translation2d.kZero, 0.45, 0.45);
        assertTrue(piece.velocity().getX() >= -1e-9, "resting contact must never pull the piece inward");
    }

    @Test
    void aNonsenseGamePieceIsRejected() {
        assertThrows(IllegalStateException.class,
                () -> SimulatedGamePiece.builder().massKg(0).build());
        assertThrows(IllegalStateException.class,
                () -> SimulatedGamePiece.builder().radius(-1).build());
    }
}
