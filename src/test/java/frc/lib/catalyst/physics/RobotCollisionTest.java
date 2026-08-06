package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.catalyst.physics.contact.CollisionField;
import frc.lib.catalyst.physics.contact.ContactMaterial;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.sim.SimulatedRobot;
import org.junit.jupiter.api.Test;

/** Pure-Java tests for robot-versus-field collision. No HAL, no NetworkTables, no robot. */
class RobotCollisionTest {

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(0.29, 0.29), new Translation2d(0.29, -0.29),
            new Translation2d(-0.29, 0.29), new Translation2d(-0.29, -0.29));

    private static RobotModel chassis() {
        return RobotModel.builder()
                .massKg(55.0).footprintMeters(0.86, 0.86)
                .centerOfMassHeightMeters(0.22).coefficientOfFriction(1.05).build();
    }

    /** Parked mid-field, well clear of everything, so a test has to drive somewhere to hit anything. */
    private static SimulatedRobot robotIn(CollisionField field) {
        return SimulatedRobot.builder()
                .robotModel(chassis()).kinematics(KINEMATICS).loopPeriod(0.02)
                .startingPose(new Pose2d(8.0, 4.0, Rotation2d.kZero))
                .collisionField(field).build();
    }

    // ------------------------------------------------------------------ geometry

    @Test
    void aRobotInOpenFieldTouchesNothing() {
        CollisionField field = CollisionField.rebuilt().build();
        assertTrue(field.contact(new Pose2d(8.0, 4.0, Rotation2d.kZero), 0.43, 0.43).isEmpty());
    }

    @Test
    void aRobotOverlappingTheWallIsDetected() {
        CollisionField field = CollisionField.rebuilt().build();
        var hit = field.contact(new Pose2d(0.2, 4.0, Rotation2d.kZero), 0.43, 0.43);
        assertTrue(hit.isPresent());
        assertEquals(1.0, hit.get().normal().getX(), 1e-9, "it should be pushed back into the field");
        assertEquals(0.23, hit.get().penetration(), 1e-6);
    }

    @Test
    void aDiagonalRobotNeedsMoreRoomThanASquareOne() {
        CollisionField field = CollisionField.rebuilt().build();
        Pose2d square = new Pose2d(0.45, 4.0, Rotation2d.kZero);
        Pose2d turned = new Pose2d(0.45, 4.0, Rotation2d.fromDegrees(45));

        assertTrue(field.contact(square, 0.43, 0.43).isEmpty(),
                "square to the wall, 0.43 of frame fits inside 0.45");
        assertTrue(field.contact(turned, 0.43, 0.43).isPresent(),
                "at 45 degrees the same robot reaches 0.61 and must be in the wall");
    }

    @Test
    void anObstacleIsDetectedAndTheNormalPointsAway() {
        CollisionField field = CollisionField.rebuilt()
                .addObstacle("hub", new Translation2d(8.0, 4.0), 0.6, 0.6, ContactMaterial.ALUMINIUM)
                .build();

        // Approaching from -x, overlapping slightly.
        var hit = field.contact(new Pose2d(7.0, 4.0, Rotation2d.kZero), 0.43, 0.43);
        assertTrue(hit.isPresent());
        assertEquals("hub", hit.get().what());
        assertTrue(hit.get().normal().getX() < 0, "the way out is back the way it came");
    }

    @Test
    void aRobotClearOfAnObstacleIsNotInContact() {
        CollisionField field = CollisionField.rebuilt()
                .addObstacle("hub", new Translation2d(8.0, 4.0), 0.6, 0.6, ContactMaterial.ALUMINIUM)
                .build();
        assertTrue(field.contact(new Pose2d(6.0, 4.0, Rotation2d.kZero), 0.43, 0.43).isEmpty());
    }

    // ------------------------------------------------------------------ response

    @Test
    void aRobotWithoutACollisionFieldStillDrivesThroughWalls() {
        // The historical behaviour, and what every simulation written before this expects.
        SimulatedRobot sim = SimulatedRobot.builder()
                .robotModel(chassis()).kinematics(KINEMATICS).loopPeriod(0.02).build();
        sim.command(new ChassisSpeeds(-4.0, 0, 0));
        sim.step(150);
        assertTrue(sim.truePose().getX() < -1.0, "no field configured means nothing to hit");
        assertEquals(0, sim.collisionCount());
    }

    @Test
    void theWallStopsARobotDrivingIntoIt() {
        SimulatedRobot sim = robotIn(CollisionField.rebuilt().build());
        sim.command(new ChassisSpeeds(-4.0, 0, 0));   // from mid-field, straight at the x- wall

        for (int i = 0; i < 400; i++) sim.step();

        assertTrue(sim.collisionCount() > 0, "it drove at a wall and should have hit it");
        assertTrue(sim.truePose().getX() >= 0.0,
                "the robot ended up outside the field at x=" + sim.truePose().getX());
    }

    @Test
    void theRobotNeverEndsUpOutsideTheField() {
        SimulatedRobot sim = robotIn(CollisionField.rebuilt().build());
        // Drive hard into a corner, diagonally, while spinning — the case that breaks naive push-out.
        sim.command(new ChassisSpeeds(-4.0, -4.0, 2.0));
        for (int i = 0; i < 400; i++) {
            sim.step();
            Pose2d at = sim.truePose();
            assertTrue(at.getX() > -0.05 && at.getX() < 16.6, "escaped at x=" + at.getX());
            assertTrue(at.getY() > -0.05 && at.getY() < 8.12, "escaped at y=" + at.getY());
        }
    }

    @Test
    void anObstacleStopsTheRobotShortOfIt() {
        CollisionField field = CollisionField.rebuilt()
                .addObstacle("pillar", new Translation2d(11.0, 4.0), 0.5, 0.5, ContactMaterial.ALUMINIUM)
                .build();
        SimulatedRobot sim = robotIn(field);
        sim.command(new ChassisSpeeds(3.0, 0, 0));
        for (int i = 0; i < 300; i++) sim.step();

        assertTrue(sim.collisionCount() > 0);
        assertTrue(sim.truePose().getX() < 11.0,
                "it should be held on the near side of the pillar, got x=" + sim.truePose().getX());
        assertTrue(sim.lastContact().isPresent());
        assertEquals("pillar", sim.lastContact().get().what());
    }

    @Test
    void aSofterWallTakesLessSpeedOffThanAHarderOne() {
        double offHard = reboundSpeed(ContactMaterial.POLYCARBONATE);
        double offDead = reboundSpeed(ContactMaterial.CARPET.withRestitution(0.0));
        assertTrue(offHard > offDead,
                "wall material has to change the outcome: hard " + offHard + " vs dead " + offDead);
    }

    private static double reboundSpeed(ContactMaterial wall) {
        SimulatedRobot sim = robotIn(CollisionField.rebuilt().wallMaterial(wall).build());
        sim.command(new ChassisSpeeds(-4.0, 0, 0));
        while (sim.collisionCount() == 0 && sim.timestamp() < 5.0) sim.step();
        return sim.trueVelocityVector().getX();   // positive means it came back off the wall
    }

    @Test
    void anImpactShowsUpAsAccelerationTheWheelsCannotExplain() {
        // This is the point of simulating collisions at all: the disturbance has to be visible.
        SimulatedRobot sim = robotIn(CollisionField.rebuilt().build());
        sim.command(new ChassisSpeeds(-4.0, 0, 0));
        while (sim.collisionCount() == 0 && sim.timestamp() < 5.0) sim.step();

        Translation2d body = sim.trueAcceleration();
        assertTrue(body.getNorm() > 20.0,
                "a 4 m/s stop inside one loop is a large acceleration, got " + body.getNorm());
        assertTrue(sim.wheelVelocity().getX() < -1.0,
                "the wheels should still be claiming forward motion after the body stopped");
    }

    @Test
    void aRobotThatIsNotDrivingAtAnythingNeverCollides() {
        SimulatedRobot sim = robotIn(CollisionField.rebuilt().build());
        sim.command(new ChassisSpeeds(0.5, 0, 0));
        sim.step(100);
        assertEquals(0, sim.collisionCount());
        assertFalse(sim.isTouchingField());
    }

    @Test
    void resetClearsTheCollisionRecord() {
        SimulatedRobot sim = robotIn(CollisionField.rebuilt().build());
        sim.command(new ChassisSpeeds(-4.0, 0, 0));
        sim.step(200);
        assertTrue(sim.collisionCount() > 0);
        sim.reset();
        assertEquals(0, sim.collisionCount());
        assertTrue(sim.lastContact().isEmpty());
    }
}
