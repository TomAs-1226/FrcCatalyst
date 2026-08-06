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
import frc.lib.catalyst.physics.contact.FieldHeightmap;
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

    // ------------------------------------------------------ settling against CAD-derived geometry
    //
    // The heightmap path is the one the console actually drives, and it is where the robot was seen
    // buzzing against walls and sitting visibly inside them. These four tests are the shape of that
    // complaint: does it stop, does it stay stopped, does it stay outside, and can it still slide.

    private static final double HALF_ROBOT = 0.43;
    /** Inner face of the +x wall of {@link #walledBox()}. */
    private static final double WALL_X = 3.85;

    /** A 4 x 10 m box on 5 cm cells: flat carpet inside a 50 cm perimeter wall three cells thick. */
    private static FieldHeightmap walledBox() {
        int cols = 80, rows = 200;
        StringBuilder h = new StringBuilder(cols * rows * 4);
        StringBuilder c = new StringBuilder(cols * rows * 5);
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                if (h.length() > 0) { h.append(','); c.append(','); }
                h.append(x < 3 || y < 3 || x >= cols - 3 || y >= rows - 3 ? 500 : 0);
                c.append(9999);
            }
        }
        return FieldHeightmap.parse("{"
                + "\"cellMeters\":0.05,\"cols\":" + cols + ",\"rows\":" + rows + ","
                + "\"lengthMeters\":4.0,\"widthMeters\":10.0,"
                + "\"heightsMillimetres\":[" + h + "],"
                + "\"clearanceMillimetres\":[" + c + "]}");
    }

    private static SimulatedRobot robotOn(FieldHeightmap map, double x, double y) {
        return SimulatedRobot.builder()
                .robotModel(chassis()).kinematics(KINEMATICS).loopPeriod(0.02)
                .startingPose(new Pose2d(x, y, Rotation2d.kZero))
                .heightmap(map, 0.85, 0.0).build();
    }

    /** How far the front bumper reaches past the wall face. Negative means clear of it. */
    private static double depthIntoWall(SimulatedRobot sim) {
        return (sim.truePose().getX() + HALF_ROBOT) - WALL_X;
    }

    @Test
    void aRobotHeldAgainstAWallComesToRest() {
        SimulatedRobot sim = robotOn(walledBox(), 2.0, 5.0);
        sim.command(new ChassisSpeeds(4.0, 0, 0));   // and never let go of the stick
        sim.step(400);

        // Everything below is measured after the impact has had eight seconds to settle. The old
        // solver never got here: the normal swung 90 degrees between passes, so the robot buzzed in a
        // 34 mm band with the velocity changing sign 58 times in 136 steps.
        Pose2d settled = sim.truePose();
        double fastest = 0;
        double furthest = 0;
        int reversals = 0;
        double previous = sim.trueVelocityVector().getX();
        for (int i = 0; i < 200; i++) {
            sim.step();
            double vx = sim.trueVelocityVector().getX();
            if (vx * previous < 0) reversals++;
            previous = vx;
            fastest = Math.max(fastest, sim.trueSpeed());
            furthest = Math.max(furthest,
                    sim.truePose().getTranslation().getDistance(settled.getTranslation()));
        }
        System.out.printf(
                "held against a wall: |v| <= %.3e m/s, pose band %.3e m, %d sign changes in 200 steps%n",
                fastest, furthest, reversals);

        assertTrue(fastest < 1e-6, "it should be at rest, still moving at " + fastest + " m/s");
        assertTrue(furthest < 1e-6, "and staying put, wandering " + furthest + " m");
        assertEquals(0, reversals, "a robot leaning on a wall does not change direction");
    }

    @Test
    void theRobotIsPushedOutInsideTheStepThatDroveItIn() {
        // Not "eventually". The console draws truePose() once a step, so anything still overlapping
        // when step() returns is overlap somebody watches. It used to take seven steps — 140 ms, eight
        // rendered frames — to climb out of a 20 cm overlap, one fixed push at a time.
        SimulatedRobot sim = robotOn(walledBox(), 2.0, 5.0);
        sim.command(new ChassisSpeeds(4.0, 0, 0));

        double deepest = 0;
        for (int i = 0; i < 400; i++) {
            sim.step();
            deepest = Math.max(deepest, depthIntoWall(sim));
        }
        assertTrue(sim.collisionCount() > 0, "it drove at a wall and should have hit it");
        assertTrue(deepest < 0.003,
                "the robot was visibly " + deepest * 1000 + " mm inside the wall at the end of a step");
    }

    @Test
    void aRobotSlidingAlongAWallKeepsSliding() {
        // Friction should scrub some speed off the slide, not weld the robot to the wall. The failure
        // this guards against is the normal picking up a component along the wall, which turns every
        // contact into a brake and, at 45 degrees out, into a shove in the wrong direction entirely.
        SimulatedRobot sim = robotOn(walledBox(), 2.0, 1.0);
        sim.command(new ChassisSpeeds(4.0, 4.0, 0));   // diagonally into the +x wall
        sim.step(60);                                  // reach it and settle into the slide

        double startY = sim.truePose().getY();
        double closestApproach = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 50; i++) {
            sim.step();
            closestApproach = Math.min(closestApproach, -depthIntoWall(sim));
        }
        double slid = sim.truePose().getY() - startY;
        double alongWall = sim.trueVelocityVector().getY();
        System.out.printf("sliding along a wall: %.3f m in 1.0 s, %.3f m/s, %.1f mm of overlap%n",
                slid, alongWall, -closestApproach * 1000);

        assertTrue(closestApproach < 0.003,
                "it should still be against the wall, standing " + closestApproach + " m off it");
        assertTrue(alongWall > 1.0, "it should still be sliding, at " + alongWall + " m/s");
        assertTrue(slid > 1.0, "and covering ground: " + slid + " m in one second");
    }

    @Test
    void aRobotStrandedOutsideTheFieldIsNotFlungFurtherOut() {
        // Every sample off the grid used to pick the rear corner as the worst thing under the robot,
        // so the escape pointed forward and outward and the robot walked away from the field at
        // 1.77 m/s of pure position, with no impulse and no limit.
        SimulatedRobot sim = robotOn(walledBox(), 6.0, 5.0);
        sim.stop();
        sim.step(50);

        assertEquals(6.0, sim.truePose().getX(), 1e-9, "at rest outside the field it should stay put");
        assertTrue(sim.lastContact().isPresent());
        assertTrue(sim.lastContact().get().normal().getX() < 0,
                "and the way home is back toward the field, got " + sim.lastContact().get().normal());
    }
}
