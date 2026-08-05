package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.lib.catalyst.physics.model.ArticulatedRobotModel;
import frc.lib.catalyst.physics.model.MechanismModel;
import frc.lib.catalyst.physics.model.RobotModel;

/**
 * Pure-Java tests for {@link MechanismModel} and {@link ArticulatedRobotModel}. Mechanism positions
 * come from mutable arrays the test drives by hand, so there is no HAL and no robot.
 */
class ArticulatedModelTest {

    /** 50 kg of chassis with its centre of mass 0.2 m up. */
    private static RobotModel chassis() {
        return RobotModel.builder()
                .massKg(50.0)
                .footprintMeters(0.7, 0.7)
                .centerOfMassHeightMeters(0.2)
                .build();
    }

    /** A 10 kg carriage that rides straight up from 0.15 m, driven by {@code height[0]}. */
    private static MechanismModel elevator(double[] height) {
        return MechanismModel.linear("Elevator", 10.0)
                .mountedAt(new Translation3d(0.0, 0.0, 0.15))
                .along(new Translation3d(0.0, 0.0, 1.0))
                .position(() -> height[0])
                .build();
    }

    @Test
    void aChassisWithNoLinksKeepsItsStaticCenterOfMass() {
        ArticulatedRobotModel robot = ArticulatedRobotModel.ofChassisOnly(chassis());

        assertEquals(0.2, robot.centerOfMassHeightMeters(), 1e-9);
        assertEquals(50.0, robot.totalMassKg(), 1e-9);
        assertEquals(0.0, robot.centerOfMassShiftMeters(), 1e-9);
    }

    @Test
    void raisingAnElevatorRaisesTheWholeRobotsCenterOfMass() {
        double[] height = {0.0};
        ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
                .chassis(chassis())
                .add(elevator(height))
                .build();

        // Stowed: 50 kg at 0.20 m and 10 kg at 0.15 m.
        assertEquals((50 * 0.20 + 10 * 0.15) / 60.0, robot.centerOfMassHeightMeters(), 1e-9);

        height[0] = 1.2;
        // Raised: the carriage is now at 0.15 + 1.2 = 1.35 m.
        assertEquals((50 * 0.20 + 10 * 1.35) / 60.0, robot.centerOfMassHeightMeters(), 1e-9);
        assertEquals(60.0, robot.totalMassKg(), 1e-9);
    }

    @Test
    void theCenterOfMassMovesFarEnoughToMatter() {
        double[] height = {0.0};
        ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
                .chassis(chassis())
                .add(elevator(height))
                .build();

        double stowed = robot.centerOfMassHeightMeters();
        height[0] = 1.2;
        double raised = robot.centerOfMassHeightMeters();

        // 10 kg moving 1.2 m on a 60 kg robot is 10/60 * 1.2 = 0.20 m of CoM travel.
        assertEquals(0.20, raised - stowed, 1e-9);
        assertTrue(robot.centerOfMassShiftMeters() > 0.15);
    }

    @Test
    void aChildLinkRidesOnItsParent() {
        double[] height = {0.5};
        double[] angle = {0.0};

        MechanismModel arm = MechanismModel.rotational("Arm", 5.0)
                .childOf("Elevator")
                .mountedAt(new Translation3d(0.0, 0.0, 0.05))
                .comAt(new Translation3d(0.30, 0.0, 0.0))
                .angle(() -> angle[0])
                .build();

        ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
                .chassis(chassis())
                .add(elevator(height))
                .add(arm)
                .build();

        // Arm pivot = chassis origin + elevator mount (0.15) + extension (0.5) + arm mount (0.05).
        Pose3d pivot = robot.poseOf("Arm").orElseThrow();
        assertEquals(0.70, pivot.getZ(), 1e-9);
        assertEquals(0.0, pivot.getX(), 1e-9);

        // Raise the elevator and the arm goes with it.
        height[0] = 1.0;
        assertEquals(1.20, robot.poseOf("Arm").orElseThrow().getZ(), 1e-9);
    }

    @Test
    void rotatingAnArmSwingsItsCenterOfMassThroughTheCorrectArc() {
        double[] height = {0.0};
        double[] angle = {0.0};
        MechanismModel arm = MechanismModel.rotational("Arm", 5.0)
                .childOf("Elevator")
                .comAt(new Translation3d(0.40, 0.0, 0.0))   // 40 cm out along the arm
                .angle(() -> angle[0])
                .build();

        ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
                .chassis(chassis()).add(elevator(height)).add(arm).build();

        // Horizontal: the arm's mass is 0.40 m forward of the pivot, at the pivot's height.
        double pivotHeight = 0.15;
        double flatCom = robot.centerOfMass().getZ();
        assertEquals((50 * 0.20 + 10 * 0.15 + 5 * pivotHeight) / 65.0, flatCom, 1e-9);

        // Straight up. Right-handed rotation about +Y takes +X toward -Z, so raising the far end is
        // a NEGATIVE angle - the sign trap the builder javadoc warns about.
        angle[0] = -Math.PI / 2.0;
        double raisedCom = robot.centerOfMass().getZ();
        assertEquals((50 * 0.20 + 10 * 0.15 + 5 * (pivotHeight + 0.40)) / 65.0, raisedCom, 1e-9);
        assertTrue(raisedCom > flatCom);
    }

    @Test
    void aFieldPoseComposesTheChassisPoseWithTheChain() {
        double[] height = {0.8};
        ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
                .chassis(chassis()).add(elevator(height)).build();

        Pose2d chassisPose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90));
        Pose3d carriage = robot.fieldPoseOf("Elevator", chassisPose).orElseThrow();

        // The carriage is straight above the chassis origin, so rotating the robot moves it nowhere
        // horizontally - it just inherits the chassis position and the extension height.
        assertEquals(3.0, carriage.getX(), 1e-9);
        assertEquals(4.0, carriage.getY(), 1e-9);
        assertEquals(0.95, carriage.getZ(), 1e-9);
    }

    @Test
    void aForwardOffsetLinkRotatesWithTheChassis() {
        MechanismModel gripper = MechanismModel.fixed("Gripper", 2.0, new Translation3d(0.5, 0.0, 0.4));
        ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
                .chassis(chassis()).add(gripper).build();

        // Facing +Y, a link 0.5 m "forward" of the robot ends up 0.5 m along the field's +Y.
        Pose3d field = robot.fieldPoseOf("Gripper",
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90))).orElseThrow();
        assertEquals(1.0, field.getX(), 1e-9);
        assertEquals(1.5, field.getY(), 1e-9);
    }

    @Test
    void aFixedLinkContributesMassWithoutMoving() {
        ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
                .chassis(chassis())
                .add(MechanismModel.fixed("Battery", 6.0, new Translation3d(-0.2, 0.0, 0.10)))
                .build();

        assertEquals(56.0, robot.totalMassKg(), 1e-9);
        assertEquals((50 * 0.20 + 6 * 0.10) / 56.0, robot.centerOfMassHeightMeters(), 1e-9);
        assertEquals((50 * 0.0 + 6 * -0.2) / 56.0, robot.centerOfMass().getX(), 1e-9);
    }

    @Test
    void aLinkNamingAnUnknownParentFallsBackToTheChassisRatherThanVanishing() {
        double[] angle = {0.0};
        MechanismModel orphan = MechanismModel.rotational("Wrist", 3.0)
                .childOf("NoSuchLink")
                .mountedAt(new Translation3d(0.0, 0.0, 0.6))
                .angle(() -> angle[0])
                .build();

        ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
                .chassis(chassis()).add(orphan).build();

        // Its mass still counts - a typo must not silently remove 3 kg from the model.
        assertEquals(53.0, robot.totalMassKg(), 1e-9);
        assertEquals(0.6, robot.poseOf("Wrist").orElseThrow().getZ(), 1e-9);
    }

    @Test
    void linkOrderDoesNotChangeTheAnswer() {
        double[] height = {0.9};
        double[] angle = {0.3};
        MechanismModel arm = MechanismModel.rotational("Arm", 5.0)
                .childOf("Elevator").comAt(new Translation3d(0.3, 0, 0)).angle(() -> angle[0]).build();

        // Child added before its parent - the topological pass has to sort this out.
        ArticulatedRobotModel childFirst = ArticulatedRobotModel.builder()
                .chassis(chassis()).add(arm).add(elevator(height)).build();
        ArticulatedRobotModel parentFirst = ArticulatedRobotModel.builder()
                .chassis(chassis()).add(elevator(height)).add(arm).build();

        assertEquals(parentFirst.centerOfMass().getZ(), childFirst.centerOfMass().getZ(), 1e-12);
        assertEquals(parentFirst.poseOf("Arm").orElseThrow().getZ(),
                childFirst.poseOf("Arm").orElseThrow().getZ(), 1e-12);
    }

    @Test
    void badModelsAreRejected() {
        assertThrows(IllegalStateException.class, () -> ArticulatedRobotModel.builder().build());

        double[] h = {0.0};
        assertThrows(IllegalStateException.class, () -> ArticulatedRobotModel.builder()
                .chassis(chassis()).add(elevator(h)).add(elevator(h)).build());

        assertThrows(IllegalArgumentException.class,
                () -> MechanismModel.fixed("Bad", 0.0, Translation3d.kZero));
        assertThrows(IllegalStateException.class,
                () -> MechanismModel.linear("NoPosition", 1.0).build());
        assertThrows(IllegalStateException.class,
                () -> MechanismModel.rotational("NoAngle", 1.0).build());
        assertThrows(IllegalStateException.class, () -> MechanismModel.linear("ZeroAxis", 1.0)
                .along(Translation3d.kZero).position(() -> 0).build());
    }

    @Test
    void aParentCycleIsRejectedRatherThanLoopingForever() {
        MechanismModel a = MechanismModel.fixed("A", 1.0, Translation3d.kZero);
        MechanismModel selfParented = MechanismModel.linear("B", 1.0)
                .childOf("B").position(() -> 0.0).build();

        assertThrows(IllegalStateException.class, () -> ArticulatedRobotModel.builder()
                .chassis(chassis()).add(a).add(selfParented).build());
    }
}
