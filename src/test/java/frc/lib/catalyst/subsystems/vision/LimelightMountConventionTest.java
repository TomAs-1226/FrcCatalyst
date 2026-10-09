package frc.lib.catalyst.subsystems.vision;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

import static org.junit.jupiter.api.Assertions.*;

/** Exercise actual NT overrides: classic output can also be enabled on a modern camera. */
class LimelightMountConventionTest {
    private static final Transform3d MOUNT = new Transform3d(
            new Translation3d(-0.321, 0.07, 0.410),
            new Rotation3d(Math.toRadians(5), Math.toRadians(-20), Math.toRadians(180)));
    private static final double[] NWU = {-0.321, 0.07, 0.410, 5, -20, 180};
    private static final double[] CLASSIC = {-0.321, -0.07, 0.410, 5, 20, 180};

    private NetworkTable table(String suffix) {
        return NetworkTableInstance.getDefault().getTable("limelight-mount-" + suffix);
    }

    @Test
    void disconnectedCameraDoesNotReceiveAGuessedConvention() {
        var t = table("unknown");
        var src = new LimelightSource(t.getPath().substring(1), MOUNT, false);
        assertTrue(src.getEstimatedPose().isEmpty());
        assertFalse(t.getEntry("camerapose_robotspace_set").exists());
    }

    @Test
    void oldCameraReceivesRightPositiveAndPitchUp() {
        var t = table("classic");
        var src = new LimelightSource(t.getPath().substring(1), MOUNT, false);
        t.getEntry("tv").setDouble(1);
        t.getEntry("botpose_wpiblue").setDoubleArray(
                new double[] {3, 4, 0, 0, 0, 90, 25, 2, 1, 2, 1});
        assertTrue(src.getEstimatedPose().isPresent(), "legacy pose reading must still work");
        assertTrue(src.isUsingLegacyApi());
        assertArrayEquals(CLASSIC, t.getEntry("camerapose_robotspace_set").getDoubleArray(new double[0]), 1e-9);
    }

    @Test
    void modernProtocolWinsEvenWhenClassicKeysArriveBeforeResults() {
        var t = table("modern");
        var src = new LimelightSource(t.getPath().substring(1), MOUNT, false);
        t.getEntry("tv").setDouble(1);
        t.getEntry("protover").setInteger(1);
        src.getEstimatedPose();
        assertFalse(src.isUsingLegacyApi());
        assertArrayEquals(NWU, t.getEntry("camerapose_robotspace_set").getDoubleArray(new double[0]), 1e-9);
    }

    @Test
    void delayedModernAnnouncementCorrectsAnEarlyClassicSelection() {
        var t = table("late-protocol");
        var src = new LimelightSource(t.getPath().substring(1), MOUNT, false);
        t.getEntry("tv").setDouble(0);
        src.getEstimatedPose();
        assertTrue(src.isUsingLegacyApi());
        assertArrayEquals(CLASSIC, t.getEntry("camerapose_robotspace_set").getDoubleArray(new double[0]), 1e-9);
        t.getEntry("protover").setInteger(1);
        src.getEstimatedPose();
        assertFalse(src.isUsingLegacyApi());
        assertArrayEquals(NWU, t.getEntry("camerapose_robotspace_set").getDoubleArray(new double[0]), 1e-9);
        // An output pause must not reinterpret a known-modern camera's mount.
        t.getEntry("protover").setInteger(0);
        src.getEstimatedPose();
        assertFalse(src.isUsingLegacyApi());
        assertArrayEquals(NWU, t.getEntry("camerapose_robotspace_set").getDoubleArray(new double[0]), 1e-9);
    }
}
