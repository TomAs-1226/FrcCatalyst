package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class AllianceFlipUtilTest {

    private static final double L = 16.54, W = 8.21;

    @Test
    void rotationalFlipMirrorsBothAxesAndAddsHalfTurn() {
        AllianceFlipUtil.configure(L, W, AllianceFlipUtil.Symmetry.ROTATIONAL);
        Translation2d t = AllianceFlipUtil.flip(new Translation2d(2.0, 3.0));
        assertEquals(L - 2.0, t.getX(), 1e-9);
        assertEquals(W - 3.0, t.getY(), 1e-9);

        Rotation2d r = AllianceFlipUtil.flip(Rotation2d.fromDegrees(30));
        assertEquals(-150.0, r.getDegrees(), 1e-9);   // 30 + 180 wrapped
    }

    @Test
    void mirroredFlipMirrorsXOnly() {
        AllianceFlipUtil.configure(L, W, AllianceFlipUtil.Symmetry.MIRRORED);
        Translation2d t = AllianceFlipUtil.flip(new Translation2d(2.0, 3.0));
        assertEquals(L - 2.0, t.getX(), 1e-9);
        assertEquals(3.0, t.getY(), 1e-9);            // Y unchanged

        Rotation2d r = AllianceFlipUtil.flip(Rotation2d.fromDegrees(30));
        assertEquals(150.0, r.getDegrees(), 1e-9);    // 180 - 30
    }

    @Test
    void flippingTwiceIsIdentity() {
        for (AllianceFlipUtil.Symmetry s : AllianceFlipUtil.Symmetry.values()) {
            AllianceFlipUtil.configure(L, W, s);
            Pose2d p = new Pose2d(4.2, 1.7, Rotation2d.fromDegrees(57));
            Pose2d twice = AllianceFlipUtil.flip(AllianceFlipUtil.flip(p));
            assertEquals(p.getX(), twice.getX(), 1e-9);
            assertEquals(p.getY(), twice.getY(), 1e-9);
            assertEquals(p.getRotation().getRadians(), twice.getRotation().getRadians(), 1e-9);
        }
    }

    @Test
    void flipXUsesFieldLength() {
        AllianceFlipUtil.setFieldLength(L);
        assertEquals(L - 2.0, AllianceFlipUtil.flipX(2.0), 1e-9);
    }
}
