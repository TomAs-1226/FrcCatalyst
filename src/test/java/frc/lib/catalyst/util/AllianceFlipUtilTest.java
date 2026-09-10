package frc.lib.catalyst.util;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class AllianceFlipUtilTest {

    private static final double L = 16.54, W = 8.07;   // REBUILT 2026

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

    /**
     * The DEFAULT field, unconfigured, must be the current season's.
     *
     * <p>Every other test here calls {@code configure(...)} first, so none of them ever looked at
     * the default - and the default width sat at 8.21 m, the REEFSCAPE number, a season after the
     * field changed to 8.07 m. A team that never called configure got every red-alliance Y flipped
     * about an axis 7 cm off centre: 14 cm of error, on one alliance only, in a value that looks
     * entirely reasonable.
     *
     * <p>Pinned against WPILib's own 2026-rebuilt layout (16.541 x 8.069) rather than against the
     * constant this class holds, so the two cannot drift together and still agree.
     */
    @Test
    void theUnconfiguredDefaultIsTheCurrentField() {
        AllianceFlipUtil.configure(CatalystMath.FIELD_LENGTH, CatalystMath.FIELD_WIDTH,
                AllianceFlipUtil.Symmetry.ROTATIONAL);

        assertEquals(16.541, CatalystMath.FIELD_LENGTH, 0.01,
                "field length must match the REBUILT 2026 layout");
        assertEquals(8.069, CatalystMath.FIELD_WIDTH, 0.01,
                "field width must match the REBUILT 2026 layout - 8.21 is REEFSCAPE");

        // A point on the centre line must flip to itself. This is the property the wrong width
        // actually broke, and it holds for no other value.
        Translation2d centre = new Translation2d(4.0, CatalystMath.FIELD_WIDTH / 2);
        assertEquals(centre.getY(), AllianceFlipUtil.flip(centre).getY(), 1e-9,
                "the field's centre line must be its own mirror");
    }
}
