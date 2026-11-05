package frc.lib.catalyst.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.catalyst.util.AutoStartPose.Source;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class AutoStartPoseTest {

    private static final Pose2d MARK = new Pose2d(3.5, 4.0, Rotation2d.fromDegrees(180));
    private static final Optional<Pose2d> START = Optional.of(MARK);

    private static Pose2d placed(double x, double headingDeg) {
        return new Pose2d(x, 4.1, Rotation2d.fromDegrees(headingDeg));
    }

    @Test
    void aRecentFixWithTheRoutinesHeadingStandsEvenOffTheMark() {
        var d = AutoStartPose.decide(placed(3.7, 175), START, true, true, true);
        assertEquals(Source.CURRENT, d.source(), "20 cm off the mark, and the cameras know it");
        assertTrue(d.pose().isEmpty());
    }

    @Test
    void aRecentFixWithAQuarterTurnHeadingResetsToTheMark() {
        // Switched on sideways: the fix was made with a heading 90 degrees out.
        var d = AutoStartPose.decide(placed(3.7, 90), START, true, true, true);
        assertEquals(Source.START_POSE, d.source());
        assertEquals(MARK, d.pose().orElseThrow());
        assertTrue(d.detail().contains("heading"));
    }

    @Test
    void theHeadingToleranceWrapsAcross180() {
        assertEquals(Source.CURRENT, AutoStartPose.decide(placed(3.5, -170), START, true, true, true).source(),
                "-170 is 10 degrees from 180");
        assertEquals(Source.START_POSE,
                AutoStartPose.decide(placed(3.5, -170), START, true, true, true, 5).source());
    }

    @Test
    void noRecentFixKeepsATrustedPoseAndResetsAnUntrustedOne() {
        assertEquals(Source.CURRENT, AutoStartPose.decide(placed(3.7, 90), START, false, true, true).source());
        var d = AutoStartPose.decide(placed(0, 0), START, false, false, true);
        assertEquals(Source.START_POSE, d.source());
        assertEquals("START POSE", d.detail());
    }

    @Test
    void aResetWithNoMarkOrWithResetsOffChangesNothingAndSaysWhy() {
        var none = AutoStartPose.decide(placed(0, 0), Optional.empty(), false, false, true);
        assertEquals(Source.NOTHING, none.source());
        assertTrue(none.detail().contains("no start"));
        var off = AutoStartPose.decide(placed(0, 0), START, false, false, false);
        assertEquals(Source.NOTHING, off.source());
        assertTrue(off.detail().contains("switched off"));
    }

    @Test
    void aRoutineWithNoStartLetsAnyRecentFixStand() {
        assertEquals(Source.CURRENT, AutoStartPose.decide(placed(1, 33), Optional.empty(), true, false, true).source());
    }
}
