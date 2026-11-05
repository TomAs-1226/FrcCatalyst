package frc.lib.catalyst.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.catalyst.subsystems.vision.TargetTagPriority.Filter;
import frc.lib.catalyst.subsystems.vision.TargetTagPriority.Mode;

import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * The scenarios team 5805's {@code HubTagModeAndSteadinessTest} drove through NetworkTables with real
 * sleeps, here on the pure core with a clock the test owns, plus one end-to-end Limelight check.
 */
class TargetTagPriorityTest {

    private static final double LOOP = 0.02;

    /** Run loops from {@code from} to {@code to}, s, with the same sightings each loop; returns the last time. */
    private static double run(TargetTagPriority p, double from, double to, boolean wanted, boolean... sees) {
        double t = from;
        for (; t <= to + 1e-9; t += LOOP) p.update(t, wanted, sees);
        return t - LOOP;
    }

    @Test
    void theFirstListedCameraSeeingTheTargetLeadsAndTheOthersPause() {
        TargetTagPriority p = TargetTagPriority.allMayLead(3);
        run(p, 0, 0.06, true, true, false, false);
        assertEquals(Mode.WAITING, p.mode(), "debounced: not before 0.1 s in view");
        run(p, 0.08, 0.2, true, true, false, true);
        assertEquals(Mode.TARGET_TAGS, p.mode());
        assertEquals(0, p.leader(), "the shooter camera, first in the list, leads");
        assertEquals(Filter.TARGET, p.filterFor(0));
        assertEquals(Filter.NONE, p.filterFor(1));
        assertEquals(Filter.NONE, p.filterFor(2), "a second camera on the target still pauses");
        p.update(0.22, false, new boolean[] {true, false, true});
        assertEquals(Mode.ALL_TAGS, p.mode());
        for (int c = 0; c < 3; c++) assertEquals(Filter.ALL, p.filterFor(c));
    }

    @Test
    void aSideCameraLeadsWhenItAloneSeesTheTarget() {
        TargetTagPriority p = TargetTagPriority.allMayLead(3);
        run(p, 0, 0.2, true, false, false, true);
        assertEquals(2, p.leader());
        assertEquals(Filter.NONE, p.filterFor(0));
        assertEquals(Filter.TARGET, p.filterFor(2));
    }

    @Test
    void anotherCameraSeeingTheTargetNeverTakesOverFromAHoldingLeader() {
        TargetTagPriority p = TargetTagPriority.allMayLead(2);
        run(p, 0, 0.2, true, false, true);
        assertEquals(1, p.leader());
        run(p, 0.22, 0.6, true, true, true);
        assertEquals(1, p.leader(), "camera 0 is first in the list, but the leader still sees the target");
    }

    @Test
    void aLostLeaderIsHeldThenEveryCameraSearchesOnTargetTagsThenAllTagsReturn() {
        TargetTagPriority p = TargetTagPriority.allMayLead(3);
        double t = run(p, 0, 0.2, true, true, false, false);
        assertEquals(0, p.leader());
        t = run(p, t + LOOP, t + 0.28, true, false, false, false);
        assertEquals(Mode.TARGET_TAGS, p.mode(), "a dropped frame or two is held");
        t = run(p, t + LOOP, t + 0.1, true, false, false, false);
        assertEquals(Mode.TARGET_SEARCH, p.mode(), "past the 0.3 s hold");
        for (int c = 0; c < 3; c++) {
            assertEquals(Filter.TARGET, p.filterFor(c), "a handover never mixes other field tags in");
        }
        assertEquals(-1, p.leader());
        run(p, t + LOOP, t + 1.0, true, false, false, false);
        assertEquals(Mode.WAITING, p.mode(), "after the 1.0 s search, every tag again");
        assertEquals(Filter.ALL, p.filterFor(0));
    }

    @Test
    void aSearchingCameraThatFindsTheTargetTakesTheLead() {
        TargetTagPriority p = TargetTagPriority.allMayLead(3);
        double t = run(p, 0, 0.2, true, true, false, false);
        t = run(p, t + LOOP, t + 0.4, true, false, false, false);
        assertEquals(Mode.TARGET_SEARCH, p.mode());
        run(p, t + LOOP, t + 0.14, true, false, false, true);
        assertEquals(Mode.TARGET_TAGS, p.mode());
        assertEquals(2, p.leader());
        assertEquals(Filter.NONE, p.filterFor(0));
        assertEquals(Filter.NONE, p.filterFor(1));
    }

    @Test
    void aCameraThatMayNotLeadKeepsEveryTagAndIsNeverALeader() {
        TargetTagPriority p = new TargetTagPriority(new boolean[] {false, true});
        run(p, 0, 0.2, true, true, false);
        assertEquals(Mode.WAITING, p.mode(), "its sighting does not count");
        run(p, 0.22, 0.4, true, true, true);
        assertEquals(1, p.leader());
        assertEquals(Filter.ALL, p.filterFor(0));
    }

    @Test
    void timingIsConfigurableAndChecked() {
        TargetTagPriority p = new TargetTagPriority(new boolean[] {true},
                new TargetTagPriority.Config().debounceSeconds(0).holdSeconds(0).searchSeconds(0));
        p.update(0, true, new boolean[] {true});
        assertEquals(Mode.TARGET_TAGS, p.mode(), "no debounce");
        p.update(0.02, true, new boolean[] {false});
        assertEquals(Mode.WAITING, p.mode(), "no hold and no search");
        assertThrows(IllegalArgumentException.class, () -> new TargetTagPriority.Config().holdSeconds(-1));
        assertThrows(IllegalArgumentException.class, () -> p.update(0.04, true, new boolean[2]));
    }

    // ---- Limelight wiring -----------------------------------------------------------------------

    @Test
    void rawFiducialsAreReadSevenNumbersATagWhateverTheOrder() {
        Set<Integer> blue = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
        double[] raw = {14, .1, .2, .3, 2.5, 2.6, .01, 25, .1, .2, .3, 2.5, 2.6, .01};
        assertTrue(LimelightTagPriority.seesAny(raw, blue));
        assertFalse(LimelightTagPriority.seesAny(raw, Set.of(2, 3)));
        assertFalse(LimelightTagPriority.seesAny(new double[] {25, .1}, blue), "a torn array");
        double[] swapped = {21, 0, 0, 1, 3, 3, 0, 20, 0, 0, 1, 3, 3, 0,
            21, 0, 0, 1, 3, 3, 0, Double.NaN, 0, 0, 1, 3, 3, 0, 25, 0};
        assertArrayEquals(new double[] {20, 21}, LimelightTagPriority.visibleTagIds(swapped, blue));
    }

    @Test
    void frameFreshnessIsInNanoseconds() {
        long now = 2_000_000_000L;
        assertTrue(LimelightTagPriority.freshFrame(now, now));
        assertTrue(LimelightTagPriority.freshFrame(now, now - 250_000_000L));
        assertFalse(LimelightTagPriority.freshFrame(now, now - 250_000_001L));
        assertFalse(LimelightTagPriority.freshFrame(now, now + 1));
        assertFalse(LimelightTagPriority.freshFrame(now, 0));
    }

    @Test
    void theLimelightFiltersFollowTheModeAndAStaleArrayIsNotASighting() throws Exception {
        assertTrue(HAL.initialize());
        var nt = NetworkTableInstance.getDefault();
        var names = List.of("limelight-ttp-a", "limelight-ttp-b");
        var priority = new LimelightTagPriority(names, Set.copyOf(names), new int[] {3, 1, 2, 4, 5, 6});
        Set<Integer> target = Set.of(4, 5);
        for (int i = 0; i < 2; i++) {
            nt.getTable(names.get(0)).getEntry("rawfiducials").setDoubleArray(new double[] {1, i * .01, 0, 1, 3, 3, 0});
            nt.getTable(names.get(1)).getEntry("rawfiducials").setDoubleArray(new double[] {5, i * .01, 0, 1, 3, 3, 0});
            nt.flush();
            Thread.sleep(5);
            priority.update(true, target);
            if (i == 0) Thread.sleep(120);
        }
        assertEquals(Mode.TARGET_TAGS, priority.mode());
        assertEquals(1, priority.leader());
        assertArrayEquals(new double[] {4, 5}, filter(nt, names.get(1)));
        assertArrayEquals(LimelightTagPriority.NO_TAGS, filter(nt, names.get(0)));
        priority.update(false, target);
        assertArrayEquals(new double[] {1, 2, 3, 4, 5, 6}, filter(nt, names.get(0)), "every tag, sorted");
        // The last array stays on the server after a camera drops off; once it is old it sees nothing.
        Thread.sleep(300);
        priority.update(true, target);
        assertEquals(Mode.WAITING, priority.mode());
    }

    private static double[] filter(NetworkTableInstance nt, String camera) {
        return nt.getTable(camera).getEntry("fiducial_id_filters_set").getDoubleArray(new double[0]);
    }
}
