package frc.lib.catalyst.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.catalyst.autonomy.ChaseCore;
import frc.lib.catalyst.autonomy.TaskArbiter;

import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/** Regressions for the bugs found vetting 2.0.0 before 2.0.1, one test each. */
class Vetting201Test {

    @AfterEach
    void tidy() {
        RobotSafety.reset();
        RobotSafety.configure(null);
        AlertManager.getInstance().clearAll();
        if (SimHooks.isTimingPaused()) SimHooks.resumeTiming();
    }

    @Test
    void alliancePoseRotatesOnTheRotationalFieldInsteadOfMirroring() {
        assertTrue(HAL.initialize());
        DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
        DriverStationSim.notifyNewData();
        RobotState.refresh();
        try {
            Pose2d red = DynamicAutoBuilder.alliancePose(new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(30)));
            assertEquals(AllianceFlipUtil.fieldLength() - 2.0, red.getX(), 1e-9);
            assertEquals(AllianceFlipUtil.fieldWidth() - 1.0, red.getY(), 1e-9, "the far side, not y = 1");
            assertEquals(-150.0, red.getRotation().getDegrees(), 1e-9);
        } finally {
            DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
            DriverStationSim.notifyNewData();
            RobotState.refresh();
        }
    }

    @Test
    void aManualTripIsNotUndoneByTheNextQuietTick() {
        assertTrue(HAL.initialize());
        SimHooks.pauseTiming();
        RobotSafety.configure(RobotSafety.Config.builder().maxConcurrentErrors(1).autoReset(2.0).build());
        RobotSafety.tick(0, 0);
        SimHooks.stepTiming(30.0);           // healthy for a long time
        RobotSafety.tick(0, 0);
        RobotSafety.trip("brownout predicted");
        RobotSafety.tick(0, 0);
        assertTrue(RobotSafety.isTripped(), "the trip must survive the next loop");
        SimHooks.stepTiming(2.1);
        RobotSafety.tick(0, 0);
        assertFalse(RobotSafety.isTripped(), "and clear after autoReset seconds of health");
    }

    @Test
    void anInfoAlertCanBeCleared() {
        AlertManager a = AlertManager.getInstance();
        a.info("Elevator", "Mechanism has not been zeroed since boot");
        assertEquals(1, a.getInfos().size());
        a.clearInfo("Elevator", "Mechanism has not been zeroed since boot");
        assertTrue(a.getInfos().isEmpty());
    }

    @Test
    void poseHistoryReportsNoVelocityFromItsFirstSampleOrAfterClear() {
        PoseHistory h = new PoseHistory();
        h.addSample(new Pose2d(8, 4, Rotation2d.ZERO), 2.0);
        assertEquals(0.0, h.getVelocityVector().getNorm(), 1e-12, "one sample is not a velocity");
        h.addSample(new Pose2d(8.1, 4, Rotation2d.ZERO), 2.1);
        assertEquals(1.0, h.getVelocityVector().getNorm(), 1e-9);
        h.clear();
        h.addSample(new Pose2d(0, 0, Rotation2d.ZERO), 2.2);
        assertEquals(0.0, h.getVelocityVector().getNorm(), 1e-12, "nor is the first after a reset");
    }

    @Test
    void oneNanDoesNotPoisonAMovingAverage() {
        MovingAverage m = new MovingAverage(3);
        m.calculate(1);
        m.calculate(Double.NaN);
        m.calculate(2);
        m.calculate(3);
        m.calculate(4);
        assertEquals(3.0, m.get(), 1e-12);
    }

    @Test
    void aTargetWithNoValueIsNeverChased() {
        var good = new ChaseCore.Target<>("a", "good", 2.0, 2.0, true, "");
        var bad = new ChaseCore.Target<>("b", "nan", Double.NaN, 1.0, true, "");
        var choice = ChaseCore.choose(List.of(bad, good), Double.NaN, 0.0);
        assertEquals("good", choice.chosen().orElseThrow().name());
    }

    @Test
    void aCandidateNeedingNothingCanWin() {
        var c = new TaskArbiter.Candidate<String, String>("t", "free", 1.0, null, true);
        var d = new TaskArbiter.Candidate<String, String>("u", "arm", 0.5, Set.of("arm"), true);
        var sel = TaskArbiter.select(List.of(c, d), 0.0);
        assertEquals(2, sel.winners().size());
    }
}
