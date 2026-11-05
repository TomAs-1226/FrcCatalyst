package frc.lib.catalyst.fieldlab;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;

import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * {@link CampaignRunner}'s battery gate, run for real through the three-arg constructor's injected
 * {@code DoubleSupplier} - the one piece of the runner that is testable without a robot, because that
 * supplier exists specifically so the gate's pause-and-resume can be exercised on a desktop.
 *
 * <p>{@link FieldLabInput} wires real NetworkTables entries in its constructor, so this needs the HAL
 * initialised first, the same way {@code SystemCoreAvailabilityTest} and
 * {@code DifferentialWristTuningKeysTest} do it. The campaign here has one procedure with nothing but
 * a battery gate, so no operator step and no real robot {@code Command} are needed to see the gate
 * itself work.
 */
class CampaignRunnerBatteryGateTest {

    private Scheduler scheduler;

    @BeforeEach
    void freshSchedulerAndHal() {
        assertTrue(HAL.initialize(), "no HAL, no NetworkTables-backed FieldLabInput");
        scheduler = Scheduler.createIndependentScheduler();
    }

    /** Run the scheduler n times, which is n robot loops. */
    private void tick(int n) {
        for (int i = 0; i < n; i++) {
            scheduler.run();
        }
    }

    @Test
    void theCampaignPausesBelowTheGateAndResumesOnceTheBatteryRecovers(@TempDir Path dir) {
        double[] volts = {11.0};
        CampaignCheckpoint checkpoint = new CampaignCheckpoint(dir.resolve("fieldlab-checkpoint.json"));
        TestCampaign campaign = TestCampaign.builder("Gate Test")
                .add(TestProcedure.builder("gate-only", "Gate only").needsBattery(12.0).build())
                .build();

        CampaignRunner runner = new CampaignRunner(campaign, checkpoint, () -> volts[0]);
        scheduler.schedule(runner.command());

        tick(5);
        assertEquals(CampaignRunner.State.PAUSED_BATTERY, runner.state(),
                "11.0 V is below the 12.0 V floor, so the campaign should be held at the gate");

        volts[0] = 12.5;   // above the 12.3 V resume point - 0.3 V clear of the floor, on purpose
        tick(5);

        assertEquals(CampaignRunner.State.DONE, runner.state(),
                "the only step was the gate, so clearing it should finish the one-procedure campaign");
        assertTrue(runner.notes().stream().anyMatch(n -> n.contains("paused at")),
                "the pause should have left a note: " + runner.notes());
        assertTrue(runner.notes().stream().anyMatch(n -> n.contains("resumed at")),
                "the resume should have left a note: " + runner.notes());
    }
}
