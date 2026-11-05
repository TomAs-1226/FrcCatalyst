package frc.lib.catalyst.fieldlab;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A campaign's own guard - duplicate procedure ids would make a resume land in the wrong
 * procedure, so they are refused at build time - plus the time estimate and the operator count a
 * dashboard shows before a session starts.
 */
class TestCampaignTest {

    private static TestProcedure robotOnly(String id) {
        return TestProcedure.builder(id, id)
                .step(TestStep.Act.of("act", () -> null, 1.0))
                .build();
    }

    private static TestProcedure withConfirm(String id) {
        return TestProcedure.builder(id, id)
                .step(TestStep.Confirm.of("confirm"))
                .build();
    }

    @Test
    void twoProceduresSharingAnIdAreRefusedBecauseIdsAreTheCheckpointKeys() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class,
                () -> TestCampaign.builder("Campaign")
                        .add(robotOnly("wheel-radius"))
                        .add(withConfirm("wheel-radius"))
                        .build());
        assertTrue(ex.getMessage().contains("wheel-radius"),
                "the exception should name the duplicated id so it can be found in a long campaign: " + ex.getMessage());
    }

    @Test
    void estimatedSecondsIsTheSumOfEveryProcedureAndEstimateFormatsASubHourCampaignAsMinutesOnly() {
        TestCampaign campaign = TestCampaign.builder("Campaign")
                .add(TestProcedure.builder("p1", "P1").step(TestStep.Settle.of("settle", 300.0)).build())
                .add(TestProcedure.builder("p2", "P2").step(TestStep.Settle.of("settle", 420.0)).build())
                .build();

        assertEquals(720.0, campaign.estimatedSeconds(), 1e-9);
        assertEquals("12 min", campaign.estimate());
    }

    @Test
    void estimateFormatsAnHourOrMoreAsHoursAndMinutes() {
        TestCampaign campaign = TestCampaign.builder("Campaign")
                .add(TestProcedure.builder("p1", "P1").step(TestStep.Settle.of("settle", 3600.0 + 12 * 60)).build())
                .build();

        assertEquals("1 h 12 min", campaign.estimate());
    }

    @Test
    void operatorProceduresCountsOnlyProceduresThatNeedAPerson() {
        TestCampaign campaign = TestCampaign.builder("Campaign")
                .add(robotOnly("act-only"))
                .add(TestProcedure.builder("gate-and-settle", "Gate and settle")
                        .needsBattery(12.0)
                        .step(TestStep.Settle.of("settle", 1.0))
                        .build())
                .add(withConfirm("with-confirm"))
                .add(TestProcedure.builder("with-measure", "With measure")
                        .step(TestStep.Measure.of("measure", "k", "m"))
                        .build())
                .build();

        assertEquals(2, campaign.operatorProcedures(),
                "only the confirm and the measure procedures should count as needing an operator");
    }
}
