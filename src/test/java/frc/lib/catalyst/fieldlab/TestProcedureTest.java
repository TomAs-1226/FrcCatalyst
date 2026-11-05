package frc.lib.catalyst.fieldlab;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A procedure's own validation - its id is both a log namespace and a checkpoint key, so it is
 * checked harder than an ordinary name would be - plus the time estimate and the operator check
 * that a campaign relies on.
 */
class TestProcedureTest {

    @Test
    void aBlankIdIsRefused() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class,
                () -> new TestProcedure("   ", "Title", "why", List.of()));
        assertTrue(ex.getMessage().contains("id"), "the message should explain that an id is required: " + ex.getMessage());
    }

    @Test
    void anIdContainingASlashIsRefusedBecauseItBecomesPartOfANetworkTablesKey() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class,
                () -> new TestProcedure("wheel/radius", "Title", "why", List.of()));
        assertTrue(ex.getMessage().contains("wheel/radius"),
                "the message should name the offending id so it is findable in a long campaign: " + ex.getMessage());
    }

    @Test
    void estimatedSecondsIsTheSumOfItsStepsWithABatteryGateCostingNothingAndASettleCostingItsSeconds() {
        TestProcedure procedure = TestProcedure.builder("p1", "Procedure")
                .needsBattery(12.0)                             // BatteryGate: 0.0
                .step(TestStep.Settle.of("settle", 12.5))       // 12.5
                .step(TestStep.Confirm.of("confirm"))           // 30.0
                .step(TestStep.Measure.of("measure", "k", "m")) // 60.0
                .build();

        assertEquals(102.5, procedure.estimatedSeconds(), 1e-9,
                "the procedure's estimate should be exactly the sum of its steps' estimates");
    }

    @Test
    void needsAnOperatorIsTrueOnlyWhenAMeasureOrAConfirmIsPresent() {
        TestProcedure robotOnly = TestProcedure.builder("robot-only", "Robot only")
                .needsBattery(12.0)
                .step(TestStep.Settle.of("settle", 1.0))
                .step(TestStep.Act.of("act", () -> null, 1.0))
                .build();
        assertFalse(robotOnly.needsAnOperator(),
                "a gate, a settle and an act are all things the robot does alone");

        TestProcedure withConfirm = TestProcedure.builder("with-confirm", "With confirm")
                .step(TestStep.Confirm.of("confirm"))
                .build();
        assertTrue(withConfirm.needsAnOperator(), "a Confirm step needs a person to press advance");

        TestProcedure withMeasure = TestProcedure.builder("with-measure", "With measure")
                .step(TestStep.Measure.of("measure", "k", "m"))
                .build();
        assertTrue(withMeasure.needsAnOperator(), "a Measure step needs a person with a tape measure");
    }
}
