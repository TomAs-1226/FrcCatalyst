package frc.lib.catalyst.identity;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import org.junit.jupiter.api.Test;

/**
 * The spec sheet's one job is to leave out what nobody knows.
 *
 * <p>Catalyst Console draws a dash for a key the robot never published, so an absent key is how the
 * robot says "unknown". Every placeholder that reaches the wire instead — a zero, a {@code -1}, an
 * empty string — arrives looking exactly like a measurement. These tests are the guard on that.
 *
 * <p>No HAL and no NetworkTables here: a sheet is assembled and inspected without either.
 */
class SpecSheetTest {

    @Test
    void blankTextIsNotAFact() {
        SpecSheet sheet = new SpecSheet()
                .putText("A", Optional.empty())
                .putText("B", Optional.of(""))
                .putText("C", Optional.of("   "))
                .putText("D", SpecSheet.text(null))
                .putText("E", Optional.of("Ratchet"));

        assertEquals(1, sheet.size(), "only the real name should have been recorded");
        assertEquals("Ratchet", sheet.entries().get("E"));
    }

    @Test
    void textIsTrimmed() {
        SpecSheet sheet = new SpecSheet().putText("Name", Optional.of("  Ratchet  "));
        assertEquals("Ratchet", sheet.entries().get("Name"));
    }

    @Test
    void nonFiniteNumbersAreNotFacts() {
        SpecSheet sheet = new SpecSheet()
                .putNumber("Nan", OptionalDouble.of(Double.NaN))
                .putNumber("Inf", OptionalDouble.of(Double.POSITIVE_INFINITY))
                .putNumber("Absent", OptionalDouble.empty())
                .putNumber("Real", OptionalDouble.of(4.5));

        assertEquals(1, sheet.size());
        assertEquals(4.5, (Double) sheet.entries().get("Real"));
    }

    @Test
    void zeroSurvivesWhenItIsAMeasurementAndNotWhenItIsAnApiSayingNothing() {
        // A module sitting on the centreline genuinely has x = 0, so putNumber keeps it.
        SpecSheet sheet = new SpecSheet().putNumber("ModuleX", OptionalDouble.of(0.0));
        assertTrue(sheet.entries().containsKey("ModuleX"));

        // positive() is the wrapper for the sources that return 0 to mean "unset" — an unassigned
        // team number, an FPGA version off a desktop.
        assertTrue(SpecSheet.positive(0.0).isEmpty());
        assertTrue(SpecSheet.positive(0).isEmpty());
        assertTrue(SpecSheet.positive(-1.0).isEmpty());
        assertTrue(SpecSheet.positive(-1).isEmpty());
        assertEquals(9973, SpecSheet.positive(9973).getAsInt());
    }

    @Test
    void emptyListsAreUnknownRatherThanNone() {
        SpecSheet sheet = new SpecSheet()
                .putList("Cameras", List.of())
                .putList("Inventory", List.of("TalonFX|8"));

        assertFalse(sheet.entries().containsKey("Cameras"),
                "no cameras seen and no cameras fitted must not look the same as an empty array");
        assertEquals(1, sheet.size());
    }

    @Test
    void arraysAreDroppedWholeIfAnyElementIsNotFinite() {
        SpecSheet sheet = new SpecSheet()
                .putNumbers("Bad", new double[] {0.3, Double.NaN})
                .putNumbers("Empty", new double[0])
                .putNumbers("Good", new double[] {0.3, -0.3});

        assertEquals(1, sheet.size(), "a half-known geometry is not a geometry");
        assertEquals(2, ((double[]) sheet.entries().get("Good")).length);
    }

    @Test
    void intsAndFlagsRecordWhatTheyAreGiven() {
        SpecSheet sheet = new SpecSheet()
                .putInt("Team", OptionalInt.of(9973))
                .putInt("Missing", OptionalInt.empty())
                .putFlag("CanFd", true);

        assertEquals(9973L, sheet.entries().get("Team"));
        assertEquals(Boolean.TRUE, sheet.entries().get("CanFd"));
        assertEquals(2, sheet.size());
    }

    @Test
    void entriesKeepInsertionOrderAndCannotBeEditedFromOutside() {
        SpecSheet sheet = new SpecSheet()
                .putText("Identity/Name", Optional.of("Ratchet"))
                .putInt("Identity/TeamNumber", OptionalInt.of(9973));

        assertEquals(List.of("Identity/Name", "Identity/TeamNumber"),
                List.copyOf(sheet.entries().keySet()));
        org.junit.jupiter.api.Assertions.assertThrows(UnsupportedOperationException.class,
                () -> sheet.entries().put("Sneaky", "value"));
    }
}
