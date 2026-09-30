package frc.lib.catalyst.fieldlab;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * {@link Procedures.Setpoint} is the only piece of {@link Procedures} that needs no
 * {@code SwerveSubsystem} or {@code SysIdRoutine} to exercise: its label, and the NetworkTables
 * key an observation is recorded under.
 */
class ProceduresTest {

    @Test
    void aSetpointBuiltFromItsValueLabelsItselfFromThatValue() {
        Procedures.Setpoint sp = Procedures.Setpoint.of(2.5, 1800, 42);

        assertEquals("2.50", sp.label(), "the label should read off the setpoint's own independent value");
        assertEquals(2.5, sp.at(), 0.0);
        assertEquals(1800, sp.first(), 0.0);
        assertEquals(42, sp.second(), 0.0);
    }

    @Test
    void keyIsStableAcrossCalls() {
        Procedures.Setpoint sp = Procedures.Setpoint.of(3.0, 2100, 38);

        String first = sp.key();
        String second = sp.key();
        assertEquals(first, second, "the same setpoint should always produce the same key");
    }

    @Test
    void aDecimalSetpointProducesACleanKeyWithNoSpacesOrSlashes() {
        Procedures.Setpoint sp = Procedures.Setpoint.of(2.5, 1800, 42);

        String key = sp.key();

        assertEquals("at-2.50", key);
        assertTrue(key.matches("[A-Za-z0-9._-]+"),
                "an NT key should contain only characters NetworkTables accepts in a name: " + key);
        assertFalse(key.contains(" "), "a key with a space would split across dashboards differently than intended: " + key);
        assertFalse(key.contains("/"), "a '/' in a key would be read as a table separator, not part of the name: " + key);
    }
}
