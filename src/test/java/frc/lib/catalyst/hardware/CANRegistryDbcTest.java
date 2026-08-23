package frc.lib.catalyst.hardware;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The DBC export.
 *
 * <p>Generated text is where silent corruption lives — a file that is subtly malformed still gets
 * written, still gets uploaded, and only fails once it is in the CAN Bus Monitor at an event. These
 * assert the structure the format requires rather than that the method returns something.
 */
class CANRegistryDbcTest {

    @BeforeEach
    void reset() {
        CANRegistry.clear();
    }

    @Test
    void producesTheHeaderTheFormatRequires() {
        CANRegistry.register("FrontLeftDrive", 1, "can_s0", "TalonFX");
        String dbc = CANRegistry.toDbc();

        assertTrue(dbc.startsWith("VERSION \"\""), "a DBC file opens with a VERSION line");
        assertTrue(dbc.contains("NS_ :"), "NS_ section is required");
        assertTrue(dbc.contains("BS_:"), "BS_ section is required");
        assertTrue(dbc.contains("BU_:"), "BU_ node list is required");
    }

    @Test
    void everyDeviceBecomesAMessageNamedAsRobotCodeNamesIt() {
        CANRegistry.register("FrontLeftDrive", 1, "can_s0", "TalonFX");
        CANRegistry.register("ArmEncoder", 2, "can_s3", "CANcoder");

        String dbc = CANRegistry.toDbc();
        assertTrue(dbc.contains("BO_ 1 FrontLeftDrive:"), "device name should survive into the file");
        assertTrue(dbc.contains("BO_ 2 ArmEncoder:"), "second device missing: " + dbc);
    }

    @Test
    void busesBecomeNodesSoTheMonitorGroupsThemAsWired() {
        CANRegistry.register("A", 1, "can_s0", "TalonFX");
        CANRegistry.register("B", 2, "can_s3", "TalonFX");

        String dbc = CANRegistry.toDbc();
        String nodeLine = dbc.lines().filter(l -> l.startsWith("BU_:")).findFirst().orElseThrow();
        assertTrue(nodeLine.contains("can_s0"), "bus should appear as a node: " + nodeLine);
        assertTrue(nodeLine.contains("can_s3"), "bus should appear as a node: " + nodeLine);
    }

    @Test
    void namesIllegalInDbcAreSanitisedRatherThanEmitted() {
        // Teams name things with spaces, dots and dashes. DBC identifiers allow none of those, and
        // emitting them produces a file the monitor rejects with no useful message.
        CANRegistry.register("Front Left.Drive-1", 3, "can_s0", "TalonFX");

        String dbc = CANRegistry.toDbc();
        String messageLine = dbc.lines().filter(l -> l.startsWith("BO_ 3")).findFirst().orElseThrow();
        assertFalse(messageLine.contains(" Left"), "spaces must not survive: " + messageLine);
        assertFalse(messageLine.contains("."), "dots must not survive: " + messageLine);
        assertFalse(messageLine.contains("-"), "dashes must not survive: " + messageLine);
        assertTrue(messageLine.contains("Front_Left_Drive_1"), messageLine);
    }

    @Test
    void everyMessageHasExactlyOneSignal() {
        for (int id = 1; id <= 5; id++) {
            CANRegistry.register("Motor" + id, id, "can_s0", "TalonFX");
        }
        String dbc = CANRegistry.toDbc();

        long messages = dbc.lines().filter(l -> l.startsWith("BO_ ")).count();
        long signals = dbc.lines().filter(l -> l.trim().startsWith("SG_ ")).count();
        assertEquals(5, messages, "one message per device");
        assertEquals(messages, signals, "a DBC message with no signal is not useful to the monitor");
    }

    @Test
    void anEmptyRegistryStillProducesAValidFile() {
        String dbc = CANRegistry.toDbc();
        assertTrue(dbc.startsWith("VERSION \"\""),
                "a robot with nothing registered should produce an empty-but-valid file, not junk");
        assertEquals(0, dbc.lines().filter(l -> l.startsWith("BO_ ")).count());
    }
}
