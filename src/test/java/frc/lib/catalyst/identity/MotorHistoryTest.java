package frc.lib.catalyst.identity;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.nio.file.Files;
import java.util.List;

/**
 * The accounting and the file, with no Phoenix and no HAL: devices arrive as the diagnostic server
 * would describe them, samples arrive as the status frames would carry them.
 */
class MotorHistoryTest {

    private static final String SERIAL = "000E0B500C776800000A0001160000E3";

    @BeforeEach
    void fresh() {
        System.setProperty("catalyst.motorhistory.noPhoenix", "true");
        MotorHistory.clearForTest();
    }

    @AfterEach
    void clean() {
        MotorHistory.clearForTest();
        System.clearProperty("catalyst.motorhistory.noPhoenix");
    }

    private static MotorHistory.DeviceInfo talon(int id, String name, String fw) {
        return new MotorHistory.DeviceInfo("Talon FX", id, name, SERIAL, "can_s2", fw, "2.2", "Aug 17, 2020");
    }

    @Test
    void theSerialIsTheMotorAndTheIdIsJustWhatItIsCalledToday() {
        // The X1's back-left drive arrived as "Intake Roller" on 45 - a different robot's label.
        MotorHistory.observe(talon(45, "Intake Roller", "26.1.0.0 (Phoenix 6)"), 1_000L);
        MotorHistory.observe(talon(45, "Intake Roller", "26.1.0.0 (Phoenix 6)"), 2_000L);
        MotorHistory.observe(talon(45, "BL_Drive", "26.1.1.1 (Phoenix 6)"), 3_000L);

        List<MotorHistory.Record> records = List.copyOf(MotorHistory.records());
        assertEquals(1, records.size(), "one serial, one motor, whatever it was called");
        MotorHistory.Record r = records.get(0);
        assertEquals("motor", r.kind);
        assertEquals(2, r.identities.size(), "the rename and the firmware update are one new identity");
        assertEquals("Intake Roller", r.identities.get(0).name);
        assertEquals(2_000L, r.identities.get(0).lastSeenMs, "the old identity's last sighting is kept");
        assertEquals("BL_Drive", r.latest().name);
        assertEquals(1_000L, r.firstSeenMs);
        assertEquals(3_000L, r.lastSeenMs);
        assertEquals(1, r.boots, "three sightings in one boot are one session");
    }

    @Test
    void useIsAccountedFromTheStatusFramesAndTheBootIsASession() {
        MotorHistory.observe(talon(30, "FL_Drive", "26.1.1.1"), 5_000L);
        // Ten seconds turning at 50 rps under 40 A at 12 V, 75 C: hot.
        for (int i = 0; i < 100; i++) {
            MotorHistory.sampleFor(SERIAL, new MotorHistory.Sample(50.0, 40.0, 30.0, 12.0, 75.0, 0), 0.1);
        }
        // Ten seconds parked, cool, one sticky fault.
        for (int i = 0; i < 100; i++) {
            MotorHistory.sampleFor(SERIAL, new MotorHistory.Sample(0.0, 0.2, 0.1, 0.0, 40.0, 0b100), 0.1);
        }

        MotorHistory.Record r = MotorHistory.records().iterator().next();
        assertEquals(20.0, r.poweredSeconds, 1e-6);
        assertEquals(10.0, r.runningSeconds, 1e-6);
        assertEquals(10.0, r.loadedSeconds, 1e-6);
        assertEquals(500.0, r.revolutions, 1e-6, "50 rps for 10 s");
        assertEquals(4800.0, r.energyJoules, 1e-6, "12 V * 40 A * 10 s");
        assertEquals(40.0, r.peakStatorAmps, 1e-9);
        assertEquals(75.0, r.peakTempC, 1e-9);
        assertEquals(10.0, r.hotSeconds, 1e-6, "above 70 C for the working half");
        assertEquals(0b100, r.stickyFaults);
        assertEquals(1, r.sessions.size());
        assertEquals(20.0, r.sessions.get(0).seconds, 1e-6);
        assertEquals(500.0, r.sessions.get(0).revolutions, 1e-6);
    }

    @Test
    void theFileIsTheRecordAndComesBackWhole() throws Exception {
        MotorHistory.observe(talon(45, "Intake Roller", "26.1.0.0"), 1_000L);
        MotorHistory.observe(talon(45, "BL_Drive", "26.1.1.1"), 2_000L);
        MotorHistory.sampleFor(SERIAL, new MotorHistory.Sample(20.0, 10.0, 8.0, 6.0, 50.0, 0), 2.0);
        String json = MotorHistory.toJson();

        MotorHistory.clearForTest();
        System.setProperty("catalyst.motorhistory.noPhoenix", "true");
        MotorHistory.loadJson(json);

        MotorHistory.Record r = MotorHistory.records().iterator().next();
        assertEquals(SERIAL, r.serial);
        assertEquals(2, r.identities.size());
        assertEquals("BL_Drive", r.latest().name);
        assertEquals(2.0, r.poweredSeconds, 1e-9);
        assertEquals(40.0, r.revolutions, 1e-9);
        assertEquals(1, r.sessions.size());
        assertEquals(1, r.boots);

        // The next boot on the loaded record is a new session, and the totals carry on.
        MotorHistory.observe(talon(45, "BL_Drive", "26.1.1.1"), 9_000L);
        MotorHistory.sampleFor(SERIAL, new MotorHistory.Sample(20.0, 10.0, 8.0, 6.0, 50.0, 0), 1.0);
        r = MotorHistory.records().iterator().next();
        assertEquals(2, r.boots);
        assertEquals(2, r.sessions.size());
        assertEquals(3.0, r.poweredSeconds, 1e-9);
        assertEquals(60.0, r.revolutions, 1e-9);
        assertEquals(2, r.identities.size(), "same name, same id, same firmware: no new identity");
    }

    @Test
    void theSessionLogIsBoundedAndTheTotalsAreNot() {
        MotorHistory.Config cfg = new MotorHistory.Config();
        cfg.maxSessions = 3;
        MotorHistory.configure(cfg);
        for (int boot = 0; boot < 5; boot++) {
            MotorHistory.observe(talon(30, "FL_Drive", "26.1.1.1"), 1_000L * (boot + 1));
            MotorHistory.sampleFor(SERIAL, new MotorHistory.Sample(10.0, 1.0, 1.0, 1.0, 30.0, 0), 1.0);
            // A new boot: reload from the file, which is what a boot does.
            String json = MotorHistory.toJson();
            MotorHistory.clearForTest();
            System.setProperty("catalyst.motorhistory.noPhoenix", "true");
            MotorHistory.configure(cfg);
            MotorHistory.loadJson(json);
        }
        MotorHistory.Record r = MotorHistory.records().iterator().next();
        assertEquals(5, r.boots);
        assertEquals(3, r.sessions.size(), "only the last three boots are kept in detail");
        assertEquals(5.0, r.poweredSeconds, 1e-9, "but every second is in the totals");
        assertEquals(50.0, r.revolutions, 1e-9);
    }

    @Test
    void devicesThatAreNotMotorsAreKnownButNotSampled() {
        MotorHistory.observe(new MotorHistory.DeviceInfo("CANCoder", 13, "FL_Encoder", "ABC", "can_s2", "26.1.1.0", "", ""), 1L);
        MotorHistory.observe(new MotorHistory.DeviceInfo("Pigeon 2", 23, "Pigeon", "DEF", "can_s2", "26.1.0.0", "", ""), 1L);
        List<MotorHistory.Record> records = List.copyOf(MotorHistory.records());
        assertEquals("encoder", records.get(0).kind);
        assertEquals("imu", records.get(1).kind);
        assertTrue(MotorHistory.toJson().contains("\"FL_Encoder\""));
    }

    @Test
    void aDeviceWithNoSerialIsKeyedByWhereItIs() {
        MotorHistory.observe(new MotorHistory.DeviceInfo("Talon FX", 7, "spare", "", "can_s0", "", "", ""), 1L);
        MotorHistory.observe(new MotorHistory.DeviceInfo("Talon FX", 7, "spare", "", "can_s0", "", "", ""), 2L);
        assertEquals(1, MotorHistory.records().size());
        assertFalse(MotorHistory.toJson().contains("\"serial\": \"can_s0"), "the made-up key is not written as a serial");
    }

    @Test
    void theDeviceListFromTheDiagnosticServerIsReadCorrectly() {
        String body = "{\"DeviceArray\": [{\"CANbus\": \"can_s2\", \"ID\": 27, \"Model\": \"Talon FX\", \"Name\": \"FR_steer\","
                + " \"SerialNo\": \"000E0B500C776800000A0001060000F5\", \"CurrentVers\": \"26.1.1.1 (Phoenix 6)\","
                + " \"HardwareRev\": \"2.2\", \"ManDate\": \"Aug 17, 2020\"}, {\"CANbus\": \"can_s2\", \"ID\": 19,"
                + " \"Model\": \"CANCoder\", \"Name\": \"FR_Encoder\", \"SerialNo\": \"X\"}], \"GeneralReturn\": {\"Error\": 0}}";
        List<MotorHistory.DeviceInfo> devices = MotorHistory.parseDevices(body);
        assertEquals(2, devices.size());
        assertEquals(27, devices.get(0).id());
        assertEquals("motor", devices.get(0).kind());
        assertEquals("000E0B500C776800000A0001060000F5", devices.get(0).key());
        assertEquals("encoder", devices.get(1).kind());
    }

    @Test
    void writingGoesToTheConfiguredPath() throws Exception {
        File dir = Files.createTempDirectory("motor-history").toFile();
        MotorHistory.Config cfg = new MotorHistory.Config();
        cfg.path = new File(dir, "nested/motor-history.json").getPath();
        MotorHistory.configure(cfg);
        MotorHistory.observe(talon(30, "FL_Drive", "26.1.1.1"), 1L);
        MotorHistory.flushNow();
        File f = MotorHistory.file();
        assertTrue(f.exists(), "directories are made as needed");
        assertTrue(Files.readString(f.toPath()).contains("catalyst-motor-history"));
    }
}
