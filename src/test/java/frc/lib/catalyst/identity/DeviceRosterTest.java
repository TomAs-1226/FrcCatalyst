package frc.lib.catalyst.identity;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

class DeviceRosterTest {

    private NetworkTableInstance inst;
    private NetworkTable table;

    @BeforeEach
    void setUp() {
        DeviceRoster.clear();
        inst = NetworkTableInstance.create();
        table = inst.getTable("Catalyst").getSubTable("Devices");
        DeviceRoster.useTable(table);
    }

    @AfterEach
    void tearDown() {
        DeviceRoster.clear();
        DeviceRoster.useTable(null);
        inst.close();
    }

    @Test
    void countsAndRowsFollowTheRegistrations() {
        DeviceRoster.registerMotor("frontLeft", "can_s0", 3, () -> true);
        DeviceRoster.registerMotor("frontRight", "can_s0", 4, () -> false);
        DeviceRoster.registerCamera("limelight-left", "Limelight", () -> true);
        DeviceRoster.registerController("Systemcore", () -> true);
        DeviceRoster.publishNow();

        assertEquals(2, table.getSubTable("Motors").getEntry("Expected").getInteger(-1));
        assertEquals(1, table.getSubTable("Motors").getEntry("Connected").getInteger(-1));
        assertArrayEquals(new String[] {"frontLeft|can_s0|3|true", "frontRight|can_s0|4|false"},
                table.getSubTable("Motors").getEntry("Rows").getStringArray(new String[0]));
        assertEquals(1, table.getSubTable("Cameras").getEntry("Expected").getInteger(-1));
        assertArrayEquals(new String[] {"limelight-left|true|Limelight"},
                table.getSubTable("Cameras").getEntry("Rows").getStringArray(new String[0]));
        assertEquals("Systemcore", table.getSubTable("Controller").getEntry("Kind").getString(""));
        assertTrue(table.getSubTable("Controller").getEntry("Connected").getBoolean(false));
        assertEquals("1/1 cameras · 1/2 motors · Systemcore", table.getEntry("Summary").getString(""));
    }

    @Test
    void aProbeThatThrowsIsNotConnected() {
        DeviceRoster.registerMotor("m", "can_s1", 9, () -> { throw new IllegalStateException("bus gone"); });
        assertEquals(new DeviceRoster.Count(1, 0), DeviceRoster.count(DeviceRoster.Kind.MOTOR));
    }

    @Test
    void controllerThatStopsAnsweringIsSaidSo() {
        boolean[] up = {true};
        DeviceRoster.registerController("Systemcore", () -> up[0]);
        DeviceRoster.publishNow();
        assertEquals("0/0 cameras · 0/0 motors · Systemcore", table.getEntry("Summary").getString(""));
        up[0] = false;
        DeviceRoster.publishNow();
        assertFalse(table.getSubTable("Controller").getEntry("Connected").getBoolean(true));
        assertTrue(table.getEntry("Summary").getString("").endsWith("(not answering)"));
    }

    @Test
    void publishIsRateLimitedToFourHertz() {
        boolean[] up = {true};
        DeviceRoster.registerMotor("m", "can_s0", 1, () -> up[0]);
        DeviceRoster.publish(100.0);
        assertEquals(1, table.getSubTable("Motors").getEntry("Connected").getInteger(-1));
        up[0] = false;
        DeviceRoster.publish(100.1);
        assertEquals(1, table.getSubTable("Motors").getEntry("Connected").getInteger(-1), "too soon to republish");
        DeviceRoster.publish(100.3);
        assertEquals(0, table.getSubTable("Motors").getEntry("Connected").getInteger(-1));
    }

    @Test
    void withNothingRegisteredTheControllerIsStillNamed() {
        DeviceRoster.publishNow();
        String kind = table.getSubTable("Controller").getEntry("Kind").getString("");
        assertTrue(kind.equals("Systemcore") || kind.equals("Robot controller"), kind);
    }
}
