package frc.lib.catalyst.identity;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * What the sheet says, and — mostly — what it declines to say.
 *
 * <p>These run on a bare JVM with no HAL, no roboRIO and no deployed PathPlanner settings, which is
 * the harshest version of the case that matters: almost nothing is knowable, so almost nothing may
 * be published. Every key that shows up here has a real source behind it.
 */
class RobotIdentitySheetTest {

    @BeforeEach
    @AfterEach
    void clearObservedState() {
        RobotIdentity.reset();
    }

    private static Map<String, Object> sheetFor(RobotIdentity.Builder builder) {
        return RobotIdentity.derive(builder.build()).entries();
    }

    @Test
    void theNameIsTheOnlyThingATeamMustSupply() {
        Map<String, Object> sheet = sheetFor(RobotIdentity.named("Ratchet"));
        assertEquals("Ratchet", sheet.get("Identity/Name"));
    }

    @Test
    void anUnnamedRobotIsRefusedAtTheDoor() {
        assertThrows(IllegalArgumentException.class, () -> RobotIdentity.named(null));
        assertThrows(IllegalArgumentException.class, () -> RobotIdentity.named("   "));
    }

    @Test
    void catalystNamesItsOwnBuild() {
        Map<String, Object> sheet = sheetFor(RobotIdentity.named("Ratchet"));
        assertEquals(CatalystVersion.version(), sheet.get("Software/CatalystVersion"));
        assertFalse(String.valueOf(sheet.get("Software/CatalystVersion")).isBlank());
    }

    @Test
    void noDrivetrainMeansNoDrivetrainGroup() {
        Map<String, Object> sheet = sheetFor(RobotIdentity.named("Ratchet"));
        assertTrue(sheet.keySet().stream().noneMatch(k -> k.startsWith("Drivetrain/")),
                "a robot Catalyst has no drivetrain for publishes nothing about one, not zeros: "
                        + sheet.keySet());
    }

    @Test
    void noMeasurementsMeansNoChassisGeometry() {
        Map<String, Object> sheet = sheetFor(RobotIdentity.named("Ratchet"));
        assertFalse(sheet.containsKey("Chassis/FrameLengthMeters"));
        assertFalse(sheet.containsKey("Chassis/HeightMeters"));
        assertFalse(sheet.containsKey("Chassis/BumperLengthMeters"));
        // Mass comes from deployed PathPlanner settings, and there are none on a test machine.
        assertFalse(sheet.containsKey("Chassis/MassKg"));
    }

    @Test
    void bumperFootprintIsArithmeticOnDeclaredNumbers() {
        Map<String, Object> sheet = sheetFor(RobotIdentity.named("Ratchet")
                .frameMeters(0.74, 0.70)
                .bumperThicknessMeters(0.08));

        assertEquals(0.74, (Double) sheet.get("Chassis/FrameLengthMeters"), 1e-9);
        assertEquals(0.90, (Double) sheet.get("Chassis/BumperLengthMeters"), 1e-9);
        assertEquals(0.86, (Double) sheet.get("Chassis/BumperWidthMeters"), 1e-9);
    }

    @Test
    void bumperFootprintNeedsBothHalvesOfTheSum() {
        Map<String, Object> withoutThickness = sheetFor(RobotIdentity.named("Ratchet").frameMeters(0.74, 0.70));
        assertFalse(withoutThickness.containsKey("Chassis/BumperLengthMeters"));

        Map<String, Object> withoutFrame = sheetFor(RobotIdentity.named("Ratchet").bumperThicknessMeters(0.08));
        assertFalse(withoutFrame.containsKey("Chassis/BumperLengthMeters"));
    }

    @Test
    void nonsenseMeasurementsAreDroppedRatherThanPublished() {
        Map<String, Object> sheet = sheetFor(RobotIdentity.named("Ratchet")
                .frameMeters(0, -1)
                .heightMeters(Double.NaN));

        assertFalse(sheet.containsKey("Chassis/FrameLengthMeters"));
        assertFalse(sheet.containsKey("Chassis/FrameWidthMeters"));
        assertFalse(sheet.containsKey("Chassis/HeightMeters"));
    }

    @Test
    void declaredTextIsPublishedAndBlankTextIsNot() {
        Map<String, Object> sheet = sheetFor(RobotIdentity.named("Ratchet")
                .robotCodeVersion("2026.4.1-3-gdeadbee")
                .robotCodeBuild("  ")
                .battery("MK ES17-12"));

        assertEquals("2026.4.1-3-gdeadbee", sheet.get("Software/RobotCodeVersion"));
        assertFalse(sheet.containsKey("Software/RobotCodeBuild"));
        assertEquals("MK ES17-12", sheet.get("Power/Battery"));
    }

    @Test
    void powerChannelsSerialiseInChannelOrderAsPipedRows() {
        Map<String, Object> sheet = sheetFor(RobotIdentity.named("Ratchet")
                .pdhChannel(7, "Front right drive")
                .pdhChannel(2, "Elevator")
                .pdhChannel(4, ""));

        String[] rows = (String[]) sheet.get("Power/ChannelsInUse");
        assertEquals(List.of("2|Elevator", "7|Front right drive"), List.of(rows),
                "an unlabelled channel is not a channel in use");
    }

    @Test
    void noPowerModuleMeansNoModuleKeys() {
        Map<String, Object> sheet = sheetFor(RobotIdentity.named("Ratchet"));
        assertFalse(sheet.containsKey("Power/Module"));
        assertFalse(sheet.containsKey("Power/Channels"));
    }

    @Test
    void camerasAppearOnceTheVisionSubsystemHasNamedThem() {
        assertFalse(sheetFor(RobotIdentity.named("Ratchet")).containsKey("Hardware/Cameras"));

        RobotIdentity.observeCameras(List.of("FrontCam", "BackCam"));
        String[] cameras = (String[]) sheetFor(RobotIdentity.named("Ratchet")).get("Hardware/Cameras");
        assertEquals(List.of("FrontCam", "BackCam"), List.of(cameras));
    }

    @Test
    void aSheetSurvivesAMachineThatKnowsNothing() {
        // No HAL here, so every rio-sourced fact must come back missing rather than throwing.
        Map<String, Object> sheet = sheetFor(RobotIdentity.named("Ratchet"));
        assertFalse(sheet.containsKey("Identity/TeamNumber"));
        assertFalse(sheet.containsKey("Identity/RioSerial"));
        assertFalse(sheet.containsKey("Software/RioImage"));
        // ...and the facts that need nothing but the JVM must still be there.
        assertTrue(sheet.containsKey("Software/WPILibVersion"));
        assertTrue(sheet.containsKey("Software/JavaVersion"));
        assertEquals(2026L, sheet.get("Identity/Season"));
    }
}
