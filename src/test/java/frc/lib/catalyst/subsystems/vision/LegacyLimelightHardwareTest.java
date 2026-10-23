package frc.lib.catalyst.subsystems.vision;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Exercises the documented hardware tuple through the real NetworkTables compatibility path. */
class LegacyLimelightHardwareTest {
    private NetworkTable table(String suffix) {
        return NetworkTableInstance.getDefault().getTable("limelight-hardware-test-" + suffix);
    }

    private LimelightSource source(NetworkTable table) {
        return new LimelightSource(table.getPath().substring(1), new Transform3d(), true);
    }

    @Test
    void documentedTupleReportsCpuTemperatureAndFpsInTheirOwnUnits() {
        NetworkTable table = table("documented");
        // Limelight NT API: [cpu_temp_celsius, cpu_usage, ram_usage_percent, fps].
        table.getEntry("hw").setDoubleArray(new double[] {82.6, 75.2, 41.3, 57.6});
        LimelightSource source = source(table);

        assertEquals(82.6, source.cameraTemperatureC().orElseThrow(), 1e-9);
        assertEquals(57.6, source.cameraFps().orElseThrow(), 1e-9);
    }

    @Test
    void unpublishedHardwareMetricsStayUnknown() {
        LimelightSource source = source(table("missing"));
        assertTrue(source.cameraTemperatureC().isEmpty());
        assertTrue(source.cameraFps().isEmpty());
    }

    @Test
    void shortArraysNeverSubstituteCpuOrRamUsageForMissingFps() {
        for (int length = 0; length < 4; length++) {
            NetworkTable table = table("short-" + length);
            double[] hw = java.util.Arrays.copyOf(new double[] {82.6, 75.2, 41.3}, length);
            table.getEntry("hw").setDoubleArray(hw);
            LimelightSource source = source(table);

            assertTrue(source.cameraFps().isEmpty(), "FPS is absent at length " + length);
            if (length == 0) {
                assertTrue(source.cameraTemperatureC().isEmpty());
            } else {
                assertEquals(82.6, source.cameraTemperatureC().orElseThrow(), 1e-9);
            }
        }
    }

    @Test
    void nonFiniteTemperatureDoesNotHideValidFps() {
        double[] invalidValues = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (int i = 0; i < invalidValues.length; i++) {
            NetworkTable table = table("invalid-temp-" + i);
            table.getEntry("hw").setDoubleArray(new double[] {invalidValues[i], 75.2, 41.3, 57.6});
            LimelightSource source = source(table);

            assertTrue(source.cameraTemperatureC().isEmpty());
            assertEquals(57.6, source.cameraFps().orElseThrow(), 1e-9);
        }
    }

    @Test
    void nonFiniteFpsDoesNotHideValidTemperature() {
        double[] invalidValues = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (int i = 0; i < invalidValues.length; i++) {
            NetworkTable table = table("invalid-fps-" + i);
            table.getEntry("hw").setDoubleArray(new double[] {82.6, 75.2, 41.3, invalidValues[i]});
            LimelightSource source = source(table);

            assertTrue(source.cameraFps().isEmpty());
            assertEquals(82.6, source.cameraTemperatureC().orElseThrow(), 1e-9);
        }
    }

    @Test
    void malformedUnusedMetricsAndExtraFieldsDoNotReplaceValidDiagnostics() {
        NetworkTable table = table("unused-fields");
        table.getEntry("hw").setDoubleArray(
                new double[] {82.6, Double.NaN, Double.POSITIVE_INFINITY, 57.6, 999.0});
        LimelightSource source = source(table);

        assertEquals(82.6, source.cameraTemperatureC().orElseThrow(), 1e-9);
        assertEquals(57.6, source.cameraFps().orElseThrow(), 1e-9);
    }
}
