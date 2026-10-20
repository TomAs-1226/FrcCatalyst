package frc.lib.catalyst.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class LegacySingleTagAmbiguityTest {
    @Test void singleTagMt1PreservesTheCameraAmbiguityInsteadOfCertifyingZero() {
        double[] frame = new double[18];
        frame[17] = .95;
        assertEquals(.95, LegacyLimelightReader.singleTagAmbiguity(frame, 1, false));
        frame[17] = .02;
        assertEquals(.02, LegacyLimelightReader.singleTagAmbiguity(frame, 1, false));
        frame[17] = Double.NaN;
        assertEquals(1, LegacyLimelightReader.singleTagAmbiguity(frame, 1, false));
    }
    @Test void gyroAndMultiTagSolutionsDoNotInheritSingleTagPlanarAmbiguity() {
        double[] frame = new double[18]; frame[17] = .95;
        assertEquals(0, LegacyLimelightReader.singleTagAmbiguity(frame, 1, true));
        assertEquals(0, LegacyLimelightReader.singleTagAmbiguity(frame, 2, false));
        assertEquals(0, LegacyLimelightReader.singleTagAmbiguity(new double[11], 1, false));
    }
}
