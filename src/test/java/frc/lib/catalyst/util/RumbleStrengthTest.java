package frc.lib.catalyst.util;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
class RumbleStrengthTest {
  @Test void preservesScaleAndClear() {
    assertEquals(.2, RumbleEvents.scaledStrength(.5,.4), 1e-9);
    assertEquals(0, RumbleEvents.scaledStrength(0,.4));
    assertEquals(1, RumbleEvents.scaledStrength(1,1));
  }
  @Test void clampsAndRejectsInvalidInput() {
    assertEquals(0, RumbleEvents.scaledStrength(1,Double.NaN));
    assertEquals(0, RumbleEvents.scaledStrength(Double.POSITIVE_INFINITY,1));
    assertEquals(0, RumbleEvents.scaledStrength(1,-1));
    assertEquals(1, RumbleEvents.scaledStrength(1,2));
  }
}
