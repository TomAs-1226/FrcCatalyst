package frc.lib.catalyst.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Torque-current config plumbing. Constructing a real FlywheelMechanism needs CAN hardware and the
 * HAL, which this test environment does not have — but the Config carries every decision the
 * mechanism later acts on, so the plumbing is testable on its own.
 */
class FlywheelTorqueCurrentConfigTest {
  @Test
  void torqueCurrentDefaultsOffWithPhoenixPeaks() {
    var config = FlywheelMechanism.Config.builder().name("T").motor(1).build();

    assertFalse(config.torqueCurrentFOC, "torque-current must be opt-in");
    // Phoenix's own factory peaks — writing these unconditionally must be a no-op for
    // voltage-mode users.
    assertEquals(800, config.peakForwardTorqueCurrent);
    assertEquals(-800, config.peakReverseTorqueCurrent);
  }

  @Test
  void builderCarriesTorqueCurrentSettingsThrough() {
    var config =
        FlywheelMechanism.Config.builder()
            .name("Shooter")
            .motor(26)
            .torqueCurrentFOC(true)
            .torqueCurrentLimits(200, -35)
            .pid(14.0, 0, 0)
            .feedforward(6.4, 0.13)
            .build();

    assertTrue(config.torqueCurrentFOC);
    assertEquals(200, config.peakForwardTorqueCurrent);
    assertEquals(-35, config.peakReverseTorqueCurrent);
    // In torque-current mode these are amps; the builder must not transform them.
    assertEquals(14.0, config.kP);
    assertEquals(6.4, config.kS);
    assertEquals(0.13, config.kV);
  }
}
