package frc.lib.catalyst.mechanisms;

import frc.lib.catalyst.statemachine.goals.ServoGoal;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * HAL-free checks for the servo configuration and goal. Constructing a {@link ServoMechanism} itself
 * needs the PWM HAL, so only the pure {@code Config} builder and {@link ServoGoal} are exercised here.
 */
class ServoConfigTest {

    @Test
    void buildRequiresAChannel() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class,
                () -> ServoMechanism.Config.builder().name("Hood").build());
        assertTrue(ex.getMessage().contains("channel"));
    }

    @Test
    void buildRejectsInvertedRange() {
        assertThrows(IllegalArgumentException.class, () -> ServoMechanism.Config.builder()
                .name("Hood").channel(0).angleRange(60, 20).build());
    }

    @Test
    void buildRejectsAnOutOfRangeNamedPosition() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class,
                () -> ServoMechanism.Config.builder()
                        .name("Hood").channel(0).angleRange(20, 60)
                        .position("TOO_FAR", 90)
                        .build());
        assertTrue(ex.getMessage().contains("TOO_FAR"));
    }

    @Test
    void validConfigBuilds() {
        ServoMechanism.Config c = ServoMechanism.Config.builder()
                .name("Hood").channel(1).angleRange(20, 60)
                .position("CLOSE", 20).position("FAR", 55)
                .build();
        assertEquals("Hood", c.name);
        assertEquals(2, c.namedPositions.size());
    }

    @Test
    void servoGoalClampsBadSettleAndCarriesPreset() {
        assertEquals(ServoGoal.DEFAULT_SETTLE_SECONDS,
                ServoGoal.degrees(30, -5).settleSeconds(), 1e-9,
                "a negative settle must not mean 'never arrives'");
        assertEquals("FAR", ServoGoal.preset("FAR").preset());
        // Value equality so the engine's per-loop goal compare is cheap.
        assertEquals(ServoGoal.degrees(30), ServoGoal.degrees(30));
    }
}
