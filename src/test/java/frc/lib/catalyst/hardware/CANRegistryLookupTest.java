package frc.lib.catalyst.hardware;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * {@link CANRegistry#lookup} has to agree with {@link CANRegistry#register} about what a bus is
 * called.
 *
 * <p>Systemcore devices can be declared on {@code ""}, {@code "0"} or {@code "can_s0"} and mean the
 * same wire. {@code register} normalises those to one name before storing; {@code lookup} built its
 * key from whatever string it was handed, so it looked for {@code "/12"} or {@code "0/12"} against
 * an entry stored as {@code "can_s0/12"} and came back empty. Empty reads as "no device has that
 * id", which is the opposite of what was true.
 */
class CANRegistryLookupTest {

    @AfterEach
    void clear() {
        CANRegistry.clear();
    }

    @Test
    void everySpellingOfABusFindsTheSameDevice() {
        CANRegistry.register("FrontLeftDrive", 12, "can_s0", "TalonFX");

        for (String spelling : new String[] { "can_s0", "0", "" }) {
            assertTrue(CANRegistry.lookup(12, spelling).isPresent(),
                    "lookup(12, \"" + spelling + "\") found nothing, but that is the bus it is on");
            assertEquals("FrontLeftDrive", CANRegistry.lookup(12, spelling).get().name());
        }
    }

    @Test
    void aDeviceRegisteredUnderAnAliasIsFoundByItsCanonicalName() {
        // The other direction: declared with the roboRIO-era empty string, looked up properly.
        CANRegistry.register("Intake", 5, "", "TalonFX");

        assertTrue(CANRegistry.lookup(5, "can_s0").isPresent(),
                "an empty bus name means can_s0, and has since the default changed");
    }

    @Test
    void anIdThatIsGenuinelyUnusedStillComesBackEmpty() {
        CANRegistry.register("Intake", 5, "can_s0", "TalonFX");

        assertTrue(CANRegistry.lookup(9, "can_s0").isEmpty(), "nothing owns id 9");
        assertTrue(CANRegistry.lookup(5, "can_s2").isEmpty(), "id 5 is on can_s0, not can_s2");
    }
}
