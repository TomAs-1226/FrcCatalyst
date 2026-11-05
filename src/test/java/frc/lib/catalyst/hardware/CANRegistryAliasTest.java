package frc.lib.catalyst.hardware;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

/**
 * One physical bus has several spellings, and the registry has to know that.
 *
 * <p>{@code ""}, {@code "0"} and {@code "can_s0"} all mean the same wire — {@code CatalystCANBus.of}
 * resolves all three to {@code can_s0}. The registry used to key on the raw string, so two devices
 * on that wire at the same id registered under different keys and never collided.
 *
 * <p>That is not a hypothetical combination. {@code CatalystMotor}'s default bus is the empty string
 * and a generated {@code CANIds.java} writes {@code "can_s0"}, so a robot mixing a hand-built
 * mechanism with generated constants hits it — and hits it as a Phoenix failure at an event rather
 * than as a named conflict at {@code robotInit}, which is the whole reason the registry exists.
 */
class CANRegistryAliasTest {

    @AfterEach
    void clear() {
        CANRegistry.clear();
    }

    @Test
    void theThreeSpellingsOfOneBusCollideWithEachOther() {
        CANRegistry.register("Elevator", 12, "", "TalonFX");

        assertThrows(CANRegistry.CANConflictException.class,
                () -> CANRegistry.register("Intake", 12, "can_s0", "TalonFX"),
                "the default bus and can_s0 are the same wire");
    }

    @Test
    void theNumericSpellingCollidesToo() {
        CANRegistry.register("Elevator", 12, "0", "TalonFX");

        assertThrows(CANRegistry.CANConflictException.class,
                () -> CANRegistry.register("Intake", 12, "can_s0", "TalonFX"));
    }

    @Test
    void reRegisteringTheSameDeviceUnderAnotherSpellingIsStillIdempotent() {
        // A mechanism claiming its own id twice, once through each path, must not be a conflict.
        CANRegistry.register("Elevator", 12, "", "TalonFX");
        CANRegistry.register("Elevator", 12, "can_s0", "TalonFX");

        assertEquals(1, CANRegistry.all().size());
    }

    @Test
    void genuinelyDifferentBusesStillDoNotCollide() {
        // The point of having five. Same id on two wires is legal and normal.
        CANRegistry.register("Elevator", 12, "can_s0", "TalonFX");
        CANRegistry.register("Intake", 12, "can_s2", "TalonFX");

        assertEquals(2, CANRegistry.all().size());
    }

    @Test
    void aBusIsCountedOnceHoweverItWasSpelled() {
        // byBus() feeds contention warnings and the utilisation estimate. Aliasing split one
        // overloaded bus into two half-loaded ones, which is exactly the situation those checks
        // exist to report.
        CANRegistry.register("A", 1, "", "TalonFX");
        CANRegistry.register("B", 2, "0", "TalonFX");
        CANRegistry.register("C", 3, "can_s0", "TalonFX");

        assertEquals(1, CANRegistry.byBus().size(), "one wire, one entry");
        assertEquals(3, CANRegistry.byBus().get("can_s0").size());
    }

    @Test
    void aCanivoreKeepsItsOwnName() {
        // Anything that is not a Systemcore bus name is a CANivore, and two CANivores are two buses.
        CANRegistry.register("A", 5, "Drivebase", "TalonFX");
        CANRegistry.register("B", 5, "Superstructure", "TalonFX");

        assertEquals(2, CANRegistry.byBus().size());
    }
}
