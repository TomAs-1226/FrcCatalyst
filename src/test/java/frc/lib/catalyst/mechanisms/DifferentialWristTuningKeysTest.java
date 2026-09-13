package frc.lib.catalyst.mechanisms;

import frc.lib.catalyst.hardware.CANRegistry;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Where a differential wrist's live-tunable gains land in NetworkTables.
 *
 * <p>{@link frc.lib.catalyst.util.TunableNumber} files every key under {@code /Catalyst/Tuning} on its
 * own. From 0.3.5-beta the Slot 1 (roll) gains were created with that prefix already written into the
 * key, so they published at {@code /Catalyst/Tuning/Catalyst/Tuning/<Name>/Diff/kP} while the
 * mechanisms page, and the Slot 0 gains beside them, said {@code /Catalyst/Tuning/<Name>/...}.
 * Nothing failed: a tunable reads back from whatever path it wrote, so the roll gains hot-reloaded for
 * anyone who found the doubled key and never moved for anyone who typed the documented one.
 *
 * <p>The mechanism is built for real, against the simulated HAL, because the key is assembled in its
 * constructor. A test of a copied string would pass whatever the constructor does.
 */
class DifferentialWristTuningKeysTest {

    private static final String NAME = "TuningKeysWrist";

    @BeforeAll
    static void bootTheHal() {
        assertTrue(HAL.initialize(500, 0), "no HAL, no Phoenix devices");
    }

    @AfterAll
    static void shutDownTheHal() {
        HAL.shutdown();
    }

    @AfterEach
    void tidy() {
        // The registry is global and rejects a second claim on an id.
        CANRegistry.clear();
    }

    @Test
    void rollGainsPublishBesideThePitchGains() {
        // Every Slot 1 value differs from its Slot 0 counterpart, so an entry holding the other
        // slot's gain fails rather than passing by coincidence.
        new DifferentialWristMechanism(DifferentialWristMechanism.Config.builder()
                .name(NAME)
                .leftMotor(50)
                .rightMotor(51)
                .pid(40, 0.1, 0.5)
                .feedforward(0.2, 0.12, 0.01)
                .differentialPid(30, 0.2, 0.3)
                .differentialFeedforward(0.25, 0.13, 0.02)
                .build());

        String diff = "/Catalyst/Tuning/" + NAME + "/Diff/";
        assertPublished(diff + "kP", 30);
        assertPublished(diff + "kI", 0.2);
        assertPublished(diff + "kD", 0.3);
        assertPublished(diff + "kS", 0.25);
        assertPublished(diff + "kV", 0.13);
        assertPublished(diff + "kA", 0.02);

        // The Slot 0 gains were always in the right place; the roll gains belong next to them.
        assertPublished("/Catalyst/Tuning/" + NAME + "/kP", 40);

        assertFalse(entry("/Catalyst/Tuning/Catalyst/Tuning/" + NAME + "/Diff/kP").exists(),
                "the table prefix was applied twice");
    }

    private static NetworkTableEntry entry(String topic) {
        return NetworkTableInstance.getDefault().getEntry(topic);
    }

    private static void assertPublished(String topic, double expected) {
        NetworkTableEntry e = entry(topic);
        assertTrue(e.exists(), topic + " was never published");
        assertEquals(expected, e.getDouble(Double.NaN), 1e-9, topic);
    }
}
