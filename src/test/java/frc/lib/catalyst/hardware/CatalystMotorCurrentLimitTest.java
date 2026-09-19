package frc.lib.catalyst.hardware;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Changing a motor's current limits at runtime - state-based power budgeting, a turbo button - reaches every motor
 * on the mechanism. The builder gives the followers the leader's limits at boot; a runtime change that wrote the
 * leader alone left a follower drawing its boot limit through every state, so a pair "cut to 5 A" drew 5 A on one
 * motor and the old limit on the other.
 */
class CatalystMotorCurrentLimitTest {

    @AfterEach
    void clear() {
        CANRegistry.clear();
    }

    private static CurrentLimitsConfigs read(TalonFX motor) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        assertTrue(motor.getConfigurator().refresh(limits).isOK(), "id " + motor.getDeviceID() + " unread");
        return limits;
    }

    @Test
    void aRuntimeSupplyLimitReachesEveryFollowerAndKeepsTheStatorLimit() {
        assertTrue(HAL.initialize(), "no HAL, no Phoenix devices");
        CatalystMotor motor = CatalystMotor.builder(41).name("Pair").canBus("can_s0")
                .withFollower(42, true).withFollower(43, false)
                .currentLimit(20).statorCurrentLimit(60).build();

        motor.setSupplyCurrentLimit(35);

        assertEquals(35.0, read(motor.getTalonFX()).SupplyCurrentLimit, 1e-9);
        for (TalonFX follower : motor.getFollowerTalonFXs()) {
            CurrentLimitsConfigs limits = read(follower);
            assertEquals(35.0, limits.SupplyCurrentLimit, 1e-9, "follower " + follower.getDeviceID());
            assertEquals(60.0, limits.StatorCurrentLimit, 1e-9, "follower " + follower.getDeviceID());
        }
        assertEquals(35.0, motor.getSupplyCurrentLimit(), 0.0);
        assertEquals(60.0, motor.getStatorCurrentLimit(), 0.0);
    }

    @Test
    void aRuntimeStatorLimitReachesEveryFollowerAndKeepsTheSupplyLimit() {
        assertTrue(HAL.initialize(), "no HAL, no Phoenix devices");
        CatalystMotor motor = CatalystMotor.builder(44).name("Pair2").canBus("can_s0")
                .withFollower(45, false)
                .currentLimit(20).statorCurrentLimit(60).build();

        motor.setCurrentLimits(25, 80);
        motor.setStatorCurrentLimit(70);

        CurrentLimitsConfigs follower = read(motor.getFollowerTalonFX());
        assertEquals(25.0, follower.SupplyCurrentLimit, 1e-9);
        assertEquals(70.0, follower.StatorCurrentLimit, 1e-9);
    }
}
