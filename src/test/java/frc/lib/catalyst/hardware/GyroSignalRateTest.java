package frc.lib.catalyst.hardware;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The Pigeon's signal rates, and the limit of what a desktop test can say about them.
 *
 * <h2>What the fix is for</h2>
 *
 * <p>Phoenix's defaults are not uniform, and the two signals this library reads every loop are the
 * slow ones. From CTRE's own source for the Pigeon 2, on CAN 2.0 — which is every Systemcore bus and
 * the roboRIO's:
 *
 * <ul>
 *   <li>{@code Yaw}, {@code Pitch}, {@code Roll} — 100 Hz
 *   <li>{@code AngularVelocityZWorld} — <b>10 Hz</b>
 *   <li>{@code AccelerationX/Y} — <b>10 Hz</b>
 * </ul>
 *
 * <p>Left alone, {@link CatalystGyro#getYawRate()} is read at 50 Hz and answers with a value that
 * can be 100 ms old — five loops of a heading controller's derivative term acting on a number that
 * has not moved. Acceleration is worse, because {@link DualIMU} differences it against Systemcore's
 * own IMU sampled now: under a hard turn that gap is not two sensors disagreeing, it is one of them
 * being a tenth of a second behind. Nothing about it fails; it produces a plausible number, late.
 *
 * <h2>Why there is no test that the default was raised</h2>
 *
 * <p>There was one, and it passed with the fix removed. Simulation reports
 * {@code getAppliedUpdateFrequency()} as 100.0 for <em>every</em> signal, including the ones a real
 * device sends at 10 Hz — measured, by asking an untouched simulated Pigeon. The 10 Hz default is
 * device firmware behaviour that simulation does not model, so a desktop assertion that "the rate is
 * now 100" holds whether or not the code does anything at all.
 *
 * <p>Deleting that test rather than keeping it is the point. A test that cannot fail is worse than
 * no test, because it reads as coverage. What survives is the part simulation can genuinely
 * distinguish — that an explicitly requested rate is the rate that gets applied — and the default
 * itself has to be confirmed against hardware.
 */
class GyroSignalRateTest {

    @AfterEach
    void clear() {
        CANRegistry.clear();
    }

    @Test
    void anExplicitRateIsTheRateThatGetsApplied() {
        assertTrue(HAL.initialize(), "no HAL, no Phoenix devices");

        // 50, not 100: simulation's default is already 100, so asserting 100 would prove nothing.
        // Asking for something the simulator would not have chosen is what makes this a test.
        CatalystGyro gyro = new CatalystGyro(24, "can_s0").withSignalRate(50.0);
        var pigeon = gyro.getPigeon();

        for (var signal : new com.ctre.phoenix6.BaseStatusSignal[] {
                pigeon.getAngularVelocityZWorld(),
                pigeon.getAccelerationX(),
                pigeon.getAccelerationY(),
        }) {
            assertEquals(50.0, signal.getAppliedUpdateFrequency(), 1e-6,
                    signal.getName() + " did not take the requested rate");
        }
    }

    @Test
    void everySignalCatalystReadsIsCovered() {
        assertTrue(HAL.initialize(), "no HAL, no Phoenix devices");

        // The rate call names signals one at a time, so a reading added to CatalystGyro later can
        // quietly keep the 10 Hz default. Setting an unusual rate and checking all three come back
        // with it is what catches a fourth signal being read but never listed.
        CatalystGyro gyro = new CatalystGyro(25, "can_s0").withSignalRate(37.0);
        var pigeon = gyro.getPigeon();

        // 37 Hz comes back as 37.037, and that is Phoenix rather than a rounding error here: it
        // stores a period in whole milliseconds, so 1/37 = 27.027 ms becomes 27 ms, which is
        // 37.037 Hz. Rates whose period is a whole number of milliseconds - 100, 50, 20 - come back
        // exactly; nothing else does. The tolerance is one millisecond of period at this rate.
        double tolerance = 37.0 * 37.0 / 1000.0;
        assertEquals(37.0, pigeon.getAngularVelocityZWorld().getAppliedUpdateFrequency(), tolerance);
        assertEquals(37.0, pigeon.getAccelerationX().getAppliedUpdateFrequency(), tolerance);
        assertEquals(37.0, pigeon.getAccelerationY().getAppliedUpdateFrequency(), tolerance);
    }

    @Test
    void nothingElseOnTheDeviceIsSilenced() {
        assertTrue(HAL.initialize(), "no HAL, no Phoenix devices");

        // Deliberately no optimizeBusUtilization() alongside the rate change. That call silences
        // every signal nobody explicitly asked for, which would be a quiet trap for a team reading
        // something else off getPigeon() - their reading would stop updating with nothing to say
        // why. A motor is closed enough for Catalyst to make that call for it; a gyro a team also
        // talks to directly is not.
        CatalystGyro gyro = new CatalystGyro(26, "can_s0").withSignalRate(50.0);

        assertTrue(gyro.getPigeon().getTemperature().getAppliedUpdateFrequency() > 0.0,
                "a signal Catalyst does not read should still be arriving, not switched off");
    }
}
