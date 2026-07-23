package frc.lib.catalyst.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The internal Phoenix sim thread must get out of the way when an external physics engine takes
 * over. Both write the same module rotor states, and at 200 Hz {@code updateSimState()} wins — the
 * external physics stops reaching the robot, silently.
 *
 * <p>These tests stay away from {@link edu.wpi.first.wpilibj.Notifier} and the HAL on purpose: a
 * real {@link SwerveSubsystem} needs a Phoenix {@code SwerveDrivetrain}, which needs CAN hardware,
 * and {@code Notifier} needs the HAL JNI. Neither is available in a plain unit test. The behaviour
 * that matters here is the yield bookkeeping, which needs neither.
 */
class SwerveSimYieldTest {
  @Test
  void disableInternalSimIsIdempotentAndSafeWithNoThread() throws Exception {
    var subsystem = allocateWithoutConstructor();

    subsystem.disableInternalSim();
    assertTrue(readYieldedFlag(subsystem), "disableInternalSim should record the yield");
    assertFalse(subsystem.isInternalSimRunning());

    // Twice must not throw, including when there was never a thread to stop.
    subsystem.disableInternalSim();
    assertTrue(readYieldedFlag(subsystem));
  }

  @Test
  void startSimThreadRespectsAnEarlierYield() throws Exception {
    var subsystem = allocateWithoutConstructor();
    subsystem.disableInternalSim();

    // Without the guard this constructs a Notifier and starts it — which would both revive the
    // conflict and, here, blow up on the missing HAL. Reaching the end proves the guard held.
    var start = SwerveSubsystem.class.getDeclaredMethod("startSimThread");
    start.setAccessible(true);
    start.invoke(subsystem);

    assertFalse(
        subsystem.isInternalSimRunning(),
        "startSimThread must not revive the internal sim after it has yielded");
  }

  @Test
  void isInternalSimRunningReportsFalseOnceYielded() throws Exception {
    var subsystem = allocateWithoutConstructor();
    assertFalse(subsystem.isInternalSimRunning(), "no thread was ever started");

    subsystem.disableInternalSim();
    assertFalse(subsystem.isInternalSimRunning());
  }

  /** Builds a SwerveSubsystem without running its constructor, which would need CAN hardware. */
  private static SwerveSubsystem allocateWithoutConstructor() throws Exception {
    var unsafeField = sun.misc.Unsafe.class.getDeclaredField("theUnsafe");
    unsafeField.setAccessible(true);
    var unsafe = (sun.misc.Unsafe) unsafeField.get(null);
    return (SwerveSubsystem) unsafe.allocateInstance(SwerveSubsystem.class);
  }

  private static boolean readYieldedFlag(SwerveSubsystem subsystem) throws Exception {
    var f = SwerveSubsystem.class.getDeclaredField("internalSimYielded");
    f.setAccessible(true);
    return f.getBoolean(subsystem);
  }
}
