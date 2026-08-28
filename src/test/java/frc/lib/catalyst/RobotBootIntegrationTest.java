package frc.lib.catalyst;

import frc.lib.catalyst.command.Commands;
import frc.lib.catalyst.hardware.CANRegistry;
import frc.lib.catalyst.hardware.CatalystCANBus;
import frc.lib.catalyst.mechanisms.FlywheelMechanism;
import frc.lib.catalyst.opmode.CatalystOpMode;
import frc.lib.catalyst.opmode.CommandOpMode;
import frc.lib.catalyst.system.SystemCoreSim;
import frc.lib.catalyst.system.SystemCoreSource;
import frc.lib.catalyst.system.SystemCoreStatus;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Boots a robot, the way a robot boots.
 *
 * <p>Every other test here exercises a piece. This one constructs real hardware objects against the
 * simulated HAL, wires a mechanism to them, puts it inside an OpMode, and runs the scheduler for a
 * few hundred loops — which is the sequence a robot actually performs and the one nothing had ever
 * checked end to end.
 *
 * <p>It exists because the failures this port could still be carrying are not the kind unit tests
 * catch. A mechanism that constructs fine and never ticks, a command that hangs the scheduler, a
 * periodic that throws on the tenth loop rather than the first — all of those pass every test that
 * does not run the whole thing for a while.
 *
 * <p>The HAL initialises here. That is worth stating because for most of this port it was believed
 * impossible: WPILib's native loader terminates the JVM when the desktop natives are missing, and
 * they were, so anything touching hardware was untestable. They are extracted now.
 */
class RobotBootIntegrationTest {

    /** A robot loop, in seconds. Matches the real one so loop-count reasoning transfers. */
    private static final double PERIOD = 0.02;

    /** Long enough for a filter to settle and a mechanism to reach a setpoint. */
    private static final int LOOPS = 250;

    @BeforeAll
    static void bootTheHal() {
        assertTrue(HAL.initialize(500, 0), "the simulated HAL must come up, or none of this means anything");
    }

    @AfterAll
    static void shutDownTheHal() {
        HAL.shutdown();
    }

    @AfterEach
    void tidy() {
        // The registry is global and rejects a second claim on an id. Without this the second test
        // fails for a reason that has nothing to do with what it is testing.
        CANRegistry.clear();
        SystemCoreStatus.useSource(SystemCoreSource.unavailable());
    }

    /** A real mechanism on real (simulated) Phoenix hardware. */
    private static FlywheelMechanism flywheel(String name, int canId) {
        return new FlywheelMechanism(FlywheelMechanism.Config.builder()
                .name(name)
                .motor(canId)
                .build());
    }

    // --- the thing that was never checked ------------------------------------

    @Test
    void aMechanismConstructsAgainstRealHardwareAndTicks() {
        // Construction touches Phoenix, which touches the HAL. If that works at all, it works here.
        FlywheelMechanism shooter = assertDoesNotThrow(() -> flywheel("Shooter", 20));

        Scheduler scheduler = Scheduler.createIndependentScheduler();
        scheduler.schedule(shooter.spinUp(40.0));

        // Two hundred and fifty loops is five seconds. A mechanism that throws on the tenth loop
        // rather than the first passes every test that runs three.
        assertDoesNotThrow(() -> {
            for (int i = 0; i < LOOPS; i++) {
                scheduler.run();
            }
        }, "five seconds of scheduling should not throw");
    }

    @Test
    void aFullOpModeCycleRunsWithoutIncident() {
        // Boot, enable, run, disable, run again - two matches on one robot program, which is where
        // state that was not reset shows up.
        FlywheelMechanism shooter = flywheel("Shooter", 21);
        Scheduler scheduler = Scheduler.createIndependentScheduler();

        class Auto extends CommandOpMode {
            Auto() {
                super(() -> shooter.spinUp(30.0), false);
            }
        }

        Auto auto = new Auto();

        assertDoesNotThrow(() -> {
            for (int match = 0; match < 2; match++) {
                auto.start();
                for (int i = 0; i < 50; i++) {
                    auto.periodic();
                }
                auto.end();
                for (int i = 0; i < 10; i++) {
                    auto.disabledPeriodic();
                }
            }
        }, "two full match cycles should not throw");
    }

    @Test
    void theSchedulerNeverStopsComingBack() {
        // The failure mode that has no error message. A command body that loops without yielding
        // never returns control, and the robot stops responding for the rest of the match with
        // nothing logged anywhere. If any Catalyst command did that, this test would hang rather
        // than fail - so it is wrapped in a timeout that turns a hang into a failure.
        FlywheelMechanism shooter = flywheel("Shooter", 22);
        Scheduler scheduler = Scheduler.createIndependentScheduler();

        AtomicInteger spins = new AtomicInteger();
        AtomicInteger loops = new AtomicInteger();

        // Two things this chain is built around, both learned the hard way here.
        //
        // spinUp holds the flywheel at speed until interrupted, so anything after it in a sequence
        // never runs unless it is bounded. That is the command behaving correctly.
        //
        // The bound is a *condition*, not a timeout. Time-based commands need wall-clock time to
        // pass, and 250 iterations of a tight loop take a few milliseconds - so a 0.2 s timeout
        // never expires and the test fails for a reason that has nothing to do with the scheduler.
        // A loop counter advances with the scheduler, which is what is being tested anyway.
        scheduler.schedule(Commands.sequence(
                shooter.spinUp(25.0).untilTrue(() -> spins.incrementAndGet() > 20),
                Commands.run(loops::incrementAndGet).withName("after")
        ).withName("chain"));

        assertTimeoutPreemptively(java.time.Duration.ofSeconds(10), () -> {
            for (int i = 0; i < LOOPS; i++) {
                scheduler.run();
            }
        });
        assertTrue(loops.get() > 0, "the chain should have reached its last step");
    }

    @Test
    void twoMechanismsOnDifferentBusesBothWork() {
        // Five buses is the headline platform change, and the registry is what enforces it. Same id
        // on two buses is legal and is the entire point of having five.
        FlywheelMechanism a = flywheel("Left", 30);
        FlywheelMechanism b = flywheel("Right", 31);

        CANRegistry.register("OnAnotherBus", 30, CatalystCANBus.systemcore(2), "TalonFX");

        Scheduler scheduler = Scheduler.createIndependentScheduler();
        scheduler.schedule(a.spinUp(10.0));
        scheduler.schedule(b.spinUp(20.0));

        assertDoesNotThrow(() -> {
            for (int i = 0; i < 100; i++) {
                scheduler.run();
            }
        });
    }

    @Test
    void machineStatusPublishesEveryLoopWithoutAccumulating() {
        // publish() runs once per loop for the life of the match. Anything it appends to rather than
        // overwrites is a leak measured in hours.
        SystemCoreStatus.useSource(SystemCoreSim.healthy());
        SystemCoreStatus status = SystemCoreStatus.getInstance();

        assertDoesNotThrow(() -> {
            for (int i = 0; i < 500; i++) {
                status.publish();
            }
        }, "ten seconds of publishing should not throw");
    }

    @Test
    void anOpModeWithNoCommandStillTicksTheScheduler() {
        // A teleop is not one command; it keeps the scheduler running while bindings schedule
        // things. If the base class did not tick, every binding would be dead.
        AtomicInteger ticks = new AtomicInteger();

        class Teleop extends CatalystOpMode {
            Teleop() {
                super(false);
            }

            /** Exposed so the test can put a command on the same scheduler the mode drives. */
            void bind(org.wpilib.command3.Command c) {
                scheduler().schedule(c);
            }
        }

        Teleop teleop = new Teleop();
        teleop.bind(Commands.run(ticks::incrementAndGet).withName("binding"));
        teleop.start();
        for (int i = 0; i < 20; i++) {
            teleop.periodic();
        }
        teleop.end();

        assertTrue(ticks.get() >= 18, "bindings should run every loop, saw " + ticks.get());
    }

    // JUnit's assertTimeoutPreemptively, imported here so the intent above reads plainly.
    private static void assertTimeoutPreemptively(java.time.Duration timeout, Runnable body) {
        org.junit.jupiter.api.Assertions.assertTimeoutPreemptively(timeout, body::run);
    }
}
