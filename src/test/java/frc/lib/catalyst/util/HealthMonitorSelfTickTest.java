package frc.lib.catalyst.util;

import frc.lib.catalyst.opmode.CatalystOpMode;
import frc.lib.catalyst.system.SystemCoreSim;
import frc.lib.catalyst.system.SystemCoreSource;
import frc.lib.catalyst.system.SystemCoreStatus;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * That the health kit evaluates on a robot that owns no mechanism.
 *
 * <p>It did not. {@link HealthMonitor#update()} is the only evaluator of every registered check, the
 * only writer of the health topics, the only feeder of {@code HealthHistory} and the only caller of
 * {@code RobotSafety.tick()} — and its only callers were the nine {@code updateTelemetry()} bodies
 * inside the mechanism classes. A robot of swerve, vision and the team's own subsystems owns no
 * {@code CatalystMechanism}, so nothing ever called it: every check stayed un-evaluated, the
 * dashboard showed a healthy robot, and a watchdog a team had configured to stop the robot sat there
 * having never been asked. The discriminating variable was precisely "does this robot own a
 * mechanism", so these tests deliberately build none.
 *
 * <p>The first fix for it was worse than it looked: {@code Scheduler.addPeriodic} stamps its
 * callback with an opmode-scoped {@code BindingScope}, and {@code runPeriodicSideloads} removes any
 * callback whose scope has gone inactive — so a monitor installed during autonomous would stop
 * ticking at the switch to teleop, silently. Driving it from the OpMode's own periodic cannot be
 * reaped, which is what these tests pin.
 */
class HealthMonitorSelfTickTest {

    /** A robot with no mechanism at all. That is the whole point. */
    private static final class BareOpMode extends CatalystOpMode {
        BareOpMode() {
            super(false);
        }
    }

    @BeforeEach
    void setUp() {
        HealthMonitor.getInstance().clear();
        SystemCoreStatus.useSource(SystemCoreSim.healthy().withAvailable(false));
        assertTrue(HAL.initialize(500, 0), "no HAL, no Driver Station state");
    }

    @AfterEach
    void tidy() {
        HealthMonitor.getInstance().clear();
        SystemCoreStatus.useSource(SystemCoreSource.unavailable());
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    private static void enabled(boolean on) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(on);
        DriverStationSim.notifyNewData();
    }

    /**
     * Tick the mode, letting real time pass.
     *
     * <p>The sleep is not padding. {@code update()} is throttled to 5 ms on purpose — it is called
     * from nine mechanism telemetry bodies as well as from here, and re-evaluating every check
     * several times a loop is waste. The throttle is state on the singleton, so a sibling test can
     * consume the window and a single tight tick then evaluates nothing. A robot never notices,
     * because its loop is 20 ms.
     */
    private static void tick(CatalystOpMode mode, boolean whileEnabled, int n) {
        for (int i = 0; i <= n; i++) {
            if (whileEnabled) {
                mode.periodic();
            } else {
                mode.disabledPeriodic();
            }
            try {
                Thread.sleep(8);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    @Test
    void aMechanismLessRobotStillEvaluatesItsChecks() {
        enabled(true);
        AtomicBoolean asked = new AtomicBoolean(false);

        HealthCheck.builder("CAN", "BusDown")
                .severity(HealthCheck.Severity.ERROR)
                .description("the bus stopped reporting")
                .debounce(0)
                .when(() -> {
                    asked.set(true);
                    return false;
                })
                .register();

        tick(new BareOpMode(), true, 3);

        assertTrue(asked.get(),
                "nothing evaluated the check on a robot that owns no CatalystMechanism");
    }

    @Test
    void aDeadBusActuallyFires() {
        enabled(true);
        AtomicBoolean down = new AtomicBoolean(false);

        HealthCheck check = HealthCheck.builder("CAN", "BusDown2")
                .severity(HealthCheck.Severity.ERROR)
                .description("the bus stopped reporting")
                .debounce(0)
                .when(down::get)
                .register();

        BareOpMode mode = new BareOpMode();
        tick(mode, true, 2);
        assertFalse(check.isFiring(), "a healthy bus must not fire");

        down.set(true);
        tick(mode, true, 3);

        assertTrue(check.isFiring(), "a dead CAN bus went unreported on a mechanism-less robot");
    }

    @Test
    void healthIsAlsoEvaluatedWhileDisabled() {
        // Between matches is exactly when a pit crew reads this to decide whether the robot goes
        // back out, and it is the window periodic() now declines - so disabledPeriodic() has to
        // carry it too, or health goes dark precisely when someone is looking at it.
        enabled(false);
        AtomicBoolean asked = new AtomicBoolean(false);

        HealthCheck.builder("CAN", "BusDown3")
                .severity(HealthCheck.Severity.ERROR)
                .description("the bus stopped reporting")
                .debounce(0)
                .when(() -> {
                    asked.set(true);
                    return false;
                })
                .register();

        tick(new BareOpMode(), false, 3);

        assertTrue(asked.get(), "health stopped being evaluated while disabled");
    }
}
