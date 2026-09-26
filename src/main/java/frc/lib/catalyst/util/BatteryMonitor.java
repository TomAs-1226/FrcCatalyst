package frc.lib.catalyst.util;

import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.system.RobotController;

/**
 * Battery voltage on NetworkTables, with no setup at all.
 *
 * <h2>Why this class exists</h2>
 *
 * <p>Until this was added, <b>no Catalyst class published a battery voltage that existed on every
 * robot.</b> There were three paths to one and each needed something:
 *
 * <ul>
 *   <li>{@code /Catalyst/Systemcore/BatteryVolts} — a Systemcore, plus a call to
 *       {@code HealthMonitor.systemCoreChecks()}.
 *   <li>{@code /Catalyst/Brownout/MeasuredVoltage} — a {@link BrownoutMonitor} built and
 *       {@code update()}d every loop.
 *   <li>{@code /Catalyst/Status/BatteryVolts} — not a library topic at all. It was a line in the
 *       example robot's own {@code RobotContainer}, which is why every dashboard reads it first and
 *       why it was never there.
 * </ul>
 *
 * <p>So a robot that skipped both opt-ins published no battery voltage, and the single most
 * important number in a pit showed as "not published" for the whole match — on Catalyst Tab and on
 * Catalyst Console alike, since both read the same three keys in the same order. WPILib does not put
 * battery voltage on NetworkTables by itself, so nothing else filled the gap.
 *
 * <p>This publishes the key the dashboards already look for first, so nothing downstream changes:
 * the topic simply starts existing.
 *
 * <h2>Use</h2>
 *
 * <p>One line, once per loop:
 *
 * <pre>{@code
 * @Override
 * public void robotPeriodic() {
 *     BatteryMonitor.update();
 *     // ...
 * }
 * }</pre>
 *
 * <p>There is nothing to construct and nothing to configure, on purpose. A class that needed a
 * builder would be the fourth opt-in on a list of three that were each skipped often enough to be
 * the reason this exists.
 *
 * <h2>What it does not do</h2>
 *
 * <p>It reports, and nothing else. It does not predict a sag, throttle output or trip anything —
 * {@link BrownoutMonitor} does all of that and is still the right class for it, with a real internal
 * resistance and a real current measurement behind it. This one answers "what is the pack at,"
 * which is the question a dashboard asks and the one nothing was answering.
 *
 * @since 2.0.0
 */
public final class BatteryMonitor {

    private BatteryMonitor() {}

    /** The key both Catalyst Console and Catalyst Tab try first. */
    private static final String KEY = "Status/BatteryVolts";

    /**
     * Publish the battery voltage. Call once per loop, from {@code robotPeriodic()}.
     *
     * <p>Goes through {@link CatalystLog}, so a robot that has installed a {@code WpilogSink} gets
     * the voltage in its log beside everything else rather than only on NetworkTables — which is
     * what makes a post-match "was this a brownout?" answerable.
     */
    public static void update() {
        CatalystLog.log(KEY, RobotController.getBatteryVoltage());
    }

    /**
     * Publish a voltage measured somewhere else — a PDH's own voltage channel, say, which is a
     * better number than the controller's input on a robot wired through one.
     *
     * @param volts the pack voltage to report
     */
    public static void update(double volts) {
        CatalystLog.log(KEY, volts);
    }

    /** The NetworkTables key this publishes, without the {@code /Catalyst/} root. */
    public static String key() {
        return KEY;
    }
}
