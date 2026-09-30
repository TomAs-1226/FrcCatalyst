package frc.lib.catalyst.util;

import frc.lib.catalyst.hardware.CANBusHealth;
import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;

/**
 * "Is this robot all right?" in one topic, and the match clock in another.
 *
 * <h2>The rollup</h2>
 *
 * <p>Both shipped Catalyst dashboards answer the same headline question — battery, loop time, CAN
 * load, alerts, preflight — and both reconstruct it from about fifteen separate subscriptions. Catalyst
 * Tab does it across six reader functions; Catalyst Console does it across as many tile definitions.
 * Every new dashboard, and every team's own, has to do it again, and each one gets the thresholds
 * slightly differently, which is how two screens in the same pit end up disagreeing about whether the
 * robot is fine.
 *
 * <p>This publishes the conclusion, as JSON at {@code /Catalyst/Tablet/Summary}:
 *
 * <pre>
 * {"ok":true,"battery":12.4,"worstAlert":"","canWorst":0.31,"canWorstBus":"can_s0",
 *  "preflightReady":true,"errors":0,"warnings":1}
 * </pre>
 *
 * <p>It is a reduction of numbers the library already computes, so it costs one string write per loop
 * and nothing else. It does not replace the individual topics — a dashboard that wants to draw a CAN
 * bus still reads that bus — it replaces the *judgement*, so that the judgement is made in one place
 * and agrees with itself everywhere.
 *
 * <h2>The match clock</h2>
 *
 * <p>{@code /Catalyst/Match/TimeLeft} is the other thing here, and it is a smaller story with the same
 * shape. Both dashboards read it, and nothing has ever published it: on the tablet it is the first of
 * three fallbacks and on Console it is a tile's default topic. The FMS number is available to the robot
 * for free, so the topic being empty was an oversight rather than a decision. One line fills it.
 *
 * <h2>Thresholds</h2>
 *
 * <p>The ones the dashboards already use, so the rollup cannot disagree with the screen beside it:
 * battery below 12.2 V wants a swap, below 11.8 V fails; a CAN bus above 0.70 warns and above 0.85
 * faults. {@code ok} is false if anything is at fault level or any error alert is up.
 *
 * <h2>Use</h2>
 *
 * <pre>{@code
 * @Override
 * public void robotPeriodic() {
 *     BatteryMonitor.update();
 *     RobotSummary.update();
 * }
 * }</pre>
 *
 * @since 2.0.0
 */
public final class RobotSummary {

    private RobotSummary() {}

    /** A pack at or above this is charged enough for a match. */
    public static final double BATTERY_SWAP_VOLTS = 12.2;

    /** Below this, the robot should not be on the field. */
    public static final double BATTERY_FAIL_VOLTS = 11.8;

    /** A bus above this fraction warns. */
    public static final double CAN_WARN = 0.70;

    /** A bus above this fraction faults. */
    public static final double CAN_FAULT = 0.85;

    /** The rollup topic, without the {@code /Catalyst/} root. */
    public static final String SUMMARY_KEY = "Tablet/Summary";

    /** The match clock topic, without the {@code /Catalyst/} root. */
    public static final String MATCH_KEY = "Match/TimeLeft";

    /** The rollup's rate. A headline for a human does not need 50 Hz; see {@link #update()}. */
    private static final double SUMMARY_PERIOD_S = 0.25;

    private static double lastSummaryAt = Double.NEGATIVE_INFINITY;

    /**
     * Publish the rollup and the match clock. Call once per loop.
     *
     * <p><b>The rollup is rate-limited to 4 Hz and the match clock is not.</b> That split is the whole
     * subtlety of this method, and it is not a micro-optimisation.
     *
     * <p>The claim this class shipped with — "nothing here polls hardware that was not already being
     * polled" — was wrong. {@link CANBusHealth#readAll()} turned out to have no other caller, and each
     * call asks Phoenix for a status per registered bus. On a Systemcore with five buses, at 50 Hz,
     * that is 250 CAN status queries a second spent filling one field of a summary that a person reads
     * a few times a minute. Four times a second is imperceptible on a dashboard and a twelfth of the
     * traffic.
     *
     * <p>The match clock stays at loop rate because it costs a field read from {@link RobotState} and a
     * countdown that updates four times a second looks like it is stuttering.
     */
    public static void update() {
        double matchTime = RobotState.matchTimeRemaining();
        // WPILib reports -1 when there is no match clock — practice, or the bench. A dashboard drawing
        // a countdown from -1 looks broken, and "absent" is the honest answer, so it is not published
        // at all rather than published as a negative number or as zero. Catalyst's rule throughout is
        // that a value the robot does not have shows as absent, never as 0.
        if (matchTime >= 0) {
            CatalystLog.log(MATCH_KEY, matchTime);
        }

        double now = Timer.getTimestamp();
        if (now - lastSummaryAt < SUMMARY_PERIOD_S) {
            return;
        }
        lastSummaryAt = now;
        CatalystLog.log(SUMMARY_KEY, json());
    }

    /** The rollup as JSON, without publishing it. Exposed for tests and for a team's own dashboard. */
    public static String json() {
        double battery = RobotController.getBatteryVoltage();

        AlertManager alerts = AlertManager.getInstance();
        java.util.List<String> errorList = alerts.getErrors();
        java.util.List<String> warningList = alerts.getWarnings();
        int errors = errorList.size();
        int warnings = warningList.size();
        String worstAlert = firstOrEmpty(errorList);
        if (worstAlert.isEmpty()) {
            worstAlert = firstOrEmpty(warningList);
        }

        double canWorst = 0;
        String canWorstBus = "";
        for (CANBusHealth.BusStatus bus : CANBusHealth.readAll()) {
            if (bus.utilization() > canWorst) {
                canWorst = bus.utilization();
                canWorstBus = bus.bus();
            }
        }

        // Empty when preflight has never run, which is not the same as "not ready" and must not be
        // reported as it: a robot mid-match has a stale preflight by definition, and a dashboard that
        // draws "preflight: failed" because nobody ran one is worse than one that draws nothing.
        java.util.Optional<Boolean> preflightReady = Preflight.lastReady();

        boolean ok = errors == 0
                && battery >= BATTERY_FAIL_VOLTS
                && canWorst <= CAN_FAULT
                && preflightReady.orElse(true);

        StringBuilder sb = new StringBuilder(160);
        sb.append("{\"ok\":").append(ok)
                .append(",\"battery\":").append(round(battery, 2))
                .append(",\"batteryWantsSwap\":").append(battery < BATTERY_SWAP_VOLTS)
                .append(",\"errors\":").append(errors)
                .append(",\"warnings\":").append(warnings)
                .append(",\"worstAlert\":").append(quote(worstAlert))
                .append(",\"canWorst\":").append(round(canWorst, 3))
                .append(",\"canWorstBus\":").append(quote(canWorstBus));
        preflightReady.ifPresent(r -> sb.append(",\"preflightReady\":").append(r.booleanValue()));
        sb.append('}');
        return sb.toString();
    }

    private static String firstOrEmpty(java.util.List<String> values) {
        return values == null || values.isEmpty() ? "" : values.get(0);
    }

    private static String round(double v, int places) {
        if (!Double.isFinite(v)) {
            return "null";
        }
        double factor = Math.pow(10, places);
        return Double.toString(Math.round(v * factor) / factor);
    }

    private static String quote(String s) {
        StringBuilder sb = new StringBuilder(s.length() + 2).append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"' -> sb.append("\\\"");
                case '\\' -> sb.append("\\\\");
                case '\n' -> sb.append("\\n");
                case '\r' -> sb.append("\\r");
                case '\t' -> sb.append("\\t");
                default -> {
                    if (c < 0x20) {
                        sb.append(String.format("\\u%04x", (int) c));
                    } else {
                        sb.append(c);
                    }
                }
            }
        }
        return sb.append('"').toString();
    }
}
