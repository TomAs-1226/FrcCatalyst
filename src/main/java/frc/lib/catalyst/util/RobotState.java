package frc.lib.catalyst.util;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.command3.Trigger;

import java.util.Optional;

/**
 * Snapshot of "what's the robot doing right now?" in one place.
 *
 * <p>Wraps the assorted {@link DriverStation} and {@link RobotController}
 * calls every subsystem ends up making — alliance color, match time,
 * enable state, mode, battery voltage. Cached for the lifetime of a
 * scheduler tick so re-reading is cheap.
 *
 * <p>{@link RobotState} doesn't need explicit installation — every read
 * is a static call that refreshes the cache if the last refresh was
 * more than {@code 5 ms} ago. To force a refresh (e.g. between scheduler
 * ticks in a unit test) call {@link #refresh()}.
 *
 * <p>Example:
 * <pre>{@code
 * if (RobotState.isAutonomous()) { ... }
 * if (RobotState.alliance() == Alliance.RED) { ... }
 * if (RobotState.matchTimeRemaining() < 10) { climber.runDefault(); }
 *
 * // As a Trigger:
 * RobotState.lateMatch(20).onTrue(forcedClimbCommand);
 * }</pre>
 */
public final class RobotState {

    private static final double STALE_AFTER_S = 0.005;

    // Cached fields — refreshed lazily, no thread-safety story here
    // because every consumer runs on the main scheduler thread anyway.
    private static double lastRefresh = -1;
    private static boolean isAuto, isTele, isTest, isDisabled, isEStopped, isDsAttached;
    private static Alliance alliance = null;
    private static int stationLocation = 1;
    private static double matchTime = -1;
    private static double batteryVolts;
    private static double enabledTimestamp = -1;

    private RobotState() {}

    /**
     * Force a refresh. Normal callers don't need this — every read
     * auto-refreshes if more than 5 ms have elapsed.
     */
    public static void refresh() {
        // DriverStation.refreshData() was removed in 2027 - the DS cache refreshes itself.
        isAuto       = org.wpilib.driverstation.RobotState.isAutonomous();
        isTele       = org.wpilib.driverstation.RobotState.isTeleop();
        isTest       = org.wpilib.driverstation.RobotState.isUtility();
        isDisabled   = org.wpilib.driverstation.RobotState.isDisabled();
        isEStopped   = org.wpilib.driverstation.RobotState.isEStopped();
        isDsAttached = org.wpilib.driverstation.RobotState.isDSAttached();
        alliance     = MatchState.getAlliance().orElse(null);
        stationLocation = MatchState.getLocation().orElse(1);
        matchTime    = MatchState.getMatchTime();
        batteryVolts = RobotController.getBatteryVoltage();
        if (!isDisabled && enabledTimestamp < 0) {
            enabledTimestamp = Timer.getTimestamp();
        } else if (isDisabled) {
            enabledTimestamp = -1;
        }
        lastRefresh = Timer.getTimestamp();
    }

    private static void maybeRefresh() {
        double t = Timer.getTimestamp();
        if (lastRefresh < 0 || (t - lastRefresh) > STALE_AFTER_S) refresh();
    }

    public static boolean isAutonomous()  { maybeRefresh(); return isAuto; }
    public static boolean isTeleop()      { maybeRefresh(); return isTele; }
    public static boolean isTest()        { maybeRefresh(); return isTest; }
    public static boolean isDisabled()    { maybeRefresh(); return isDisabled; }
    public static boolean isEnabled()     { maybeRefresh(); return !isDisabled; }
    public static boolean isEStopped()    { maybeRefresh(); return isEStopped; }
    public static boolean isDsAttached()  { maybeRefresh(); return isDsAttached; }

    /** Alliance reported by the FMS / Driver Station. Empty until the DS connects. */
    public static Optional<Alliance> allianceOpt() {
        maybeRefresh();
        return Optional.ofNullable(alliance);
    }

    /** Alliance, or {@link Alliance#BLUE} as a safe default before the DS connects. */
    public static Alliance alliance() {
        maybeRefresh();
        return alliance == null ? Alliance.BLUE : alliance;
    }

    /** True if the alliance is red. False before DS connects. */
    public static boolean isRed()  { return alliance() == Alliance.RED; }
    /** True if the alliance is blue. */
    public static boolean isBlue() { return alliance() == Alliance.BLUE; }

    /** Driver station position 1, 2 or 3. */
    public static int stationLocation() { maybeRefresh(); return stationLocation; }

    /** Match time remaining in seconds. {@code -1} when not in a real match. */
    public static double matchTimeRemaining() { maybeRefresh(); return matchTime; }

    /** Battery voltage from the PDH/PDP. */
    public static double batteryVoltage() { maybeRefresh(); return batteryVolts; }

    /** Seconds since the robot was last enabled. {@code 0} while disabled. */
    public static double timeSinceEnable() {
        maybeRefresh();
        if (enabledTimestamp < 0) return 0;
        return Timer.getTimestamp() - enabledTimestamp;
    }

    // ===========================================
    //                 TRIGGERS
    // ===========================================

    /** Trigger that fires the entire time the robot is in autonomous mode. */
    public static Trigger autonomous() { return new Trigger(RobotState::isAutonomous); }
    /** Trigger that fires during teleop. */
    public static Trigger teleop()     { return new Trigger(RobotState::isTeleop); }
    /** Trigger that fires while disabled. */
    public static Trigger disabled()   { return new Trigger(RobotState::isDisabled); }
    /** Trigger that fires while enabled (auto, teleop, or test). */
    public static Trigger enabled()    { return new Trigger(RobotState::isEnabled); }

    /**
     * Trigger that fires when match time drops below {@code seconds}. Useful
     * for end-game routines:
     *
     * <pre>{@code
     * RobotState.lateMatch(20).onTrue(climber.deployCommand());
     * }</pre>
     */
    public static Trigger lateMatch(double seconds) {
        return new Trigger(() -> {
            double t = matchTimeRemaining();
            return t >= 0 && t <= seconds;
        });
    }

    /**
     * Trigger that fires when battery voltage drops below {@code volts}. Pair
     * with {@link RobotSafety} to back off aggressive draws before brownout.
     */
    public static Trigger lowBattery(double volts) {
        return new Trigger(() -> batteryVoltage() < volts);
    }
}
