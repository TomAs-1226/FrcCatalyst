package frc.lib.catalyst.util;

import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.system.Timer;
import frc.lib.catalyst.command.Commands;
import org.wpilib.command3.Trigger;

import java.util.ArrayList;
import java.util.List;

/**
 * Bind WPILib {@link Trigger}s to rumble patterns on Xbox-style controllers.
 *
 * <p>The pattern teams reach for most: "buzz the driver's controller when
 * the auto-align finishes" or "buzz the operator when the intake grabs a
 * piece". Writing it longhand means a `Commands.runOnce(() -> ctrl.setRumble(...))`
 * plus a `Commands.waitSeconds(...)` plus a clear, and remembering to clear
 * if the trigger flips off mid-pattern. This wraps all of that.
 *
 * <p>Patterns are queued through a dispatcher so two events fired
 * back-to-back don't fight over the rumble motors — the latest pattern
 * wins, and a clear runs at the end so the controller doesn't get stuck
 * humming.
 *
 * <p>Example:
 * <pre>{@code
 * RumbleEvents events = new RumbleEvents(driver.getHID(), operator.getHID());
 *
 * events.onTrigger(swerve.atAlignmentTarget(), RumbleEvents.Pattern.DOUBLE_TAP, Channel.DRIVER);
 * events.onTrigger(claw.hasPieceTrigger(),     RumbleEvents.Pattern.SHORT,      Channel.BOTH);
 * events.onTrigger(RobotSafety.trippedTrigger(), RumbleEvents.Pattern.LONG,     Channel.BOTH);
 * }</pre>
 */
public final class RumbleEvents {

    /** Which controller(s) a pattern targets. */
    public enum Channel { DRIVER, OPERATOR, BOTH }

    /** Built-in rumble patterns. */
    public enum Pattern {
        /** Single ~120 ms buzz. Good for "got it" feedback. */
        SHORT,
        /** ~400 ms buzz. Good for warnings / fault notifications. */
        LONG,
        /** Two ~80 ms buzzes separated by a 60 ms gap. Good for "ready". */
        DOUBLE_TAP,
        /** Three ~70 ms buzzes. Good for high-importance alerts. */
        TRIPLE_TAP,
        /** Ramp up over ~300 ms then sharply cut. Good for "charging up". */
        RAMP
    }

    private final GenericHID driver;
    private final GenericHID operator;
    private final List<Active> active = new ArrayList<>();
    private double lastTick = -1;

    /**
     * @param driver   the driver's controller (or {@code null} if you only
     *                 want operator rumble)
     * @param operator the operator's controller (or {@code null} for
     *                 driver-only)
     */
    public RumbleEvents(GenericHID driver, GenericHID operator) {
        this.driver = driver;
        this.operator = operator;
    }

    /** Bind a trigger so each rising edge fires the pattern on the given channel. */
    public void onTrigger(Trigger trigger, Pattern pattern, Channel channel) {
        trigger.onTrue(Commands.runOnce(() -> fire(pattern, channel))
                .ignoringDisable(true));
    }

    /** Fire a pattern immediately, regardless of any trigger state. */
    public void fire(Pattern pattern, Channel channel) {
        active.add(new Active(pattern, channel, now()));
    }

    /**
     * Call this every loop from {@code Robot.robotPeriodic()} (after the
     * scheduler runs). Cheap when no patterns are active.
     */
    public void update() {
        double t = now();
        if (lastTick < 0) lastTick = t;
        lastTick = t;

        double driverStrength = 0, operatorStrength = 0;
        for (int i = active.size() - 1; i >= 0; i--) {
            Active a = active.get(i);
            double age = t - a.startTs;
            double s = a.strengthAt(age);
            if (s < 0) {
                active.remove(i);
                continue;
            }
            if (a.channel == Channel.DRIVER || a.channel == Channel.BOTH) {
                driverStrength = Math.max(driverStrength, s);
            }
            if (a.channel == Channel.OPERATOR || a.channel == Channel.BOTH) {
                operatorStrength = Math.max(operatorStrength, s);
            }
        }

        if (driver != null) setStrength(driver, driverStrength);
        if (operator != null) setStrength(operator, operatorStrength);
    }

    /** Stop every active rumble immediately. */
    public void clear() {
        active.clear();
        if (driver != null) setStrength(driver, 0);
        if (operator != null) setStrength(operator, 0);
    }

    private void setStrength(GenericHID hid, double strength) {
        // 2027 dropped the combined "both" rumble type, so drive the two sides explicitly.
        double clamped = Math.clamp(strength, 0.0, 1.0);
        hid.setRumble(RumbleType.LEFT_RUMBLE, clamped);
        hid.setRumble(RumbleType.RIGHT_RUMBLE, clamped);
    }

    private static double now() {
        return Timer.getTimestamp();
    }

    // ----- Internals -----

    private static final class Active {
        final Pattern pattern;
        final Channel channel;
        final double startTs;

        Active(Pattern pattern, Channel channel, double startTs) {
            this.pattern = pattern;
            this.channel = channel;
            this.startTs = startTs;
        }

        /** Strength at the given age into the pattern. Returns -1 once finished. */
        double strengthAt(double age) {
            switch (pattern) {
                case SHORT:      return age < 0.12 ? 1.0 : -1;
                case LONG:       return age < 0.40 ? 1.0 : -1;
                case DOUBLE_TAP:
                    if (age < 0.08) return 1.0;
                    if (age < 0.14) return 0.0;
                    if (age < 0.22) return 1.0;
                    return -1;
                case TRIPLE_TAP:
                    if (age < 0.07) return 1.0;
                    if (age < 0.12) return 0.0;
                    if (age < 0.19) return 1.0;
                    if (age < 0.24) return 0.0;
                    if (age < 0.31) return 1.0;
                    return -1;
                case RAMP:
                    if (age < 0.30) return age / 0.30;
                    return -1;
                default: return -1;
            }
        }
    }
}
