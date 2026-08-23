package frc.lib.catalyst.driverstation;

import frc.lib.catalyst.system.SystemCoreStatus;
import frc.lib.catalyst.util.HealthCheck;
import frc.lib.catalyst.util.HealthMonitor;

import org.wpilib.command3.Scheduler;

import org.wpilib.driverstation.DriverStationDisplay;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.function.Supplier;

/**
 * Catalyst's status, written to the Driver Station itself.
 *
 * <h2>Why this exists</h2>
 *
 * <p>Catalyst reports a great deal, and all of it lands somewhere the driver is not looking. Console
 * is on a second screen or a second laptop; AdvantageScope is for afterwards; the health dashboard is
 * a browser tab. In the thirty seconds before a match the driver is looking at exactly one thing —
 * the Driver Station — and until 2027 there was no way to put anything there.
 *
 * <p>{@link DriverStationDisplay} is that way. This puts the handful of facts worth interrupting
 * someone with onto it, and nothing else.
 *
 * <pre>{@code
 * DriverBoard.standard()          // health, battery, machine
 *     .line("Auto", () -> selectedAutoName)
 *     .start();                   // updates itself from then on
 * }</pre>
 *
 * <h2>What goes on it</h2>
 *
 * <p>Deliberately little. A display that scrolls is a display nobody reads, and this one competes
 * with a match for attention. The rule used here: a line earns its place if a driver would do
 * something differently because of it in the next minute.
 *
 * <p>So: whether anything is wrong and what, battery, and the mode. Not loop times, not CAN
 * utilisation, not the twelve mechanism states — those matter, and they matter in the pit, on
 * Console, where there is time to read them.
 *
 * <p>Failing health checks are shown because they are the case where a driver <em>can</em> act: a
 * match not started is a match that can be fixed.
 *
 * @since 2.0.0
 */
public final class DriverBoard {

    /** A labelled value that is recomputed each update. */
    private record Line(String key, Supplier<String> value) {
    }

    /**
     * How many failing checks to name before summarising.
     *
     * <p>Three is about what fits without the display becoming a list. Past that the count is the
     * useful fact anyway — a robot with nine problems has one problem, and it is not any of the
     * nine.
     */
    private static final int MAX_PROBLEMS_SHOWN = 3;

    private final List<Line> lines = new ArrayList<>();
    private boolean started;

    private DriverBoard() {
    }

    /** An empty board. Add what you want on it. */
    public static DriverBoard empty() {
        return new DriverBoard();
    }

    /**
     * The board most robots want: what is wrong, the battery, and the machine.
     *
     * <p>Each of these is on it because a driver would act on it before the match rather than after.
     */
    public static DriverBoard standard() {
        return empty()
                .line("Status", DriverBoard::healthSummary)
                .line("Battery", DriverBoard::batterySummary);
    }

    /**
     * Add a line.
     *
     * @param key   the label, shown as given
     * @param value recomputed each update, so it can read live state
     */
    public DriverBoard line(String key, Supplier<String> value) {
        lines.add(new Line(key, value));
        return this;
    }

    /** Add a line that never changes. */
    public DriverBoard line(String key, String value) {
        return line(key, () -> value);
    }

    /**
     * Start updating the Driver Station, and keep updating it.
     *
     * <p>Registers a periodic callback on the command scheduler, so nothing needs calling from robot
     * code afterwards. Calling it twice is harmless — the second call does nothing rather than
     * registering a second callback that would write every line twice.
     *
     * <p>If you would rather drive it yourself, skip this and call {@link #update()} each loop.
     */
    public DriverBoard start() {
        return start(Scheduler.getDefault());
    }

    /**
     * As {@link #start()}, on a scheduler of your choosing.
     *
     * @param scheduler where the periodic callback is registered
     */
    public DriverBoard start(Scheduler scheduler) {
        if (started) {
            return this;
        }
        started = true;
        DriverStationDisplay.setMode(DriverStationDisplay.Mode.Line);
        scheduler.addPeriodic(this::update);
        return this;
    }

    /** Whether {@link #start()} has been called and the board is updating itself. */
    public boolean isStarted() {
        return started;
    }

    /**
     * Write the current values.
     *
     * <p>Call once per loop, or let {@link #start()} arrange it. Cheap: the display batches lines and
     * sends them on {@code updateLines()}.
     */
    public void update() {
        for (Line line : lines) {
            DriverStationDisplay.addData(line.key(), valueOf(line.key()));
        }
        DriverStationDisplay.updateLines();
    }

    /** How many lines this board carries. */
    public int lineCount() {
        return lines.size();
    }

    /**
     * The current value of one line, as it would be written.
     *
     * <p>The same path {@link #update()} takes, including the guards, so what a test reads here is
     * what a driver would see.
     *
     * @return the value, {@code "-"} when the supplier returned nothing, {@code "error"} when it
     *         threw, or null if there is no such line
     */
    public String valueOf(String key) {
        for (Line line : lines) {
            if (!line.key().equals(key)) {
                continue;
            }
            try {
                String value = line.value().get();
                return value == null ? "-" : value;
            } catch (RuntimeException e) {
                // A supplier that throws must not take the display down with it. A driver seeing one
                // line read "error" still has the other five.
                return "error";
            }
        }
        return null;
    }

    // --- the standard lines ---------------------------------------------------

    /**
     * What is wrong, in the fewest words that let someone act.
     *
     * <p>Names the failing checks while there are few enough to name, then counts them. Errors are
     * reported ahead of warnings, since a driver reading one line should be told the worst thing
     * first.
     */
    static String healthSummary() {
        List<HealthCheck> firing = HealthMonitor.getInstance().checks().stream()
                .filter(HealthCheck::isFiring)
                .toList();
        if (firing.isEmpty()) {
            return "OK";
        }

        List<HealthCheck> errors = firing.stream()
                .filter(c -> c.severity() == HealthCheck.Severity.ERROR)
                .toList();
        List<HealthCheck> worst = errors.isEmpty() ? firing : errors;
        String label = errors.isEmpty() ? "WARN" : "FAULT";

        if (worst.size() <= MAX_PROBLEMS_SHOWN) {
            return label + ": " + String.join(", ", worst.stream().map(HealthCheck::id).toList());
        }
        return label + ": " + worst.size() + " problems";
    }

    /**
     * Battery, against the thresholds the machine itself holds.
     *
     * <p>Not a roboRIO constant. Systemcore publishes its own brownout voltage and it is not the same
     * number, so a board comparing against 6.8 V would say "low" at the wrong point on the one
     * reading a driver actually acts on.
     */
    static String batterySummary() {
        SystemCoreStatus status = SystemCoreStatus.getInstance();
        OptionalDouble volts = status.batteryVolts();
        if (volts.isEmpty()) {
            return "-";
        }
        double v = volts.getAsDouble();
        String text = String.format("%.2f V", v);

        if (status.isBrownedOut()) {
            return text + " BROWNOUT";
        }
        OptionalDouble floor = status.brownoutVolts();
        if (floor.isPresent() && v < floor.getAsDouble() + 1.0) {
            return text + " LOW";
        }
        return text;
    }
}
