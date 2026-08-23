package frc.lib.catalyst.util;

import frc.lib.catalyst.command.CatalystCommand;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.command3.Command;
import frc.lib.catalyst.command.Commands;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Pre-match self-test — run every subsystem through a verification routine
 * and get a green/red board before you queue.
 *
 * <p>This catches the failures that actually lose matches: a loose
 * connector, an inverted motor, a dead follower, an encoder stuck at zero,
 * a CAN device that didn't boot. Run it from a button in the pit; read the
 * result on the dashboard.
 *
 * <p>Two kinds of check:
 * <ul>
 *   <li><b>instant</b> — a {@link BooleanSupplier} evaluated once
 *       ({@code check("Battery OK", () -> RobotState.batteryVoltage() > 12)}),</li>
 *   <li><b>timed</b> — apply an action for a duration, then evaluate a pass
 *       condition, then clean up. Use this to spin a motor and confirm the
 *       encoder actually moves and the current stays sane.</li>
 * </ul>
 *
 * <p>Results publish to {@code /Catalyst/SystemCheck/<name>/...}:
 * one entry per test ({@code "PASS"} or {@code "FAIL: <reason>"}), a
 * {@code Ready} boolean (true only if every test passed), and a
 * {@code Report} string you can copy into the pit chat.
 *
 * <pre>{@code
 * SystemCheck check = SystemCheck.builder("PreMatch")
 *     .check("Battery >12V", () -> RobotState.batteryVoltage() > 12.0)
 *     .check("Gyro alive",   () -> Double.isFinite(drive.getHeading().getDegrees()))
 *     .timed("Elevator moves up",
 *            () -> elevator.getMotor().setVoltage(1.5),       // action each loop
 *            1.0,                                              // for 1.0 s
 *            () -> elevator.getVelocity() > 0.05,              // pass if it's moving
 *            () -> elevator.getMotor().stop())                 // cleanup
 *     .timed("Elevator current sane",
 *            () -> elevator.getMotor().setVoltage(1.5),
 *            0.5,
 *            () -> elevator.getMotor().getStatorCurrent() < 60,
 *            () -> elevator.getMotor().stop())
 *     .build();
 *
 * pit.start().onTrue(check.run());   // runs when nothing else commands the subsystems
 * }</pre>
 *
 * <p>SystemCheck declares no subsystem requirements — run it in the pit when
 * no other command is driving the mechanisms.
 */
public final class SystemCheck {

    private interface Step {
        String name();
        Command toCommand(Results results, NetworkTable nt);
    }

    /** Live result for one test. */
    public record Result(String name, boolean passed, String detail) {}

    /** Accumulator passed through the run. */
    public static final class Results {
        private final Map<String, Result> map = new LinkedHashMap<>();

        void record(Result r) { map.put(r.name(), r); }

        /** All results in declaration order. */
        public List<Result> all() { return new ArrayList<>(map.values()); }

        /** True only if every test passed. */
        public boolean ready() {
            return map.values().stream().allMatch(Result::passed);
        }

        /** Plain-text report, one line per test. */
        public String report() {
            StringBuilder sb = new StringBuilder();
            sb.append(ready() ? "READY ✓" : "NOT READY ✗").append('\n');
            for (Result r : map.values()) {
                sb.append(r.passed() ? "  PASS  " : "  FAIL  ")
                  .append(r.name());
                if (!r.passed() && !r.detail().isEmpty()) sb.append("  — ").append(r.detail());
                sb.append('\n');
            }
            return sb.toString();
        }
    }

    private final String name;
    private final List<Step> steps;
    private final Results results = new Results();

    private SystemCheck(String name, List<Step> steps) {
        this.name = name;
        this.steps = steps;
    }

    /** The latest results (populated as {@link #run()} executes). */
    public Results results() {
        return results;
    }

    /**
     * Command that runs every check in order and publishes results. Bind to
     * a pit button.
     */
    public CatalystCommand run() {
        NetworkTable nt = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("SystemCheck").getSubTable(name);

        List<Command> stepCommands = new ArrayList<>(steps.size());
        for (Step step : steps) {
            stepCommands.add(step.toCommand(results, nt));
        }

        return Commands.sequence(stepCommands.toArray(new Command[0]))
                .beforeStarting(() -> {
                    results.map.clear();
                    nt.getEntry("Ready").setBoolean(false);
                    nt.getEntry("Report").setString("(running…)");
                })
                .then(Commands.runOnce(() -> {
                    nt.getEntry("Ready").setBoolean(results.ready());
                    nt.getEntry("Report").setString(results.report());
                }))
                .withName("SystemCheck:" + name);
    }

    // ============================================================
    //                        BUILDER
    // ============================================================

    public static Builder builder(String name) {
        return new Builder(name);
    }

    public static class Builder {
        private final String name;
        private final List<Step> steps = new ArrayList<>();

        private Builder(String name) {
            this.name = name;
        }

        /** An instant pass/fail check. */
        public Builder check(String testName, BooleanSupplier pass) {
            steps.add(new Step() {
                public String name() { return testName; }
                public CatalystCommand toCommand(Results results, NetworkTable nt) {
                    return Commands.runOnce(() -> {
                        boolean ok;
                        String detail = "";
                        try {
                            ok = pass.getAsBoolean();
                        } catch (Throwable t) {
                            ok = false;
                            detail = "threw " + t.getClass().getSimpleName();
                        }
                        publish(results, nt, testName, ok, detail);
                    });
                }
            });
            return this;
        }

        /**
         * A timed check: run {@code action} every loop for {@code seconds},
         * then evaluate {@code pass}, then always run {@code cleanup}.
         *
         * @param testName label
         * @param action   applied each loop (e.g. {@code () -> motor.setVoltage(1.5)})
         * @param seconds  how long to apply it
         * @param pass     evaluated after the duration — true = pass
         * @param cleanup  always run at the end (e.g. {@code () -> motor.stop()})
         */
        public Builder timed(String testName, Runnable action, double seconds,
                             BooleanSupplier pass, Runnable cleanup) {
            steps.add(new Step() {
                public String name() { return testName; }
                public CatalystCommand toCommand(Results results, NetworkTable nt) {
                    return Commands.run(() -> safe(action))
                            .timeoutAfter(seconds)
                            .then(Commands.runOnce(() -> {
                                boolean ok;
                                String detail = "";
                                try {
                                    ok = pass.getAsBoolean();
                                } catch (Throwable t) {
                                    ok = false;
                                    detail = "threw " + t.getClass().getSimpleName();
                                }
                                if (!ok && detail.isEmpty()) detail = "condition not met after " + seconds + "s";
                                publish(results, nt, testName, ok, detail);
                            }))
                            .finallyDo(interrupted -> safe(cleanup));
                }
            });
            return this;
        }

        public SystemCheck build() {
            return new SystemCheck(name, List.copyOf(steps));
        }

        private static void safe(Runnable r) {
            try { r.run(); } catch (Throwable ignored) {}
        }
    }

    private static void publish(Results results, NetworkTable nt,
                                String testName, boolean ok, String detail) {
        results.record(new Result(testName, ok, detail));
        nt.getEntry(testName).setString(ok ? "PASS" : ("FAIL: " + (detail.isEmpty() ? "—" : detail)));
    }
}
