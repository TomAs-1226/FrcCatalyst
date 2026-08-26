package frc.lib.catalyst.util;

import frc.lib.catalyst.command.CommandRuntime;
import frc.lib.catalyst.hardware.CANBusPlanner;
import frc.lib.catalyst.hardware.CANRegistry;
import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.system.SystemCoreStatus;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

/**
 * One call at boot that answers: is this robot fit to enable?
 *
 * <pre>{@code
 * Preflight.run().printToConsole();     // in the robot constructor
 * }</pre>
 *
 * <h2>Why this exists</h2>
 *
 * <p>Everything below is already checkable, and separately checking all of it is something nobody
 * does. The failures it covers share a shape: the robot boots, the dashboard connects, everything
 * looks normal, and the problem surfaces at the worst moment as something that does not resemble its
 * cause.
 *
 * <p>The two JVM flags are the sharpest example. Without them the robot starts perfectly and dies on
 * the first command a driver schedules, with a stack trace from inside WPILib naming no class of
 * yours. A check at boot turns that into a line of text before anyone has walked to the field.
 *
 * <h2>What it does not do</h2>
 *
 * <p>It changes nothing and blocks nothing. A preflight that refused to let a robot enable would
 * eventually refuse during a match, and the team that hit that would remove it. It reports; the
 * decision stays with the humans.
 *
 * @since 2.0.0
 */
public final class Preflight {

    /** How bad a finding is. */
    public enum Level {
        /** Worth knowing. Nothing is wrong. */
        INFO,
        /** Works, but something is not as intended. */
        WARN,
        /** Will not work. Fix before enabling. */
        BLOCKER
    }

    /** One thing the preflight found. */
    public record Finding(Level level, String what, String detail) {
        @Override
        public String toString() {
            return switch (level) {
                case BLOCKER -> "[BLOCKER] " + what + " — " + detail;
                case WARN -> "[warn]    " + what + " — " + detail;
                case INFO -> "[ok]      " + what;
            };
        }
    }

    /** Everything the preflight found, worst first. */
    public record Report(List<Finding> findings) {

        /** True when nothing will stop the robot working. Warnings do not count. */
        public boolean canEnable() {
            return findings.stream().noneMatch(f -> f.level() == Level.BLOCKER);
        }

        /** The findings at or above a level. */
        public List<Finding> atLeast(Level level) {
            return findings.stream().filter(f -> f.level().ordinal() >= level.ordinal()).toList();
        }

        /**
         * Print to the driver station console.
         *
         * <p>Deliberately the console rather than an alert: this is read once at boot by whoever is
         * standing at the laptop, and an alert that fires every boot becomes an alert nobody reads.
         */
        public Report printToConsole() {
            System.out.println("---- Catalyst preflight ----");
            for (Finding f : findings) {
                System.out.println("  " + f);
            }
            System.out.println(canEnable()
                    ? "---- ready ----"
                    : "---- NOT READY: fix the blockers above ----");
            return this;
        }

        /** A one-line summary, for {@code DriverBoard} or a dashboard. */
        public String summary() {
            List<Finding> blockers = findings.stream()
                    .filter(f -> f.level() == Level.BLOCKER).toList();
            if (!blockers.isEmpty()) {
                return "NOT READY: " + blockers.get(0).what();
            }
            long warnings = findings.stream().filter(f -> f.level() == Level.WARN).count();
            return warnings == 0 ? "Ready" : "Ready, " + warnings + " warning"
                    + (warnings == 1 ? "" : "s");
        }
    }

    private Preflight() {
    }

    /**
     * Run every check.
     *
     * <p>Safe to call before anything else is constructed — each check reports what it can and says
     * so when it cannot see enough to judge.
     */
    public static Report run() {
        List<Finding> findings = new ArrayList<>();

        checkCommandRuntime(findings);
        checkMachine(findings);
        checkStorage(findings);
        checkCanPlan(findings);
        checkBattery(findings);

        // Worst first: someone reading three lines should read the three that matter.
        findings.sort((a, b) -> b.level().compareTo(a.level()));

        CatalystLog.log("Preflight/Ready", findings.stream()
                .noneMatch(f -> f.level() == Level.BLOCKER));
        CatalystLog.log("Preflight/Summary", new Report(findings).summary());
        CatalystLog.log("Preflight/Findings",
                findings.stream().map(Finding::toString).toArray(String[]::new));

        return new Report(findings);
    }

    /**
     * The one that stops the robot dead.
     *
     * <p>Commands v3 needs two {@code --add-opens} flags. Without them nothing fails until a command
     * is scheduled, which is usually the first time a driver presses a button.
     */
    private static void checkCommandRuntime(List<Finding> out) {
        if (CommandRuntime.isAvailable()) {
            out.add(new Finding(Level.INFO, "Commands v3 runtime", "continuations reachable"));
            return;
        }
        out.add(new Finding(Level.BLOCKER, "Commands v3 cannot run",
                "add --add-opens java.base/jdk.internal.vm=ALL-UNNAMED and "
                        + "--add-opens java.base/java.lang=ALL-UNNAMED to the robot's JVM arguments. "
                        + "Both are needed. Nothing will fail until the first command is scheduled."));
    }

    private static void checkMachine(List<Finding> out) {
        SystemCoreStatus status = SystemCoreStatus.getInstance();
        if (!status.isAvailable()) {
            // Not a blocker. Simulation is a legitimate place to be, and so is a bench test.
            out.add(new Finding(Level.INFO, "Systemcore", "not present — simulation or desktop"));
            return;
        }

        var team = status.teamNumber();
        out.add(new Finding(Level.INFO, "Systemcore",
                team.isPresent() ? "team " + team.getAsInt() : "reporting"));

        status.cpuTemperatureCelsius().ifPresent(c -> {
            if (c >= 80) {
                out.add(new Finding(Level.WARN, "Systemcore is hot",
                        String.format("%.0f °C. The cores throttle rather than reporting anything, "
                                + "so the symptom is a loop overrun, not a temperature.", c)));
            }
        });
    }

    /**
     * Storage, both kinds.
     *
     * <p>A full disk stops logging, then stops the robot program, and nothing about that symptom
     * points at the disk. Flash wear is slower and does not recover.
     */
    private static void checkStorage(List<Finding> out) {
        SystemCoreStatus status = SystemCoreStatus.getInstance();

        status.storageFraction().ifPresent(f -> {
            if (f >= 0.95) {
                out.add(new Finding(Level.BLOCKER, "Storage is full",
                        String.format("%.0f%% used. Logging will stop and the robot program will "
                                + "follow it.", f * 100)));
            } else if (f >= 0.85) {
                out.add(new Finding(Level.WARN, "Storage is filling",
                        String.format("%.0f%% used.", f * 100)));
            }
        });

        if (status.emmcNeedsAttention()) {
            out.add(new Finding(Level.WARN, "Flash is wearing out",
                    "the eMMC is near the end of its rated write life, and unlike a full disk this "
                            + "does not come back when you delete something."));
        }
    }

    /**
     * The CAN plan, judged before the robot is enabled.
     *
     * <p>Everything here is knowable from the device list alone, which is why it is worth asking at
     * boot rather than discovering under load.
     */
    private static void checkCanPlan(List<Finding> out) {
        List<String> problems = CANBusPlanner.validate();
        for (String problem : problems) {
            out.add(new Finding(Level.WARN, "CAN plan", problem));
        }
        List<String> contention = CANRegistry.contentionWarnings();
        for (String warning : contention) {
            out.add(new Finding(Level.WARN, "CAN layout", warning));
        }
        if (problems.isEmpty() && contention.isEmpty()) {
            out.add(new Finding(Level.INFO, "CAN plan", "no conflicts"));
        }
    }

    /**
     * Battery, against the machine's own brownout threshold rather than a roboRIO constant.
     *
     * <p>A blocker rather than a warning below the threshold, because a robot that is already
     * browning out on the cart will not survive being driven.
     */
    private static void checkBattery(List<Finding> out) {
        SystemCoreStatus status = SystemCoreStatus.getInstance();
        OptionalDouble volts = status.batteryVolts();
        if (volts.isEmpty()) {
            return;
        }
        double v = volts.getAsDouble();
        double floor = status.brownoutVolts().orElse(BrownoutMonitor.LEGACY_FLOOR_VOLTS);

        if (status.isBrownedOut() || v <= floor) {
            out.add(new Finding(Level.BLOCKER, "Battery is below the brownout threshold",
                    String.format("%.2f V against a floor of %.2f V.", v, floor)));
        } else if (v < 12.0) {
            out.add(new Finding(Level.WARN, "Battery is low",
                    String.format("%.2f V. Fine on a cart, short of a match.", v)));
        } else {
            out.add(new Finding(Level.INFO, "Battery", String.format("%.2f V", v)));
        }
    }
}
