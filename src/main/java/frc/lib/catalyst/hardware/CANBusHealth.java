package frc.lib.catalyst.hardware;

import com.ctre.phoenix6.CANBus;

import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.util.HealthCheck;

import java.util.ArrayList;
import java.util.List;

/**
 * Live health of every CAN bus the robot uses.
 *
 * <p>On a roboRIO there was one bus, and if it was saturated the robot simply behaved badly with no
 * obvious cause. Systemcore has five, each reporting its own utilisation, bus-off count, TX-full
 * count and error counters — which turns "the robot is twitchy" into a number attached to a specific
 * bus.
 *
 * <p>Two things make this more than a readout:
 *
 * <ul>
 *   <li><b>Error counters lead failures.</b> A bus does not go from healthy to bus-off; the receive
 *       and transmit error counters climb first. Watching REC and TEC catches a marginal
 *       connection — a nearly-seated connector, a terminator working loose — during a practice match
 *       rather than during an elimination.</li>
 *   <li><b>TX-full means messages were dropped.</b> Utilisation being high is a warning; the
 *       transmit queue overflowing means commands did not reach a motor. Those deserve different
 *       severities, and get them.</li>
 * </ul>
 *
 * <p>Which buses are watched comes from {@link CANRegistry} — whatever the robot actually declared,
 * so nothing needs listing twice.
 *
 * @since 2.0.0
 */
public final class CANBusHealth {

    /**
     * Utilisation at which Systemcore's own web UI starts warning.
     *
     * <p>Matching the OS's threshold on purpose: a team that sees a warning in one place and not the
     * other will reasonably assume one of them is broken.
     */
    public static final double UTILIZATION_WARN = 0.90;

    /** CAN error-counter value at which a controller enters error-passive state. */
    private static final int ERROR_PASSIVE_THRESHOLD = 128;

    private CANBusHealth() {}

    /** A single bus's current state. */
    public record BusStatus(
            String bus,
            boolean ok,
            double utilization,
            int busOffCount,
            int txFullCount,
            int receiveErrorCount,
            int transmitErrorCount) {

        /** Whether either error counter is high enough to indicate a marginal bus. */
        public boolean hasErrorActivity() {
            return receiveErrorCount >= ERROR_PASSIVE_THRESHOLD
                    || transmitErrorCount >= ERROR_PASSIVE_THRESHOLD;
        }
    }

    /** Read the current state of every bus the CAN registry knows about. */
    public static List<BusStatus> readAll() {
        List<BusStatus> out = new ArrayList<>();
        for (String busName : CANRegistry.byBus().keySet()) {
            read(busName).ifPresent(out::add);
        }
        return out;
    }

    /** Read one bus by the name the registry stores, tolerating anything unreadable. */
    public static java.util.Optional<BusStatus> read(String busName) {
        try {
            CatalystCANBus bus = CatalystCANBus.of(busName);
            CANBus.CANBusStatus raw = bus.status();
            return java.util.Optional.of(new BusStatus(
                    bus.name(),
                    raw.Status.isOK(),
                    raw.BusUtilization,
                    raw.BusOffCount,
                    raw.TxFullCount,
                    raw.REC,
                    raw.TEC));
        } catch (RuntimeException ignored) {
            // A bus that cannot be queried (simulation, or a CANivore that is not attached) is not
            // an error worth surfacing - it simply has no status.
            return java.util.Optional.empty();
        }
    }

    /** Publish every bus's state under {@code /Catalyst/CAN/Health/<bus>/...}. Call once per loop. */
    public static void publish() {
        for (BusStatus s : readAll()) {
            String prefix = "CAN/Health/" + s.bus() + "/";
            CatalystLog.log(prefix + "OK", s.ok());
            CatalystLog.log(prefix + "Utilization", s.utilization());
            CatalystLog.log(prefix + "BusOffCount", s.busOffCount());
            CatalystLog.log(prefix + "TxFullCount", s.txFullCount());
            CatalystLog.log(prefix + "REC", s.receiveErrorCount());
            CatalystLog.log(prefix + "TEC", s.transmitErrorCount());
        }
    }

    /**
     * Register health checks for every bus the registry knows about.
     *
     * <p>Call once, after every device has been constructed — the registry has to know about a bus
     * before this can watch it.
     */
    public static void registerChecks() {
        for (String busName : CANRegistry.byBus().keySet()) {
            CatalystCANBus bus;
            try {
                bus = CatalystCANBus.of(busName);
            } catch (RuntimeException ignored) {
                continue;
            }
            final String name = bus.name();
            final String subsystem = "CAN";

            HealthCheck.builder(subsystem, name + "/Down")
                    .severity(HealthCheck.Severity.ERROR)
                    .description("CAN bus " + name + " is not reporting OK")
                    .when(() -> read(name).map(s -> !s.ok()).orElse(false))
                    .register();

            HealthCheck.builder(subsystem, name + "/HighUtilization")
                    .severity(HealthCheck.Severity.WARN)
                    .description("CAN bus " + name + " above "
                            + (int) (UTILIZATION_WARN * 100) + "% utilisation")
                    .when(() -> read(name).map(s -> s.utilization() > UTILIZATION_WARN).orElse(false))
                    .debounce(1.0)
                    .detail(() -> read(name)
                            .map(s -> String.format("%.0f%%", s.utilization() * 100))
                            .orElse("unknown"))
                    .register();

            // Dropped frames, not just a busy bus: a full transmit queue means commands did not go
            // out. Higher severity than utilisation for that reason.
            HealthCheck.builder(subsystem, name + "/TxFull")
                    .severity(HealthCheck.Severity.ERROR)
                    .description("CAN bus " + name + " transmit queue overflowed - frames were lost")
                    .when(() -> read(name).map(s -> s.txFullCount() > 0).orElse(false))
                    .detail(() -> read(name).map(s -> s.txFullCount() + " overflows").orElse(""))
                    .register();

            // The leading indicator. Error counters climb before a bus goes off, so this is the one
            // that catches a loose connector in the pit instead of on the field.
            HealthCheck.builder(subsystem, name + "/Errors")
                    .severity(HealthCheck.Severity.WARN)
                    .description("CAN bus " + name + " error counters are elevated - check wiring "
                            + "and termination before this becomes a bus-off")
                    .when(() -> read(name).map(BusStatus::hasErrorActivity).orElse(false))
                    .debounce(2.0)
                    .detail(() -> read(name)
                            .map(s -> "REC " + s.receiveErrorCount() + ", TEC " + s.transmitErrorCount())
                            .orElse("unknown"))
                    .register();
        }
    }
}
