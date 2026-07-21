package frc.lib.catalyst.statemachine.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.statemachine.BindingSample;
import frc.lib.catalyst.statemachine.Phase;
import frc.lib.catalyst.statemachine.StateMachineTelemetry;
import frc.lib.catalyst.statemachine.TransitionRecord;
import frc.lib.catalyst.util.AlertManager;

import java.util.List;

/**
 * Routes the whole state-machine log schema into {@link CatalystLog}, plus the two channels that
 * work with no dashboard configured at all.
 *
 * <p>Everything goes through {@code CatalystLog} rather than NetworkTables directly, so a team
 * that swaps in a WPILOG or AdvantageKit sink gets the state machine's telemetry along with
 * everything else, for free. Live keys land at {@code /Catalyst/<prefix>/...}.
 *
 * <h2>The three field-visible channels</h2>
 *
 * <p>This is the "no laptop at a regional" contract, and each channel is used for what it is
 * actually good at:
 *
 * <ol>
 *   <li><b>NetworkTables / WPILOG</b> — the full schema, for AdvantageScope and the pit display.</li>
 *   <li><b>{@link AlertManager}</b> — exactly four alert strings, all invariant. {@code AlertManager}
 *       keys messages by their composed text and clears by exact match, so putting live numbers in
 *       an alert makes it impossible to clear. The numbers go elsewhere.</li>
 *   <li><b>Driver Station console</b> — one non-deduplicated warning per fault and per rejection,
 *       carrying the full summary with mechanism names and numbers. This is the channel that needs
 *       no dashboard setup whatsoever, which is exactly when you need it most.</li>
 * </ol>
 *
 * @param <S> the machine's state enum
 * @since 1.2.0
 */
public final class CatalystStateMachineLog<S extends Enum<S>> implements StateMachineTelemetry<S> {

    private static final String ALERT_TIMED_OUT = "Transition timed out";
    private static final String ALERT_REJECTED = "Transition rejected";
    private static final String ALERT_UNCONFIRMED = "State is not confirmed";
    private static final String ALERT_FAULTED = "Superstructure faulted";

    private final String prefix;
    private final String alertSubsystem;
    private final boolean driverStationMessages;

    /**
     * @param prefix                log prefix, relative to the {@code Catalyst/} root
     * @param alertSubsystem        subsystem name used for {@link AlertManager} entries
     * @param driverStationMessages whether to emit one-shot Driver Station warnings
     */
    public CatalystStateMachineLog(String prefix, String alertSubsystem, boolean driverStationMessages) {
        this.prefix = prefix;
        this.alertSubsystem = alertSubsystem;
        this.driverStationMessages = driverStationMessages;
    }

    @Override
    public void graph(String machine, String[] states, String[] edges, String[] bindings,
                      String[] routes, String dot, List<String> errors, List<String> warnings) {
        log("Graph/Machine", machine);
        log("Graph/States", states);
        log("Graph/Edges", edges);
        log("Graph/Bindings", bindings);
        log("Graph/Routes", routes);
        log("Graph/Dot", dot);
        log("Graph/Warnings", warnings.toArray(new String[0]));
        for (String w : warnings) {
            DriverStation.reportWarning("[" + prefix + "] " + w, false);
        }
    }

    @Override
    public void state(S current, boolean confirmed, S target, S nextHop, Phase phase, int stageIndex) {
        log("State", current == null ? "" : current.name());
        log("StateOrdinal", current == null ? -1L : (long) current.ordinal());
        log("StateConfirmed", confirmed);
        log("Target", target == null ? "" : target.name());
        log("NextHop", nextHop == null ? "" : nextHop.name());
        log("Phase", phase.name());
        log("PhaseOrdinal", (long) phase.ordinal());
        log("Stage", (long) stageIndex);
        log("Transitioning", phase == Phase.MOVING || phase == Phase.SETTLING);

        AlertManager alerts = AlertManager.getInstance();
        if (confirmed) {
            // Clear the alerts whose condition a confirmed arrival genuinely resolves. The rejection
            // alert is deliberately NOT cleared here: a request refused while the machine is otherwise
            // confirmed would raise it and then have it wiped on the very next loop, before anyone
            // could see it. It is cleared in transition() when the next request is accepted instead.
            alerts.clearWarning(alertSubsystem, ALERT_UNCONFIRMED);
            alerts.clearWarning(alertSubsystem, ALERT_TIMED_OUT);
        } else {
            alerts.warning(alertSubsystem, ALERT_UNCONFIRMED);
        }
    }

    @Override
    public void blocker(String blocker, String blockerDetail, String summary, List<String> waitingOn) {
        log("Blocker", blocker);
        log("BlockerDetail", blockerDetail);
        log("Summary", summary);
        log("WaitingOn", waitingOn.toArray(new String[0]));
    }

    @Override
    public void progress(long seq, double fraction, double elapsedSeconds, double timeoutSeconds,
                         String route, String trigger) {
        log("Progress", fraction);
        log("ElapsedSeconds", elapsedSeconds);
        log("TimeoutSeconds", timeoutSeconds);
        log("Route", route);
        log("Trigger", trigger);
    }

    @Override
    public void binding(BindingSample s) {
        String base = "Bindings/" + s.key() + "/";
        log(base + "Goal", s.goalLabel());
        log(base + "GoalDetail", s.goalDetail());
        log(base + "Measured", s.measured());
        log(base + "Error", s.error());
        log(base + "Tolerance", s.tolerance());
        log(base + "AtGoal", s.arrived());
        log(base + "Owned", s.owned());
        log(base + "Observable", s.observable());
        log(base + "Gating", s.gating());
        log(base + "ArrivalSeconds", s.arrivalSeconds());
        log(base + "Note", s.note());
    }

    @Override
    public void transition(TransitionRecord<S> r) {
        log("Transition/Seq", r.seq());
        log("Transition/From", r.from() == null ? "" : r.from().name());
        log("Transition/To", r.to() == null ? "" : r.to().name());
        log("Transition/Route", r.routeString());
        log("Transition/Trigger", r.trigger());
        log("Transition/Outcome", r.outcome().name());
        log("Transition/OutcomeOrdinal", (long) r.outcome().ordinal());
        log("Transition/Reason", r.reason().name());
        log("Transition/Detail", r.detail());
        log("Transition/DurationSeconds", r.durationSeconds());

        String[] arrivals = new String[r.arrivals().size()];
        for (int i = 0; i < arrivals.length; i++) arrivals[i] = r.arrivals().get(i).serialize();
        log("Transition/Arrivals", arrivals);

        AlertManager alerts = AlertManager.getInstance();
        switch (r.outcome()) {
            case ARRIVED:
                // A completed transition means the driver's last request was honoured, so any
                // lingering rejection alert from an earlier refused request is now stale.
                alerts.clearWarning(alertSubsystem, ALERT_REJECTED);
                break;
            case TIMED_OUT:
                alerts.warning(alertSubsystem, ALERT_TIMED_OUT);
                announce("timed out " + describe(r) + " — " + r.detail());
                break;
            case REJECTED:
                log("Rejected/Last", describe(r) + ": " + r.reason()
                        + (r.detail().isEmpty() ? "" : " (" + r.detail() + ")"));
                log("Rejected/LastTimestamp", r.endTimestamp());
                alerts.warning(alertSubsystem, ALERT_REJECTED);
                announce("refused " + describe(r) + ": " + r.reason()
                        + (r.detail().isEmpty() ? "" : " — " + r.detail()));
                break;
            case FAULTED:
                announce("faulted " + describe(r) + " — " + r.detail());
                break;
            default:
                break;
        }
    }

    @Override
    public void history(String[] newestFirst) {
        log("Transition/History", newestFirst);
    }

    @Override
    public void legalTargets(String[] targets) {
        log("LegalTargets", targets);
    }

    @Override
    public void counters(long transitions, long rejections, long timeouts, long aborts, long yields) {
        log("Counters/Transitions", transitions);
        log("Counters/Rejections", rejections);
        log("Counters/Timeouts", timeouts);
        log("Counters/Aborts", aborts);
        log("Counters/Yields", yields);
    }

    @Override
    public void heartbeat(long ticks, double uptimeSeconds, boolean enabled) {
        log("Ticks", ticks);
        log("UptimeSeconds", uptimeSeconds);
        log("Enabled", enabled);
    }

    @Override
    public void fault(boolean faulted, String reason) {
        log("Faulted", faulted);
        log("FaultReason", reason);
        AlertManager alerts = AlertManager.getInstance();
        if (faulted) {
            alerts.error(alertSubsystem, ALERT_FAULTED);
            announce("FAULT — " + reason);
        } else {
            alerts.clearError(alertSubsystem, ALERT_FAULTED);
        }
    }

    /**
     * One-shot Driver Station warning. Deliberately not deduplicated: unlike an
     * {@link AlertManager} entry, which must carry invariant text to stay clearable, this channel
     * exists precisely to carry the live numbers, and it is emitted once per event rather than
     * every loop.
     */
    private void announce(String message) {
        if (driverStationMessages) {
            DriverStation.reportWarning("[" + prefix + "] " + message, false);
        }
    }

    private String describe(TransitionRecord<S> r) {
        return (r.from() == null ? "?" : r.from().name()) + "->" + (r.to() == null ? "?" : r.to().name());
    }

    private void log(String key, String value)   { CatalystLog.log(prefix + "/" + key, value); }
    private void log(String key, String[] value) { CatalystLog.log(prefix + "/" + key, value); }
    private void log(String key, double value)   { CatalystLog.log(prefix + "/" + key, value); }
    private void log(String key, long value)     { CatalystLog.log(prefix + "/" + key, value); }
    private void log(String key, boolean value)  { CatalystLog.log(prefix + "/" + key, value); }
}
