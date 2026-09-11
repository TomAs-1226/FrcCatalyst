package frc.lib.catalyst.autonomy;

import frc.lib.catalyst.logging.CatalystLog;

import java.util.List;

/**
 * The one place the autonomy layer says what it is doing and why.
 *
 * <p>Every decision core in this package is a pure function that returns a record. That makes them
 * testable and it makes them silent: a robot running them would decide beautifully and tell nobody.
 * This is the other half - it takes those records and publishes them under
 * {@code /Catalyst/Autonomy/}, on a fixed schema, so a dashboard can show the robot's reasoning
 * without knowing anything about the game.
 *
 * <p><b>Why one publisher rather than each core publishing itself.</b> Keeping the cores pure is
 * worth more than the convenience: a core that logs cannot be run a thousand times in a test, cannot
 * be run twice in a loop to compare options, and drags a sink into every unit test. The cost is this
 * class, which is a hundred lines of string formatting and no logic.
 *
 * <p>Everything is published only when it changes. These values are stable for seconds at a time and
 * the robot loop has 20 ms for everything.
 *
 * <h2>The schema</h2>
 *
 * <pre>
 * Autonomy/Situation/{Valid,Confidence,Speed,Slip,BusVolts,Headroom,Binding}
 * Autonomy/Tasks/{Running,Held,Explain}
 * Autonomy/Chase/{Target,Why,Rejected}
 * Autonomy/Authority/{Scale,Binding,Explain}
 * Autonomy/Power/{Deficit,Shed,Short,Explain}
 * Autonomy/Intent/{Guess,HitRate,Samples,Explain}
 * </pre>
 *
 * @since 2.1.0
 */
public final class AutonomyBoard {

    private static final String ROOT = "Autonomy/";

    // Last published values, so an unchanging robot costs nothing on the wire.
    private String lastTasks;
    private String lastChase;
    private String lastAuthority;
    private String lastPower;
    private String lastIntent;
    private String lastBinding;

    /** The physics snapshot, and how much of it can be trusted. */
    public void publish(Situation s) {
        if (s == null) {
            return;
        }
        CatalystLog.log(ROOT + "Situation/Valid", !s.blindfolded());
        if (s.localization().valid()) {
            CatalystLog.log(ROOT + "Situation/Confidence", s.localization().confidence());
        }
        if (s.motion().valid()) {
            CatalystLog.log(ROOT + "Situation/Speed", s.motion().speedMps());
        }
        if (s.traction().valid()) {
            CatalystLog.log(ROOT + "Situation/Slip", s.traction().slipFactor());
        }
        CatalystLog.log(ROOT + "Situation/BusVolts", s.power().busVolts());
        if (s.power().valid()) {
            CatalystLog.log(ROOT + "Situation/Headroom", s.power().headroomAmps());
        }
        if (!s.power().bindingLimit().equals(lastBinding)) {
            lastBinding = s.power().bindingLimit();
            CatalystLog.log(ROOT + "Situation/Binding", lastBinding);
        }
    }

    /** What the multi-winner arbiter chose, and what it held back. */
    public <T, R> void publish(TaskArbiter.Selection<T, R> selection) {
        if (selection == null) {
            return;
        }
        String running = join(selection.winners().stream().map(TaskArbiter.Candidate::name).toList());
        if (!running.equals(lastTasks)) {
            lastTasks = running;
            CatalystLog.log(ROOT + "Tasks/Running", running);
            CatalystLog.log(ROOT + "Tasks/Held",
                    join(selection.skipped().stream()
                            .map(k -> k.name() + " (" + k.reason() + ")").toList()));
            CatalystLog.log(ROOT + "Tasks/Explain", selection.explain());
        }
    }

    /** What the chaser is going after, and why not the others. */
    public <T> void publish(ChaseCore.Choice<T> choice) {
        if (choice == null) {
            return;
        }
        String target = choice.chosen().map(ChaseCore.Target::name).orElse("(none)");
        if (!target.equals(lastChase)) {
            lastChase = target;
            CatalystLog.log(ROOT + "Chase/Target", target);
            CatalystLog.log(ROOT + "Chase/Why", choice.reason());
            CatalystLog.log(ROOT + "Chase/Rejected",
                    join(choice.rejected().stream().map(ChaseCore.Target::name).toList()));
        }
    }

    /** How much of the robot's output is allowed right now, and by whom. */
    public void publish(AuthorityCore.Authority authority) {
        if (authority == null) {
            return;
        }
        // The scale itself moves continuously, so it is published every loop; the explanation is
        // stable and is not.
        CatalystLog.log(ROOT + "Authority/Scale", authority.scale());
        if (!authority.explain().equals(lastAuthority)) {
            lastAuthority = authority.explain();
            CatalystLog.log(ROOT + "Authority/Binding", authority.binding());
            CatalystLog.log(ROOT + "Authority/Explain", authority.explain());
        }
    }

    /** What was shed to stay inside the power budget. */
    public void publish(ShedCore.Plan plan, double deficitAmps) {
        if (plan == null) {
            return;
        }
        CatalystLog.log(ROOT + "Power/Deficit", deficitAmps);
        CatalystLog.log(ROOT + "Power/Shed", plan.shedAmps());
        CatalystLog.log(ROOT + "Power/Short", plan.shortfall());
        if (!plan.explain().equals(lastPower)) {
            lastPower = plan.explain();
            CatalystLog.log(ROOT + "Power/Explain", plan.explain());
        }
    }

    /** The intention guess and, more importantly, how often it has been right. */
    public void publish(IntentCore.Reading reading) {
        if (reading == null) {
            return;
        }
        CatalystLog.log(ROOT + "Intent/HitRate", reading.hitRate());
        CatalystLog.log(ROOT + "Intent/Samples", (long) reading.samples());
        String explain = reading.explain();
        if (!explain.equals(lastIntent)) {
            lastIntent = explain;
            CatalystLog.log(ROOT + "Intent/Guess",
                    reading.best().map(IntentCore.Guess::name).orElse("(none)"));
            CatalystLog.log(ROOT + "Intent/Explain", explain);
        }
    }

    /** Comma-separated, or a dash when empty - never an empty string, which reads as a bug. */
    private static String join(List<String> names) {
        return names.isEmpty() ? "—" : String.join(", ", names);
    }
}
