package frc.lib.catalyst.fieldlab;

import frc.lib.catalyst.command.CatalystCommand;
import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.util.AlertManager;
import frc.lib.catalyst.util.RobotState;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Time;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;

import static org.wpilib.units.Units.Seconds;

/**
 * Runs a {@link TestCampaign}: one command, an hour long, that survives being interrupted.
 *
 * <h2>How to use it</h2>
 *
 * <pre>{@code
 * @Utility(name = "Field Lab", group = "Data", description = "The field-day measurement campaign")
 * public class FieldLab extends FieldLabOpMode {
 *     public FieldLab() {
 *         super(MyCampaigns.fieldDay(drive, shooter));
 *     }
 * }
 * }</pre>
 *
 * <p>{@link FieldLabOpMode} does the wiring. Use this class directly only if you are driving a
 * campaign from somewhere that is not an op mode.
 *
 * <h2>What it guarantees</h2>
 *
 * <p><b>It resumes.</b> Every finished procedure is banked to {@link CampaignCheckpoint} before the
 * next one starts, so a disable, a brownout, a battery swap or a power cycle costs the procedure that
 * was in flight and nothing else. This is the difference between a campaign that gets used and one
 * that gets abandoned three procedures in.
 *
 * <p><b>It stops rather than lie.</b> A {@link TestStep.BatteryGate} pauses the campaign instead of
 * collecting characterisation data on a flat pack. Data taken at 11 V is not slightly worse than data
 * taken at 12.6 V; it is wrong in a direction that looks like a real result, which is worse than
 * having none.
 *
 * <p><b>It asks.</b> A {@link TestStep.Measure} or {@link TestStep.Confirm} waits for a person,
 * indefinitely, and says what it wants on {@code /Catalyst/FieldLab/Prompt}. A campaign is expected to
 * spend a good part of its hour waiting for someone with a tape measure, and that is not a stall.
 *
 * <p><b>It never commands anything itself.</b> The runner has no {@link Mechanism} requirements: it
 * only awaits the commands its procedures supply, which carry their own. So it cannot conflict with a
 * procedure, and a step that wants the drivetrain gets it the ordinary way.
 *
 * <h2>What it publishes</h2>
 *
 * <p>Under {@code /Catalyst/FieldLab/}: {@code State}, {@code Session}, {@code Campaign},
 * {@code Procedure}, {@code ProcedureIndex}, {@code ProcedureCount}, {@code Step},
 * {@code StepIndex}, {@code StepCount}, {@code Progress}, {@code ElapsedSeconds},
 * {@code RemainingSeconds}, {@code Prompt}, {@code Notes} and {@code Measurements}. A dashboard needs
 * nothing but these to show a campaign's progress and its current prompt.
 *
 * <h2>Where the numbers go</h2>
 *
 * <p>Samples and measurements go through {@link CatalystLog} under
 * {@code FieldLab/<session>/<procedure>/...}, which means they land in whatever sink the robot has
 * installed — a {@code WpilogSink} writes them into the same {@code .wpilog} as everything else, so
 * one file is one campaign and the offline analysis has the robot's full context beside the
 * measurements. Nothing is fitted on the robot: {@code tools/fieldlab.py} reads the log afterwards and
 * produces the report. A fit you can re-run against the log with a different model is worth more than
 * a number computed once on a 20 ms loop with the data thrown away.
 *
 * @since 2.0.0
 */
public final class CampaignRunner {

    /** Where the campaign is. */
    public enum State {
        /** Built, not started. */
        IDLE,
        /** A robot step is running. */
        RUNNING,
        /** Waiting for a person to measure something or acknowledge. */
        AWAITING_OPERATOR,
        /** Held at a battery gate. */
        PAUSED_BATTERY,
        /** Every procedure banked. */
        DONE,
        /** Stopped early. */
        ABORTED
    }

    private static final String LOG = "FieldLab/";

    private final TestCampaign campaign;
    private final CampaignCheckpoint checkpoint;
    private final FieldLabInput input;
    private final DoubleSupplier battery;

    private State state = State.IDLE;
    private String sessionId = "";
    private int procedureIndex = -1;
    private int stepIndex = -1;
    private String procedureTitle = "";
    private String stepLabel = "";
    private double startedAt = Double.NaN;
    private final List<String> notes = new ArrayList<>();
    private final Map<String, Double> measured = new LinkedHashMap<>();

    /** A runner with the default checkpoint location and the robot's own battery voltage. */
    public CampaignRunner(TestCampaign campaign) {
        this(campaign, new CampaignCheckpoint(), RobotState::batteryVoltage);
    }

    /**
     * A runner with an explicit checkpoint and battery source.
     *
     * @param battery volts; injected so the gates are testable without a HAL, which is the only way
     *     the resume-and-pause behaviour gets tested at all
     */
    public CampaignRunner(TestCampaign campaign, CampaignCheckpoint checkpoint, DoubleSupplier battery) {
        this.campaign = campaign;
        this.checkpoint = checkpoint;
        this.battery = battery;
        this.input = new FieldLabInput();
    }

    /** Where the campaign is. */
    public State state() {
        return state;
    }

    /** The session id this run is logging under. */
    public String sessionId() {
        return sessionId;
    }

    /** Everything a person has measured so far, keyed {@code <procedure>/<key>}. */
    public Map<String, Double> measurements() {
        return Map.copyOf(measured);
    }

    /** Notes the run has accumulated: gates that fired, steps that timed out, odd values. */
    public List<String> notes() {
        return List.copyOf(notes);
    }

    /** Fraction of the campaign's procedures banked, 0 to 1. */
    public double progress() {
        int total = campaign.procedures().size();
        if (total == 0) {
            return 1.0;
        }
        return (double) checkpoint.completed().size() / total;
    }

    /** The campaign's remaining time estimate, from the procedures not yet banked. */
    public double remainingSeconds() {
        double remaining = 0;
        for (TestProcedure p : campaign.procedures()) {
            if (!checkpoint.isComplete(p.id())) {
                remaining += p.estimatedSeconds();
            }
        }
        return remaining;
    }

    /**
     * The whole campaign, as one command.
     *
     * <p>Schedule it once; it runs to the end of the campaign or until it is cancelled. Cancelling is
     * safe and is the normal way a session ends when the field time runs out — everything banked stays
     * banked, and scheduling it again resumes from the next unfinished procedure.
     */
    public CatalystCommand command() {
        return CatalystCommand.of(new CampaignCommand()).withName("FieldLab." + campaign.name());
    }

    /**
     * Publish the campaign's state. Call every loop, from {@code onPeriodic} <em>and</em>
     * {@code onDisabledPeriodic}.
     *
     * <p>Both, because the two states an operator most needs to see are the two where no command is
     * running: paused at a battery gate with the robot disabled for the swap, and waiting for a
     * measurement. A runner that only published while enabled would go blank at exactly the moments
     * someone is looking at it.
     */
    public void publish() {
        CatalystLog.log(LOG + "State", state.name());
        CatalystLog.log(LOG + "Session", sessionId);
        CatalystLog.log(LOG + "Campaign", campaign.name());
        CatalystLog.log(LOG + "Procedure", procedureTitle);
        CatalystLog.log(LOG + "ProcedureIndex", (long) Math.max(0, procedureIndex));
        CatalystLog.log(LOG + "ProcedureCount", (long) campaign.procedures().size());
        CatalystLog.log(LOG + "Step", stepLabel);
        CatalystLog.log(LOG + "StepIndex", (long) Math.max(0, stepIndex));
        CatalystLog.log(LOG + "StepCount", (long) currentStepCount());
        CatalystLog.log(LOG + "Progress", progress());
        CatalystLog.log(LOG + "ElapsedSeconds", Double.isNaN(startedAt) ? 0.0 : Timer.getTimestamp() - startedAt);
        CatalystLog.log(LOG + "RemainingSeconds", remainingSeconds());
        CatalystLog.log(LOG + "BatteryVolts", battery.getAsDouble());
        CatalystLog.log(LOG + "Notes", notes.toArray(new String[0]));
        CatalystLog.log(LOG + "Measurements", measurementRows());
    }

    private int currentStepCount() {
        if (procedureIndex < 0 || procedureIndex >= campaign.procedures().size()) {
            return 0;
        }
        return campaign.procedures().get(procedureIndex).steps().size();
    }

    /** {@code "<procedure>/<key>|<value>|<unit>"}, the shape Catalyst's other row topics use. */
    private String[] measurementRows() {
        List<String> rows = new ArrayList<>(measured.size());
        measured.forEach((k, v) -> rows.add(k + "|" + v));
        return rows.toArray(new String[0]);
    }

    private void note(String note) {
        notes.add(note);
        checkpoint.note(note);
        CatalystLog.log(LOG + "LastNote", note);
    }

    /** The campaign itself. No requirements, so it never conflicts with the steps it awaits. */
    private final class CampaignCommand implements Command {

        @Override
        public String name() {
            return "FieldLab." + campaign.name();
        }

        @Override
        public Set<Mechanism> requirements() {
            return Set.of();
        }

        @Override
        public void run(Coroutine coroutine) {
            begin();

            for (int i = 0; i < campaign.procedures().size(); i++) {
                TestProcedure procedure = campaign.procedures().get(i);
                if (checkpoint.isComplete(procedure.id())) {
                    continue;
                }
                procedureIndex = i;
                procedureTitle = procedure.title();
                CatalystLog.log(LOG + procedure.id() + "/Started", Timer.getTimestamp());

                runProcedure(coroutine, procedure);

                checkpoint.complete(procedure.id());
                CatalystLog.log(LOG + procedure.id() + "/Finished", Timer.getTimestamp());
                checkpoint.save().ifPresent(CampaignRunner.this::note);
            }

            finish();
        }

        @Override
        public void onCancel() {
            // Cancelling is the normal end of a session that ran out of field time, not a fault.
            // Everything banked is on disk already; say so, and leave the checkpoint alone so the
            // next run resumes rather than restarts.
            stepLabel = "";
            input.clear();
            state = State.ABORTED;
            CatalystLog.log(LOG + "State", state.name());
            AlertManager.getInstance().info("FieldLab",
                    "campaign stopped after " + checkpoint.completed().size() + " of "
                            + campaign.procedures().size() + " procedures; re-enable to resume");
        }

        private void begin() {
            Optional<String> loadProblem = checkpoint.load();
            boolean resuming = checkpoint.sessionId() != null
                    && !checkpoint.sessionId().isEmpty()
                    && campaign.name().equals(checkpoint.campaignName())
                    && !checkpoint.completed().isEmpty();

            if (resuming) {
                sessionId = checkpoint.sessionId();
                measured.putAll(checkpoint.measurements());
                notes.addAll(checkpoint.notes());
                AlertManager.getInstance().info("FieldLab",
                        "resuming session " + sessionId + " with " + checkpoint.completed().size()
                                + " of " + campaign.procedures().size() + " procedures already banked");
            } else {
                // A session id a person can say out loud and match to a log file. Not a UUID: the
                // point of it is that someone reads it off a dashboard and finds the right .wpilog.
                sessionId = String.format("%08x", System.currentTimeMillis() / 1000L);
                checkpoint.begin(sessionId, campaign.name());
                loadProblem.filter(p -> !p.startsWith("no checkpoint")).ifPresent(CampaignRunner.this::note);
                AlertManager.getInstance().info("FieldLab",
                        "session " + sessionId + ", " + campaign.procedures().size() + " procedures, estimated "
                                + campaign.estimate() + ", " + campaign.operatorProcedures() + " needing an operator");
            }

            startedAt = Timer.getTimestamp();
            state = State.RUNNING;
            CatalystLog.log(LOG + "Session", sessionId);
            CatalystLog.log(LOG + "EstimatedSeconds", campaign.estimatedSeconds());
        }

        private void runProcedure(Coroutine coroutine, TestProcedure procedure) {
            List<TestStep> steps = procedure.steps();
            for (int s = 0; s < steps.size(); s++) {
                stepIndex = s;
                TestStep step = steps.get(s);
                stepLabel = step.label();
                CatalystLog.log(LOG + "Step", stepLabel);

                switch (step) {
                    case TestStep.BatteryGate gate -> awaitBattery(coroutine, gate);
                    case TestStep.Settle settle -> {
                        state = State.RUNNING;
                        coroutine.wait(seconds(settle.seconds()));
                    }
                    case TestStep.Confirm confirm -> awaitOperator(coroutine, confirm.label(), "");
                    case TestStep.Measure measure -> takeMeasurement(coroutine, procedure, measure);
                    case TestStep.Act act -> runAction(coroutine, procedure, act);
                }
            }
            stepLabel = "";
        }

        private void awaitBattery(Coroutine coroutine, TestStep.BatteryGate gate) {
            if (battery.getAsDouble() >= gate.minVolts()) {
                return;
            }
            state = State.PAUSED_BATTERY;
            note(String.format("paused at %.2f V, wanted %.2f V", battery.getAsDouble(), gate.resumeVolts()));
            input.ask(gate.label());
            CatalystLog.log(LOG + "State", state.name());
            coroutine.waitUntil(() -> battery.getAsDouble() >= gate.resumeVolts());
            input.clear();
            state = State.RUNNING;
            note(String.format("resumed at %.2f V", battery.getAsDouble()));
        }

        private void awaitOperator(Coroutine coroutine, String label, String unit) {
            state = State.AWAITING_OPERATOR;
            CatalystLog.log(LOG + "State", state.name());
            input.armed();
            input.ask(label, unit);
            coroutine.waitUntil(input::advanced);
            input.clear();
            state = State.RUNNING;
            CatalystLog.log(LOG + "State", state.name());
        }

        private void takeMeasurement(Coroutine coroutine, TestProcedure procedure, TestStep.Measure measure) {
            String key = procedure.id() + "/" + measure.key();

            // A resumed campaign must not ask again for something already answered: the whole point of
            // the checkpoint is that nobody re-measures.
            if (measured.containsKey(key)) {
                note("kept the measurement already taken for " + key + " (" + measured.get(key) + ")");
                return;
            }

            awaitOperator(coroutine, measure.label(), measure.unit());
            double value = input.value();

            measured.put(key, value);
            checkpoint.measured(procedure.id(), measure.key(), value);
            CatalystLog.log(LOG + sessionId + "/" + key, value);
            CatalystLog.log(LOG + sessionId + "/" + key + ".unit", measure.unit());

            if (!measure.plausible(value)) {
                // Recorded, not rejected. A value outside the expected range is usually either the
                // interesting result or a units mistake, and both are worth seeing in the report;
                // throwing it away is how a real finding gets lost.
                String warning = String.format(
                        "%s = %s %s is outside the expected %s..%s - recorded anyway; check the units",
                        key, value, measure.unit(), measure.min(), measure.max());
                note(warning);
                AlertManager.getInstance().warning("FieldLab", warning);
            }
        }

        private void runAction(Coroutine coroutine, TestProcedure procedure, TestStep.Act act) {
            state = State.RUNNING;
            CatalystLog.log(LOG + "State", state.name());

            Command command;
            try {
                command = act.command().get();
            } catch (RuntimeException e) {
                // A procedure that cannot even build its command should cost that step, not the
                // campaign. Nine procedures with one hole in them is a good afternoon; nine
                // procedures abandoned at the second is not.
                note("could not build '" + act.label() + "' (" + e + "); skipped");
                AlertManager.getInstance().error("FieldLab", "step '" + act.label() + "' could not start: " + e);
                return;
            }

            double began = Timer.getTimestamp();
            var result = coroutine.await(command.withTimeout(seconds(act.timeoutSeconds())));
            double took = Timer.getTimestamp() - began;

            CatalystLog.log(LOG + sessionId + "/" + procedure.id() + "/" + safe(act.label()) + ".seconds", took);

            if (!result.successful()) {
                note("'" + act.label() + "' did not complete (" + result.getFailedCommands().size() + " failed)");
            } else if (took >= act.timeoutSeconds() - 1e-3) {
                // A step that ran exactly to its cap probably did not finish what it was doing, and a
                // measurement taken from it should be read with that in mind.
                note(String.format("'%s' hit its %.0f s cap; treat its data as incomplete",
                        act.label(), act.timeoutSeconds()));
            }
        }

        private void finish() {
            state = State.DONE;
            stepLabel = "";
            input.clear();
            CatalystLog.log(LOG + "State", state.name());
            CatalystLog.log(LOG + "Progress", 1.0);

            double took = Double.isNaN(startedAt) ? 0 : Timer.getTimestamp() - startedAt;
            AlertManager.getInstance().info("FieldLab", String.format(
                    "session %s finished: %d procedures in %.0f min, %d measurements, %d notes."
                            + " Pull the .wpilog and run tools/fieldlab.py.",
                    sessionId, campaign.procedures().size(), took / 60.0, measured.size(), notes.size()));

            // Cleared on purpose, and only here: a finished campaign must not resume into itself the
            // next time the op mode is enabled, which would look like the campaign refusing to run.
            checkpoint.clear().ifPresent(CampaignRunner.this::note);
        }
    }

    private static Time seconds(double s) {
        return Seconds.of(s);
    }

    /** A label reduced to something safe to put in a NetworkTables key. */
    private static String safe(String label) {
        return label.replaceAll("[^A-Za-z0-9._-]+", "_");
    }
}
