package frc.lib.catalyst.behavior;

import frc.lib.catalyst.command.CatalystCommand;
import frc.lib.catalyst.logging.CatalystLog;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ToDoubleFunction;
import frc.lib.catalyst.identity.CatalystFeatures;

/**
 * Utility-based action selector — each loop it scores every registered
 * behaviour and runs the highest scorer whose precondition holds, switching
 * when a different behaviour starts winning.
 *
 * <p>This is the "experimental decision layer" done the field-appropriate
 * way: it's <b>utility AI</b>, not search. Every behaviour returns a number
 * ("how good is it to do this right now?"); the highest wins. It's instant,
 * inspectable (every score publishes to NT), and degrades gracefully — there's
 * no plan to fall apart, just a continuous "what's best now?".
 *
 * <p>The canonical use: an autonomous that chases scattered game pieces but
 * bails to a guaranteed score when time runs short. Express it as two
 * behaviours whose scores cross over at the deadline:
 *
 * <pre>{@code
 * Command auto = Strategist.named("FuelAuto")
 *     .add("ChasePiece", chaseNearestPiece,
 *          ctx -> (scored < goal && ctx.matchTimeRemaining() > 4.0
 *                  && vision.hasPieceTarget()) ? 10.0 : 0.0)
 *     .add("AlignAndShoot", alignAndShoot,
 *          ctx -> (scored >= goal || ctx.matchTimeRemaining() <= 4.0) ? 20.0 : 0.0)
 *     .build();
 * }</pre>
 *
 * <p>While there's time and pieces to chase, {@code ChasePiece} scores 10 and
 * wins. The moment match time drops to 4 s (or the goal is met),
 * {@code AlignAndShoot} jumps to 20 and the Strategist switches to it
 * mid-stride. No explicit state machine — the scores encode the strategy.
 *
 * <p>Introspection publishes to
 * {@code /Catalyst/Behavior/<name>/{Active, Scores/<behaviour>}}.
 */
public final class Strategist {

    private record Behavior(String name, Action action, ToDoubleFunction<BehaviorContext> scorer) {}

    private Strategist() {}

    public static Builder named(String name) {
        return new Builder(name);
    }

    public static class Builder {
        private final String name;
        private final List<Behavior> behaviors = new ArrayList<>();
        private double minScore = 0.0;
        private double switchMargin = 0.0;
        private double minDwellSeconds = 0.0;

        private Builder(String name) {
            this.name = name;
        }

        /**
         * Register a behaviour. The scorer returns its desirability right now;
         * the highest scorer above {@link #minScore} whose action can start is
         * run. Return 0 (or negative) to take the behaviour out of contention.
         */
        public Builder add(String label, Action action, ToDoubleFunction<BehaviorContext> scorer) {
            behaviors.add(new Behavior(label, action, scorer));
            return this;
        }

        /** Behaviours must score strictly above this to be eligible. Default 0. */
        /**
         * How far a challenger must beat the incumbent before control changes hands.
         *
         * <p>Utility selection has no memory: every loop the highest scorer wins, so two behaviours
         * whose scores cross repeatedly hand control back and forth at the loop rate. Each handover
         * cancels a command and schedules a new one, so the mechanism restarts fifty times a second
         * and finishes nothing. The robot twitches and nothing in the scores looks wrong.
         *
         * <p><b>Defaults to 0.0, which is exactly today's behaviour</b> — a non-zero default would
         * change what every existing robot does on a library upgrade. The units are your own score
         * units, which is why there is no safe default: only you know whether your scores run 0–1 or
         * 0–100.
         *
         * <p>Set too high this freezes the robot on its first choice instead of thrashing, which is
         * just as silent. Watch {@code /Catalyst/Behavior/<name>/HeldSeconds} — a stuck incumbent
         * shows up there as a number that keeps climbing.
         */
        public Builder switchMargin(double margin) {
            this.switchMargin = margin;
            return this;
        }

        /**
         * How long a behaviour keeps control before a challenger may take it, however far ahead.
         *
         * <p>Complements {@link #switchMargin(double)}: margin stops close scores trading places,
         * dwell stops any handover happening faster than a mechanism can usefully act.
         *
         * <p>Defaults to 0.0, today's behaviour. A behaviour that becomes <em>ineligible</em> or
         * drops below {@link #minScore(double)} is released immediately regardless — a dwell timer
         * must never hold on to something that cannot run.
         */
        public Builder minDwellSeconds(double seconds) {
            this.minDwellSeconds = seconds;
            return this;
        }

        public Builder minScore(double minScore) {
            this.minScore = minScore;
            return this;
        }

        public CatalystCommand build() {
            CatalystFeatures.record(CatalystFeatures.STRATEGIST, name);
            return CatalystCommand.of(new SelectorCommand(
                    name, List.copyOf(behaviors), minScore, switchMargin, minDwellSeconds));
        }
    }

    /**
     * The running selector. Requires no subsystems itself — it schedules the
     * winning behaviour's command (which reserves its own subsystems) and
     * cancels it when a different behaviour takes over.
     */
    private static final class SelectorCommand implements Command {
        private final List<Behavior> behaviors;
        private final double minScore;
        private final BehaviorArbiter arbiter;
        private final String key;
        /** Active and the switch reason are stable for seconds; this evaluates every loop. */
        private String lastActive;
        private String lastReason;
        private final String commandName;
        private BehaviorContext ctx;

        private String activeName = "";
        private Command activeCommand;

        SelectorCommand(String name, List<Behavior> behaviors, double minScore,
                        double switchMargin, double minDwellSeconds) {
            this.behaviors = behaviors;
            this.minScore = minScore;
            this.arbiter = new BehaviorArbiter(minScore, switchMargin, minDwellSeconds);
            this.key = "Behavior/" + name + "/";
            this.commandName = "Strategist:" + name;
        }

        @Override
        public String name() {
            return commandName;
        }

        /**
         * The selector reserves nothing itself. It schedules the winning behaviour's command,
         * which reserves its own mechanisms, and cancels it when a different behaviour wins.
         */
        @Override
        public java.util.Set<Mechanism> requirements() {
            return java.util.Set.of();
        }

        /**
         * Commands v3 replaces initialize/execute/end with one coroutine body. The former
         * {@code isFinished()} returned false, so this loops until cancelled; the former
         * {@code end(boolean)} became {@link #onCancel()}.
         */
        @Override
        public void run(Coroutine coroutine) {
            ctx = new BehaviorContext(Timer.getTimestamp());
            activeName = "";
            activeCommand = null;

            while (true) {
                evaluate();
                coroutine.yield();
            }
        }

        /** Published only when it changes - see the fields. */
        private void publishActive(String value) {
            if (!value.equals(lastActive)) {
                lastActive = value;
                CatalystLog.log(key + "Active", value);
            }
        }

        private void publishReason(String value) {
            if (value != null && !value.equals(lastReason)) {
                lastReason = value;
                CatalystLog.log(key + "LastSwitchReason", value);
            }
        }

        private void evaluate() {
            // Clear a finished behaviour so it can be re-evaluated next loop.
            if (activeCommand != null && !Scheduler.getDefault().isScheduledOrRunning(activeCommand)) {
                activeCommand = null;
                activeName = "";
            }

            List<BehaviorArbiter.Candidate> candidates = new java.util.ArrayList<>(behaviors.size());
            java.util.Map<String, Behavior> byName = new java.util.HashMap<>();
            for (Behavior b : behaviors) {
                double score = safeScore(b);
                CatalystLog.log(key + "Scores/" + b.name(), score);
                candidates.add(new BehaviorArbiter.Candidate(b.name(), score, safeCanStart(b)));
                byName.put(b.name(), b);
            }

            double now = Timer.getTimestamp();
            BehaviorArbiter.Decision decision = arbiter.decide(candidates, activeName, now);
            publishArbitration(now);

            if (!decision.switched()) {
                return;
            }

            // A handover. Every one of these cancels a running command and starts another, which is
            // why they are counted and published rather than left invisible.
            if (activeCommand != null) {
                Scheduler.getDefault().cancel(activeCommand);
                activeCommand = null;
            }
            if (decision.winner() == null) {
                activeName = "";
                publishActive("(none)");
                return;
            }
            activeCommand = byName.get(decision.winner()).action().toCommand();
            activeName = decision.winner();
            Scheduler.getDefault().schedule(activeCommand);
            publishActive(activeName);
        }

        @Override
        public void onCancel() {
            if (activeCommand != null) Scheduler.getDefault().cancel(activeCommand);
            activeCommand = null;
            activeName = "";
            publishActive("(stopped)");
        }

        /**
         * Publish how settled the decision is, whether or not the knobs are in use.
         *
         * <p>On by default and free: thrashing is invisible today, and a team that has not set a
         * margin is exactly the team that needs to see the switch rate. A frozen selector is visible
         * on the same keys, as a HeldSeconds that keeps climbing with a stale reason.
         */
        private void publishArbitration(double now) {
            // These three move every loop by construction, so gating them would only add a compare.
            CatalystLog.log(key + "Switches", arbiter.switches());
            CatalystLog.log(key + "SwitchesPerSecond", arbiter.switchesPerSecond(now));
            CatalystLog.log(key + "HeldSeconds", arbiter.heldSeconds(now));
            publishReason(arbiter.lastSwitchReason());
        }

        /** A scorer that throws must not be able to decide the match; nor must canStart(). */
        private boolean safeCanStart(Behavior b) {
            try {
                return b.action().canStart();
            } catch (Throwable t) {
                return false;
            }
        }

        private double safeScore(Behavior b) {
            try {
                return b.scorer().applyAsDouble(ctx);
            } catch (Throwable t) {
                return 0.0;
            }
        }
    }
}
