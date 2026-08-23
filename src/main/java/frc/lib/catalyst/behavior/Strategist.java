package frc.lib.catalyst.behavior;

import frc.lib.catalyst.command.CatalystCommand;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
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
        public Builder minScore(double minScore) {
            this.minScore = minScore;
            return this;
        }

        public CatalystCommand build() {
            CatalystFeatures.record(CatalystFeatures.STRATEGIST, name);
            return CatalystCommand.of(new SelectorCommand(name, List.copyOf(behaviors), minScore));
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
        private final NetworkTable nt;
        private final String commandName;
        private BehaviorContext ctx;

        private String activeName = "";
        private Command activeCommand;

        SelectorCommand(String name, List<Behavior> behaviors, double minScore) {
            this.behaviors = behaviors;
            this.minScore = minScore;
            this.nt = NetworkTableInstance.getDefault()
                    .getTable("Catalyst").getSubTable("Behavior").getSubTable(name);
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

        private void evaluate() {
            // Clear a finished behaviour so it can be re-evaluated next loop.
            if (activeCommand != null && !Scheduler.getDefault().isScheduledOrRunning(activeCommand)) {
                activeCommand = null;
                activeName = "";
            }

            Behavior best = null;
            double bestScore = minScore;
            for (Behavior b : behaviors) {
                double score = safeScore(b);
                nt.getSubTable("Scores").getEntry(b.name()).setDouble(score);
                if (score > bestScore && b.action().canStart()) {
                    bestScore = score;
                    best = b;
                }
            }

            if (best == null) {
                // Nothing wants to run — release whatever was running.
                if (activeCommand != null) {
                    Scheduler.getDefault().cancel(activeCommand);
                    activeCommand = null;
                }
                activeName = "";
                nt.getEntry("Active").setString("(none)");
                return;
            }

            if (!best.name().equals(activeName)) {
                if (activeCommand != null) Scheduler.getDefault().cancel(activeCommand);
                activeCommand = best.action().toCommand();
                activeName = best.name();
                Scheduler.getDefault().schedule(activeCommand);
                nt.getEntry("Active").setString(activeName);
            }
        }

        @Override
        public void onCancel() {
            if (activeCommand != null) Scheduler.getDefault().cancel(activeCommand);
            activeCommand = null;
            activeName = "";
            nt.getEntry("Active").setString("(stopped)");
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
