package frc.lib.catalyst.behavior;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;

/**
 * Reactive action sequencer — runs a list of {@link Action}s in order,
 * checking each one's precondition <b>at the moment it's reached</b> and
 * taking a fallback when it can't start. This is the resilient-auto
 * pattern: if a game piece isn't where you expected, skip to the next
 * action or substitute a different one instead of blindly driving to an
 * empty spot.
 *
 * <p>The whole sequence honours a <b>bail condition</b> — when it goes
 * true (typically "match time running low") the sequence stops and an
 * optional bail action runs, e.g. "give up chasing and go align + shoot".
 *
 * <p>Everything here is game-agnostic. The actions encapsulate the
 * game-specific work; the engine only sequences and reacts.
 *
 * <pre>{@code
 * Command auto = BehaviorEngine.sequence("ThreePiece")
 *     .then(scorePreload)
 *     .attempt(grabPieceA).orElse(grabPieceB)   // substitute if A isn't there
 *     .then(scoreIfHolding)
 *     .attempt(grabPieceC).orElseSkip()          // skip if C isn't there
 *     .then(scoreIfHolding)
 *     .bailWhen(() -> RobotState.matchTimeRemaining() < 3.0)
 *     .onBail(alignAndShoot)                      // last-ditch points
 *     .build();
 *
 * autonomousCommand = auto;
 * }</pre>
 *
 * <p>Live introspection publishes to
 * {@code /Catalyst/Behavior/<name>/{Step, Action, FellBack, Bailed}} so you
 * can see on the dashboard exactly what the sequence is doing and why.
 */
public final class BehaviorEngine {

    private enum Fallback { SKIP, ABORT, SUBSTITUTE }

    private static final class Step {
        final Action action;
        Fallback fallback = Fallback.SKIP;
        Action substitute; // only for SUBSTITUTE

        Step(Action action) {
            this.action = action;
        }
    }

    private BehaviorEngine() {}

    public static Builder sequence(String name) {
        return new Builder(name);
    }

    public static class Builder {
        private final String name;
        private final List<Step> steps = new ArrayList<>();
        private BooleanSupplier bailCondition = () -> false;
        private Action bailAction;
        private double deadlineSeconds = -1;

        private Builder(String name) {
            this.name = name;
        }

        /** Run an action; if its precondition fails when reached, skip it. */
        public Builder then(Action action) {
            steps.add(new Step(action));
            return this;
        }

        /**
         * Begin a step whose fallback you set with the next {@code orElse*}
         * call. Defaults to skip if you don't chain one.
         */
        public Builder attempt(Action action) {
            steps.add(new Step(action));
            return this;
        }

        /** If the last {@link #attempt} can't start, run this action instead. */
        public Builder orElse(Action substitute) {
            Step s = last();
            s.fallback = Fallback.SUBSTITUTE;
            s.substitute = substitute;
            return this;
        }

        /** If the last {@link #attempt} can't start, skip it (explicit). */
        public Builder orElseSkip() {
            last().fallback = Fallback.SKIP;
            return this;
        }

        /** If the last {@link #attempt} can't start, end the whole sequence. */
        public Builder orElseAbort() {
            last().fallback = Fallback.ABORT;
            return this;
        }

        /** Stop the sequence when this becomes true (e.g. low match time). */
        public Builder bailWhen(BooleanSupplier condition) {
            this.bailCondition = condition;
            return this;
        }

        /**
         * Convenience: bail this many seconds after the sequence is
         * <em>scheduled</em> (not built), so it's safe to construct the
         * command once in robotInit and run it later.
         */
        public Builder deadline(double seconds) {
            this.deadlineSeconds = seconds;
            return this;
        }

        /** Action to run once if the sequence bails (or an attempt aborts). */
        public Builder onBail(Action action) {
            this.bailAction = action;
            return this;
        }

        private Step last() {
            if (steps.isEmpty()) {
                throw new IllegalStateException("orElse* must follow an attempt(...)");
            }
            return steps.get(steps.size() - 1);
        }

        public Command build() {
            NetworkTable nt = NetworkTableInstance.getDefault()
                    .getTable("Catalyst").getSubTable("Behavior").getSubTable(name);

            AtomicBoolean abort = new AtomicBoolean(false);
            AtomicBoolean completed = new AtomicBoolean(false);
            AtomicInteger stepIndex = new AtomicInteger(0);
            // Start time stamped at schedule time so deadline() measures from
            // when the sequence actually runs, not when it was built.
            final double[] startTime = { 0.0 };

            // Union of all requirements so each deferred step reserves the
            // right subsystems regardless of which branch it takes.
            Set<Subsystem> reqs = new HashSet<>();
            for (Step s : steps) {
                reqs.addAll(s.action.requirements());
                if (s.substitute != null) reqs.addAll(s.substitute.requirements());
            }

            List<Command> stepCommands = new ArrayList<>(steps.size());
            for (int i = 0; i < steps.size(); i++) {
                final Step step = steps.get(i);
                final int idx = i;
                stepCommands.add(Commands.defer(() -> {
                    nt.getEntry("Step").setInteger(idx);
                    stepIndex.set(idx);
                    if (step.action.canStart()) {
                        nt.getEntry("Action").setString(step.action.name());
                        nt.getEntry("FellBack").setBoolean(false);
                        return step.action.toCommand();
                    }
                    switch (step.fallback) {
                        case SUBSTITUTE:
                            if (step.substitute != null && step.substitute.canStart()) {
                                nt.getEntry("Action").setString(step.substitute.name() + " (sub)");
                                nt.getEntry("FellBack").setBoolean(true);
                                return step.substitute.toCommand();
                            }
                            nt.getEntry("Action").setString("(skipped " + step.action.name() + ")");
                            nt.getEntry("FellBack").setBoolean(true);
                            return Commands.none();
                        case ABORT:
                            nt.getEntry("Action").setString("(abort at " + step.action.name() + ")");
                            nt.getEntry("FellBack").setBoolean(true);
                            return Commands.runOnce(() -> abort.set(true));
                        case SKIP:
                        default:
                            nt.getEntry("Action").setString("(skipped " + step.action.name() + ")");
                            nt.getEntry("FellBack").setBoolean(true);
                            return Commands.none();
                    }
                }, reqs));
            }

            Command main = Commands.sequence(stepCommands.toArray(new Command[0]))
                    .andThen(Commands.runOnce(() -> completed.set(true)));

            // End the sequence early on bail OR abort. The deadline (if set) is
            // measured from schedule time via startTime.
            Command guarded = main.until(() ->
                    abort.get()
                            || safeBool(bailCondition)
                            || (deadlineSeconds > 0
                                && Timer.getFPGATimestamp() - startTime[0] > deadlineSeconds));

            // After the (possibly interrupted) sequence, run the bail action
            // only if we didn't finish cleanly.
            Command bailCmd = bailAction != null ? bailAction.toCommand() : Commands.none();
            Command full = guarded.andThen(Commands.either(
                    Commands.none(),
                    bailCmd.beforeStarting(() -> nt.getEntry("Bailed").setBoolean(true)),
                    completed::get));

            return full.beforeStarting(() -> {
                abort.set(false);
                completed.set(false);
                stepIndex.set(0);
                startTime[0] = Timer.getFPGATimestamp();
                nt.getEntry("Bailed").setBoolean(false);
                nt.getEntry("Step").setInteger(-1);
                nt.getEntry("Action").setString("(start)");
            }).withName("Behavior:" + name);
        }

        private static boolean safeBool(BooleanSupplier s) {
            try {
                return s.getAsBoolean();
            } catch (Throwable t) {
                return false;
            }
        }
    }
}
