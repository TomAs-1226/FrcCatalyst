package frc.lib.catalyst.goal;

import frc.lib.catalyst.command.CatalystCommand;
import frc.lib.catalyst.logging.CatalystLog;
import org.wpilib.command3.Command;
import frc.lib.catalyst.command.Commands;
import org.wpilib.command3.Trigger;
import frc.lib.catalyst.mechanisms.SuperstructureCoordinator;
import frc.lib.catalyst.statemachine.robot.SuperstructureLike;
import frc.lib.catalyst.identity.CatalystFeatures;

/**
 * The optional "give the robot an intent, it figures out the details" layer.
 * A {@code GoalDirector} turns a {@link Goal} into a runnable command that
 * drives the superstructure to the right state, runs the goal's setup work,
 * reports readiness, and hands control back to the driver the moment they
 * want it.
 *
 * <p>This is the capstone that ties together pieces you already have
 * ({@link SuperstructureCoordinator}, mechanisms, {@code AimingSolver},
 * {@code RobotSafety}). It is deliberately thin and <b>completely optional</b>
 * — skip this package entirely and every other part of Catalyst is unchanged.
 *
 * <h2>Driver override is free</h2>
 * A pursued goal {@linkplain Command#requirements() requires} the same
 * subsystems its transition and setup touch (via the coordinator and your
 * setup command). So the instant the driver triggers anything that needs
 * those subsystems — including their default commands — WPILib interrupts the
 * goal. There is no special "abort" path to get wrong: it's the standard
 * command-requirement model. Bind a goal with {@code whileTrue} and releasing
 * the button ends it; bind with {@code onTrue} and it latches until something
 * else claims the subsystems.
 *
 * <h2>Telemetry</h2>
 * Everything publishes under {@code /Catalyst/Goal}:
 * <ul>
 *   <li>{@code Active} — the goal currently being pursued (or the default)</li>
 *   <li>{@code TargetState} — the superstructure state it's driving toward</li>
 *   <li>{@code Ready} — is the active goal ready (per its readiness test)?</li>
 *   <li>{@code WhyNotReady} — a short reason when it isn't</li>
 * </ul>
 * so you can watch on the dashboard exactly what the robot thinks it's doing
 * and why it isn't ready yet.
 *
 * <h2>Example</h2>
 * <pre>{@code
 * GoalDirector director = GoalDirector.builder()
 *     .coordinator(superstructure)   // optional
 *     .defaultGoal(STOW)             // safe fallback (optional)
 *     .build();
 *
 * // Hold to pursue, release to hand control back to the driver:
 * driver.y().whileTrue(director.pursue(SCORE_HIGH));
 * driver.a().whileTrue(director.pursue(INTAKE));
 *
 * // Buzz the driver when the active goal becomes ready to fire:
 * director.readyTrigger().onTrue(rumble.shortPulse());
 * }</pre>
 */
public class GoalDirector {

    private final SuperstructureLike coordinator;          // nullable
    private final Goal defaultGoal;                          // nullable
    /**
     * Key prefix under CatalystLog's root. {@code Goal/} by default, which is byte-identical to the
     * table this used to write to directly; {@code Goal/<name>/} when the director is named. Two
     * directors on one robot - a scoring one and a climbing one, say - both wrote {@code Goal/Active}
     * and each overwrote the other every loop, so the dashboard showed whichever ran last.
     */
    private final String key;

    // Last published values. Every one of these is stable for seconds at a time, and this runs in
    // the pursue command's monitor at loop rate, so the unchanged writes were pure cost.
    private String lastActive;
    private String lastTargetState;
    private Boolean lastReady;
    private String lastWhy;

    private volatile Goal activeGoal;        // the goal currently being pursued (nullable)
    private volatile boolean activeReady;    // cached readiness of the active goal

    private GoalDirector(SuperstructureLike coordinator, Goal defaultGoal, String name) {
        this.coordinator = coordinator;
        this.defaultGoal = defaultGoal;
        this.key = name == null || name.isBlank() ? "Goal/" : "Goal/" + name + "/";
        this.activeGoal = defaultGoal;
        publishIdle();
    }

    /**
     * Build a command that pursues {@code goal}: it drives the superstructure
     * to the goal's state (if any) while running the goal's setup work, and
     * continuously publishes readiness. The command holds until interrupted,
     * so bind it with {@code whileTrue} (hold) or {@code onTrue} (latch).
     */
    public CatalystCommand pursue(Goal goal) {
        Command transition = (coordinator != null && goal.hasSuperstructureState())
                ? coordinator.transitionTo(goal.superstructureState())
                : Commands.none();

        Command setup = goal.hasSetup() ? goal.newSetupCommand() : Commands.none();
        if (setup == null) {
            setup = Commands.none();
        }

        // A requirement-free monitor that never finishes. It keeps the whole
        // composition alive (so the goal "holds" until the driver releases it)
        // and publishes live readiness / why-not telemetry every loop.
        Command monitor = Commands.run(() -> updateReadiness(goal));

        return CatalystCommand.of(transition).together(setup, monitor)
                .beforeStarting(() -> onPursueStart(goal))
                .finallyDo(interrupted -> onPursueEnd(goal))
                .withName("Goal.Pursue(" + goal.name() + ")");
    }

    /**
     * Pursue the configured default goal (the safe fallback). No-op command if
     * no default was configured.
     */
    public CatalystCommand pursueDefault() {
        return defaultGoal == null
                ? Commands.none().withName("Goal.NoDefault")
                : pursue(defaultGoal);
    }

    /** Is the currently-active goal ready (per its readiness test)? */
    public boolean isReady() {
        return activeReady;
    }

    /** The name of the goal currently being pursued (or the default). */
    public String activeGoalName() {
        Goal g = activeGoal;
        return g == null ? "None" : g.name();
    }

    /**
     * A trigger that fires while the active goal is ready. Handy for driver
     * feedback: {@code director.readyTrigger().onTrue(rumble.shortPulse())}.
     */
    public Trigger readyTrigger() {
        return new Trigger(this::isReady);
    }

    /** A trigger that fires while {@code goal} is the active goal. */
    public Trigger pursuing(Goal goal) {
        return new Trigger(() -> activeGoal == goal);
    }

    // --- internals ---

    private void onPursueStart(Goal goal) {
        activeGoal = goal;
        activeReady = false;
        publishActive(goal.name());
        publishTargetState(goal.hasSuperstructureState() ? goal.superstructureState() : "—");
        publishReady(false);
        publishWhy("starting");
    }

    private void onPursueEnd(Goal goal) {
        // Only fall back to idle telemetry if this goal is still the active one
        // (a newer goal may have already taken over).
        if (activeGoal == goal) {
            activeGoal = defaultGoal;
            activeReady = false;
            publishIdle();
        }
    }

    /**
     * Package-private, like {@link Goal#readyNow()} and for the same reason: the guard below is the
     * whole point of this method and a test must be able to reach it without a scheduler.
     */
    void updateReadiness(Goal goal) {
        boolean ready;
        String why;
        // Both branches below call code the team wrote - a readiness lambda, and a coordinator whose
        // state table the team populated. This runs inside the pursue command's monitor, so a throw
        // from either used to propagate into the scheduler and take the command down. A goal that
        // cannot tell you whether it is ready is simply not ready, and says so.
        if (goal.hasReadinessTest()) {
            try {
                ready = goal.readyNow();
                why = ready ? "" : "setup not complete";
            } catch (Throwable t) {
                ready = false;
                why = "readiness test failed: " + t;
            }
        } else if (coordinator != null && goal.hasSuperstructureState()) {
            // No explicit test: ready when the superstructure has arrived.
            try {
                ready = coordinator.isAtState(goal.superstructureState());
                why = ready ? "" : "in transition";
            } catch (Throwable t) {
                ready = false;
                why = "coordinator failed: " + t;
            }
        } else {
            // Nothing to wait on — ready as soon as it's running.
            ready = true;
            why = "";
        }
        activeReady = ready;
        publishReady(ready);
        publishWhy(why);
    }

    private void publishIdle() {
        Goal g = defaultGoal;
        publishActive(g == null ? "None" : g.name());
        publishTargetState((g != null && g.hasSuperstructureState()) ? g.superstructureState() : "—");
        publishReady(false);
        publishWhy("idle");
    }

    // --- telemetry, published only when it changes ---

    private void publishActive(String value) {
        if (!value.equals(lastActive)) {
            lastActive = value;
            CatalystLog.log(key + "Active", value);
        }
    }

    private void publishTargetState(String value) {
        if (!value.equals(lastTargetState)) {
            lastTargetState = value;
            CatalystLog.log(key + "TargetState", value);
        }
    }

    private void publishReady(boolean value) {
        if (lastReady == null || lastReady != value) {
            lastReady = value;
            CatalystLog.log(key + "Ready", value);
        }
    }

    private void publishWhy(String value) {
        if (!value.equals(lastWhy)) {
            lastWhy = value;
            CatalystLog.log(key + "WhyNotReady", value);
        }
    }

    /** Start building a director. */
    public static Builder builder() {
        return new Builder();
    }

    /** Fluent builder for {@link GoalDirector}. */
    public static final class Builder {
        private String name;
        private SuperstructureLike coordinator;
        private Goal defaultGoal;

        private Builder() {}

        /**
         * The superstructure coordinator goals drive. Optional — leave it out
         * and goals are pure setup-command intents (still get telemetry and
         * override for free).
         */
        public Builder coordinator(SuperstructureCoordinator coordinator) {
            this.coordinator = coordinator;
            return this;
        }

        /**
         * The superstructure goals drive, as of 1.2.0 accepting either the legacy
         * {@link SuperstructureCoordinator} or the new
         * {@link frc.lib.catalyst.statemachine.robot.Superstructure}.
         *
         * <p>Pointing a goal layer at the new state machine is a one-line change here; goals
         * still name their state as a {@code String}, so a director on the new engine and one on
         * the old coordinator remain interchangeable. With the new engine, readiness also becomes
         * truthful for free — {@code isAtState} maps to a measured, confirmed arrival rather than
         * to a flag that was set when a transition command ended for any reason at all.
         *
         * @since 1.2.0
         */
        public Builder superstructure(SuperstructureLike superstructure) {
            this.coordinator = superstructure;
            return this;
        }

        /**
         * The safe fallback goal — what the robot "wants" when nothing else is
         * requested (usually STOW). Used by {@link #pursueDefault()} and shown
         * as the idle {@code Active} goal. Optional.
         */
        /**
         * Namespace this director's telemetry as {@code /Catalyst/Goal/<name>/...}. Leave it unset
         * and the keys are exactly what they have always been. Set it on every director as soon as
         * a robot has more than one, or they overwrite each other.
         */
        public Builder name(String name) {
            this.name = name;
            return this;
        }

        public Builder defaultGoal(Goal defaultGoal) {
            this.defaultGoal = defaultGoal;
            return this;
        }

        /** Build the director. */
        public GoalDirector build() {
            /* The default goal names the director usefully: a robot with a "Stow" default and one
             * with a "Ready to score" default are describing different intents, and the goal is the
             * only thing at build time that says which. */
            CatalystFeatures.record(CatalystFeatures.GOAL_DIRECTOR,
                    defaultGoal == null ? null : defaultGoal.name());
            return new GoalDirector(coordinator, defaultGoal, name);
        }
    }
}
