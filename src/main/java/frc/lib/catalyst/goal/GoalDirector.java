package frc.lib.catalyst.goal;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.mechanisms.SuperstructureCoordinator;
import frc.lib.catalyst.statemachine.robot.SuperstructureLike;

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
 * A pursued goal {@linkplain Command#getRequirements() requires} the same
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
    private final NetworkTable table;

    private volatile Goal activeGoal;        // the goal currently being pursued (nullable)
    private volatile boolean activeReady;    // cached readiness of the active goal

    private GoalDirector(SuperstructureLike coordinator, Goal defaultGoal) {
        this.coordinator = coordinator;
        this.defaultGoal = defaultGoal;
        this.table = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("Goal");
        this.activeGoal = defaultGoal;
        publishIdle();
    }

    /**
     * Build a command that pursues {@code goal}: it drives the superstructure
     * to the goal's state (if any) while running the goal's setup work, and
     * continuously publishes readiness. The command holds until interrupted,
     * so bind it with {@code whileTrue} (hold) or {@code onTrue} (latch).
     */
    public Command pursue(Goal goal) {
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

        return transition.alongWith(setup, monitor)
                .beforeStarting(() -> onPursueStart(goal))
                .finallyDo(interrupted -> onPursueEnd(goal))
                .withName("Goal.Pursue(" + goal.name() + ")");
    }

    /**
     * Pursue the configured default goal (the safe fallback). No-op command if
     * no default was configured.
     */
    public Command pursueDefault() {
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
        table.getEntry("Active").setString(goal.name());
        table.getEntry("TargetState").setString(
                goal.hasSuperstructureState() ? goal.superstructureState() : "—");
        table.getEntry("Ready").setBoolean(false);
        table.getEntry("WhyNotReady").setString("starting");
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

    private void updateReadiness(Goal goal) {
        boolean ready;
        String why;
        if (goal.hasReadinessTest()) {
            ready = goal.readyNow();
            why = ready ? "" : "setup not complete";
        } else if (coordinator != null && goal.hasSuperstructureState()) {
            // No explicit test: ready when the superstructure has arrived.
            ready = coordinator.isAtState(goal.superstructureState());
            why = ready ? "" : "in transition";
        } else {
            // Nothing to wait on — ready as soon as it's running.
            ready = true;
            why = "";
        }
        activeReady = ready;
        table.getEntry("Ready").setBoolean(ready);
        table.getEntry("WhyNotReady").setString(why);
    }

    private void publishIdle() {
        Goal g = defaultGoal;
        table.getEntry("Active").setString(g == null ? "None" : g.name());
        table.getEntry("TargetState").setString(
                (g != null && g.hasSuperstructureState()) ? g.superstructureState() : "—");
        table.getEntry("Ready").setBoolean(false);
        table.getEntry("WhyNotReady").setString("idle");
    }

    /** Start building a director. */
    public static Builder builder() {
        return new Builder();
    }

    /** Fluent builder for {@link GoalDirector}. */
    public static final class Builder {
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
        public Builder defaultGoal(Goal defaultGoal) {
            this.defaultGoal = defaultGoal;
            return this;
        }

        /** Build the director. */
        public GoalDirector build() {
            return new GoalDirector(coordinator, defaultGoal);
        }
    }
}
