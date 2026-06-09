package frc.lib.catalyst.behavior;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A single atomic capability the robot has — "intake from the ground",
 * "drive to a scoring pose and score", "chase the nearest game piece".
 * The building block of the Catalyst behavior framework
 * ({@link BehaviorEngine}, {@link Strategist}, {@link Autopilot}).
 *
 * <p>An action is deliberately game-agnostic: it wraps a WPILib
 * {@link Command} together with the metadata the orchestration layers
 * need to reason about it —
 * <ul>
 *   <li>a <b>precondition</b> ("can this even start right now?"),</li>
 *   <li>a <b>success condition</b> ("did it accomplish its goal?"),</li>
 *   <li>and an <b>estimated cost</b> in seconds, for time budgeting.</li>
 * </ul>
 *
 * <p>The command is supplied as a factory ({@code Supplier<Command>}) so
 * the action can be run more than once — WPILib commands cannot be
 * rescheduled while running, so a fresh instance is built each time.
 *
 * <pre>{@code
 * Action grabNearestPiece = Action.named("GrabNearestPiece")
 *     .when(() -> vision.hasGamePieceTarget())          // precondition
 *     .run(() -> drive.driveToDetectedPiece()
 *                     .andThen(intake.intakeUntilPiece()))
 *     .until(claw::hasPiece)                             // success
 *     .estimatedSeconds(2.5)
 *     .build();
 * }</pre>
 */
public final class Action {

    private final String name;
    private final BooleanSupplier precondition;
    private final Supplier<Command> commandFactory;
    private final BooleanSupplier successCondition;
    private final double estimatedSeconds;
    private final Set<Subsystem> requirements;

    private Action(Builder b) {
        this.name = b.name;
        this.precondition = b.precondition;
        this.commandFactory = b.commandFactory;
        this.successCondition = b.successCondition;
        this.estimatedSeconds = b.estimatedSeconds;
        this.requirements = Set.copyOf(b.requirements);
    }

    /**
     * The subsystems this action's command will require. The orchestration
     * layers need these to build deferred commands correctly — declare every
     * subsystem the command touches.
     */
    public Set<Subsystem> requirements() {
        return requirements;
    }

    /** Display name (used in logs and NT introspection). */
    public String name() {
        return name;
    }

    /** True if the action's precondition currently holds. */
    public boolean canStart() {
        try {
            return precondition.getAsBoolean();
        } catch (Throwable t) {
            // A buggy precondition lambda shouldn't crash the whole strategy.
            return false;
        }
    }

    /** True if the action's success condition currently holds. */
    public boolean succeeded() {
        try {
            return successCondition.getAsBoolean();
        } catch (Throwable t) {
            return false;
        }
    }

    /** Best-guess duration in seconds, for time budgeting. */
    public double estimatedSeconds() {
        return estimatedSeconds;
    }

    /**
     * Build a fresh command for this action. The returned command runs the
     * supplied work and ends when either that work finishes or the success
     * condition is met, whichever comes first — so an action whose command
     * loops forever (e.g. "run intake") still terminates once it succeeds.
     */
    public Command toCommand() {
        Command work;
        try {
            work = commandFactory.get();
        } catch (Throwable t) {
            return Commands.none();
        }
        if (work == null) return Commands.none();
        // End on success OR command completion. If no explicit success
        // condition was given, this is just the work itself.
        return work.until(successCondition).withName("Action:" + name);
    }

    // ============================================================
    //                        BUILDER
    // ============================================================

    /** Start building an action with the given display name. */
    public static Builder named(String name) {
        return new Builder(name);
    }

    public static class Builder {
        private final String name;
        private BooleanSupplier precondition = () -> true;
        private Supplier<Command> commandFactory = Commands::none;
        private BooleanSupplier successCondition = () -> false;
        private double estimatedSeconds = 1.0;
        private final Set<Subsystem> requirements = new HashSet<>();

        private Builder(String name) {
            this.name = name;
        }

        /** Precondition — the action can only start when this is true. Default: always. */
        public Builder when(BooleanSupplier precondition) {
            this.precondition = precondition;
            return this;
        }

        /** Command factory — fresh command built each run. */
        public Builder run(Supplier<Command> commandFactory) {
            this.commandFactory = commandFactory;
            return this;
        }

        /**
         * Convenience for a command that is safe to reuse, or when you build
         * the command inline. Prefer {@link #run(Supplier)} for anything with
         * internal state.
         */
        public Builder run(Command command) {
            this.commandFactory = () -> command;
            return this;
        }

        /**
         * Success condition — when true the action is considered done and its
         * command is interrupted. Default: never (the command decides when it
         * finishes).
         */
        public Builder until(BooleanSupplier successCondition) {
            this.successCondition = successCondition;
            return this;
        }

        /** Estimated duration in seconds, for time budgeting. Default 1.0. */
        public Builder estimatedSeconds(double seconds) {
            this.estimatedSeconds = seconds;
            return this;
        }

        /**
         * Declare the subsystems the command requires. Needed so the
         * orchestration layers ({@link BehaviorEngine}, {@link Strategist},
         * {@link Autopilot}) can build deferred commands that reserve them
         * correctly. List every subsystem the command touches.
         */
        public Builder requires(Subsystem... subsystems) {
            for (Subsystem s : subsystems) this.requirements.add(s);
            return this;
        }

        public Action build() {
            return new Action(this);
        }
    }
}
