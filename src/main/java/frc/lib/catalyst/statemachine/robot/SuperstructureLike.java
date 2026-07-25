package frc.lib.catalyst.statemachine.robot;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

/**
 * The string-keyed contract that {@link frc.lib.catalyst.goal.GoalDirector} consumes.
 *
 * <p>All five signatures already existed verbatim on
 * {@link frc.lib.catalyst.mechanisms.SuperstructureCoordinator}, so declaring that class to
 * implement this interface is a one-word, source- and binary-compatible change — and it lets a
 * team move from the old coordinator to the new {@link Superstructure} without touching their
 * goal layer at all.
 *
 * @since 1.2.0
 */
public interface SuperstructureLike {

    /**
     * A command that drives the superstructure to the named state.
     *
     * <p>Must return a fresh, uncomposed command on every call — {@code GoalDirector} composes the
     * result with its own setup and monitor commands, and WPILib refuses to compose one command
     * instance twice.
     */
    Command transitionTo(String stateName);

    /** Is the superstructure at the named state right now? */
    boolean isAtState(String stateName);

    /** The current state's name, or {@code ""}. */
    default String getCurrentState() { return ""; }

    /** Is a transition in flight? */
    default boolean isTransitioning() { return false; }

    /** Every state name this superstructure knows. */
    default List<String> getStateNames() { return List.of(); }
}
