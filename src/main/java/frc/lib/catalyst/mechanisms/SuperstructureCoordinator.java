package frc.lib.catalyst.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Coordinates multiple mechanisms into safe state transitions.
 * Prevents mechanical collisions by enforcing ordering constraints
 * when moving between states.
 *
 * <p>A "state" is a named configuration of all mechanism positions.
 * For example, "STOW" might mean elevator=0, arm=0, wrist=0,
 * while "SCORE_HIGH" means elevator=1.1, arm=90, wrist=45.
 *
 * <p>The coordinator ensures mechanisms move in the right order.
 * For example, when going from STOW to SCORE_HIGH:
 * 1. Move elevator up first (so arm doesn't hit the chassis)
 * 2. Then deploy the arm
 * 3. Then angle the wrist
 *
 * <p>Example usage:
 * <pre>{@code
 * SuperstructureCoordinator coord = new SuperstructureCoordinator()
 *     .withLinear("elevator", elevator)
 *     .withRotational("arm", arm)
 *     .defineState("STOW")
 *         .setLinear("elevator", 0.0)
 *         .setRotational("arm", 0.0)
 *         .done()
 *     .defineState("SCORE_HIGH")
 *         .setLinear("elevator", 1.1)
 *         .setRotational("arm", 90.0)
 *         .done()
 *     .addTransitionRule("STOW", "SCORE_HIGH", (from, to) -> {
 *         // When going to SCORE_HIGH, raise elevator first, then arm
 *         return to.goToLinear("elevator")
 *             .andThen(Commands.waitUntil(elevator.atPositionTrigger(1.1, 0.1)))
 *             .andThen(to.goToRotational("arm"));
 *     });
 *
 * // Bind to button
 * controller.a().onTrue(coord.transitionTo("SCORE_HIGH"));
 * controller.b().onTrue(coord.transitionTo("STOW"));
 * }</pre>
 */
public class SuperstructureCoordinator {

    private final Map<String, LinearMechanism> linearMechanisms = new HashMap<>();
    private final Map<String, RotationalMechanism> rotationalMechanisms = new HashMap<>();
    private final Map<String, StateDefinition> states = new HashMap<>();
    private final Map<String, TransitionRule> transitionRules = new HashMap<>();

    private String currentState = "";

    /** Register a linear mechanism. */
    public SuperstructureCoordinator withLinear(String key, LinearMechanism mechanism) {
        linearMechanisms.put(key, mechanism);
        return this;
    }

    /** Register a rotational mechanism. */
    public SuperstructureCoordinator withRotational(String key, RotationalMechanism mechanism) {
        rotationalMechanisms.put(key, mechanism);
        return this;
    }

    /** Begin defining a state. */
    public StateBuilder defineState(String stateName) {
        return new StateBuilder(stateName);
    }

    /**
     * Add a custom transition rule between two states.
     * @param fromState source state name ("*" for any state)
     * @param toState target state name
     * @param rule function that creates the transition command
     */
    public SuperstructureCoordinator addTransitionRule(String fromState, String toState,
                                                        TransitionRule rule) {
        transitionRules.put(fromState + "->" + toState, rule);
        return this;
    }

    /**
     * Create a command to transition to the target state.
     * If a custom transition rule exists, it is used.
     * Otherwise, all mechanisms move simultaneously (parallel).
     */
    public Command transitionTo(String targetState) {
        StateDefinition target = states.get(targetState);
        if (target == null) {
            throw new IllegalArgumentException(
                    "Unknown state: " + targetState + ". Available: " + states.keySet());
        }

        // Check for custom transition rule
        String ruleKey = currentState + "->" + targetState;
        String wildcardKey = "*->" + targetState;
        TransitionRule rule = transitionRules.getOrDefault(ruleKey,
                transitionRules.get(wildcardKey));

        Command transition;
        if (rule != null) {
            transition = rule.createTransition(
                    states.getOrDefault(currentState, target), target);
        } else {
            // Default: move all mechanisms in parallel
            transition = createParallelTransition(target);
        }

        return transition.beforeStarting(() -> {})
                .finallyDo(() -> currentState = targetState)
                .withName("Superstructure.To(" + targetState + ")");
    }

    /** Get the name of the current state. */
    public String getCurrentState() {
        return currentState;
    }

    /** Create a trigger that fires when in a specific state. */
    public Trigger inState(String stateName) {
        return new Trigger(() -> currentState.equals(stateName));
    }

    /** Check if all mechanisms are at their target positions for a given state. */
    public boolean isAtState(String stateName) {
        StateDefinition state = states.get(stateName);
        if (state == null) return false;

        for (var entry : state.linearPositions.entrySet()) {
            LinearMechanism mech = linearMechanisms.get(entry.getKey());
            if (mech != null && !mech.atPosition(entry.getValue(), 0.02)) return false;
        }
        for (var entry : state.rotationalPositions.entrySet()) {
            RotationalMechanism mech = rotationalMechanisms.get(entry.getKey());
            if (mech != null && !mech.atAngle(entry.getValue(), 2.0)) return false;
        }
        return true;
    }

    private Command createParallelTransition(StateDefinition target) {
        Command combined = Commands.none();

        for (var entry : target.linearPositions.entrySet()) {
            LinearMechanism mech = linearMechanisms.get(entry.getKey());
            if (mech != null) {
                combined = combined.alongWith(mech.goTo(entry.getValue()));
            }
        }
        for (var entry : target.rotationalPositions.entrySet()) {
            RotationalMechanism mech = rotationalMechanisms.get(entry.getKey());
            if (mech != null) {
                combined = combined.alongWith(mech.goTo(entry.getValue()));
            }
        }
        return combined;
    }

    // --- Inner Classes ---

    /** Defines mechanism positions for a named state. */
    public static class StateDefinition {
        final String name;
        final Map<String, Double> linearPositions = new HashMap<>();
        final Map<String, Double> rotationalPositions = new HashMap<>();

        StateDefinition(String name) {
            this.name = name;
        }

        /** Create a goTo command for a linear mechanism in this state. */
        public Command goToLinear(String key, Map<String, LinearMechanism> mechanisms) {
            Double pos = linearPositions.get(key);
            LinearMechanism mech = mechanisms.get(key);
            if (pos == null || mech == null) return Commands.none();
            return mech.goTo(pos);
        }

        /** Create a goTo command for a rotational mechanism in this state. */
        public Command goToRotational(String key, Map<String, RotationalMechanism> mechanisms) {
            Double pos = rotationalPositions.get(key);
            RotationalMechanism mech = mechanisms.get(key);
            if (pos == null || mech == null) return Commands.none();
            return mech.goTo(pos);
        }
    }

    /** Builder for defining a state's mechanism positions. */
    public class StateBuilder {
        private final StateDefinition state;

        StateBuilder(String name) {
            this.state = new StateDefinition(name);
        }

        /** Set the target position for a linear mechanism in this state (meters). */
        public StateBuilder setLinear(String key, double positionMeters) {
            state.linearPositions.put(key, positionMeters);
            return this;
        }

        /** Set the target position for a rotational mechanism in this state (degrees). */
        public StateBuilder setRotational(String key, double positionDegrees) {
            state.rotationalPositions.put(key, positionDegrees);
            return this;
        }

        /** Finish defining this state and return to the coordinator. */
        public SuperstructureCoordinator done() {
            states.put(state.name, state);
            return SuperstructureCoordinator.this;
        }
    }

    /** Functional interface for custom transition rules. */
    @FunctionalInterface
    public interface TransitionRule {
        Command createTransition(StateDefinition fromState, StateDefinition toState);
    }
}
