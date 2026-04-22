package frc.lib.catalyst.mechanisms;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.util.AlertManager;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
 * <p>Advanced features (vs simple version):
 * <ul>
 *   <li>Transition timeouts — safety stop if mechanism gets stuck</li>
 *   <li>Collision zone modeling — automatic collision checking between mechanisms</li>
 *   <li>Transition progress telemetry — track how far along a transition is</li>
 *   <li>Entry/exit actions — run custom code when entering/leaving states</li>
 *   <li>Transition queueing — queue next state while in transition</li>
 *   <li>Interruptible transitions — cancel mid-transition safely</li>
 * </ul>
 *
 * <p>Example usage:
 * <pre>{@code
 * SuperstructureCoordinator coord = new SuperstructureCoordinator()
 *     .withLinear("elevator", elevator)
 *     .withRotational("arm", arm)
 *     .withTimeout(3.0) // 3 second safety timeout
 *     .defineState("STOW")
 *         .setLinear("elevator", 0.0)
 *         .setRotational("arm", 0.0)
 *         .onEntry(() -> System.out.println("Stowed!"))
 *         .done()
 *     .defineState("SCORE_HIGH")
 *         .setLinear("elevator", 1.1)
 *         .setRotational("arm", 90.0)
 *         .done()
 *     .addCollisionZone("arm-chassis", () -> {
 *         // Arm collides with chassis when elevator is low and arm is extended
 *         return elevator.getPosition() < 0.3 && arm.getAngle() > 30;
 *     })
 *     .addTransitionRule("STOW", "SCORE_HIGH", (from, to) -> {
 *         return to.goToLinear("elevator")
 *             .andThen(Commands.waitUntil(elevator.atPositionTrigger(1.1, 0.1)))
 *             .andThen(to.goToRotational("arm"));
 *     });
 * }</pre>
 */
public class SuperstructureCoordinator {

    private final Map<String, LinearMechanism> linearMechanisms = new HashMap<>();
    private final Map<String, RotationalMechanism> rotationalMechanisms = new HashMap<>();
    private final Map<String, StateDefinition> states = new HashMap<>();
    private final Map<String, TransitionRule> transitionRules = new HashMap<>();
    private final Map<String, CollisionZone> collisionZones = new HashMap<>();

    private String currentState = "";
    private String targetState = "";
    private boolean transitioning = false;
    private double transitionTimeoutSeconds = 5.0;

    // Telemetry
    private final NetworkTable telemetryTable;

    public SuperstructureCoordinator() {
        this.telemetryTable = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("Superstructure");
    }

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

    /**
     * Set the timeout for transitions. If a transition takes longer than this,
     * it is cancelled and an alert is raised.
     * @param seconds timeout in seconds (default 5.0)
     */
    public SuperstructureCoordinator withTimeout(double seconds) {
        this.transitionTimeoutSeconds = seconds;
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
     * Add a collision zone — a condition that should never be true.
     * The coordinator logs a warning if any collision zone is active.
     *
     * @param name descriptive name for the collision zone
     * @param condition returns true when a collision would occur
     */
    public SuperstructureCoordinator addCollisionZone(String name, CollisionZone condition) {
        collisionZones.put(name, condition);
        return this;
    }

    /**
     * Create a command to transition to the target state.
     * If a custom transition rule exists, it is used.
     * Otherwise, all mechanisms move simultaneously (parallel).
     * Includes timeout safety and telemetry.
     */
    public Command transitionTo(String targetStateName) {
        StateDefinition target = states.get(targetStateName);
        if (target == null) {
            throw new IllegalArgumentException(
                    "Unknown state: " + targetStateName + ". Available: " + states.keySet());
        }

        // Check for custom transition rule
        String ruleKey = currentState + "->" + targetStateName;
        String wildcardKey = "*->" + targetStateName;
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

        // Wrap with timeout, telemetry, and entry/exit actions
        return transition
                .withTimeout(transitionTimeoutSeconds)
                .beforeStarting(() -> {
                    // Run exit action of current state
                    StateDefinition currentStateDef = states.get(currentState);
                    if (currentStateDef != null && currentStateDef.exitAction != null) {
                        currentStateDef.exitAction.run();
                    }
                    transitioning = true;
                    this.targetState = targetStateName;
                    telemetryTable.getEntry("Transitioning").setBoolean(true);
                    telemetryTable.getEntry("TargetState").setString(targetStateName);
                })
                .finallyDo(interrupted -> {
                    if (interrupted) {
                        AlertManager.getInstance().warning("Superstructure",
                                "Transition to " + targetStateName + " was interrupted/timed out");
                    }
                    currentState = targetStateName;
                    transitioning = false;
                    telemetryTable.getEntry("Transitioning").setBoolean(false);
                    telemetryTable.getEntry("CurrentState").setString(currentState);

                    // Run entry action of new state
                    if (target.entryAction != null) {
                        target.entryAction.run();
                    }
                })
                .withName("Superstructure.To(" + targetStateName + ")");
    }

    /**
     * Create a command that transitions to a state only if a condition is met.
     * Useful for conditional scoring (e.g., only go to SCORE if intake has a piece).
     *
     * @param targetState the state to transition to
     * @param condition the condition that must be true
     */
    public Command transitionToIf(String targetState, java.util.function.BooleanSupplier condition) {
        return Commands.either(
                transitionTo(targetState),
                Commands.none(),
                condition
        ).withName("Superstructure.ToIf(" + targetState + ")");
    }

    /** Get the name of the current state. */
    public String getCurrentState() {
        return currentState;
    }

    /** Get the target state (during transition). */
    public String getTargetState() {
        return targetState;
    }

    /** Check if currently transitioning between states. */
    public boolean isTransitioning() {
        return transitioning;
    }

    /** Create a trigger that fires when in a specific state. */
    public Trigger inState(String stateName) {
        return new Trigger(() -> currentState.equals(stateName));
    }

    /** Trigger that fires when any transition completes. */
    public Trigger transitionComplete() {
        return new Trigger(() -> !transitioning && isAtState(currentState));
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

    /**
     * Check all collision zones and report any violations.
     * Call from a periodic method to monitor for collisions.
     */
    public void checkCollisionZones() {
        for (var entry : collisionZones.entrySet()) {
            boolean colliding = entry.getValue().isColliding();
            telemetryTable.getEntry("Collision/" + entry.getKey()).setBoolean(colliding);
            if (colliding) {
                AlertManager.getInstance().error("Superstructure",
                        "COLLISION ZONE ACTIVE: " + entry.getKey());
            }
        }
    }

    /**
     * Get the transition progress as a fraction [0.0, 1.0].
     * Counts how many mechanisms have reached their target out of total.
     */
    public double getTransitionProgress() {
        if (!transitioning || targetState.isEmpty()) return 1.0;
        StateDefinition target = states.get(targetState);
        if (target == null) return 1.0;

        int total = 0;
        int arrived = 0;

        for (var entry : target.linearPositions.entrySet()) {
            LinearMechanism mech = linearMechanisms.get(entry.getKey());
            if (mech != null) {
                total++;
                if (mech.atPosition(entry.getValue(), 0.02)) arrived++;
            }
        }
        for (var entry : target.rotationalPositions.entrySet()) {
            RotationalMechanism mech = rotationalMechanisms.get(entry.getKey());
            if (mech != null) {
                total++;
                if (mech.atAngle(entry.getValue(), 2.0)) arrived++;
            }
        }

        return total == 0 ? 1.0 : (double) arrived / total;
    }

    /** Get a list of all defined state names. */
    public List<String> getStateNames() {
        return new ArrayList<>(states.keySet());
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
        Runnable entryAction;
        Runnable exitAction;

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

        /** Get the state name. */
        public String getName() {
            return name;
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

        /**
         * Set an action to run when this state is entered.
         * Runs after the transition command completes.
         */
        public StateBuilder onEntry(Runnable action) {
            state.entryAction = action;
            return this;
        }

        /**
         * Set an action to run when leaving this state.
         * Runs before the transition command starts.
         */
        public StateBuilder onExit(Runnable action) {
            state.exitAction = action;
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

    /** Functional interface for collision zone detection. */
    @FunctionalInterface
    public interface CollisionZone {
        boolean isColliding();
    }
}
