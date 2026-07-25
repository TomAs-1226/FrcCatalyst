package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.ServoMechanism;
import frc.lib.catalyst.statemachine.goals.ServoGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

/**
 * Binds a {@link ServoMechanism} into a state machine.
 *
 * <p>A servo is open-loop, so arrival is a settle timer rather than a sensor read — this binding
 * reports {@link #observable} as {@code false} so nobody reading a log mistakes the timer for a
 * measurement. Preset goals are resolved to degrees once, at {@link #validate} time, so an unknown
 * preset or an out-of-range angle fails the build on a laptop instead of throwing from inside a
 * command factory during a match.
 *
 * @since 1.3.0
 */
public final class ServoBinding implements Actuator<ServoGoal> {

    private final ServoMechanism mechanism;
    private final String key;
    private final Set<Subsystem> requirements;

    /** Angles resolved once at validate() time, so pursue/atGoal use the same clamped value. */
    private final Map<ServoGoal, Double> resolved = new HashMap<>();

    /**
     * @param mechanism the servo to drive; must not be {@code null}
     * @param key       stable, unique, log-safe telemetry key; must not be blank
     */
    public ServoBinding(ServoMechanism mechanism, String key) {
        if (mechanism == null) throw new IllegalArgumentException("ServoBinding requires a ServoMechanism");
        if (key == null || key.isBlank()) throw new IllegalArgumentException("ServoBinding requires a key");
        this.mechanism = mechanism;
        this.key = key;
        this.requirements = Set.of(mechanism);
    }

    @Override
    public String key() {
        return key;
    }

    @Override
    public String kind() {
        return "servo";
    }

    @Override
    public String unit() {
        return "deg";
    }

    @Override
    public Command pursueCommand(ServoGoal goal) {
        return mechanism.goTo(target(goal));
    }

    // holdCommand defaults to null: goTo already re-asserts the angle every loop, so the servo holds.

    @Override
    public Set<Subsystem> requirements() {
        return requirements;
    }

    @Override
    public boolean atGoal(ServoGoal goal, double secondsSinceApplied) {
        // Open-loop: there is nothing to sense, so arrival is "the settle window has elapsed".
        return secondsSinceApplied >= goal.settleSeconds();
    }

    @Override
    public boolean observable(ServoGoal goal) {
        return false;   // a timer, not a sensor — say so
    }

    @Override
    public double measured() {
        return mechanism.getAngle();
    }

    @Override
    public double error(ServoGoal goal) {
        return mechanism.getAngle() - target(goal);
    }

    @Override
    public double tolerance(ServoGoal goal) {
        return Double.NaN;   // no meaningful tolerance for an open-loop settle
    }

    @Override
    public String label(ServoGoal goal) {
        return goal.preset() != null ? goal.preset() : String.format("%.0fdeg", target(goal));
    }

    @Override
    public String detail(ServoGoal goal) {
        return String.format("%s -> %.1f deg (settle %.2fs)", label(goal), target(goal), goal.settleSeconds());
    }

    @Override
    public void validate(ServoGoal goal, Consumer<String> problems) {
        double degrees;
        if (goal.preset() != null) {
            Double p = mechanism.getNamedPositions().get(goal.preset());
            if (p == null) {
                problems.accept("unknown servo position '" + goal.preset() + "'; known: "
                        + mechanism.getNamedPositions().keySet());
                return;
            }
            degrees = p;
        } else {
            degrees = goal.degrees();
        }
        if (degrees < mechanism.getMinAngle() || degrees > mechanism.getMaxAngle()) {
            problems.accept("servo angle " + degrees + " deg is outside the mechanism's range ["
                    + mechanism.getMinAngle() + ", " + mechanism.getMaxAngle() + "]");
            return;
        }
        resolved.put(goal, degrees);
    }

    /** The clamped/resolved target angle — cached from validate(), with a safe fallback. */
    private double target(ServoGoal goal) {
        Double cached = resolved.get(goal);
        if (cached != null) return cached;
        double degrees = goal.preset() != null
                ? mechanism.getNamedPositions().getOrDefault(goal.preset(), mechanism.getMinAngle())
                : goal.degrees();
        return Math.max(mechanism.getMinAngle(), Math.min(mechanism.getMaxAngle(), degrees));
    }
}
