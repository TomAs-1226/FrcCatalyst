package frc.lib.catalyst.statemachine.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.statemachine.Binding;

import java.util.Set;

/**
 * A {@link Binding} that drives real hardware through WPILib {@link Command}s.
 *
 * <p>All nine Catalyst mechanism bindings in {@link frc.lib.catalyst.statemachine.mech.Mechanisms}
 * are ordinary implementations of this interface — there is no special-casing, so a team's own
 * subsystem is exactly as first-class as {@code LinearMechanism}.
 *
 * <p>Commands returned here are <b>hosted</b> by {@link GoalRunner} rather than scheduled:
 * {@code initialize}/{@code execute}/{@code isFinished}/{@code end} are called directly. That is
 * why the contract below is strict.
 *
 * @param <G> the goal type; see {@link Binding}
 * @since 1.2.0
 */
public interface Actuator<G> extends Binding<G> {

    /**
     * Command that drives the mechanism toward {@code goal}.
     *
     * <p>Contract, enforced by a probe at {@link Superstructure.Builder#build()}:
     * <ul>
     *   <li>Its requirements must be a subset of {@link #requirements()}.</li>
     *   <li>It must return a <b>fresh instance on every call</b> — a hosted command is
     *       initialised more than once over a match.</li>
     *   <li>It must either never end, or end with a <em>persistent</em> effect (a
     *       {@code runOnce} that latches a Phoenix control request counts).</li>
     * </ul>
     *
     * <p>A {@code goToAndWait}-style factory must never be used. Arrival is the state
     * machine's job, decided by {@link #atGoal}, never by a command ending.
     */
    Command pursueCommand(G goal);

    /**
     * Command to run once {@link #atGoal} first becomes {@code true}.
     *
     * <p>Return {@code null} to keep running the pursue command — correct for closed-loop
     * holds like {@code LinearMechanism.holdPosition()}, which keeps re-driving Motion Magic
     * to its setpoint.
     *
     * <p>Non-null is <b>required</b> for open-loop mechanisms that would otherwise keep
     * driving into a hard stop after arrival: a winch {@code EXTEND}, a roller {@code EJECT},
     * any duty-cycle goal.
     */
    default Command holdCommand(G goal) { return null; }

    /** Subsystems this actuator owns. Exactly one for all nine Catalyst bindings. */
    Set<Subsystem> requirements();

    /**
     * If the pursue command finishes while {@link #atGoal} is still {@code false}, re-run its
     * {@code initialize()} every N loops. {@code 0} disables.
     *
     * <p>The pneumatic binding uses 25 (~0.5 s at 50 Hz) so an actuation that was silently
     * refused for low pressure is retried instead of quietly never happening.
     */
    default int reassertPeriodLoops() { return 0; }
}
