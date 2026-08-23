package frc.lib.catalyst.command;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.RobotBase;

/**
 * What Catalyst subsystems implement in place of the removed {@code SubsystemBase}.
 *
 * <p>Commands v3's {@link Mechanism} is an interface with every method defaulted, which makes it a
 * near drop-in for {@code SubsystemBase} — except for two things Catalyst relied on heavily:
 *
 * <ol>
 *   <li>{@code SubsystemBase} offered {@code run(Runnable)}, {@code runOnce(Runnable)} and
 *       {@code startEnd(...)} as command factories bound to {@code this}. {@code Mechanism} has
 *       {@code run(Consumer<Coroutine>)} instead — same name, different shape — so every existing
 *       call site would otherwise have to be rewritten. The overloads below restore the old shapes.
 *       They coexist with the inherited coroutine version because Java picks by lambda arity:
 *       {@code run(() -> …)} is the {@link Runnable}, {@code run(co -> …)} is the coroutine.</li>
 *   <li>{@code SubsystemBase} registered itself so {@code periodic()} was called every loop.
 *       {@code Mechanism} has no such hook — v3 uses {@link Scheduler#addPeriodic(Runnable)}.
 *       Implementors call {@link #registerPeriodic()} once, from their constructor.</li>
 * </ol>
 *
 * <p>The second point is a genuine behaviour change and the one worth being careful about: a
 * subsystem that defines {@code periodic()} but never calls {@code registerPeriodic()} will compile
 * and silently never tick. Catalyst's own subsystems all register in their constructors.
 *
 * @since 2.0.0
 */
public interface CatalystSubsystem extends Mechanism {

    /** Run {@code action} every loop until the command is cancelled. Requires this mechanism. */
    default CatalystCommand run(Runnable action) {
        return Commands.run(action, this);
    }

    /** Run {@code action} once, then finish. Requires this mechanism. */
    default CatalystCommand runOnce(Runnable action) {
        return Commands.runOnce(action, this);
    }

    /** Run {@code start} on entry and {@code end} on exit, holding this mechanism in between. */
    default CatalystCommand startEnd(Runnable start, Runnable end) {
        return Commands.startEnd(start, end, this);
    }

    /** Hold this mechanism, doing nothing, until cancelled. */
    default CatalystCommand idleCommand() {
        return Commands.idle(this);
    }

    /**
     * Called every scheduler loop, once {@link #registerPeriodic()} has been called.
     *
     * <p>Default is a no-op so subsystems that do not need it can ignore it entirely.
     */
    default void periodic() {}

    /**
     * Called every scheduler loop, but only when the robot program is running in simulation.
     *
     * <p>Same contract {@code SubsystemBase} had. Mechanism sim models update here so the real
     * {@link #periodic()} stays free of simulation-only work.
     */
    default void simulationPeriodic() {}

    /**
     * Register {@link #periodic()} — and {@link #simulationPeriodic()} when running in simulation —
     * with the default scheduler. Call once, from the constructor.
     *
     * <p>This replaces the automatic registration {@code SubsystemBase} performed in its own
     * constructor. It is explicit here because v3 has no constructor to hook.
     */
    default void registerPeriodic() {
        Scheduler.getDefault().addPeriodic(this::periodic);
        if (RobotBase.isSimulation()) {
            Scheduler.getDefault().addPeriodic(this::simulationPeriodic);
        }
    }
}
