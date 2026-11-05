package frc.lib.catalyst.opmode;

import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.command3.Command;

import java.util.function.Supplier;

/**
 * An {@link org.wpilib.opmode.OpMode} that runs one command.
 *
 * <p>Which is what nearly every autonomous routine is. A team with autos already written should not
 * have to restructure them to get Driver Station selection — the routine is already a command, and
 * this is the adapter.
 *
 * <pre>{@code
 * @Autonomous(name = "Three Piece Left", group = "Competition")
 * public class ThreePieceLeft extends CommandOpMode {
 *     public ThreePieceLeft() {
 *         super(() -> RobotContainer.get().auto().threePieceLeft());
 *     }
 * }
 * }</pre>
 *
 * <h2>Why a supplier and not a command</h2>
 *
 * <p>Because the OpMode is constructed when the Driver Station lists the modes, which is at startup,
 * and the command must be built when the mode <em>starts</em>. A command built at construction
 * captures whatever the robot's state was before the match — the pose it had at boot, the alliance
 * before the FMS said, the game data that had not arrived. It runs, it looks plausible, and it
 * drives the wrong way.
 *
 * <p>This is the same reason {@code Commands.defer} exists, and it is a mistake worth making
 * impossible rather than documenting.
 *
 * @since 2.0.0
 */
public abstract class CommandOpMode extends CatalystOpMode {

    private final Supplier<Command> supplier;
    private Command running;

    /**
     * @param supplier builds the command when the mode starts, not when it is constructed
     */
    protected CommandOpMode(Supplier<Command> supplier) {
        this.supplier = supplier;
    }

    /**
     * @param supplier            builds the command when the mode starts
     * @param useDefaultScheduler false to drive a scheduler of this mode's own, which is what a
     *                            test wants — two modes sharing the default scheduler inherit each
     *                            other's commands
     */
    protected CommandOpMode(Supplier<Command> supplier, boolean useDefaultScheduler) {
        super(useDefaultScheduler);
        this.supplier = supplier;
    }

    @Override
    protected void onStart() {
        running = supplier.get();
        if (running == null) {
            // A supplier that returns nothing is a bug in robot code, not a mode that does nothing.
            // Saying so is the difference between "the auto did not run" and knowing why.
            CatalystLog.log("OpMode/Error", name() + ": the command supplier returned null");
            return;
        }
        CatalystLog.log("OpMode/Command", running.name());
        run(running);
    }

    @Override
    protected void onEnd() {
        // The match ending does not cancel what the mode scheduled. Without this a fifteen-second
        // auto keeps driving into teleop, still holding the drivetrain, and the driver cannot work
        // out why the robot will not respond.
        if (running != null) {
            scheduler().cancel(running);
            running = null;
        }
    }

    /** The command currently running, or null. */
    protected final Command runningCommand() {
        return running;
    }
}
