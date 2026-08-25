package frc.lib.catalyst.command;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static org.wpilib.units.Units.Seconds;

/**
 * The command factories Catalyst uses, rebuilt on commands v3.
 *
 * <p>This deliberately mirrors the names and argument shapes of the old
 * {@code edu.wpi.first.wpilibj2.command.Commands}, which WPILib removed in 2027. Catalyst call
 * sites change their import and nothing else, and teams who learned Catalyst against commands v2
 * keep the vocabulary they already know.
 *
 * <p>Everything returns a {@link CatalystCommand}, so the fluent decorators
 * ({@code withName}, {@code finallyDo}, {@code beforeStarting}, …) are available on the result.
 *
 * <p><b>The one rule v3 adds that teams must respect:</b> a command body that loops has to call
 * {@link Coroutine#yield()} on every pass, or it never gives the scheduler control back. Every
 * factory here that loops does so correctly; if you write a raw body of your own, do the same.
 *
 * @since 2.0.0
 */
public final class Commands {

    private Commands() {}

    private static Set<Mechanism> reqs(Mechanism... mechanisms) {
        return new LinkedHashSet<>(java.util.Arrays.asList(mechanisms));
    }

    /**
     * Build a command from a raw coroutine body.
     *
     * @param name         command name, required by v3 and shown in telemetry
     * @param body         the body; call {@link Coroutine#yield()} inside any loop
     * @param requirements mechanisms this command controls
     */
    public static CatalystCommand of(String name, java.util.function.Consumer<Coroutine> body,
                                     Mechanism... requirements) {
        Set<Mechanism> r = reqs(requirements);
        return new CatalystCommand(new Command() {
            @Override public void run(Coroutine coroutine) { body.accept(coroutine); }
            @Override public String name() { return name; }
            @Override public Set<Mechanism> requirements() { return r; }
        }, null);
    }

    /** A command that does nothing and finishes immediately. */
    public static CatalystCommand none() {
        return of("None", coroutine -> {});
    }

    /** Run {@code action} once, then finish. */
    public static CatalystCommand runOnce(Runnable action, Mechanism... requirements) {
        return of("RunOnce", coroutine -> action.run(), requirements);
    }

    /** Run {@code action} every loop, forever, until cancelled or decorated with an end condition. */
    public static CatalystCommand run(Runnable action, Mechanism... requirements) {
        return of("Run", coroutine -> {
            while (true) {
                action.run();
                coroutine.yield();
            }
        }, requirements);
    }

    /** Hold the given mechanisms doing nothing until cancelled. */
    public static CatalystCommand idle(Mechanism... requirements) {
        return of("Idle", coroutine -> {
            while (true) {
                coroutine.yield();
            }
        }, requirements);
    }

    /** Run {@code start} once at the beginning and {@code end} once when the command stops. */
    public static CatalystCommand startEnd(Runnable start, Runnable end, Mechanism... requirements) {
        return of("StartEnd", coroutine -> {
            start.run();
            while (true) {
                coroutine.yield();
            }
        }, requirements).finallyDo(interrupted -> end.run());
    }

    /** Do nothing until {@code condition} becomes true. */
    public static CatalystCommand waitUntil(BooleanSupplier condition) {
        return CatalystCommand.of(Command.waitUntil(condition).named("WaitUntil"));
    }

    /** Do nothing for {@code seconds}. */
    public static CatalystCommand waitSeconds(double seconds) {
        return CatalystCommand.of(Command.waitFor(Seconds.of(seconds)).named("Wait"));
    }

    /** Run the given commands one after another. */
    public static CatalystCommand sequence(Command... commands) {
        return CatalystCommand.of(Command.sequence(commands).named("Sequence"));
    }

    /** Run the given commands one after another, restarting from the top forever. */
    public static CatalystCommand repeatingSequence(Command... commands) {
        return of("RepeatingSequence", coroutine -> {
            while (true) {
                for (Command command : commands) {
                    coroutine.await(command);
                }
                coroutine.yield();
            }
        }, requirementsOf(commands));
    }

    /** Run the given commands at the same time, finishing when all of them are done. */
    public static CatalystCommand parallel(Command... commands) {
        return CatalystCommand.of(Command.parallel(commands).named("Parallel"));
    }

    /** Run the given commands at the same time, finishing as soon as any one is done. */
    public static CatalystCommand race(Command... commands) {
        return CatalystCommand.of(Command.race(commands).named("Race"));
    }

    /**
     * Choose between two commands when the command actually starts, not when it is constructed.
     *
     * @param onTrue    run when {@code selector} is true at start
     * @param onFalse   run when {@code selector} is false at start
     * @param selector  evaluated once, at start
     */
    public static CatalystCommand either(Command onTrue, Command onFalse, BooleanSupplier selector) {
        // Both branches' requirements, because the selector is not read until start and either
        // branch may be the one that runs. Reserving both is the conservative answer and the only
        // correct one at schedule time.
        return of("Either", coroutine -> coroutine.await(selector.getAsBoolean() ? onTrue : onFalse),
                requirementsOf(onTrue, onFalse));
    }

    /**
     * Every mechanism the given commands between them control.
     *
     * <p>{@code sequence}, {@code parallel} and {@code race} get this from v3's group builders.
     * {@code either} and {@code repeatingSequence} are built by hand here, and both used to declare
     * no requirements at all — which is not a cosmetic gap.
     *
     * <p>A repeating sequence with no requirements releases its mechanisms at every cycle boundary,
     * so the drivetrain's default command gets a full loop of stick input back between iterations.
     * On {@code Autopilot}, whose documentation promises the driver's steering stays suspended, that
     * is a visible twitch of the swerve every time the co-pilot finishes an action.
     *
     * <p>Requirements hidden inside an {@code either} are worse: v3 rejects
     * {@code parallel(a, b)} at construction when both need one mechanism, and
     * {@code parallel(either(a), either(b))} sailed straight through. The build-time check that
     * would have caught two commands driving one motor was simply skipped, and the fault surfaced
     * on the field instead as one branch cancelling the other's whole composition.
     */
    private static Mechanism[] requirementsOf(Command... commands) {
        Set<Mechanism> union = new LinkedHashSet<>();
        for (Command command : commands) {
            if (command != null) {
                union.addAll(command.requirements());
            }
        }
        return union.toArray(new Mechanism[0]);
    }

    /**
     * Build the command at start time rather than construction time.
     *
     * <p>Used wherever a command depends on state that is not known when the binding is created —
     * an alliance colour, a selected auto, a live goal.
     *
     * @param supplier     produces the command to run, called once at start
     * @param requirements mechanisms the produced command will control
     */
    public static CatalystCommand defer(Supplier<Command> supplier, Mechanism... requirements) {
        return of("Defer", coroutine -> coroutine.await(supplier.get()), requirements);
    }

    // --- Set-valued requirement overloads ------------------------------------
    //
    // Catalyst's state machine and behaviour layers carry requirements around as a Set, because
    // that is what an Actuator declares. Varargs alone would force every one of those call sites
    // to spread the set out, so the common factories take a Set directly.

    /** {@link #of(String, java.util.function.Consumer, Mechanism...)} with requirements as a set. */
    public static CatalystCommand of(String name, java.util.function.Consumer<Coroutine> body,
                                     Set<Mechanism> requirements) {
        return of(name, body, toArray(requirements));
    }

    /** {@link #runOnce(Runnable, Mechanism...)} with requirements as a set. */
    public static CatalystCommand runOnce(Runnable action, Set<Mechanism> requirements) {
        return runOnce(action, toArray(requirements));
    }

    /** {@link #run(Runnable, Mechanism...)} with requirements as a set. */
    public static CatalystCommand run(Runnable action, Set<Mechanism> requirements) {
        return run(action, toArray(requirements));
    }

    /** {@link #defer(Supplier, Mechanism...)} with requirements as a set. */
    public static CatalystCommand defer(Supplier<Command> supplier, Set<Mechanism> requirements) {
        return defer(supplier, toArray(requirements));
    }

    private static Mechanism[] toArray(Set<Mechanism> requirements) {
        return requirements == null ? new Mechanism[0] : requirements.toArray(new Mechanism[0]);
    }
}
