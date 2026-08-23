package frc.lib.catalyst.command;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import static org.wpilib.units.Units.Seconds;

/**
 * A {@link Command} with the fluent decorators Catalyst has always used.
 *
 * <p><b>Why this exists.</b> WPILib 2027 replaced commands v2 with commands v3, and v3 deliberately
 * ships a much smaller surface: there is no {@code withName}, no {@code finallyDo}, no
 * {@code beforeStarting}. Those are not oversights — v3 expects a command to be named once, at
 * construction, through its builder.
 *
 * <p>Catalyst still needs them, for two reasons. Internally, they are used several hundred times.
 * Externally, Catalyst promised teams that upgrading the library would not mean relearning it. This
 * class keeps that promise: it is a {@code Command} in every sense the v3 scheduler cares about, and
 * it carries the decorators on top.
 *
 * <p>Every decorator returns a <em>new</em> instance; nothing here mutates. Instances are cheap —
 * a wrapper around another command plus, at most, a name override.
 *
 * <p><b>One caveat worth reading.</b> {@link #ignoringDisable(boolean)} is currently a no-op that
 * returns {@code this}. Commands v2 had an explicit run-while-disabled flag; v3's scheduler does not
 * expose an equivalent that could be verified against the alpha-6 jars, and guessing at disabled
 * behaviour is exactly the kind of assumption that gets a robot penalised. The method is kept so
 * call sites compile and intent stays visible in the code, and it is flagged for confirmation
 * against real hardware before anyone relies on it.
 *
 * @since 2.0.0
 */
public final class CatalystCommand implements Command {

    private final Command inner;
    private final String nameOverride;

    CatalystCommand(Command inner, String nameOverride) {
        // Every Catalyst command - every factory, every decorator - is built here, which makes this
        // the one place that can catch a JVM missing the continuation flags and say so at the line
        // that built the command, rather than several seconds later inside WPILib. See
        // CommandRuntime for what goes wrong without it.
        CommandRuntime.require();
        this.inner = inner;
        this.nameOverride = nameOverride;
    }

    /** Adopt an existing {@link Command} so the Catalyst decorators become available on it. */
    public static CatalystCommand of(Command command) {
        return command instanceof CatalystCommand cc ? cc : new CatalystCommand(command, null);
    }

    // --- Command contract ---------------------------------------------------

    @Override public void run(Coroutine coroutine)  { inner.run(coroutine); }
    @Override public String name()                  { return nameOverride != null ? nameOverride : inner.name(); }
    @Override public Set<Mechanism> requirements()  { return inner.requirements(); }
    @Override public int priority()                 { return inner.priority(); }
    @Override public void onCancel()                { inner.onCancel(); }

    // --- Decorators ---------------------------------------------------------

    /**
     * Rename this command. v3 requires a name at construction; this overrides it afterwards, which
     * is what the several hundred existing {@code .withName(...)} call sites expect.
     */
    public CatalystCommand withName(String name) {
        return new CatalystCommand(inner, name);
    }

    /**
     * Run {@code action} when this command ends, whether it finished on its own or was interrupted.
     * The flag is {@code true} when the command was cancelled.
     */
    public CatalystCommand finallyDo(Consumer<Boolean> action) {
        Command base = inner;
        String name = name();
        return new CatalystCommand(new Command() {
            @Override public void run(Coroutine coroutine) {
                base.run(coroutine);
                action.accept(false);
            }
            @Override public void onCancel() {
                base.onCancel();
                action.accept(true);
            }
            @Override public String name() { return name; }
            @Override public Set<Mechanism> requirements() { return base.requirements(); }
            @Override public int priority() { return base.priority(); }
        }, null);
    }

    /**
     * Run {@code action} when this command ends, whether it finished or was interrupted.
     *
     * <p>The form for callers that do not care which of the two happened — the majority.
     */
    public CatalystCommand finallyDo(Runnable action) {
        return finallyDo(interrupted -> action.run());
    }

    /** Run {@code action} once, immediately before this command's body starts. */
    public CatalystCommand beforeStarting(Runnable action) {
        Command base = inner;
        String name = name();
        return new CatalystCommand(new Command() {
            @Override public void run(Coroutine coroutine) {
                action.run();
                base.run(coroutine);
            }
            @Override public void onCancel() { base.onCancel(); }
            @Override public String name() { return name; }
            @Override public Set<Mechanism> requirements() { return base.requirements(); }
            @Override public int priority() { return base.priority(); }
        }, null);
    }

    /**
     * End this command as soon as {@code condition} becomes true.
     *
     * <p>Named {@code untilTrue} rather than {@code until} because v3's {@code Command.until}
     * returns a group <em>builder</em>, and Java will not let an override narrow that to a finished
     * command. v3's own {@code until} is still inherited and still available.
     */
    public CatalystCommand untilTrue(BooleanSupplier condition) {
        return new CatalystCommand(
                Command.race(this, Command.waitUntil(condition).named(name() + "/until"))
                        .named(name()),
                null);
    }

    /** End this command after {@code seconds}, if it has not already finished. */
    public CatalystCommand timeoutAfter(double seconds) {
        return new CatalystCommand(
                Command.race(this, Command.waitFor(Seconds.of(seconds)).named(name() + "/timeout"))
                        .named(name()),
                null);
    }

    /** Run {@code next} after this command completes. See {@link #untilTrue} on the naming. */
    public CatalystCommand then(Command next) {
        return new CatalystCommand(
                Command.sequence(this, next).named(name() + " -> " + next.name()), null);
    }

    /** Run {@code others} alongside this command, finishing when all are done. */
    public CatalystCommand together(Command... others) {
        Command[] all = new Command[others.length + 1];
        all[0] = this;
        System.arraycopy(others, 0, all, 1, others.length);
        return new CatalystCommand(Command.parallel(all).named(name()), null);
    }

    /** Run {@code others} alongside this command, finishing when any one is done. */
    public CatalystCommand racing(Command... others) {
        Command[] all = new Command[others.length + 1];
        all[0] = this;
        System.arraycopy(others, 0, all, 1, others.length);
        return new CatalystCommand(Command.race(all).named(name()), null);
    }

    /**
     * Kept so existing call sites compile and their intent stays readable.
     *
     * <p><b>Currently a no-op.</b> See the class javadoc — v3's disabled-mode behaviour could not be
     * confirmed against the alpha-6 jars, and this must be verified on hardware before any code
     * depends on it.
     */
    public CatalystCommand ignoringDisable(boolean ignore) {
        return this;
    }

    /** Schedule this command on the default v3 scheduler. */
    public void schedule() {
        org.wpilib.command3.Scheduler.getDefault().schedule(this);
    }

    /** Cancel this command on the default v3 scheduler. */
    public void cancel() {
        org.wpilib.command3.Scheduler.getDefault().cancel(this);
    }

    /** Whether the default scheduler currently has this command scheduled or running. */
    public boolean isScheduled() {
        return org.wpilib.command3.Scheduler.getDefault().isScheduledOrRunning(this);
    }
}
