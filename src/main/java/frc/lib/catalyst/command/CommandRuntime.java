package frc.lib.catalyst.command;

import java.lang.invoke.MethodHandles;

/**
 * Checks that the JVM is actually able to run Commands v3, and says so plainly when it is not.
 *
 * <h2>Why this exists</h2>
 *
 * <p>Commands v3 is built on JDK continuations. Those live in {@code jdk.internal.vm}, which is not
 * exported to anybody, so v3 reaches them by reflection through
 * {@link MethodHandles#privateLookupIn}. That only works if the JVM was started with two flags:
 *
 * <pre>
 *   --add-opens java.base/jdk.internal.vm=ALL-UNNAMED
 *   --add-opens java.base/java.lang=ALL-UNNAMED
 * </pre>
 *
 * <p>Without them nothing fails at build time and nothing fails at startup. The robot boots, the
 * dashboard connects, and then the first command to be scheduled throws
 * {@code ExceptionInInitializerError} caused by
 * {@code IllegalAccessException: module java.base does not open java.lang} — from inside WPILib,
 * naming no Catalyst class, at whatever unrelated moment the driver first pressed a button.
 *
 * <p>Two flags are needed, not one, and the second is only discovered after the first is fixed:
 * {@code jdk.internal.vm} gets the scheduler <em>constructed</em>, and {@code java.lang} gets a
 * command <em>scheduled</em>. A team that finds the first flag will conclude they are done, deploy,
 * and hit the second one on the field.
 *
 * <p>So Catalyst checks once and reports it as what it is: a launch configuration problem, with the
 * two lines that fix it.
 *
 * <h2>Where the flags go</h2>
 *
 * <p>On Systemcore the robot program is launched from {@code /home/systemcore/robotCommand}, which
 * GradleRIO writes at deploy time. In simulation and unit tests they are the test JVM's arguments —
 * Catalyst's own build passes them in its {@code test} block.
 *
 * @since 2.0.0
 */
public final class CommandRuntime {

    /**
     * Whether continuations are reachable.
     *
     * <p>Probed exactly the way v3 does it, but without letting the failure escape: this must be
     * able to report a broken JVM, not become one.
     */
    private static final boolean AVAILABLE = probe();

    private CommandRuntime() {
    }

    private static boolean probe() {
        try {
            // Same two lookups v3's Continuation and ContinuationScope perform in their static
            // initialisers. If either is refused, so is every command.
            MethodHandles.privateLookupIn(
                    Class.forName("jdk.internal.vm.ContinuationScope"), MethodHandles.lookup());
            MethodHandles.privateLookupIn(Class.class, MethodHandles.lookup());
            return true;
        } catch (Throwable t) {
            return false;
        }
    }

    /** Whether this JVM can run Commands v3 at all. */
    public static boolean isAvailable() {
        return AVAILABLE;
    }

    /**
     * Throw a legible error if this JVM cannot run commands.
     *
     * <p>Called when a Catalyst command is built, so the failure lands at the line that built it
     * rather than several seconds later inside WPILib.
     *
     * @throws IllegalStateException if the required {@code --add-opens} flags are missing
     */
    public static void require() {
        if (AVAILABLE) {
            return;
        }
        throw new IllegalStateException("""
                Commands v3 cannot run in this JVM.

                It needs access to JDK continuations, which requires two launch flags:

                    --add-opens java.base/jdk.internal.vm=ALL-UNNAMED
                    --add-opens java.base/java.lang=ALL-UNNAMED

                Add them to the robot program's launch arguments (on Systemcore that is
                /home/systemcore/robotCommand, written by GradleRIO at deploy time), or to the JVM
                arguments of whatever is running this - a test task, a simulation, or an IDE run
                configuration.

                Both are required. Supplying only the first lets the scheduler be constructed and
                then fails when the first command is scheduled.""");
    }
}
