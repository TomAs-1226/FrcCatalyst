package frc.lib.catalyst.command;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

import java.util.Set;

/**
 * Runs a commands v2 command on the v3 scheduler.
 *
 * <p><b>Why this is needed.</b> WPILib 2027 ships both command frameworks, and vendor libraries are
 * migrating at their own pace. PathPlannerLib {@code 2027.0.0-alpha-3} is still compiled against
 * {@code org.wpilib.command2} — every {@code AutoBuilder} factory hands back a v2 command. Catalyst
 * is v3 throughout, so without a bridge the entire PathPlanner integration would have to be dropped
 * for the season, taking {@code AutoSelector}, {@code DynamicAutoBuilder} and path following with it.
 *
 * <p>The translation is honest rather than clever, because the two models line up almost exactly.
 * A v2 command is a state machine with four callbacks; a v3 command is a coroutine. One is the other
 * written out longhand:
 *
 * <pre>{@code
 * initialize();
 * while (!isFinished()) { execute(); yield(); }
 * end(false);
 * }</pre>
 *
 * <p>with {@code end(true)} on cancellation. That is precisely what the v2 scheduler does, so a
 * bridged command behaves the way its author intended.
 *
 * <p><b>What does not carry across.</b> v2's {@code runsWhenDisabled} has no v3 equivalent that
 * could be verified, and v2 requirements are v2 {@code Subsystem}s, which are not v3
 * {@link Mechanism}s — so a bridged command declares no requirements of its own. Give the wrapper
 * the Catalyst mechanisms it should reserve, or the v3 scheduler will not know to stop anything else
 * that wants them. For PathPlanner that means passing your drive subsystem.
 *
 * <p>This class is expected to be temporary. When PathPlanner ships a v3 build, delete it and the
 * {@code commandsv2-java} dependency together.
 *
 * @since 2.0.0
 */
public final class LegacyCommands {

    private LegacyCommands() {}

    /**
     * Wrap a commands v2 command so the v3 scheduler can run it.
     *
     * @param legacy       the v2 command, typically from PathPlanner's {@code AutoBuilder}
     * @param requirements Catalyst mechanisms the wrapped command controls. v2 requirements cannot
     *                     be translated, so state them here or nothing will be reserved.
     */
    public static CatalystCommand fromV2(org.wpilib.command2.Command legacy,
                                         Mechanism... requirements) {
        Set<Mechanism> required = Set.of(requirements);
        String name = legacy.getName();

        return CatalystCommand.of(new Command() {
            @Override
            public void run(Coroutine coroutine) {
                legacy.initialize();
                while (!legacy.isFinished()) {
                    legacy.execute();
                    coroutine.yield();
                }
                legacy.end(false);
            }

            @Override
            public void onCancel() {
                legacy.end(true);
            }

            @Override public String name() { return name; }
            @Override public Set<Mechanism> requirements() { return required; }
        });
    }
}
