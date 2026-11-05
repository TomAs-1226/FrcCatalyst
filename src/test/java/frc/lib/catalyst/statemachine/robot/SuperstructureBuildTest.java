package frc.lib.catalyst.statemachine.robot;

import frc.lib.catalyst.command.CatalystSubsystem;

import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * That a Superstructure can be built at all.
 *
 * <p>It could not. {@code checkExistingDefaults} treated a non-null {@code getDefaultCommand()} as
 * "the team already installed one", which was correct under commands v2 where the default stayed
 * null until somebody set it. Commands v3's {@code Mechanism} constructor installs {@code idle()}
 * unconditionally, so on the released alpha-6 <em>every</em> freshly constructed mechanism already
 * had a default, the guard rejected every managed binding, and {@code build()} threw for every robot
 * using the state machine's default management — which is the default.
 *
 * <p>There was no test that built a Superstructure at all. That is why it shipped, and it is the
 * gap this file closes rather than only the bug.
 *
 * <p>The fix keys on priority, measured rather than assumed: the constructor's idle carries
 * {@code LOWEST_PRIORITY} and a team's {@code setDefaultCommand(holdPosition())} carries
 * {@code DEFAULT_PRIORITY}. So the guard still refuses to silently stomp a default somebody
 * installed on purpose, which is the entire reason it exists — and the second test here is what
 * stops a future "fix" from throwing that protection away to make the first one pass.
 */
class SuperstructureBuildTest {

    private enum State { STOW, SCORE }

    /** A mechanism, which in v3 arrives with an idle default command already installed. */
    private static final class Arm extends CatalystSubsystem {
        Arm(String name) {
            super(name);
        }
    }

    /** The smallest thing that satisfies Actuator: it owns a mechanism and is always at goal. */
    private record StubActuator(String key, Mechanism owned) implements Actuator<String> {
        @Override
        public Command pursueCommand(String goal) {
            return Command.noRequirements(co -> {
                while (true) {
                    co.yield();
                }
            }).named(key + ":" + goal);
        }

        @Override
        public boolean atGoal(String goal, double secondsSinceApplied) {
            return true;
        }

        @Override
        public Set<Mechanism> requirements() {
            return Set.of(owned);
        }
    }

    /**
     * A default command of the shape a team would actually write.
     *
     * <p>It must {@code require} the mechanism: WPILib refuses a default that does not, so
     * {@code noRequirements()} cannot stand in for one. It also carries DEFAULT_PRIORITY, which is
     * exactly what distinguishes it from the constructor's LOWEST_PRIORITY idle and therefore what
     * the guard under test keys on.
     */
    private static Command teamHold(Mechanism m) {
        return Command.requiring(m).executing(co -> {
            while (true) {
                co.yield();
            }
        }).named("TeamInstalledHold");
    }

    @Test
    void aSuperstructureWithAManagedBindingBuilds() {
        var b = Superstructure.builder(State.class, "BuildTest1");
        var arm = b.bind("arm", new StubActuator("arm", new Arm("BuildTestArm1")));

        Superstructure<State> s = assertDoesNotThrow(() ->
                        b.state(State.STOW, x -> x.set(arm, "stowed"))
                                .state(State.SCORE, x -> x.set(arm, "scoring"))
                                .allowBoth(State.STOW, State.SCORE)
                                .initialState(State.STOW)
                                .build(),
                "build() threw for an ordinary managed binding — v3 installs an idle default on "
                        + "every mechanism, so this is every robot, not an edge case");

        assertNotNull(s);
    }

    @Test
    void aDefaultTheTeamInstalledIsStillRefused() {
        // The protection the guard exists for. A team that set its own default command would have it
        // silently replaced by the state machine's runner, and the mechanism would stop doing the
        // thing they wrote. Loosening the check must not lose this.
        Arm arm = new Arm("BuildTestArm2");
        arm.setDefaultCommand(teamHold(arm));

        var b = Superstructure.builder(State.class, "BuildTest2");
        var handle = b.bind("arm", new StubActuator("arm", arm));

        // Every state declared and reachable, so the only thing left to complain about is the
        // default command. Without this the assertion passes on an unrelated configuration error -
        // which is exactly what it did on the first run, reporting success for the wrong reason.
        var thrown = assertThrows(Exception.class, () ->
                        b.state(State.STOW, x -> x.set(handle, "stowed"))
                                .state(State.SCORE, x -> x.set(handle, "scoring"))
                                .allowBoth(State.STOW, State.SCORE)
                                .initialState(State.STOW)
                                .build(),
                "a default the team installed must still be reported, not quietly overwritten");

        assertTrue(thrown.getMessage().contains("already has a default command"),
                "it must throw ABOUT the default command, not about something else: "
                        + thrown.getMessage());
    }

    @Test
    void manageDefaultsFalseSkipsTheCheckEntirely() {
        // The documented escape hatch: the team schedules the runners itself, so an existing
        // default is theirs to manage and nothing should complain about it.
        Arm arm = new Arm("BuildTestArm3");
        arm.setDefaultCommand(teamHold(arm));

        var b = Superstructure.builder(State.class, "BuildTest3").manageDefaults(false);
        var handle = b.bind("arm", new StubActuator("arm", arm));

        assertDoesNotThrow(() ->
                b.state(State.STOW, x -> x.set(handle, "stowed"))
                        .state(State.SCORE, x -> x.set(handle, "scoring"))
                        .allowBoth(State.STOW, State.SCORE)
                        .initialState(State.STOW)
                        .build());
    }
}
