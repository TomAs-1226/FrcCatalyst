package frc.lib.catalyst.command;

import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Seconds;

/**
 * That {@code withTimeout} is back, and that both versions still resolve.
 *
 * <p>Four of the five renamed decorators were forced: v3 declares {@code until}, {@code andThen},
 * {@code alongWith} and {@code raceWith} taking exactly the parameters Catalyst's versions take and
 * returning group <em>builders</em>, and Java will not let an override narrow a builder to a
 * finished command. {@code withTimeout} was assumed to be the fifth and is not — v3's takes a
 * {@code Time}, Catalyst's takes a {@code double}, so they are overloads and both compile.
 *
 * <p>This test exists because the thing worth pinning is that neither call became ambiguous. If a
 * future v3 adds {@code withTimeout(double)}, this file stops compiling, which is the correct time
 * to find out.
 */
class WithTimeoutOverloadTest {

    private static CatalystCommand base() {
        return Commands.runOnce(() -> { }).withName("base");
    }

    @Test
    void theDoubleOverloadIsCatalystsAndReturnsACatalystCommand() {
        // Typed as CatalystCommand deliberately: if this resolved to v3's Command-returning version
        // the assignment would not compile, so the declaration is the assertion.
        CatalystCommand timed = base().withTimeout(2.0);
        assertNotNull(timed);
        assertTrue(timed.name().contains("base"));
    }

    @Test
    void theTimeOverloadStillReachesV3() {
        Command timed = base().withTimeout(Seconds.of(2));
        assertNotNull(timed);
    }

    @Test
    void anIntLiteralWidensToTheDoubleOverloadRatherThanBeingAmbiguous() {
        // 2 is not a Time and does not convert to one, so this must widen to double. Worth pinning:
        // an ambiguity here would be a compile error in team code, not a test failure.
        CatalystCommand timed = base().withTimeout(2);
        assertNotNull(timed);
    }

    @Test
    void theOldNameStillWorksWhileItIsDeprecated() {
        @SuppressWarnings("removal")
        CatalystCommand timed = base().timeoutAfter(2.0);
        assertNotNull(timed);
    }
}
