package frc.lib.catalyst.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

/**
 * One field, several writers. This library has already shipped the failure this prevents, in the
 * other direction: a speed governor that six of eight drive commands ignored.
 */
class AuthorityCoreTest {

    @Test
    void theSmallestLimitWinsAndIsNamed() {
        var a = AuthorityCore.combine(List.of(
                new AuthorityCore.Limit("physics", 0.75, "moderate confidence"),
                new AuthorityCore.Limit("slowMode", 0.35, "driver holding"),
                new AuthorityCore.Limit("power", 0.90, "")));

        assertEquals(0.35, a.scale(), 1e-9);
        assertEquals("slowMode", a.binding());
        assertTrue(a.limited());
        assertTrue(a.explain().contains("slowMode holding at 35%"), a.explain());
        assertTrue(a.explain().contains("driver holding"), a.explain());
    }

    @Test
    void noLimitsMeansFullAuthority() {
        assertEquals(1.0, AuthorityCore.combine(List.of()).scale(), 1e-9);
        assertEquals("none", AuthorityCore.combine(List.of()).binding());
        assertFalse(AuthorityCore.combine(List.of()).limited());
    }

    @Test
    void limitersThatAreNotLimitingDoNotClaimToBe() {
        var a = AuthorityCore.combine(List.of(
                AuthorityCore.Limit.none("physics"),
                AuthorityCore.Limit.none("power")));
        assertEquals(1.0, a.scale(), 1e-9);
        assertEquals("none", a.binding());
        assertEquals("no limits", a.explain());
    }

    @Test
    void aSecondLimiterNearTheFloorIsNamedToo() {
        // A driver told only about slow mode keeps wondering why it is still slow after releasing it.
        var a = AuthorityCore.combine(List.of(
                new AuthorityCore.Limit("slowMode", 0.35, "driver holding"),
                new AuthorityCore.Limit("physics", 0.37, "slipping")));
        assertTrue(a.explain().contains("also physics"), a.explain());
    }

    @Test
    void noLimiterCanGrantAuthorityItDoesNotHave() {
        // The safety property the whole design rests on: a limiter can slow the robot and can never
        // speed it up, which is what makes adding one safe without auditing the others.
        var a = AuthorityCore.combine(List.of(
                new AuthorityCore.Limit("optimist", 5.0, "turbo!"),
                new AuthorityCore.Limit("physics", 0.5, "")));
        assertEquals(0.5, a.scale(), 1e-9, "a limit above 1 is clamped and cannot raise the result");
    }

    @Test
    void aNaNLimitIsTreatedAsAStop() {
        // A limiter whose arithmetic broke must fail closed. Passing NaN through would make every
        // comparison downstream false and the robot would run unlimited.
        var a = AuthorityCore.combine(List.of(new AuthorityCore.Limit("broken", Double.NaN, "")));
        assertEquals(0.0, a.scale(), 1e-9);
    }

    @Test
    void aBlindSituationLimitsNothing() {
        // No measurement is not evidence of a problem. A robot with no physics must not be throttled
        // for it.
        var limit = AuthorityCore.fromSituation(Situation.blind());
        assertEquals(1.0, limit.scale(), 1e-9);
    }

    @Test
    void aSlippingSituationEasesOff() {
        Situation s = new Situation(0,
                Situation.Localization.unknown(),
                Situation.Motion.unknown(),
                new Situation.Traction(0.8, 0.9, 0.1, true),
                Situation.Power.unmeasured(12.0),
                Situation.Match.unknown());

        var limit = AuthorityCore.fromSituation(s);
        assertEquals(0.6, limit.scale(), 1e-9, "1 - 0.5 * 0.8");
        assertTrue(limit.reason().contains("slip"), limit.reason());
    }

    @Test
    void beingOverThePowerBudgetEasesOff() {
        Situation s = new Situation(0,
                Situation.Localization.unknown(),
                Situation.Motion.unknown(),
                Situation.Traction.unknown(),
                new Situation.Power(11.0, -25.0, 145.0, "breaker", true),
                Situation.Match.unknown());

        var limit = AuthorityCore.fromSituation(s);
        assertEquals(0.6, limit.scale(), 1e-9);
        assertTrue(limit.reason().contains("25 A over budget"), limit.reason());
    }

    @Test
    void anUnmeasuredPowerFacetNeverThrottles() {
        // The BrownoutMonitor lesson: an unmeasured number must not read as a bad number.
        Situation s = new Situation(0,
                Situation.Localization.unknown(),
                Situation.Motion.unknown(),
                Situation.Traction.unknown(),
                Situation.Power.unmeasured(8.0),
                Situation.Match.unknown());
        assertEquals(1.0, AuthorityCore.fromSituation(s).scale(), 1e-9);
    }
}
