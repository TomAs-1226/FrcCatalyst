package frc.lib.catalyst.statemachine;

import frc.lib.catalyst.statemachine.StateMachineFixtures.FakeBinding;
import frc.lib.catalyst.statemachine.StateMachineFixtures.FakeClock;
import frc.lib.catalyst.statemachine.StateMachineFixtures.St;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Regression tests for defects found by the adversarial review pass — the ones that would have
 * shipped a robot-loop crash or a machine that could not be recovered.
 */
class StateMachineRobustnessTest {

    private static final class Rig {
        final FakeClock clock = new FakeClock();
        final FakeBinding elevator = new FakeBinding("elevator");
        final Handle<Double> hElevator;
        final StateMachineCore<St> sm;
        final boolean[] guardThrows = {false};
        final boolean[] interlockThrows = {false};

        Rig(double timeout) {
            StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Robust").clock(clock);
            hElevator = b.bind("elevator", elevator);
            sm = b.defaultTimeout(timeout)
                    .initialState(St.STOW)
                    .state(St.STOW, s -> s.set(hElevator, 0.0))
                    .state(St.MID, s -> s.set(hElevator, 0.5)
                            .entryGuard(() -> {
                                if (guardThrows[0]) throw new IllegalStateException("boom");
                                return true;
                            }, "test"))
                    .state(St.HIGH, s -> s.set(hElevator, 1.0))
                    .state(St.INTAKE, s -> s.set(hElevator, 0.1))
                    .state(St.CLIMB, s -> s.set(hElevator, 0.0))
                    .hub(St.STOW)
                    .allowBoth(St.MID, St.HIGH)
                    .interlock("throwsMaybe", () -> {
                        if (interlockThrows[0]) throw new IllegalStateException("bang");
                        return true;
                    }, s -> s == St.HIGH)
                    .build();
        }

        void run(int steps) {
            for (int i = 0; i < steps; i++) {
                clock.advance(0.02);
                sm.step();
                elevator.drive(sm.activeGoalOf(hElevator));
            }
        }
    }

    @Test
    void aThrowingEntryGuardBlocksInsteadOfCrashingTheLoop() {
        Rig r = new Rig(2.0);
        r.sm.seed(St.STOW);
        r.guardThrows[0] = true;
        // Must not propagate — a throwing guard would otherwise kill CommandScheduler.run().
        r.run(5);
        TransitionResult<St> result = r.sm.request(St.MID, "test");
        assertTrue(result.rejected(), "a throwing guard must fail closed, refusing the transition");
        assertEquals(RejectReason.ENTRY_GUARD_BLOCKED, result.reason());
        assertEquals(St.STOW, r.sm.current());
    }

    @Test
    void aThrowingInterlockBlocksInsteadOfCrashingTheLoop() {
        Rig r = new Rig(2.0);
        r.sm.seed(St.STOW);
        r.interlockThrows[0] = true;
        r.run(5);   // legalTargets() runs every loop and evaluates the interlock; must not throw
        TransitionResult<St> result = r.sm.request(St.HIGH, "test");
        assertTrue(result.rejected());
        assertEquals(RejectReason.INTERLOCK_BLOCKED, result.reason());
    }

    @Test
    void afterATimeoutTheMachineCanBeCommandedBackToItsProvenState() {
        Rig r = new Rig(1.0);
        r.sm.seed(St.STOW);
        r.sm.request(St.MID, "go");
        for (int i = 0; i < 60 && !r.sm.isSettledAt(St.MID); i++) r.run(1);
        assertTrue(r.sm.isSettledAt(St.MID));

        r.elevator.jam();
        r.sm.request(St.HIGH, "go");
        r.run(200);
        assertFalse(r.sm.stateConfirmed(), "the timed-out transition must leave the belief unconfirmed");
        assertEquals(St.MID, r.sm.current());

        r.sm.clearFault();
        r.elevator.unjam();
        // The critical case: request the state we are actually at, to re-settle and clear the doubt.
        TransitionResult<St> back = r.sm.request(St.MID, "recover");
        assertTrue(back.accepted(),
                "request(current) while unconfirmed must be a legal recovery, not a NO_EDGE dead end");
        r.run(60);
        assertTrue(r.sm.isSettledAt(St.MID), "re-seeking the proven state must re-confirm it");
    }

    @Test
    void requestingTheConfirmedCurrentStateIsAnInformationalNoOp() {
        Rig r = new Rig(2.0);
        r.sm.seed(St.STOW);
        TransitionResult<St> result = r.sm.request(St.STOW, "again");
        assertTrue(result.rejected());
        assertEquals(RejectReason.ALREADY_THERE, result.reason());
        assertTrue(r.sm.stateConfirmed(), "a no-op request must not disturb the confirmed belief");
    }

    @Test
    void diagnosticsMeasureTheHeldGoalsAfterATimeoutNotTheConfirmedState() {
        Rig r = new Rig(1.0);
        r.sm.seed(St.STOW);
        r.sm.request(St.MID, "go");
        for (int i = 0; i < 60 && !r.sm.isSettledAt(St.MID); i++) r.run(1);

        r.elevator.jam();
        r.sm.request(St.HIGH, "go");
        r.run(200);
        // We are holding toward HIGH (elevator 1.0) but stuck; current() is MID. The diagnostics must
        // describe the goal actually being pursued (HIGH's), not MID's — otherwise they would claim
        // the machine is happily at its goal while it is visibly stuck.
        assertTrue(r.sm.waitingOn().contains("elevator"),
                "waitingOn must report the mechanism holding up the held goal: " + r.sm.waitingOn());
        assertTrue(r.sm.progress() < 1.0, "progress must reflect the held goal, not the confirmed state");
    }
}
