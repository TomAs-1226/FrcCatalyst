package frc.lib.catalyst.statemachine;

import frc.lib.catalyst.statemachine.StateMachineFixtures.FakeBinding;
import frc.lib.catalyst.statemachine.StateMachineFixtures.FakeClock;
import frc.lib.catalyst.statemachine.StateMachineFixtures.St;
import frc.lib.catalyst.statemachine.StateMachineFixtures.TimerBinding;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The invariants that separate this engine from a hand-rolled state machine: the current state is
 * never a state the robot merely tried to reach, and "am I there?" is always a measurement.
 */
class StateMachineTruthTest {

    /** A two-mechanism machine with a linear STOW - MID - HIGH chain. */
    private static final class Rig {
        final FakeClock clock = new FakeClock();
        final FakeBinding elevator = new FakeBinding("elevator");
        final FakeBinding arm = new FakeBinding("arm");
        final Handle<Double> hElevator;
        final Handle<Double> hArm;
        final StateMachineCore<St> sm;
        final AtomicInteger entries = new AtomicInteger();
        final AtomicInteger exits = new AtomicInteger();

        Rig(double timeout) {
            arm.rate = 5.0;
            StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Truth").clock(clock);
            hElevator = b.bind("elevator", elevator);
            hArm = b.bind("arm", arm);
            sm = b.defaultTimeout(timeout)
                    .initialState(St.STOW)
                    .state(St.STOW, s -> s.set(hElevator, 0.0).set(hArm, 0.0))
                    .state(St.MID, s -> s.set(hElevator, 0.5).set(hArm, 30.0))
                    .state(St.HIGH, s -> s.set(hElevator, 1.0).set(hArm, 90.0)
                            .onEnter(entries::incrementAndGet)
                            .onExit(exits::incrementAndGet))
                    .state(St.INTAKE, s -> s.set(hElevator, 0.1).set(hArm, -10.0))
                    .state(St.CLIMB, s -> s.set(hElevator, 0.0).set(hArm, 5.0))
                    .hub(St.STOW)
                    .allowBoth(St.MID, St.HIGH)
                    .build();
        }

        /** Step the machine and let the fakes follow their goals. */
        void run(int steps) {
            for (int i = 0; i < steps; i++) {
                clock.advance(0.02);
                sm.step();
                elevator.drive(sm.activeGoalOf(hElevator));
                arm.drive(sm.activeGoalOf(hArm));
            }
        }

        void runUntilSettled(St state, int maxSteps) {
            for (int i = 0; i < maxSteps && !sm.isSettledAt(state); i++) run(1);
        }
    }

    @Test
    void requestBeforeSeedIsRejectedRatherThanGuessed() {
        Rig r = new Rig(2.0);
        TransitionResult<St> result = r.sm.request(St.MID, "test");
        assertTrue(result.rejected(), "a machine that has never been seeded must not guess its origin");
        assertEquals(RejectReason.NOT_SEEDED, result.reason());
    }

    @Test
    void seedConfirmsWithoutMovingAnything() {
        Rig r = new Rig(2.0);
        r.sm.seed(St.STOW);
        assertTrue(r.sm.stateConfirmed());
        assertEquals(St.STOW, r.sm.current());
        assertEquals(0.0, r.elevator.position, 1e-9);
    }

    @Test
    void timeoutNeverConfirmsTheTarget() {
        Rig r = new Rig(1.0);
        r.sm.seed(St.STOW);
        r.elevator.jam();
        r.sm.request(St.MID, "test");
        r.run(200);

        assertEquals(St.STOW, r.sm.current(),
                "current() must stay at the last PROVEN state, not jump to the one we failed to reach");
        assertFalse(r.sm.stateConfirmed(), "a blown deadline must leave the belief unconfirmed");
        assertFalse(r.sm.isSettledAt(St.MID));
        assertEquals(1, r.sm.timeoutCount());
    }

    @Test
    void abortNeverConfirmsTheTarget() {
        Rig r = new Rig(30.0);
        r.sm.seed(St.STOW);
        r.sm.request(St.MID, "test");
        r.run(1);
        r.sm.abort("driver let go");

        assertEquals(St.STOW, r.sm.current());
        assertFalse(r.sm.stateConfirmed());
        assertEquals(1, r.sm.abortCount());
    }

    @Test
    void aTimedOutTransitionStillPlansFromTheProvenState() {
        Rig r = new Rig(1.0);
        r.sm.seed(St.STOW);
        r.runUntilSettled(St.STOW, 5);
        r.sm.request(St.MID, "first");
        r.runUntilSettled(St.MID, 300);
        assertTrue(r.sm.isSettledAt(St.MID));

        r.elevator.jam();
        r.sm.request(St.HIGH, "second");
        r.run(200);
        assertEquals(St.MID, r.sm.current());
        r.sm.clearFault();
        r.elevator.unjam();

        // The machine knows it is at MID, so a retry is legal even though the belief was unconfirmed.
        TransitionResult<St> retry = r.sm.request(St.HIGH, "retry");
        assertTrue(retry.accepted(),
                "an unconfirmed belief must not brick the machine — it still knows its last proven state");
        r.runUntilSettled(St.HIGH, 300);
        assertTrue(r.sm.isSettledAt(St.HIGH));
    }

    @Test
    void onEnterRunsOnlyOnConfirmedArrival() {
        Rig r = new Rig(1.0);
        r.sm.seed(St.STOW);
        r.elevator.jam();
        r.sm.request(St.HIGH, "test");
        r.run(200);
        assertEquals(0, r.entries.get(), "entry actions must not fire when the machine merely gave up");

        r.sm.clearFault();
        r.elevator.unjam();
        r.sm.request(St.HIGH, "retry");
        r.runUntilSettled(St.HIGH, 400);
        assertEquals(1, r.entries.get());
    }

    @Test
    void isAtIsMeasuredNotLatched() {
        Rig r = new Rig(10.0);
        r.sm.seed(St.STOW);
        r.sm.request(St.MID, "test");
        r.runUntilSettled(St.MID, 400);
        assertTrue(r.sm.isAt(St.MID));

        r.elevator.position += 1.0;   // something shoved it
        assertFalse(r.sm.isAt(St.MID), "isAt must re-measure, not remember");
        assertFalse(r.sm.isSettledAt(St.MID));
        assertEquals(St.MID, r.sm.current(), "the last proven state does not change just because it drifted");
    }

    @Test
    void isAtDoesNotConsultTheAppliedGoal() {
        // A timer-arrival binding reports at-goal once its settle window elapses. If isAt() handed
        // that binding the elapsed time for goals belonging to OTHER states, every state carrying a
        // timer binding would read as "arrived" at once.
        FakeClock clock = new FakeClock();
        TimerBinding claw = new TimerBinding("claw", 0.3);
        FakeBinding elevator = new FakeBinding("elevator");

        StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Timers").clock(clock);
        Handle<String> hClaw = b.bind("claw", claw);
        Handle<Double> hElevator = b.bind("elevator", elevator);
        StateMachineCore<St> sm = b.initialState(St.STOW)
                .state(St.STOW, s -> s.set(hClaw, "CLOSED").set(hElevator, 0.0))
                .state(St.MID, s -> s.set(hClaw, "OPEN").set(hElevator, 0.5))
                .state(St.HIGH, s -> s.set(hClaw, "OPEN").set(hElevator, 1.0))
                .state(St.INTAKE, s -> s.set(hClaw, "OPEN").set(hElevator, 0.1))
                .state(St.CLIMB, s -> s.set(hClaw, "CLOSED").set(hElevator, 0.0))
                .hub(St.STOW)
                .build();

        sm.seed(St.STOW);
        for (int i = 0; i < 100; i++) {
            clock.advance(0.02);
            sm.step();
            elevator.drive(sm.activeGoalOf(hElevator));
        }
        assertTrue(sm.isAt(St.STOW));
        assertFalse(sm.isAt(St.MID),
                "MID must not read as arrived merely because the claw's settle window elapsed in STOW");
    }

    @Test
    void disableFreezesDeadlines() {
        Rig r = new Rig(1.0);
        r.sm.seed(St.STOW);
        r.elevator.rate = 0.02;      // slow, so the transition is genuinely in flight
        r.sm.request(St.MID, "test");
        r.run(5);
        assertTrue(r.sm.isTransitioning());

        r.sm.setEnabled(false);
        r.clock.advance(30.0);       // a long disabled period between matches
        r.sm.step();
        r.sm.setEnabled(true);

        r.run(5);
        assertEquals(0, r.sm.timeoutCount(),
                "a deadline must not run while the robot is disabled and physically cannot move");
    }

    @Test
    void notZeroedIsRejectedRatherThanAttempted() {
        Rig r = new Rig(2.0);
        r.sm.seed(St.STOW);
        r.elevator.zeroed = false;
        TransitionResult<St> result = r.sm.request(St.MID, "test");
        assertTrue(result.rejected());
        assertEquals(RejectReason.NOT_ZEROED, result.reason());
        assertTrue(result.detail().contains("elevator"), "the rejection must name the culprit: " + result.detail());
    }
}
