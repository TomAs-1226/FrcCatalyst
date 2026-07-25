package frc.lib.catalyst.statemachine;

import frc.lib.catalyst.statemachine.StateMachineFixtures.FakeBinding;
import frc.lib.catalyst.statemachine.StateMachineFixtures.FakeClock;
import frc.lib.catalyst.statemachine.StateMachineFixtures.RecordingTelemetry;
import frc.lib.catalyst.statemachine.StateMachineFixtures.St;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Guards, interlocks, staging, superseding, and the sweeps that pin the safety invariants. */
class StateMachineTransitionTest {

    /** A three-mechanism rig with configurable guards, so each test can flip one condition. */
    private static final class Rig {
        final FakeClock clock = new FakeClock();
        final FakeBinding elevator = new FakeBinding("elevator");
        final FakeBinding arm = new FakeBinding("arm");
        final FakeBinding leds = new FakeBinding("leds");
        final boolean[] guardOk = {true};
        final boolean[] interlockOk = {true};
        final Handle<Double> hElevator;
        final Handle<Double> hArm;
        final Handle<Double> hLeds;
        final RecordingTelemetry<St> telemetry = new RecordingTelemetry<>();
        final StateMachineCore<St> sm;

        Rig() {
            arm.rate = 5.0;
            elevator.rate = 0.05;
            leds.rate = 100.0;
            StateMachineCore.Builder<St> b =
                    StateMachineCore.builder(St.class, "Transitions").clock(clock);
            hElevator = b.bind("elevator", elevator);
            hArm = b.bind("arm", arm);
            hLeds = b.bindAdvisory("leds", leds);
            sm = b.telemetry(telemetry)
                    .defaultTimeout(20.0)
                    .initialState(St.STOW)
                    .state(St.STOW, s -> s.set(hElevator, 0.0).set(hArm, 0.0).set(hLeds, 0.0))
                    .state(St.MID, s -> s.set(hElevator, 0.5).set(hArm, 30.0).set(hLeds, 1.0))
                    .state(St.HIGH, s -> s.set(hElevator, 1.0).set(hArm, 90.0).set(hLeds, 2.0))
                    .state(St.INTAKE, s -> s.set(hElevator, 0.1).set(hArm, -10.0).set(hLeds, 3.0))
                    .state(St.CLIMB, s -> s.set(hElevator, 0.0).set(hArm, 5.0).release(hLeds))
                    .hub(St.STOW)
                    .allowBoth(St.MID, St.HIGH)
                    .edge(St.STOW, St.HIGH, e -> e.guard(() -> guardOk[0], "armClear")
                            .stage(hElevator).stage(hArm))
                    .interlock("climberStowed", () -> interlockOk[0], s -> s == St.MID)
                    .build();
            sm.seed(St.STOW);
        }

        void run(int steps) {
            for (int i = 0; i < steps; i++) {
                clock.advance(0.02);
                sm.step();
                elevator.drive(sm.activeGoalOf(hElevator));
                arm.drive(sm.activeGoalOf(hArm));
                leds.drive(sm.activeGoalOf(hLeds));
            }
        }

        void runUntilSettled(St state, int maxSteps) {
            for (int i = 0; i < maxSteps && !sm.isSettledAt(state); i++) run(1);
        }
    }

    @Test
    void aFalseGuardBlocksWithoutChangingState() {
        Rig r = new Rig();
        r.guardOk[0] = false;
        TransitionResult<St> result = r.sm.request(St.HIGH, "test");
        assertTrue(result.rejected());
        assertEquals(RejectReason.GUARD_BLOCKED, result.reason());
        assertEquals("armClear", result.detail());
        assertEquals(St.STOW, r.sm.current());
        assertTrue(r.sm.stateConfirmed());
    }

    @Test
    void anUnsatisfiedInterlockBlocksOnlyItsTargets() {
        Rig r = new Rig();
        r.interlockOk[0] = false;
        assertEquals(RejectReason.INTERLOCK_BLOCKED, r.sm.request(St.MID, "test").reason());
        assertEquals("climberStowed", r.sm.lastResult().detail());
        // INTAKE is not blocked by this interlock.
        assertTrue(r.sm.request(St.INTAKE, "test").accepted());
    }

    @Test
    void anEntryGuardBlocksFromEveryOrigin() {
        FakeClock clock = new FakeClock();
        FakeBinding fake = new FakeBinding("elevator");
        boolean[] gripped = {false};
        StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Entry").clock(clock);
        Handle<Double> h = b.bind("elevator", fake);
        StateMachineCore<St> sm = b.initialState(St.STOW)
                .state(St.STOW, s -> s.set(h, 0.0))
                .state(St.MID, s -> s.set(h, 0.5))
                .state(St.HIGH, s -> s.set(h, 1.0).entryGuard(() -> gripped[0], "no piece"))
                .state(St.INTAKE, s -> s.set(h, 0.1))
                .state(St.CLIMB, s -> s.set(h, 0.0))
                .hub(St.STOW)
                .allowBoth(St.MID, St.HIGH)
                .build();

        for (St origin : new St[]{St.STOW, St.MID}) {
            sm.seed(origin);
            TransitionResult<St> result = sm.request(St.HIGH, "test");
            assertTrue(result.rejected(), "entry guard must block from " + origin);
            assertEquals(RejectReason.ENTRY_GUARD_BLOCKED, result.reason());
            assertEquals("no piece", result.detail());
        }
    }

    @Test
    void aStagedEdgeHoldsLaterStagesAtTheOriginGoal() {
        Rig r = new Rig();
        r.sm.request(St.HIGH, "test");
        r.run(1);

        assertEquals(1.0, r.sm.activeGoalOf(r.hElevator), 1e-9);
        assertEquals(0.0, r.sm.activeGoalOf(r.hArm), 1e-9,
                "stage 1 must stay parked at the FROM goal until stage 0 arrives");
        assertEquals(0, r.sm.stageIndex());
        assertEquals(2, r.sm.stageCount());

        r.runUntilSettled(St.HIGH, 2000);
        assertTrue(r.sm.isSettledAt(St.HIGH));
    }

    @Test
    void anAdvisoryBindingDoesNotGateArrival() {
        Rig r = new Rig();
        r.leds.jam();                    // the advisory binding can never arrive
        r.sm.request(St.MID, "test");
        r.runUntilSettled(St.MID, 2000);
        assertTrue(r.sm.isSettledAt(St.MID), "an advisory binding must never hold up a transition");
    }

    @Test
    void releaseHandsTheMechanismBack() {
        Rig r = new Rig();
        r.sm.request(St.CLIMB, "test");
        r.run(1);
        assertNull(r.sm.activeGoalOf(r.hLeds), "a released binding must have no active goal");
    }

    @Test
    void aSecondRequestSupersedesAndIsRecorded() {
        Rig r = new Rig();
        r.sm.request(St.MID, "first");
        r.run(2);
        assertTrue(r.sm.isTransitioning());

        TransitionResult<St> second = r.sm.request(St.INTAKE, "second");
        assertTrue(second.accepted());

        boolean sawSuperseded = r.telemetry.transitions.stream()
                .anyMatch(t -> t.outcome() == TransitionRecord.Outcome.SUPERSEDED);
        assertTrue(sawSuperseded, "the abandoned transition must leave a record, not vanish");
        assertEquals(St.INTAKE, r.sm.target());
    }

    @Test
    void aRefusedRequestNeverDisturbsATransitionInFlight() {
        Rig r = new Rig();
        r.sm.request(St.MID, "real");
        r.run(2);
        assertTrue(r.sm.isTransitioning());

        r.guardOk[0] = false;
        r.sm.request(St.HIGH, "refused");     // rejected: guard is false
        assertTrue(r.sm.isTransitioning(), "a rejection must not silently abandon the running transition");
        assertEquals(St.MID, r.sm.target());

        r.runUntilSettled(St.MID, 2000);
        assertTrue(r.sm.isSettledAt(St.MID));
    }

    @Test
    void aTimedOutTransitionKeepsPursuingRatherThanDrivingBack() {
        FakeClock clock = new FakeClock();
        FakeBinding elevator = new FakeBinding("elevator");
        elevator.rate = 0.01;
        StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Hold").clock(clock);
        Handle<Double> h = b.bind("elevator", elevator);
        StateMachineCore<St> sm = b.defaultTimeout(0.5)
                .initialState(St.STOW)
                .state(St.STOW, s -> s.set(h, 0.0))
                .state(St.MID, s -> s.set(h, 1.0))
                .state(St.HIGH, s -> s.set(h, 2.0))
                .state(St.INTAKE, s -> s.set(h, 0.1))
                .state(St.CLIMB, s -> s.set(h, 0.0))
                .hub(St.STOW)
                .build();
        sm.seed(St.STOW);
        sm.request(St.MID, "test");
        for (int i = 0; i < 100; i++) {
            clock.advance(0.02);
            sm.step();
            elevator.drive(sm.activeGoalOf(h));
        }
        assertEquals(1, sm.timeoutCount());
        assertEquals(1.0, sm.activeGoalOf(h), 1e-9,
                "HOLD_AND_REPORT must keep the goal it had — snapping back would actively drive the "
                        + "robot backwards through whatever it just failed to clear");
    }

    @Test
    void everyRejectedRequestLeavesTheMachineByteIdentical() {
        // Sweep every ordered state pair against every guard/interlock combination. Whenever the
        // request is refused, nothing about the machine's belief may have changed.
        for (boolean guard : new boolean[]{true, false}) {
            for (boolean interlock : new boolean[]{true, false}) {
                for (St from : St.values()) {
                    for (St to : St.values()) {
                        Rig r = new Rig();
                        r.guardOk[0] = guard;
                        r.interlockOk[0] = interlock;
                        r.sm.seed(from);

                        St beforeCurrent = r.sm.current();
                        boolean beforeConfirmed = r.sm.stateConfirmed();
                        long beforeTransitions = r.sm.transitionCount();

                        TransitionResult<St> result = r.sm.request(to, "sweep");
                        if (result.rejected()) {
                            assertEquals(beforeCurrent, r.sm.current(),
                                    "rejected " + from + "->" + to + " moved current()");
                            assertEquals(beforeConfirmed, r.sm.stateConfirmed(),
                                    "rejected " + from + "->" + to + " changed confirmation");
                            assertEquals(beforeTransitions, r.sm.transitionCount(),
                                    "rejected " + from + "->" + to + " counted a transition");
                            assertFalse(result.detail().isEmpty(),
                                    "rejected " + from + "->" + to + " gave no reason");
                        }
                    }
                }
            }
        }
    }

    @Test
    void everyAcceptedRequestTerminatesWithinItsDeadline() {
        // No accepted transition may run forever, even with a jammed mechanism: the deadline is
        // always finite, so the machine must reach a resting phase.
        for (St from : St.values()) {
            for (St to : St.values()) {
                if (from == to) continue;
                Rig r = new Rig();
                r.sm.seed(from);
                if (!r.sm.request(to, "sweep").accepted()) continue;
                r.elevator.jam();
                r.arm.jam();

                int steps = 0;
                int limit = (int) (r.sm.timeoutSeconds() / 0.02) + 200;
                while (r.sm.isTransitioning() && steps++ < limit) r.run(1);

                assertFalse(r.sm.isTransitioning(),
                        from + "->" + to + " never terminated within its deadline");
            }
        }
    }
}
