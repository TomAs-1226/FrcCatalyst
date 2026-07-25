package frc.lib.catalyst.statemachine;

import frc.lib.catalyst.statemachine.StateMachineFixtures.FakeBinding;
import frc.lib.catalyst.statemachine.StateMachineFixtures.FakeClock;
import frc.lib.catalyst.statemachine.StateMachineFixtures.RecordingTelemetry;
import frc.lib.catalyst.statemachine.StateMachineFixtures.St;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Log cadence and content.
 *
 * <p>Cadence is a correctness property, not a nicety: a diagnosis string carrying live floats,
 * written "on change", degrades to a 50 Hz write exactly when the robot is stuck and the dashboard
 * is what you are staring at.
 */
class StateMachineTelemetryTest {

    private static final class Rig {
        final FakeClock clock = new FakeClock();
        final FakeBinding elevator = new FakeBinding("elevator");
        final RecordingTelemetry<St> telemetry = new RecordingTelemetry<>();
        final Handle<Double> h;
        final StateMachineCore<St> sm;

        Rig(double timeout) {
            StateMachineCore.Builder<St> b =
                    StateMachineCore.builder(St.class, "Telemetry").clock(clock);
            h = b.bind("elevator", elevator);
            sm = b.telemetry(telemetry)
                    .defaultTimeout(timeout)
                    .historyCapacity(50)
                    .initialState(St.STOW)
                    .state(St.STOW, s -> s.set(h, 0.0))
                    .state(St.MID, s -> s.set(h, 0.5))
                    .state(St.HIGH, s -> s.set(h, 1.0))
                    .state(St.INTAKE, s -> s.set(h, 0.1))
                    .state(St.CLIMB, s -> s.set(h, 0.0))
                    .hub(St.STOW)
                    .build();
        }

        void step(int n) {
            for (int i = 0; i < n; i++) {
                clock.advance(0.02);
                sm.step();
                elevator.drive(sm.activeGoalOf(h));
            }
        }
    }

    @Test
    void stateIsWrittenOnlyOnChange() {
        Rig r = new Rig(10.0);
        r.sm.seed(St.STOW);
        r.step(500);
        assertTrue(r.telemetry.stateWrites <= 3,
                "500 idle loops must not produce 500 state writes, got " + r.telemetry.stateWrites);
    }

    @Test
    void blockerIsWrittenOnlyOnChangeWhileStuck() {
        Rig r = new Rig(1000.0);
        r.sm.seed(St.STOW);
        r.elevator.jam();
        r.sm.request(St.MID, "test");
        r.step(500);
        assertTrue(r.telemetry.distinctBlockers() <= 3,
                "the stable blocker string must not churn: " + r.telemetry.blockers.stream()
                        .distinct().toList());
    }

    @Test
    void blockerDetailIsThrottledToFiveHz() {
        Rig r = new Rig(1000.0);
        r.sm.seed(St.STOW);
        r.elevator.rate = 0.0001;             // creeping, so the live detail changes every loop
        r.sm.request(St.MID, "test");
        r.step(500);                           // 10 seconds at 50 Hz

        long distinctDetails = r.telemetry.details.stream().distinct().count();
        assertTrue(distinctDetails <= 60,
                "BlockerDetail carries live numbers and must be throttled to ~5 Hz; "
                        + distinctDetails + " distinct values over 10 s");
    }

    @Test
    void heartbeatIncrementsEveryStep() {
        Rig r = new Rig(10.0);
        r.sm.seed(St.STOW);
        r.step(120);
        assertEquals(120, r.sm.ticks());
        assertEquals(120, r.telemetry.heartbeats,
                "a machine that has stopped stepping is otherwise indistinguishable from an idle one");
    }

    @Test
    void historyIsNewestFirstAndBounded() {
        Rig r = new Rig(10.0);
        r.sm.seed(St.STOW);
        for (int i = 0; i < 200; i++) {
            r.sm.request(i % 2 == 0 ? St.MID : St.STOW, "loop" + i);
            for (int s = 0; s < 60 && r.sm.isTransitioning(); s++) r.step(1);
        }
        List<TransitionRecord<St>> history = r.sm.history();
        assertEquals(50, history.size(), "the ring must stay bounded");
        for (int i = 1; i < history.size(); i++) {
            assertTrue(history.get(i - 1).endTimestamp() >= history.get(i).endTimestamp(),
                    "history must be newest-first");
        }
    }

    @Test
    void everyRejectionEmitsExactlyOneRecord() {
        Rig r = new Rig(10.0);
        // Under a STOW hub, MID's only successor is STOW — so MID->INTAKE has no edge.
        r.sm.seed(St.MID);
        int before = r.telemetry.transitions.size();
        r.sm.request(St.INTAKE, "test");
        assertEquals(RejectReason.NO_EDGE, r.sm.lastResult().reason());
        assertEquals(before + 1, r.telemetry.transitions.size());
        assertEquals(TransitionRecord.Outcome.REJECTED,
                r.telemetry.transitions.get(r.telemetry.transitions.size() - 1).outcome());
    }

    @Test
    void serializedRecordsAlwaysSplitIntoTenColumns() {
        Rig r = new Rig(10.0);
        r.sm.seed(St.STOW);
        r.sm.request(St.MID, "trigger|with|pipes");
        r.step(60);
        for (TransitionRecord<St> record : r.sm.history()) {
            String line = record.serialize();
            assertEquals(10, line.split("\\|", -1).length,
                    "a dashboard table parser relies on the column count: " + line);
        }
    }

    @Test
    void arrivalReportsCaptureEachMechanismsOwnTime() {
        FakeClock clock = new FakeClock();
        FakeBinding fast = new FakeBinding("fast");
        FakeBinding slow = new FakeBinding("slow");
        fast.rate = 0.5;
        slow.rate = 0.05;

        StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Arrivals").clock(clock);
        Handle<Double> hFast = b.bind("fast", fast);
        Handle<Double> hSlow = b.bind("slow", slow);
        StateMachineCore<St> sm = b.defaultTimeout(30.0)
                .initialState(St.STOW)
                .state(St.STOW, s -> s.set(hFast, 0.0).set(hSlow, 0.0))
                .state(St.MID, s -> s.set(hFast, 1.0).set(hSlow, 1.0))
                .state(St.HIGH, s -> s.set(hFast, 2.0).set(hSlow, 2.0))
                .state(St.INTAKE, s -> s.set(hFast, 0.1).set(hSlow, 0.1))
                .state(St.CLIMB, s -> s.set(hFast, 0.0).set(hSlow, 0.0))
                .hub(St.STOW)
                .build();
        sm.seed(St.STOW);
        sm.request(St.MID, "test");
        for (int i = 0; i < 2000 && !sm.isSettledAt(St.MID); i++) {
            clock.advance(0.02);
            sm.step();
            fast.drive(sm.activeGoalOf(hFast));
            slow.drive(sm.activeGoalOf(hSlow));
        }
        assertTrue(sm.isSettledAt(St.MID));

        TransitionRecord<St> record = sm.lastTransition();
        assertEquals(2, record.arrivals().size());
        double fastTime = record.arrivals().stream()
                .filter(a -> a.key().equals("fast")).findFirst().orElseThrow().arrivalSeconds();
        double slowTime = record.arrivals().stream()
                .filter(a -> a.key().equals("slow")).findFirst().orElseThrow().arrivalSeconds();
        assertTrue(slowTime > fastTime,
                "per-binding arrival times are what identify the critical path: fast=" + fastTime
                        + " slow=" + slowTime);
    }

    @Test
    void noopTelemetryNeverThrows() {
        StateMachineTelemetry<St> noop = StateMachineTelemetry.noop();
        noop.graph("m", new String[0], new String[0], new String[0], new String[0], "", List.of(), List.of());
        noop.state(St.STOW, true, St.MID, St.MID, Phase.MOVING, -1);
        noop.blocker("", "", "", List.of());
        noop.progress(1, 0.5, 1.0, 2.0, "", "");
        noop.heartbeat(1, 1.0, true);
        noop.fault(false, "");
        assertFalse(false);   // reaching here without an exception is the assertion
    }
}
