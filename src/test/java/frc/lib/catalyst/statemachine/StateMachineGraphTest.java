package frc.lib.catalyst.statemachine;

import frc.lib.catalyst.statemachine.StateMachineFixtures.FakeBinding;
import frc.lib.catalyst.statemachine.StateMachineFixtures.FakeClock;
import frc.lib.catalyst.statemachine.StateMachineFixtures.St;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** The legal-transition graph, routing, and build-time validation. */
class StateMachineGraphTest {

    private static StateMachineCore.Builder<St> fullyDeclared(FakeClock clock, FakeBinding fake) {
        StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Graph").clock(clock);
        Handle<Double> h = b.bind("elevator", fake);
        return b.initialState(St.STOW)
                .state(St.STOW, s -> s.set(h, 0.0))
                .state(St.MID, s -> s.set(h, 0.5))
                .state(St.HIGH, s -> s.set(h, 1.0))
                .state(St.INTAKE, s -> s.set(h, 0.1))
                .state(St.CLIMB, s -> s.set(h, 0.0));
    }

    @Test
    void anUndeclaredStateIsABuildError() {
        FakeClock clock = new FakeClock();
        FakeBinding fake = new FakeBinding("elevator");
        StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Partial").clock(clock);
        Handle<Double> h = b.bind("elevator", fake);
        b.state(St.STOW, s -> s.set(h, 0.0)).hub(St.STOW);

        ValidationReport report = b.validate();
        assertFalse(report.ok());
        String all = String.join("\n", report.errors());
        for (St s : new St[]{St.MID, St.HIGH, St.INTAKE, St.CLIMB}) {
            assertTrue(all.contains(s.name()), "every undeclared state must be named; missing " + s);
        }
    }

    @Test
    void everyProblemIsReportedNotJustTheFirst() {
        FakeClock clock = new FakeClock();
        StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Messy").clock(clock);
        // Three deliberate problems: two undeclared states, and no edges at all.
        Handle<Double> h = b.bind("elevator", new FakeBinding("elevator"));
        b.state(St.STOW, s -> s.set(h, 0.0))
         .state(St.MID, s -> s.set(h, 0.5))
         .state(St.HIGH, s -> s.set(h, 1.0));

        ValidationReport report = b.validate();
        assertTrue(report.errors().size() >= 3,
                "a pit deploy cycle is expensive; report everything at once, got: " + report.errors());

        StateMachineConfigException ex =
                assertThrows(StateMachineConfigException.class, b::build);
        assertEquals(report.errors().size(), ex.errors().size());
    }

    @Test
    void duplicateBindingKeysAreABuildError() {
        FakeClock clock = new FakeClock();
        StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Dup").clock(clock);
        b.bind("elevator", new FakeBinding("elevator"));
        b.bind("elevator", new FakeBinding("elevator"));
        assertTrue(String.join("\n", b.validate().errors()).contains("duplicate binding key"));
    }

    @Test
    void anUnreachableStateIsAnErrorUnlessAllowed() {
        FakeClock clock = new FakeClock();
        FakeBinding fake = new FakeBinding("elevator");
        StateMachineCore.Builder<St> b = fullyDeclared(clock, fake)
                .allowBoth(St.STOW, St.MID)
                .allowBoth(St.MID, St.HIGH)
                .allowBoth(St.STOW, St.INTAKE);
        // CLIMB is declared but unreachable.
        assertTrue(String.join("\n", b.validate().errors()).contains("CLIMB"));

        b.allowUnreachable(St.CLIMB);
        assertTrue(b.validate().ok(), "allowUnreachable must silence it: " + b.validate());
    }

    @Test
    void directOnlyRefusesAnUndeclaredEdge() {
        FakeClock clock = new FakeClock();
        FakeBinding fake = new FakeBinding("elevator");
        // A deliberate chain, NOT a hub: STOW reaches HIGH only by going through MID.
        StateMachineCore<St> sm = fullyDeclared(clock, fake)
                .allowBoth(St.STOW, St.MID)
                .allowBoth(St.MID, St.HIGH)
                .allowBoth(St.STOW, St.INTAKE)
                .allowBoth(St.STOW, St.CLIMB)
                .build();
        sm.seed(St.STOW);

        TransitionResult<St> result = sm.request(St.HIGH, "test");
        assertTrue(result.rejected());
        assertEquals(RejectReason.NO_EDGE, result.reason());
        assertEquals(St.STOW, sm.current(), "a refused request must not move the machine");
        assertTrue(result.detail().contains("MID"),
                "the rejection should say what IS reachable: " + result.detail());
    }

    @Test
    void shortestPathRoutesThroughTheWaypoint() {
        FakeClock clock = new FakeClock();
        FakeBinding fake = new FakeBinding("elevator");
        StateMachineCore<St> sm = fullyDeclared(clock, fake)
                .routing(Routing.SHORTEST_PATH)
                .allowBoth(St.STOW, St.MID)
                .allowBoth(St.MID, St.HIGH)
                .allowBoth(St.STOW, St.INTAKE)
                .allowBoth(St.STOW, St.CLIMB)
                .build();
        sm.seed(St.STOW);
        assertEquals("MID>HIGH", sm.plan(St.HIGH).render());
        assertTrue(sm.canReach(St.HIGH));
    }

    @Test
    void routingIsDeterministicAcrossRebuilds() {
        String first = null;
        for (int i = 0; i < 200; i++) {
            FakeClock clock = new FakeClock();
            FakeBinding fake = new FakeBinding("elevator");
            StateMachineCore<St> sm = fullyDeclared(clock, fake)
                    .routing(Routing.SHORTEST_PATH)
                    .hub(St.STOW)
                    .allowBoth(St.MID, St.HIGH)
                    .allowBoth(St.INTAKE, St.HIGH)
                    .build();
            sm.seed(St.MID);
            String rendered = sm.plan(St.INTAKE).render();
            if (first == null) first = rendered;
            assertEquals(first, rendered, "a route must never change without a graph change");
        }
        assertNotNull(first);
    }

    @Test
    void aHubMakesOmittedEdgesReachableOnlyUnderShortestPath() {
        // Worth pinning down explicitly: hub() creates real edges, so under SHORTEST_PATH an edge
        // you deliberately left out becomes reachable in two hops. DIRECT_ONLY is the safety switch,
        // not the graph.
        FakeClock clock = new FakeClock();
        FakeBinding a = new FakeBinding("elevator");
        StateMachineCore<St> direct = fullyDeclared(clock, a).hub(St.STOW).build();
        direct.seed(St.CLIMB);
        assertEquals(RejectReason.NO_EDGE, direct.request(St.HIGH, "t").reason());

        FakeClock clock2 = new FakeClock();
        FakeBinding b = new FakeBinding("elevator");
        StateMachineCore<St> routed = fullyDeclared(clock2, b)
                .routing(Routing.SHORTEST_PATH).hub(St.STOW).build();
        routed.seed(St.CLIMB);
        assertEquals("STOW>HIGH", routed.plan(St.HIGH).render(),
                "under SHORTEST_PATH the hub makes CLIMB->HIGH legal via STOW");
    }

    @Test
    void viaPinsTheRouteOverBreadthFirstSearch() {
        FakeClock clock = new FakeClock();
        FakeBinding fake = new FakeBinding("elevator");
        StateMachineCore<St> sm = fullyDeclared(clock, fake)
                .routing(Routing.SHORTEST_PATH)
                .hub(St.STOW)
                .allowBoth(St.MID, St.HIGH)
                .via(St.CLIMB, St.HIGH, St.STOW, St.MID)
                .build();
        sm.seed(St.CLIMB);
        assertEquals("STOW>MID>HIGH", sm.plan(St.HIGH).render(),
                "a pinned route must win even when a shorter one exists");
    }

    @Test
    void dotOutputListsEveryEdge() {
        FakeClock clock = new FakeClock();
        FakeBinding fake = new FakeBinding("elevator");
        StateMachineCore<St> sm = fullyDeclared(clock, fake).hub(St.STOW).build();
        String dot = sm.graph().toDot("Test");
        int arrows = dot.split("->", -1).length - 1;
        assertEquals(sm.graph().edgeCount(), arrows);
        for (St s : St.values()) assertTrue(dot.contains(s.name()));
    }

    @Test
    void legalTargetsReflectsGuardsAndInterlocks() {
        FakeClock clock = new FakeClock();
        FakeBinding fake = new FakeBinding("elevator");
        boolean[] endgame = {false};
        StateMachineCore<St> sm = fullyDeclared(clock, fake)
                .hub(St.STOW)
                .edge(St.STOW, St.CLIMB, e -> e.guard(() -> endgame[0], "endgame"))
                .build();
        sm.seed(St.STOW);
        assertFalse(sm.legalTargets().contains(St.CLIMB), "a false guard must remove the target");
        endgame[0] = true;
        assertTrue(sm.legalTargets().contains(St.CLIMB));
    }
}
