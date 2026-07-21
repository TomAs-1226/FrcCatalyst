package frc.lib.catalyst.statemachine;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Hardware-free fixtures shared by the state-machine tests.
 *
 * <p>Everything here implements {@link Binding} directly rather than
 * {@link frc.lib.catalyst.statemachine.robot.Actuator}, so no WPILib class is ever loaded. That is
 * the whole reason the engine takes no WPILib imports: this test suite runs on a laptop with no
 * HAL, no NetworkTables and no command scheduler.
 */
final class StateMachineFixtures {

    private StateMachineFixtures() {}

    /** The state enum used across the tests. */
    enum St { STOW, MID, HIGH, INTAKE, CLIMB }

    /** A mutable clock. Tests advance it explicitly; nothing ever calls {@code Timer}. */
    static final class FakeClock implements DoubleSupplier {
        double now;

        @Override
        public double getAsDouble() {
            return now;
        }

        void advance(double seconds) {
            now += seconds;
        }
    }

    /**
     * A binding whose measured position walks toward whatever goal it is driven to, at
     * {@code rate} per {@link #drive} call. Jam it with {@link #jam()} to reproduce a stuck
     * mechanism.
     */
    static final class FakeBinding implements Binding<Double> {
        private final String key;
        double position;
        double rate = 0.25;
        double tolerance = 0.01;
        boolean stuck;
        boolean zeroed = true;

        FakeBinding(String key) {
            this.key = key;
        }

        @Override public String key() { return key; }
        @Override public String kind() { return "fake"; }
        @Override public String unit() { return "m"; }
        @Override public boolean atGoal(Double goal, double since) {
            return Math.abs(position - goal) < tolerance;
        }
        @Override public double measured() { return position; }
        @Override public double error(Double goal) { return position - goal; }
        @Override public double tolerance(Double goal) { return tolerance; }
        @Override public boolean zeroed() { return zeroed; }
        @Override public String label(Double goal) { return String.format("%.2f", goal); }

        /** Advance one step toward {@code goal}, unless jammed. */
        void drive(Double goal) {
            if (stuck || goal == null) return;
            double delta = goal - position;
            position += Math.signum(delta) * Math.min(rate, Math.abs(delta));
        }

        void jam() { stuck = true; }
        void unjam() { stuck = false; }
    }

    /**
     * A binding whose arrival is purely a settle timer, like a claw opening or a solenoid firing.
     * Used to pin down that {@code isAt(otherState)} does not report true merely because some
     * settle window has elapsed.
     */
    static final class TimerBinding implements Binding<String> {
        private final String key;
        private final double settleSeconds;

        TimerBinding(String key, double settleSeconds) {
            this.key = key;
            this.settleSeconds = settleSeconds;
        }

        @Override public String key() { return key; }
        @Override public String kind() { return "timer"; }
        @Override public boolean atGoal(String goal, double since) { return since >= settleSeconds; }
        @Override public boolean observable(String goal) { return false; }
    }

    /** Records every telemetry call so cadence is assertable with no logging stack present. */
    static final class RecordingTelemetry<S extends Enum<S>> implements StateMachineTelemetry<S> {
        final List<String> states = new ArrayList<>();
        final List<String> blockers = new ArrayList<>();
        final List<String> details = new ArrayList<>();
        final List<TransitionRecord<S>> transitions = new ArrayList<>();
        int stateWrites;
        int blockerWrites;
        int progressWrites;
        int heartbeats;
        int graphWrites;

        @Override
        public void graph(String machine, String[] s, String[] e, String[] b,
                          String[] r, String d, List<String> errors, List<String> warnings) {
            graphWrites++;
        }

        @Override
        public void state(S current, boolean confirmed, S target, S nextHop, Phase phase, int stage) {
            stateWrites++;
            states.add((current == null ? "?" : current.name()) + "/" + confirmed + "/" + phase);
        }

        @Override
        public void blocker(String blocker, String blockerDetail, String summary, List<String> waitingOn) {
            blockerWrites++;
            blockers.add(blocker);
            details.add(blockerDetail);
        }

        @Override
        public void progress(long seq, double f, double el, double to, String route, String trigger) {
            progressWrites++;
        }

        @Override public void binding(BindingSample sample) {}
        @Override public void transition(TransitionRecord<S> record) { transitions.add(record); }
        @Override public void history(String[] newestFirst) {}
        @Override public void legalTargets(String[] targets) {}
        @Override public void counters(long t, long r, long to, long a, long y) {}
        @Override public void heartbeat(long ticks, double uptime, boolean enabled) { heartbeats++; }
        @Override public void fault(boolean faulted, String reason) {}

        /** How many distinct blocker strings were ever published. */
        long distinctBlockers() {
            return blockers.stream().distinct().count();
        }
    }
}
