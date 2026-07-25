package frc.lib.catalyst.statemachine.goals;

import frc.lib.catalyst.mechanisms.PneumaticMechanism;

import java.util.Objects;

/**
 * A commanded solenoid state for a {@link PneumaticMechanism}, together with the two numbers
 * the state machine needs in order to decide that the piston has actually got there.
 *
 * <p>A pneumatic actuator is the one Catalyst mechanism with no position feedback whatsoever.
 * Nothing on the robot can tell you where the rod is; all the code knows is which coil it
 * energised. Arrival is therefore never a measurement, it is a settle timer — {@code
 * settleSeconds} is how long the binding waits after commanding the state before it is willing
 * to call the goal reached, and the binding reports {@code observable() == false} for every
 * goal of this type so that nobody reading a log later mistakes that timer for a sensor.
 *
 * <p>The second number exists because of a specific, easily-missed behaviour in
 * {@code PneumaticMechanism.applyState}. When the mechanism was built with
 * {@code requirePressureAbove(psi)} and the measured pressure is below that threshold, a
 * {@code FORWARD} actuation is <b>silently refused</b>: the mechanism raises an
 * {@code AlertManager} warning, returns early without changing {@code getState()}, and yet the
 * {@code runOnce} command that asked for it still ends completely normally. To the state
 * machine that looks exactly like a successful actuation of something that then never arrives.
 * {@code reassertPeriodLoops} is the answer — the binding re-runs the pursue command every N
 * loops for as long as {@code atGoal} is false, so that a piston refused at 30 psi fires by
 * itself the moment the compressor catches up, instead of the robot sitting in a state that
 * can never complete. The same retry incidentally repairs the case where some other command
 * (an operator-bound {@code toggle()}, or the {@code stop()} that runs on disable) moved the
 * solenoid out from under the state machine.
 *
 * <p>Arrival is consequently tested as {@code getState() == goal.state()} and never as
 * "the command finished". Only the commanded state is ground truth here.
 *
 * <p>This is a {@code record}, so it has the value-based {@code equals}/{@code hashCode} the
 * engine requires when it compares the wanted goal against the active goal every loop. Its
 * label is simply the enum constant name, which is low cardinality and free of live values.
 *
 * @param state               the solenoid state to command. {@code FORWARD} energises the
 *                            forward channel, {@code REVERSE} the reverse channel of a double
 *                            solenoid, and {@code OFF} de-energizes. Note that on a
 *                            <em>single</em> solenoid the mechanism's own {@code retract()}
 *                            factory maps to {@code OFF} rather than {@code REVERSE}; a
 *                            {@code REVERSE} goal against a single-solenoid mechanism is
 *                            structurally impossible and is reported by the binding's
 *                            {@code validate} at build time rather than discovered in a pit.
 * @param settleSeconds       how long after the state is commanded before the goal counts as
 *                            reached. This is a stand-in for the piston's travel time; too
 *                            small and a following state starts while the rod is still moving,
 *                            too large and every transition through this mechanism pays the
 *                            difference. Negative values are normalised to zero.
 * @param reassertPeriodLoops how often, in 50 Hz loops, to re-run the actuation while the goal
 *                            has not been reached. {@code 0} disables re-assertion. Negative
 *                            values are normalised to {@code 0}.
 *
 * @since 1.2.0
 */
public record PneumaticGoal(PneumaticMechanism.State state, double settleSeconds, int reassertPeriodLoops) {

    /**
     * Default settle time in seconds for a commanded piston travel.
     *
     * <p>A quarter of a second is a deliberately generous estimate for a typical FRC cylinder
     * on a healthy air system. It is a starting point to be tuned down per mechanism once the
     * real travel has been watched, not a measured constant.
     */
    public static final double DEFAULT_SETTLE_SECONDS = 0.25;

    /**
     * Default re-assertion period in 50 Hz loops, which is roughly half a second.
     *
     * <p>Fast enough that a piston refused for low pressure fires within a human-imperceptible
     * delay of the compressor recovering, slow enough that a genuinely stuck mechanism is not
     * spamming the solenoid and the alert log at loop rate.
     */
    public static final int DEFAULT_REASSERT_PERIOD_LOOPS = 25;

    /**
     * Canonical constructor, which normalises the two tunable numbers rather than rejecting
     * them.
     *
     * <p>A negative settle time and a negative re-assert period are both meaningless rather
     * than dangerous, so they are clamped to the nearest sane value — zero, and "no
     * re-assertion" respectively — instead of throwing. Values that are merely
     * <em>wrong for this robot</em>, such as a {@code REVERSE} goal on a single solenoid or a
     * settle time long enough to stall an auto routine, are far better reported by the
     * binding's build-time {@code validate}, which aggregates every such problem in the
     * superstructure into one exception on a laptop instead of one exception per deploy.
     *
     * <p>A null {@code state} is the one exception, and it is deliberately not left to
     * {@code validate}. It is a structural mistake rather than a tunable value, and its
     * downstream symptom is genuinely misleading: the binding's arrival test
     * {@code getState() == goal.state()} would simply never match, producing a state that
     * hangs forever with no obvious cause. Failing loudly at the construction site, where the
     * stack trace names the line that built the goal, is strictly more useful than a deferred
     * message.
     *
     * @throws NullPointerException when {@code state} is null
     */
    public PneumaticGoal {
        Objects.requireNonNull(state, "PneumaticGoal.state must not be null");
        if (settleSeconds < 0.0 || Double.isNaN(settleSeconds)) {
            settleSeconds = 0.0;
        }
        if (reassertPeriodLoops < 0) {
            reassertPeriodLoops = 0;
        }
    }

    /**
     * The piston driven out: {@code FORWARD}, settled over {@value #DEFAULT_SETTLE_SECONDS}
     * seconds, re-asserted every {@value #DEFAULT_REASSERT_PERIOD_LOOPS} loops.
     *
     * <p>This is the only direction the mechanism's pressure guard can refuse, which is exactly
     * why it carries a re-assert period. Without one, a {@code FORWARD} goal commanded while
     * the tank is below {@code requirePressureAbove(psi)} would be dropped on the floor — the
     * warning would appear in the alert log, the command would end normally, and the state
     * would wait on an actuation that was never going to happen.
     *
     * @return a goal that extends the actuator
     */
    public static PneumaticGoal extended() {
        return new PneumaticGoal(PneumaticMechanism.State.FORWARD,
                DEFAULT_SETTLE_SECONDS, DEFAULT_REASSERT_PERIOD_LOOPS);
    }

    /**
     * The piston driven back: {@code REVERSE}, settled over {@value #DEFAULT_SETTLE_SECONDS}
     * seconds, re-asserted every {@value #DEFAULT_REASSERT_PERIOD_LOOPS} loops.
     *
     * <p>This goal is for double solenoids, where retracting means actively energising the
     * reverse channel. For a single solenoid, retracting is de-energising, so use
     * {@link #off()} instead — the binding's {@code validate} will say so at build time if a
     * {@code REVERSE} goal is aimed at a single-solenoid mechanism.
     *
     * <p>Reverse actuation is never refused by the pressure guard, so the re-assert period is
     * not repairing a refusal here. It is retained so that a solenoid moved by anything outside
     * the state machine is driven back to the state the machine believes it is holding.
     *
     * @return a goal that retracts a double-solenoid actuator
     */
    public static PneumaticGoal retracted() {
        return new PneumaticGoal(PneumaticMechanism.State.REVERSE,
                DEFAULT_SETTLE_SECONDS, DEFAULT_REASSERT_PERIOD_LOOPS);
    }

    /**
     * Both coils de-energized: {@code OFF}, reached immediately, re-asserted every
     * {@value #DEFAULT_REASSERT_PERIOD_LOOPS} loops.
     *
     * <p>The settle time is zero because {@code OFF} has no travel to wait for. It is an
     * electrical state, not a destination — on a double solenoid the rod simply stops being
     * driven and stays wherever pressure and friction leave it, and on a single solenoid the
     * spring return has already been accounted for by whatever goal preceded this one. Waiting
     * a quarter second here would buy nothing and would slow down every state that parks the
     * mechanism.
     *
     * @return a goal that de-energizes the actuator
     */
    public static PneumaticGoal off() {
        return new PneumaticGoal(PneumaticMechanism.State.OFF,
                0.0, DEFAULT_REASSERT_PERIOD_LOOPS);
    }

    /**
     * An explicitly tuned goal, for when the defaults do not fit the mechanism.
     *
     * <p>Reach for this once the real cylinder has been watched and timed. A short-throw hatch
     * ejector may settle in well under {@value #DEFAULT_SETTLE_SECONDS} seconds and is worth
     * tightening, while a long climber cylinder on a marginal air system may need considerably
     * longer and a faster re-assert.
     *
     * @param state               the solenoid state to command; must not be null
     * @param settleSeconds       travel time to wait before declaring arrival; negatives are
     *                            normalised to zero
     * @param reassertPeriodLoops loops between re-assertions while the goal is unreached, or
     *                            {@code 0} to disable; negatives are normalised to {@code 0}
     * @return a goal with exactly these values
     */
    public static PneumaticGoal of(PneumaticMechanism.State state, double settleSeconds, int reassertPeriodLoops) {
        return new PneumaticGoal(state, settleSeconds, reassertPeriodLoops);
    }
}
