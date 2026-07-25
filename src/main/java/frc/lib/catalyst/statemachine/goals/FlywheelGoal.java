package frc.lib.catalyst.statemachine.goals;

import java.util.function.DoubleSupplier;

/**
 * A goal for a {@link frc.lib.catalyst.mechanisms.FlywheelMechanism} binding: either a fixed
 * pair of wheel speeds, a live speed re-read from a supplier every loop, or an explicit idle.
 *
 * <p>Speeds are always stored in <b>rotations per second</b>, because that is the unit
 * {@code FlywheelMechanism.spinUp} and {@code getVelocity} actually speak. The
 * {@link #rpm(double)} factories exist so that a team whose shot table is written in RPM — which
 * is most of them — converts exactly once, here, instead of scattering {@code / 60.0} across
 * every state declaration and eventually forgetting one.
 *
 * <h2>Why idle is a first-class goal rather than "just RPS zero"</h2>
 *
 * <p>{@code FlywheelMechanism.atSpeed()} opens with {@code if (primarySetpointRPS == 0) return
 * false;}. A stopped flywheel therefore reports "not at speed" forever, and a state whose only
 * gating binding is a stopped shooter would never be reported as reached — the machine would sit
 * in transition for the rest of the match. A goal that means "spun down" consequently cannot be
 * arrival-checked through {@code atSpeed()} at all. It needs its own branch, and that branch
 * needs a tolerance, because a coasting flywheel takes several seconds to reach a true zero and
 * will never sit at exactly {@code 0.0}. That is what {@link #idleToleranceRPS()} is for, and
 * {@link #isIdle()} is how a binding decides which arrival test to run:
 *
 * <pre>{@code
 * public boolean atGoal(FlywheelGoal g, double secondsSinceApplied) {
 *     if (g.isIdle()) {
 *         return Math.abs(mechanism.getVelocity()) <= g.idleToleranceRPS();
 *     }
 *     return mechanism.atSpeed();
 * }
 * }</pre>
 *
 * <h2>Equality: value on the numbers, identity on the supplier</h2>
 *
 * <p>This is a record, so {@code equals} compares every component. Four of the five components
 * are values and compare by value. The fifth, {@link #tracked()}, is a
 * {@link DoubleSupplier} — almost always a lambda — and lambdas have no value equality, so that
 * component compares by <b>reference identity</b>. Two separately-created lambdas that compute
 * the identical number are not equal and never will be.
 *
 * <p>That matters because the engine calls {@code Objects.equals(wanted, active)} every loop to
 * decide whether the actuation it already applied is still the actuation it wants. A tracked goal
 * constructed fresh inside a state declaration that is itself evaluated per loop would compare
 * unequal to itself every time, and the engine would tear down and rebuild the pursue command at
 * 50 Hz — the flywheel would be re-initialised continuously and never actually spin up. So:
 *
 * <p><b>Construct a tracked goal once and hold it in a {@code static final} or a field.</b> Pass
 * that same instance everywhere the goal is referenced. Fixed-speed goals built by
 * {@link #rps(double)} and {@link #rpm(double)} have no such constraint — they are pure values
 * and may be rebuilt freely.
 *
 * <p>The mandatory {@code label} argument on {@link #track(String, DoubleSupplier)} falls out of
 * the same problem from the logging side. A tracked goal has no fixed number to name itself
 * after, so the obvious label would interpolate the supplier's current output — and
 * {@code Binding.label} is edge-detected into the log, so a label carrying a live float would
 * write a new string every loop for the whole match. Requiring the caller to name the goal
 * ("SubwooferShot", "SOTF") makes that failure impossible to write by accident. Use
 * {@code detail(goal)}, which is never edge-detected, when you want to see the live number.
 *
 * @param primaryRPS       target speed of the primary wheel, in rotations per second. Ignored
 *                         when {@link #tracked()} is non-null, since the supplier supersedes it.
 * @param secondaryRPS     target speed of the independently-controlled second wheel, in rotations
 *                         per second. Set this different from {@code primaryRPS} to impart
 *                         backspin or topspin. On a mechanism configured without a second motor
 *                         {@code FlywheelMechanism.spinUp} quietly ignores it, so a non-matching
 *                         secondary on a one-motor flywheel is a configuration mistake that the
 *                         binding's {@code validate} reports at build rather than something this
 *                         record can judge on its own.
 * @param idleToleranceRPS how close to zero counts as spun down, in rotations per second.
 *                         Meaningful only when {@link #isIdle()} is true.
 * @param tracked          supplier re-read every loop for a speed that changes with range, or
 *                         {@code null} for a fixed goal. Compared by identity — see above.
 * @param label            stable, low-cardinality name for this goal in logs.
 *
 * @since 1.2.0
 */
public record FlywheelGoal(
        double primaryRPS,
        double secondaryRPS,
        double idleToleranceRPS,
        DoubleSupplier tracked,
        String label) {

    /**
     * Idle tolerance applied when a caller supplies none, or supplies one that cannot work.
     *
     * <p>One rotation per second is roughly the noise floor of a Talon velocity signal on a
     * geared-down flywheel, so it reads as "stopped" without waiting on the last of the coast.
     */
    public static final double DEFAULT_IDLE_TOLERANCE_RPS = 1.0;

    /**
     * Normalises the components that would otherwise break goal comparison or logging.
     *
     * <p>Two normalisations happen here, and neither is cosmetic.
     *
     * <p>First, negative zero is folded to positive zero. Record {@code equals} compares
     * {@code double} components as {@code Double.compare} does, under which {@code -0.0} and
     * {@code 0.0} are <em>not</em> equal. A shot table that produced {@code -0.0} for one goal
     * and {@code 0.0} for another spelling of the same goal would make the engine believe the
     * wanted goal had changed on every comparison and rebuild the command every loop. Adding
     * {@code 0.0} collapses the two representations without touching any other value.
     *
     * <p>Second, a non-positive or non-finite idle tolerance is replaced with
     * {@link #DEFAULT_IDLE_TOLERANCE_RPS}. A tolerance of zero is not a strict request, it is an
     * unreachable one — a real flywheel does not report exactly zero — and honouring it literally
     * would strand the machine in transition.
     *
     * <p>Nothing here throws. A genuinely wrong speed — negative, {@code NaN}, past the
     * mechanism's ceiling — is left intact so that the binding's {@code validate} can report it
     * at build time with the mechanism's actual limits in the message, alongside every other
     * problem in the configuration, instead of one exception per deploy cycle in the pit. A blank
     * label is likewise repaired rather than rejected, because losing a log name is not worth
     * taking the robot code down for.
     *
     * <p>Prefer the named factories below to this constructor; they pick a consistent label and
     * a sane tolerance for you. It stays public only because a record's canonical constructor
     * must be, and because a team generating goals from a table needs a way in.
     *
     * @param primaryRPS       primary wheel speed in rotations per second; negative zero folded
     * @param secondaryRPS     secondary wheel speed in rotations per second; negative zero folded
     * @param idleToleranceRPS stopped-band half-width in rotations per second; a non-positive or
     *                         non-finite value is replaced with {@link #DEFAULT_IDLE_TOLERANCE_RPS}
     * @param tracked          live speed supplier, or {@code null} for a fixed goal
     * @param label            log name; {@code null} or blank is replaced with a safe constant
     */
    public FlywheelGoal {
        primaryRPS = primaryRPS + 0.0;
        secondaryRPS = secondaryRPS + 0.0;

        if (!(idleToleranceRPS > 0.0) || !Double.isFinite(idleToleranceRPS)) {
            idleToleranceRPS = DEFAULT_IDLE_TOLERANCE_RPS;
        }

        if (label == null || label.isBlank()) {
            label = (tracked != null) ? "Track" : "Flywheel";
        }
    }

    /**
     * Fixed goal spinning both wheels at the same speed.
     *
     * <p>This is the ordinary shooter goal. Both wheels take the same setpoint, which on a
     * single-motor flywheel simply means the one wheel does.
     *
     * @param rps target speed in rotations per second
     * @return a fixed goal, safe to rebuild as often as you like
     */
    public static FlywheelGoal rps(double rps) {
        return rps(rps, rps);
    }

    /**
     * Fixed goal spinning the two wheels at different speeds to impart spin on the game piece.
     *
     * <p>Only meaningful on a flywheel configured with {@code secondMotor}. On a single-motor
     * mechanism {@code spinUp} delegates to the primary-only overload and the secondary value
     * has no effect, which the binding reports as a build-time problem rather than silently
     * shooting a different shot than the one that was asked for.
     *
     * @param primaryRPS   primary wheel speed in rotations per second
     * @param secondaryRPS secondary wheel speed in rotations per second
     * @return a fixed goal, safe to rebuild as often as you like
     */
    public static FlywheelGoal rps(double primaryRPS, double secondaryRPS) {
        return new FlywheelGoal(
                primaryRPS,
                secondaryRPS,
                DEFAULT_IDLE_TOLERANCE_RPS,
                null,
                labelFor(primaryRPS, secondaryRPS));
    }

    /**
     * Fixed goal in rotations per <em>minute</em>, converted once and stored as RPS.
     *
     * <p>Shot tables, dashboards and driver intuition all run in RPM while the mechanism runs in
     * RPS. Converting at the goal boundary means the factor of sixty appears exactly once in a
     * robot project, so the classic failure — one state declared in RPM against a mechanism
     * expecting RPS, producing a shot sixty times too fast that is written off as a PID problem —
     * cannot be written.
     *
     * @param rpm target speed in rotations per minute
     * @return a fixed goal holding {@code rpm / 60.0}
     */
    public static FlywheelGoal rpm(double rpm) {
        return rps(rpm / 60.0);
    }

    /**
     * Fixed dual-wheel goal in rotations per minute. See {@link #rpm(double)} for why the
     * conversion belongs here and {@link #rps(double, double)} for what the second wheel does.
     *
     * @param primaryRPM   primary wheel speed in rotations per minute
     * @param secondaryRPM secondary wheel speed in rotations per minute
     * @return a fixed goal holding both speeds divided by sixty
     */
    public static FlywheelGoal rpm(double primaryRPM, double secondaryRPM) {
        return rps(primaryRPM / 60.0, secondaryRPM / 60.0);
    }

    /**
     * Goal meaning "spun down", arrival-checked by speed rather than by
     * {@code FlywheelMechanism.atSpeed()}.
     *
     * <p>See the type javadoc for why a zero-RPS goal cannot use the mechanism's own at-speed
     * test. Uses {@link #DEFAULT_IDLE_TOLERANCE_RPS}; call {@link #idle(double)} when a heavier
     * wheel needs a wider band to stop hovering just outside it.
     *
     * @return the idle goal
     */
    public static FlywheelGoal idle() {
        return idle(DEFAULT_IDLE_TOLERANCE_RPS);
    }

    /**
     * Idle goal with an explicit tolerance, for a wheel whose measured velocity does not settle
     * inside the default band.
     *
     * <p>A tolerance that is zero, negative or non-finite is replaced with
     * {@link #DEFAULT_IDLE_TOLERANCE_RPS} rather than rejected, because an unreachable arrival
     * test costs the match and a substituted one costs nothing.
     *
     * @param idleToleranceRPS how close to zero counts as stopped, in rotations per second
     * @return the idle goal carrying that tolerance
     */
    public static FlywheelGoal idle(double idleToleranceRPS) {
        return new FlywheelGoal(0.0, 0.0, idleToleranceRPS, null, "Idle");
    }

    /**
     * Goal whose speed is re-read from {@code tracked} every loop — the shoot-on-the-fly case,
     * where required RPS is a function of range and is therefore not known when the state machine
     * is declared.
     *
     * <p><b>Call this once and keep the result.</b> The returned goal compares equal only to
     * itself, because {@code tracked} is compared by reference identity and a lambda is never
     * equal to a separately-created lambda computing the same thing. A goal rebuilt inside a
     * per-loop expression would look like a brand-new goal on every comparison and the engine
     * would restart the pursue command at 50 Hz, so the flywheel would re-initialise forever and
     * never reach speed. Holding one instance in a {@code static final} field makes the
     * comparison trivially true and the actuation stable.
     *
     * <p>{@code label} is mandatory for the complementary reason, described in full on the type:
     * a tracked goal has no fixed number to name itself after, and letting it name itself after
     * the supplier's live output would write a new log string every loop for the whole match.
     * Name the shot, not the number — {@code "SOTF"}, {@code "PodiumRange"}. A null or blank
     * label is repaired to {@code "Track"} instead of throwing, since a bad log name is not worth
     * a crash, but relying on that costs you the ability to tell your tracked goals apart in a
     * log.
     *
     * @param label   stable, low-cardinality name for this goal; must not interpolate a live value
     * @param tracked supplier of the target primary speed in rotations per second, read every loop
     * @return a tracked goal that must be stored and reused, not rebuilt
     */
    public static FlywheelGoal track(String label, DoubleSupplier tracked) {
        return new FlywheelGoal(0.0, 0.0, DEFAULT_IDLE_TOLERANCE_RPS, tracked, label);
    }

    /**
     * Is this the "spun down" goal, meaning arrival must be tested against
     * {@link #idleToleranceRPS()} rather than {@code FlywheelMechanism.atSpeed()}?
     *
     * <p>True when no supplier is attached and both wheel speeds are zero. A tracked goal is
     * never idle even while its supplier happens to be returning zero, because which arrival test
     * a binding runs must be a property of the goal — fixed for the life of the goal and the same
     * every loop — and not something that flickers with a live reading. A supplier that has
     * genuinely decided to stop the shooter still spins down through the tracking command, and
     * the state that means "stop" should be an {@link #idle()} state.
     *
     * @return {@code true} when this goal commands a stopped flywheel
     */
    public boolean isIdle() {
        return tracked == null && primaryRPS == 0.0 && secondaryRPS == 0.0;
    }

    /**
     * Builds the log label for a fixed goal, naming the setpoint rather than any live reading.
     *
     * <p>Interpolating these numbers is safe where interpolating a supplier's output is not: a
     * fixed goal's speeds never change, so the set of labels a robot can emit is bounded by the
     * number of declared states and the edge-detected log stays quiet.
     */
    private static String labelFor(double primaryRPS, double secondaryRPS) {
        if (primaryRPS == 0.0 && secondaryRPS == 0.0) {
            return "Idle";
        }
        if (primaryRPS == secondaryRPS) {
            return "Spin " + number(primaryRPS) + " rps";
        }
        return "Spin " + number(primaryRPS) + "/" + number(secondaryRPS) + " rps";
    }

    /**
     * Formats a setpoint compactly, dropping the decimal point when the value is whole, so that
     * a 50 RPS goal reads {@code "50"} rather than {@code "50.00"}. Deliberately avoids rounding
     * to whole numbers unconditionally: two goals half a rotation apart must not collapse into
     * the same label and become indistinguishable in a log.
     */
    private static String number(double value) {
        if (!Double.isFinite(value)) {
            return String.valueOf(value);
        }
        if (value == Math.rint(value) && Math.abs(value) < 1e15) {
            return Long.toString((long) value);
        }
        return String.format("%.2f", value);
    }
}
