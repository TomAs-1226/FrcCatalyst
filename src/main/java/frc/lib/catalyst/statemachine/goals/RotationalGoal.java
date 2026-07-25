package frc.lib.catalyst.statemachine.goals;

import java.util.Objects;

/**
 * A commanded angle for a {@link frc.lib.catalyst.mechanisms.RotationalMechanism} — an arm, a
 * wrist, a hood, or any other jointed mechanism that Motion Magic drives to a position in degrees.
 *
 * <p>A goal is either an <b>explicit angle</b> ({@link #degrees(double)}) or a <b>named preset</b>
 * ({@link #preset(String)}), never both. A preset carries only the name; the binding resolves it
 * against {@code RotationalMechanism.getNamedPositions()} at build time and again when it actuates.
 * Resolving late rather than at goal-construction time is deliberate: it means a state machine can
 * be declared before the mechanism exists, and it means a team that retunes {@code SCORE} from 100
 * degrees to 104 changes one number in their mechanism config instead of hunting for a copy of it
 * that got frozen into a superstructure state.
 *
 * <p>This is a {@code record}, so {@code equals}/{@code hashCode} are value-based, which is what the
 * engine relies on. {@link frc.lib.catalyst.statemachine.StateMachineCore} compares the wanted goal
 * against the active goal with {@link java.util.Objects#equals} every loop and rebuilds the pursue
 * command whenever they differ. An identity-equality goal type would therefore tear down and
 * re-initialise a Motion Magic command fifty times a second, which reads on a log as a mechanism
 * that never settles and on the field as a mechanism that stutters.
 *
 * <p>There is no {@code label} component here, unlike the supplier-bearing goals elsewhere in this
 * package. A rotational goal is a pair of constants, so a label derived from it is already
 * low-cardinality — nothing in it changes between loops. Only goals that close over a live
 * {@code DoubleSupplier} need an explicit label to keep an edge-detected log string from being
 * rewritten at 50 Hz.
 *
 * <h2>Sentinels</h2>
 * Two components use {@link Double#NaN} to mean "unspecified":
 * <ul>
 *   <li>{@link #degrees()} is {@code NaN} on a preset goal, because the angle is not known until the
 *       preset is resolved.</li>
 *   <li>{@link #toleranceDegrees()} is {@code NaN} when the caller did not name a tolerance, in
 *       which case the binding falls back to {@code RotationalMechanism.getTolerance()} — the same
 *       tolerance the mechanism's own {@code atPosition} and trigger factories use, so the state
 *       machine and a hand-written trigger agree about arrival.</li>
 * </ul>
 * {@code NaN} is safe as a sentinel inside a record: record equality compares {@code double}
 * components by their bit patterns, not with {@code ==}, so {@code NaN} equals {@code NaN} here and
 * two goals for the same preset compare equal as they must.
 *
 * @param degrees          target angle in degrees, or {@code NaN} when {@code preset} names the
 *                         target instead. Degrees are mechanism-frame, matching
 *                         {@code RotationalMechanism.getAngle()}.
 * @param toleranceDegrees half-width of the arrival band in degrees, or {@code NaN} to defer to the
 *                         mechanism's configured tolerance
 * @param preset           name of a position registered with {@code Config.Builder.position(...)}
 *                         or {@code addPositionsFromEnum(...)}, or {@code null} for an explicit
 *                         angle goal
 *
 * @since 1.2.0
 */
public record RotationalGoal(double degrees, double toleranceDegrees, String preset) {

    /**
     * Canonical constructor. It normalises rather than validates, because a bad angle is caught far
     * more usefully by {@code Binding.validate(...)}, which knows the mechanism's soft limits and
     * its preset table and can report every problem in the whole state machine in one exception on a
     * laptop. Throwing from here would instead surface as a stack trace during field construction,
     * long before anything has a chance to say which state the offending goal belongs to.
     *
     * <p>Three normalisations happen, all of them in service of value equality:
     * <ul>
     *   <li>A blank preset name becomes {@code null}, so {@code preset("")} degrades to a plainly
     *       invalid goal that {@code validate} will name, rather than to a lookup for a
     *       zero-length key.</li>
     *   <li>A preset goal has its {@code degrees} forced to {@code NaN}. Otherwise two goals that
     *       both mean {@code STOW} could carry different leftover angles, compare unequal, and make
     *       the engine rebuild the pursue command on every loop.</li>
     *   <li>A negative tolerance becomes {@code NaN}, i.e. "use the mechanism's tolerance". A
     *       negative half-width would make {@code Math.abs(error) < tolerance} unsatisfiable, and a
     *       mechanism that can never report arrival stalls the state machine at a transition
     *       forever. Zero is left alone; it is unreachable in practice but it is a legitimate thing
     *       to write in a test.</li>
     * </ul>
     * Signed zero is folded to positive zero for the same equality reason: {@code -0.0} and
     * {@code 0.0} have different bit patterns and so would be different goals despite naming the
     * same angle.
     */
    public RotationalGoal {
        if (preset != null && preset.isBlank()) {
            preset = null;
        }
        if (preset != null) {
            degrees = Double.NaN;
        } else if (degrees == 0.0) {
            degrees = 0.0; // collapses -0.0
        }
        if (toleranceDegrees < 0.0) {
            toleranceDegrees = Double.NaN;
        } else if (toleranceDegrees == 0.0) {
            toleranceDegrees = 0.0; // collapses -0.0
        }
    }

    /**
     * A goal at an explicit angle, arriving within the mechanism's configured tolerance.
     *
     * <p>Prefer this over {@link #degrees(double, double)} unless a particular state genuinely needs
     * a tighter or looser band than the rest of the mechanism. Deferring to the mechanism keeps one
     * number in one place; a tolerance copied into every state is a number that gets retuned in
     * three of them and forgotten in the fourth.
     *
     * @param degrees target angle in mechanism-frame degrees
     * @return the goal
     */
    public static RotationalGoal degrees(double degrees) {
        return new RotationalGoal(degrees, Double.NaN, null);
    }

    /**
     * A goal at an explicit angle with an explicit arrival band.
     *
     * <p>The usual reason to widen the band is a state whose next transition does not care about
     * precision — stowing before a drive across the field, say — where insisting on the scoring
     * tolerance would spend half a second of a match waiting out the last degree of settle. The
     * usual reason to tighten it is a hand-off where the piece is only captured if two mechanisms
     * are both close.
     *
     * @param degrees          target angle in mechanism-frame degrees
     * @param toleranceDegrees half-width of the arrival band in degrees; a negative value is treated
     *                         as "unspecified" and falls back to the mechanism's tolerance
     * @return the goal
     */
    public static RotationalGoal degrees(double degrees, double toleranceDegrees) {
        return new RotationalGoal(degrees, toleranceDegrees, null);
    }

    /**
     * A goal at a named position registered on the mechanism with
     * {@code Config.Builder.position(name, degrees)}.
     *
     * <p>The name is not resolved here and a misspelling is not rejected here. It is reported by
     * {@code Binding.validate(...)}, which can list the names the mechanism actually knows —
     * strictly more useful than an exception at the call site that only knows the name it was
     * handed.
     *
     * @param preset the position name, matched exactly and case-sensitively against the mechanism's
     *               named-position table
     * @return the goal
     */
    public static RotationalGoal preset(String preset) {
        return new RotationalGoal(Double.NaN, Double.NaN, preset);
    }

    /**
     * A goal at a named position identified by an enum constant, resolved through
     * {@link Enum#name()}.
     *
     * <p>This is the type-safe way to name a preset, and it lines up exactly with
     * {@code Config.Builder.addPositionsFromEnum(...)}, which registers each constant under its
     * {@code name()}. A team that declares its positions once as an enum therefore gets both the
     * mechanism's preset table and its state machine's goals from the same source, and a renamed
     * constant becomes a compile error instead of a preset lookup that silently fails on the field.
     *
     * <p>Any enum works, not only {@link frc.lib.catalyst.util.PositionEnum} implementations, since
     * only the constant's name is used. The angle still comes from the mechanism's table, so an
     * enum that carries its own target value and a mechanism that was configured from a different
     * table will disagree — register the enum on the mechanism and the two cannot drift apart.
     *
     * @param <E>      the enum type
     * @param position the constant naming the position; must not be {@code null}
     * @return the goal
     * @throws NullPointerException if {@code position} is {@code null}. This one does throw, because
     *                              a null constant is a programming error rather than a
     *                              mistuned value, and it is indistinguishable downstream from an
     *                              angle goal that was never given an angle.
     */
    public static <E extends Enum<E>> RotationalGoal preset(E position) {
        Objects.requireNonNull(position, "RotationalGoal.preset(enum): position must not be null");
        return preset(position.name());
    }

    /**
     * Does this goal name a preset rather than carry an angle? When {@code true}, {@link #degrees()}
     * is {@code NaN} and the binding must resolve {@link #preset()} before it can actuate or measure
     * error.
     *
     * @return {@code true} for a preset goal
     */
    public boolean isPreset() {
        return preset != null;
    }

    /**
     * Did the caller name an arrival band, or should the mechanism's own tolerance be used? Bindings
     * should route through {@link #toleranceOr(double)} instead of testing this directly; it exists
     * for telemetry that wants to say which of the two a log entry reflects.
     *
     * @return {@code true} when {@link #toleranceDegrees()} holds a usable number
     */
    public boolean hasExplicitTolerance() {
        return !Double.isNaN(toleranceDegrees);
    }

    /**
     * The arrival band to actually test against: this goal's tolerance when it named one, otherwise
     * {@code mechanismTolerance}.
     *
     * <p>Bindings should call this in both {@code atGoal} and {@code tolerance} so the number a log
     * reports is the same number arrival was decided with. Passing
     * {@code RotationalMechanism.getTolerance()} as the fallback is what makes the state machine
     * agree with {@code atPosition(...)} and with any trigger the team built by hand.
     *
     * @param mechanismTolerance the mechanism's configured tolerance in degrees
     * @return the effective tolerance in degrees
     */
    public double toleranceOr(double mechanismTolerance) {
        return Double.isNaN(toleranceDegrees) ? mechanismTolerance : toleranceDegrees;
    }
}
