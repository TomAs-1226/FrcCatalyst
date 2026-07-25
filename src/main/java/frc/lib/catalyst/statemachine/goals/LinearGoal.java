package frc.lib.catalyst.statemachine.goals;

/**
 * A commanded position for a {@link frc.lib.catalyst.mechanisms.LinearMechanism} — an elevator, a
 * telescoping arm, a linear slide.
 *
 * <p>This is a {@code record} rather than a class for one specific reason. The state machine engine
 * calls {@link java.util.Objects#equals} on the wanted goal against the currently applied goal every
 * loop to decide whether actuation needs to be re-applied. A goal type with identity equality would
 * compare unequal to itself on every comparison, so the engine would tear down and rebuild the pursue
 * command fifty times a second — the mechanism would re-initialise Motion Magic continuously and never
 * visibly move. Records give value-based {@code equals}/{@code hashCode} over the three components
 * below, which is exactly what the engine needs and what a hand-written class usually gets wrong.
 *
 * <h2>Two ways to name a position</h2>
 *
 * <p>A goal either names a position numerically or by preset:
 *
 * <ul>
 *   <li><b>Numeric</b> — {@link #meters(double)}. {@code preset} is {@code null} and {@code meters}
 *       holds the target immediately. Nothing needs resolving.</li>
 *   <li><b>Preset</b> — {@link #preset(String)}. {@code preset} holds a key from the mechanism's
 *       {@code namedPositions} map (the strings passed to {@code Config.Builder.position(name, meters)}),
 *       and <b>{@code meters} is {@code NaN} until the binding resolves it</b>.</li>
 * </ul>
 *
 * <p>The preset form exists because robot code should read {@code LinearGoal.preset("L4")}, not
 * {@code LinearGoal.meters(1.37)}. A magic number copied into six state definitions is six places to
 * forget when the field element gets re-measured; the preset form keeps the single source of truth in
 * the mechanism's {@code Config}.
 *
 * <h2>Why {@code meters} may legitimately be {@code NaN}</h2>
 *
 * <p>A preset goal cannot resolve itself. It has no reference to the mechanism, and it must not have
 * one — goals are plain values constructed in state tables, often before the subsystem they will
 * eventually drive has finished constructing. Resolution is therefore the binding's job, and it happens
 * at <em>build time</em>, inside {@link frc.lib.catalyst.statemachine.Binding#validate}. That method
 * looks the preset up in {@code LinearMechanism.getNamedPositions()} and reports an unknown key as a
 * build problem, so every misspelled preset in the whole superstructure is listed in a single
 * {@code StateMachineConfigException} on a laptop, rather than surfacing one at a time as a crash on
 * the field.
 *
 * <p>The consequence for callers is that <b>{@code meters()} must not be read directly on a goal whose
 * {@link #isPreset()} is true</b>. Read it through the binding, which knows the resolved value. Code
 * that reads it anyway gets {@code NaN}, and every comparison against {@code NaN} is false — an
 * elevator that silently never reports arrival. {@link #resolve(double, double)} exists so a binding
 * can cache a fully-resolved copy at build time and avoid the map lookup on every loop.
 *
 * <p>{@code NaN} is safe to carry as a sentinel here even though the engine compares goals for
 * equality, because record equality on a {@code double} component is defined in terms of
 * {@link Double#compare}, not {@code ==}. Under {@code Double.compare}, {@code NaN} equals
 * {@code NaN}. An unresolved preset goal therefore compares equal to itself and does not trigger the
 * rebuild-every-loop failure described above.
 *
 * <h2>This type never throws</h2>
 *
 * <p>The compact constructor normalises nonsense values but rejects nothing. An out-of-range setpoint,
 * an unknown preset, or a goal that names neither a position nor a preset is caught by
 * {@code validate()}, which can say <em>which state</em>, <em>which mechanism</em> and <em>what the
 * legal range was</em>. A constructor throwing {@code IllegalArgumentException} would abort the build
 * at the first bad goal with none of that context, and would hide every other bad goal behind it.
 *
 * @param meters          target position in meters, or {@code NaN} when this is a preset goal that has
 *                        not been resolved yet. Positions are measured in the mechanism's own frame,
 *                        the same one {@code LinearMechanism.getPosition()} reports.
 * @param toleranceMeters half-width of the arrival band in meters, or {@code NaN} to defer to the
 *                        mechanism's configured {@code getPositionTolerance()}. Deferring is the normal
 *                        case and is preferred: it keeps one tolerance per mechanism instead of a
 *                        different one per state. Values at or below zero are normalised to {@code NaN}
 *                        — see {@link #hasExplicitTolerance()}.
 * @param preset          key into the mechanism's named-position map, or {@code null} for a numeric
 *                        goal. Never blank; a blank string is normalised to {@code null}.
 *
 * @since 1.2.0
 */
public record LinearGoal(double meters, double toleranceMeters, String preset) {

    /**
     * Canonical constructor. Normalises the three components into the single representation the rest
     * of the package expects, so that two goals a human would call "the same goal" really do compare
     * equal.
     *
     * <p>Three normalisations happen here, each preventing a specific failure:
     *
     * <ol>
     *   <li>Negative zero collapses to positive zero. Record equality uses {@link Double#compare},
     *       under which {@code -0.0} and {@code 0.0} are <em>not</em> equal. A stow position computed
     *       as {@code -1 * 0.0} would therefore never compare equal to a literal {@code 0.0} stow goal,
     *       and the engine would rebuild the pursue command every loop for a mechanism that is already
     *       exactly where it was asked to be. This is the hardest bug in this file to find from a log,
     *       and one line here removes it.</li>
     *   <li>A tolerance at or below zero becomes {@code NaN}, meaning "use the mechanism's configured
     *       tolerance". Zero is not a stricter tolerance, it is an impossible one:
     *       {@code LinearMechanism.atPosition} tests {@code abs(error) < tolerance} with a strict
     *       comparison, so a zero band can never be satisfied and the state would hang forever waiting
     *       on arrival. Falling back to the configured default is what the caller meant.</li>
     *   <li>A preset is trimmed, and a preset that is blank becomes {@code null}. Named positions are
     *       registered from string literals, so a key with stray whitespace is always a typo rather
     *       than an intentionally distinct key, and it would otherwise fail the map lookup with a
     *       message in which the two strings look identical. Nulling a blank preset lets
     *       {@code validate()} report the clearer "goal names neither a position nor a preset".</li>
     * </ol>
     *
     * <p>Note that {@code NaN} passes through all three checks untouched: every comparison against
     * {@code NaN} is false, so neither the zero collapse nor the tolerance floor fires on it. That is
     * deliberate — {@code NaN} is a meaningful value for both {@code double} components here.
     */
    public LinearGoal {
        // -0.0 == 0.0 is true, so this assignment fires only for negative zero and replaces it with
        // positive zero. NaN == 0.0 is false, so an unresolved preset goal is left alone.
        if (meters == 0.0) {
            meters = 0.0;
        }

        // NaN <= 0.0 is false, so an already-deferred tolerance stays deferred rather than being
        // re-flagged here.
        if (toleranceMeters <= 0.0) {
            toleranceMeters = Double.NaN;
        }

        if (preset != null) {
            preset = preset.trim();
            if (preset.isEmpty()) {
                preset = null;
            }
        }
    }

    /**
     * A goal to hold a numeric position, using whatever position tolerance the mechanism was
     * configured with.
     *
     * <p>This is the right factory for a position that genuinely is a number in the caller's head — a
     * jog target, a computed handoff height, a position derived from field geometry. Prefer
     * {@link #preset(String)} for positions the mechanism already has a name for, so the number lives
     * in exactly one place.
     *
     * <p>The value is not range-checked here. A target outside the mechanism's configured travel is
     * reported by {@code validate()} at build time, which can name the state and print the legal range;
     * if one somehow reaches the hardware anyway, {@code LinearMechanism.goTo} clamps it to the soft
     * limits, so the mechanism drives to the end of travel rather than into the frame.
     *
     * @param meters target position in meters, in the same frame {@code LinearMechanism.getPosition()}
     *               reports
     * @return a numeric goal that defers its arrival tolerance to the mechanism
     */
    public static LinearGoal meters(double meters) {
        return new LinearGoal(meters, Double.NaN, null);
    }

    /**
     * A goal to hold a numeric position with an arrival tolerance specific to this goal.
     *
     * <p>Reach for this only when one position genuinely needs a different band than the rest of the
     * mechanism — a scoring height that must be tight, or a travel position that only needs to be
     * roughly right and should not stall a transition over a centimetre. Every per-goal tolerance is
     * one more number that has to be re-tuned when the mechanism's gearing changes, so the default of
     * deferring to {@code getPositionTolerance()} is usually the better engineering choice.
     *
     * <p>A tolerance at or below zero is treated as "no explicit tolerance" rather than as an error;
     * see the canonical constructor for why a zero band would otherwise hang the state forever.
     *
     * @param meters          target position in meters
     * @param toleranceMeters half-width of the arrival band in meters; values at or below zero fall
     *                        back to the mechanism's configured tolerance
     * @return a numeric goal carrying its own arrival tolerance
     */
    public static LinearGoal meters(double meters, double toleranceMeters) {
        return new LinearGoal(meters, toleranceMeters, null);
    }

    /**
     * A goal naming one of the mechanism's configured named positions.
     *
     * <p>This is the preferred way to write a goal. {@code LinearGoal.preset("L4")} keeps the actual
     * height in the mechanism's {@code Config}, where re-measuring the field element changes one
     * number instead of every state that mentions it.
     *
     * <p>The returned goal has {@code meters == NaN}. It is <b>not usable until the binding resolves
     * it</b> against {@code LinearMechanism.getNamedPositions()} during {@code validate()}, which is
     * also where an unknown key is reported as a build problem alongside the list of keys that do
     * exist. This factory deliberately does not verify the key itself: it has no mechanism to ask, and
     * even if it did, throwing here would abort the build at the first typo instead of listing all of
     * them at once.
     *
     * @param preset key registered with {@code Config.Builder.position(name, meters)}; trimmed, and
     *               treated as absent if blank
     * @return an unresolved preset goal that defers both its position and its tolerance to the
     *         mechanism
     */
    public static LinearGoal preset(String preset) {
        return new LinearGoal(Double.NaN, Double.NaN, preset);
    }

    /**
     * A goal naming one of the mechanism's configured named positions, with an arrival tolerance
     * specific to this goal.
     *
     * <p>The position still resolves from the mechanism's named-position map at build time; only the
     * tolerance is overridden. This is the combination to use when a preset is shared across several
     * states but one of them needs to be fussier about arrival than the mechanism's default — a
     * scoring pose that must settle before the claw opens, say, sharing a preset with a staging pose
     * that does not.
     *
     * @param preset          key registered with {@code Config.Builder.position(name, meters)}
     * @param toleranceMeters half-width of the arrival band in meters; values at or below zero fall
     *                        back to the mechanism's configured tolerance
     * @return an unresolved preset goal carrying its own arrival tolerance
     */
    public static LinearGoal preset(String preset, double toleranceMeters) {
        return new LinearGoal(Double.NaN, toleranceMeters, preset);
    }

    /**
     * Does this goal name its position by preset rather than numerically?
     *
     * <p>Callers should test this before reading {@link #meters()}. On a preset goal that has not been
     * through {@link #resolve(double, double)}, {@code meters()} is {@code NaN} and every arithmetic
     * comparison against it is false, which shows up as a mechanism that never reports arrival rather
     * than as an exception.
     *
     * @return {@code true} when {@link #preset()} is non-null
     */
    public boolean isPreset() {
        return preset != null;
    }

    /**
     * Does this goal carry its own arrival tolerance, or should the mechanism's configured tolerance
     * be used?
     *
     * <p>This exists so that no caller writes {@code goal.toleranceMeters() == someSentinel}. The
     * sentinel is {@code NaN}, and {@code NaN} compares unequal to everything including itself, so the
     * obvious equality test is silently always false. Ask through this method instead.
     *
     * @return {@code true} when an explicit, usable tolerance is present
     */
    public boolean hasExplicitTolerance() {
        return !Double.isNaN(toleranceMeters);
    }

    /**
     * Is this goal fully resolved — that is, does it carry a usable numeric position?
     *
     * <p>A numeric goal is resolved from construction. A preset goal becomes resolved only once a
     * binding has looked its key up and produced a copy via {@link #resolve(double, double)}.
     *
     * @return {@code true} when {@link #meters()} holds a real number
     */
    public boolean isResolved() {
        return !Double.isNaN(meters);
    }

    /**
     * Produce a fully-resolved copy of this goal, keeping the preset key for labelling.
     *
     * <p>A binding calls this at build time, from inside {@code validate()}, once it has looked the
     * preset up in {@code LinearMechanism.getNamedPositions()} and read
     * {@code LinearMechanism.getPositionTolerance()}. Caching the resolved copy keeps the map lookup
     * out of {@code atGoal}, which runs every loop for every binding and must stay cheap and
     * allocation-free.
     *
     * <p>The preset key is deliberately preserved. It is what {@code label(goal)} should publish: it is
     * stable, low-cardinality and human-meaningful in a log, where a resolved {@code 1.37} tells a
     * student nothing about which scoring position failed.
     *
     * <p>Note that the resolved copy does <b>not</b> equal the unresolved original, since its
     * {@code meters} differs. That is correct and intentional: a binding must resolve once, store the
     * result, and compare against the stored copy consistently. Mixing resolved and unresolved goals
     * for the same position would make the engine see them as two different goals and re-apply
     * actuation every loop.
     *
     * @param meters          the resolved target position in meters
     * @param toleranceMeters the resolved arrival tolerance in meters; values at or below zero are
     *                        normalised back to "defer to the mechanism" by the canonical constructor
     * @return a new goal with the same preset key and the supplied numeric values
     */
    public LinearGoal resolve(double meters, double toleranceMeters) {
        return new LinearGoal(meters, toleranceMeters, preset);
    }
}
