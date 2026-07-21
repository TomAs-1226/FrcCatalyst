package frc.lib.catalyst.statemachine.goals;

/**
 * A commanded pose for a two-axis differential wrist: a pitch angle, a roll angle, and the
 * arrival band that both axes must fall inside before the state machine calls the pose reached.
 *
 * <p>This is a {@code record}, so it has value-based {@code equals} and {@code hashCode} for
 * free. That matters more than it looks: the engine compares the wanted goal against the active
 * goal with {@link java.util.Objects#equals} on every loop to decide whether actuation needs
 * re-applying. A goal type with identity equality would compare unequal to itself and cause the
 * binding to tear down and rebuild its pursue command fifty times a second, which on a
 * differential wrist means fifty fresh Motion Magic requests per second and a mechanism that
 * shudders instead of moving.
 *
 * <p><b>Why the tolerance travels with the goal rather than being read off the mechanism.</b>
 * {@code DifferentialWristMechanism} keeps its arrival band in a package-private
 * {@code Config.toleranceDegrees} field with no public accessor, so code outside the mechanisms
 * package cannot read it at all. More importantly, the mechanism's own
 * {@code atSetpoint()} is not usable as an arrival test here: it compares the live axes against
 * {@code pitchSetpointDegrees}/{@code rollSetpointDegrees}, which {@code applyTargets} has
 * already <em>clamped</em> into the configured pitch and roll ranges. Ask for a roll of 200
 * degrees on a wrist limited to 180 and the mechanism will clamp to 180, arrive there, and
 * report {@code atSetpoint() == true} while sitting twenty degrees off what was actually
 * requested. A binding built on this goal must therefore test the raw axes against the values
 * held here — {@code Math.abs(getPitch() - pitchDegrees()) < toleranceDegrees()} and the same
 * for roll — which requires the goal to carry its own band.
 *
 * <p>Every component is a plain {@code double} in degrees. There is deliberately no
 * {@code Rotation2d} or other geometry type: wrist angles here are unwrapped mechanism angles
 * that may legitimately exceed a single revolution or run negative, and a wrapping angle type
 * would quietly fold a {@code -190} degree roll into {@code +170} and drive the wrist the long
 * way around through its hard stop.
 *
 * @param pitchDegrees      target pitch angle in degrees, in the mechanism's own unwrapped frame
 * @param rollDegrees       target roll angle in degrees, in the mechanism's own unwrapped frame
 * @param toleranceDegrees  half-width of the arrival band applied to both axes, in degrees;
 *                          always strictly positive and finite once the goal is constructed
 *
 * @since 1.2.0
 */
public record WristGoal(double pitchDegrees, double rollDegrees, double toleranceDegrees) {

    /**
     * Arrival band used when a caller does not name one, in degrees.
     *
     * <p>Two degrees is a deliberately forgiving default. A differential wrist reads its two
     * axes through the sum and difference of two separate rotor encoders, so encoder noise on
     * either motor lands on <em>both</em> derived axes; a band tight enough for a single-motor
     * pivot will chatter in and out of arrival on a diffy wrist and make the state machine
     * oscillate between "arrived" and "still moving".
     */
    public static final double DEFAULT_TOLERANCE_DEGREES = 2.0;

    /**
     * Canonical constructor, which normalises the tolerance and leaves the angles untouched.
     *
     * <p>A tolerance that is zero, negative, or not a number describes a band no real mechanism
     * can ever satisfy, and the failure it produces is one of the nastiest kinds to diagnose on
     * a field: the wrist physically arrives, sits still, and the state machine waits on it
     * forever with no error and nothing obviously wrong in the logs. Rather than propagate that,
     * any such value is replaced by {@link #DEFAULT_TOLERANCE_DEGREES}.
     *
     * <p>The pitch and roll angles are <b>not</b> validated here, and this constructor never
     * throws for them. Whether a given angle is reachable depends on the pitch and roll ranges
     * configured on the specific mechanism this goal will be attached to, which the goal knows
     * nothing about. That check belongs in the binding's
     * {@link frc.lib.catalyst.statemachine.Binding#validate} against
     * {@code getMinPitch()}/{@code getMaxPitch()} and {@code getMinRoll()}/{@code getMaxRoll()},
     * where it runs at build time on a laptop, names the offending state, and is reported
     * alongside every other configuration problem at once instead of throwing on the first one
     * during a deploy.
     */
    public WristGoal {
        if (!(toleranceDegrees > 0.0) || Double.isInfinite(toleranceDegrees)) {
            toleranceDegrees = DEFAULT_TOLERANCE_DEGREES;
        }
    }

    /**
     * A wrist pose at the given pitch and roll, using {@link #DEFAULT_TOLERANCE_DEGREES}.
     *
     * <p>This is the factory to reach for in ordinary superstructure definitions. Prefer it over
     * the three-argument form unless a particular pose genuinely needs a different band, because
     * a per-goal tolerance is a number that has to be re-tuned by hand every time the wrist's
     * gearing or gains change.
     *
     * @param pitchDegrees target pitch angle in degrees
     * @param rollDegrees  target roll angle in degrees
     * @return the goal
     */
    public static WristGoal of(double pitchDegrees, double rollDegrees) {
        return new WristGoal(pitchDegrees, rollDegrees, DEFAULT_TOLERANCE_DEGREES);
    }

    /**
     * A wrist pose at the given pitch and roll with an explicit arrival band.
     *
     * <p>Worth using for the handful of poses where the default band is the wrong call in either
     * direction: a scoring pose that has to be precise enough for the game piece to seat wants a
     * tighter band, and a stow or travel pose that only needs to be roughly right wants a looser
     * one so the machine does not stall waiting on a degree that does not matter.
     *
     * <p>A non-positive or non-finite tolerance is quietly replaced by
     * {@link #DEFAULT_TOLERANCE_DEGREES}; see the canonical constructor for why that is a
     * substitution rather than an exception.
     *
     * @param pitchDegrees     target pitch angle in degrees
     * @param rollDegrees      target roll angle in degrees
     * @param toleranceDegrees half-width of the arrival band applied to both axes, in degrees
     * @return the goal
     */
    public static WristGoal of(double pitchDegrees, double rollDegrees, double toleranceDegrees) {
        return new WristGoal(pitchDegrees, rollDegrees, toleranceDegrees);
    }

    /**
     * The neutral pose: zero pitch, zero roll, default tolerance.
     *
     * <p>Zero on both axes is whatever pose the wrist was sitting in when {@code zero()} seeded
     * the encoders, so this is only "level" in the sense that it is the mechanism's own origin.
     * On a wrist that has not been zeroed it means nothing at all, which is exactly why a
     * binding for this goal should report {@code zeroed()} honestly — a state machine that
     * drives to this goal on an unzeroed wrist will happily slew the mechanism into a hard stop.
     *
     * @return the goal at the mechanism's zero pose
     */
    public static WristGoal level() {
        return of(0.0, 0.0);
    }
}
