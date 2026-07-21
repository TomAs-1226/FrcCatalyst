package frc.lib.catalyst.statemachine.mech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.catalyst.mechanisms.RollerMechanism;
import frc.lib.catalyst.statemachine.goals.RollerGoal;
import frc.lib.catalyst.statemachine.robot.Actuator;

import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;

/**
 * Drives a {@link RollerMechanism} from a {@link RollerGoal}.
 *
 * <p>A roller is the least observable mechanism in the library. Nothing about it is positional,
 * there is no encoder reading that means "the eject finished", and the one genuine sensor it may
 * carry — a beam break, or a stall-current latch standing in for one — answers exactly one
 * question: is a game piece aboard. Every goal that is not asking that question therefore arrives
 * on a settle timer, and this binding reports {@link #observable(RollerGoal)} {@code == false} for
 * all of them so that nobody reading a log later mistakes a dwell for a measurement.
 *
 * <p>Three properties of the underlying mechanism shape the code below, and each one is worth
 * knowing before changing anything here:
 *
 * <ul>
 *   <li>{@code RollerMechanism.hasPiece()} early-returns on the beam break whenever one is
 *       configured. The stall-current latch is not consulted at all on those robots — it is
 *       unreachable dead code. A team that configures both has configured one, and this binding
 *       says so in {@link #note(RollerGoal)} rather than letting somebody spend a practice day
 *       tuning a stall threshold that can never fire.</li>
 *   <li>{@code RollerMechanism.intake()} calls {@code updateStallDetection()} in its run body;
 *       {@code intakeContinuous()} does not. Stall detection therefore only ever latches while an
 *       {@link RollerGoal.IntakeUntilPiece} goal is the one being pursued, which is why that goal
 *       maps to {@code intake()} and never to the continuous variant.</li>
 *   <li>{@code eject()}, {@code runAtSpeed(double)} and {@code feedVoltage(double)} never end.
 *       Left hosted, they keep driving the rollers for as long as the state is held, which on a
 *       compliant-wheel intake is how a motor gets cooked between matches. All three consequently
 *       supply a {@link #holdCommand(RollerGoal)} that stops the motor the moment arrival is
 *       declared.</li>
 * </ul>
 *
 * <p><b>Known limitation.</b> The specification for this binding asked it to reject, at build
 * time, an {@link RollerGoal.IntakeUntilPiece} goal aimed at a roller that has neither a beam
 * break nor stall detection configured — a combination under which {@code hasPiece()} can never
 * become true and the goal can only ever expire on its timeout. That check cannot be performed by
 * inspection: {@code RollerMechanism.Config} keeps {@code beamBreakPort} and
 * {@code stallCurrentThreshold} package-private, the {@code config} field itself is private, and
 * no accessor for either exists. From outside {@code frc.lib.catalyst.mechanisms} the two
 * settings are simply invisible, and {@code hasPiece()} returning {@code false} at build time is
 * indistinguishable between "no detection configured" and "detection configured, intake empty".
 * The check is therefore opt-in: construct the binding through
 * {@link #RollerBinding(RollerMechanism, String, PieceDetection)} to declare what the mechanism
 * actually has, and the hard build-time error comes back. Constructed without that declaration,
 * the binding degrades to a warning carried in {@link #note(RollerGoal)}, which is visible in
 * {@code BlockerDetail} at the moment the goal is actually stuck.
 *
 * @since 1.2.0
 */
public final class RollerBinding implements Actuator<RollerGoal> {

    /**
     * Widest duty cycle a roller goal may command. Values beyond this are a configuration error
     * rather than something to silently clamp, because a duty cycle of 4.0 that quietly becomes
     * 1.0 looks like it worked and hides whichever unit mix-up produced it.
     */
    private static final double MAX_DUTY_CYCLE = 1.0;

    /**
     * Widest feed voltage a roller goal may command. Twelve volts is the nominal battery, so
     * anything past it is unreachable by definition and almost always signals a duty cycle that
     * was passed to {@link RollerGoal#feed(double, double)} by mistake.
     */
    private static final double MAX_FEED_VOLTS = 12.0;

    /**
     * Duty cycle below which {@link RollerGoal.Idle} is considered to have visibly taken effect.
     * Used only for {@link #detail(RollerGoal)} prose; idle arrival itself is unconditional.
     */
    private static final double IDLE_DUTY_EPSILON = 0.02;

    /**
     * What the wrapped mechanism is known to have in the way of game-piece detection.
     *
     * <p>This exists only because the mechanism will not say. It is a declaration by the team
     * assembling the superstructure, not a measurement, and it is used for exactly one purpose:
     * deciding whether an {@link RollerGoal.IntakeUntilPiece} goal is structurally impossible to
     * satisfy and should fail the build.
     */
    public enum PieceDetection {

        /**
         * Nothing was declared. The binding assumes detection may be present, validates
         * accordingly, and carries the caveat in {@link RollerBinding#note(RollerGoal)} instead.
         * This is the default, because guessing wrong in the other direction would fail builds on
         * perfectly healthy robots.
         */
        UNDECLARED,

        /**
         * The mechanism was built with a beam break, a stall threshold, or both.
         * {@link RollerGoal.IntakeUntilPiece} can genuinely arrive on its sensor.
         */
        CONFIGURED,

        /**
         * The mechanism was built with neither. {@code hasPiece()} is hard-wired {@code false},
         * so {@link RollerGoal.IntakeUntilPiece} can only ever expire on its timeout, and
         * {@link RollerBinding#validate(RollerGoal, Consumer)} reports that as a build error.
         */
        ABSENT
    }

    /** The mechanism this binding drives. Never {@code null}. */
    private final RollerMechanism mechanism;

    /** Stable telemetry key; becomes {@code Bindings/<key>/...}. */
    private final String key;

    /** What the assembling team declared about this roller's detection hardware. */
    private final PieceDetection detection;

    /**
     * Output values resolved once at build time, keyed by the goal that produced them.
     *
     * <p>{@link RollerGoal.Speed} and {@link RollerGoal.FeedVolts} deliberately do not clamp their
     * own components — an out-of-range value is meant to be reported by {@link #validate} with the
     * offending state's name attached rather than silently corrected. Once validate has reported
     * it, though, the number still has to be handed to a motor, and handing a motor a {@code NaN}
     * duty cycle is worse than handing it a clamped one. So validate stores the sanitised value
     * here and the pursue path reads it back.
     *
     * <p>Records key this map correctly because they have value-based {@code hashCode}. The map is
     * written only from {@code validate}, which runs on the build thread before any command is
     * ever pursued, and is read-only afterwards — the pursue path never inserts, so a goal that
     * was never validated falls back to sanitising inline rather than mutating shared state from
     * the scheduler thread.
     */
    private final Map<RollerGoal, Double> resolvedOutput = new ConcurrentHashMap<>();

    /**
     * Creates a binding with no declaration about the roller's detection hardware.
     *
     * <p>This is the ordinary constructor. Because {@code RollerMechanism} does not expose whether
     * a beam break or a stall threshold was configured, a binding built this way cannot fail the
     * build on an unsatisfiable {@link RollerGoal.IntakeUntilPiece} goal and warns in
     * {@link #note(RollerGoal)} instead.
     *
     * @param mechanism the roller to drive; must not be {@code null}
     * @param key       stable, log-safe, unique key for telemetry; must not be {@code null}
     */
    public RollerBinding(RollerMechanism mechanism, String key) {
        this(mechanism, key, PieceDetection.UNDECLARED);
    }

    /**
     * Creates a binding that knows what detection hardware the roller carries.
     *
     * <p>Pass {@link PieceDetection#ABSENT} for a roller built without a beam break and without a
     * stall threshold, and the build will reject any {@link RollerGoal.IntakeUntilPiece} goal
     * aimed at it — which is the whole point, since such a goal can only ever time out. Pass
     * {@link PieceDetection#CONFIGURED} to suppress the corresponding runtime caveat in
     * {@link #note(RollerGoal)}.
     *
     * @param mechanism the roller to drive; must not be {@code null}
     * @param key       stable, log-safe, unique key for telemetry; must not be {@code null}
     * @param detection what the mechanism is known to carry; must not be {@code null}
     */
    public RollerBinding(RollerMechanism mechanism, String key, PieceDetection detection) {
        this.mechanism = Objects.requireNonNull(mechanism, "mechanism");
        this.key = Objects.requireNonNull(key, "key");
        this.detection = Objects.requireNonNull(detection, "detection");
    }

    // =========================================================================
    //                              IDENTITY
    // =========================================================================

    /** {@inheritDoc} */
    @Override
    public String key() {
        return key;
    }

    /** {@inheritDoc} */
    @Override
    public String kind() {
        return "roller";
    }

    /**
     * {@inheritDoc}
     *
     * <p>{@code "frac"}, matching what {@code RollerMechanism.describe()} publishes.
     * {@link #measured()} is {@code getSpeed()}, which is applied voltage divided by twelve — a
     * duty-cycle fraction, not a surface speed and not an RPM. Labelling it anything else would
     * invite a dashboard to draw it against a velocity axis.
     */
    @Override
    public String unit() {
        return "frac";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Applied duty cycle in {@code [-1, 1]}, positive in the configured intake direction. Note
     * that this is a commanded-output readback derived from applied voltage, not a measurement of
     * whether the wheels are actually turning — a fully stalled roller still reports its commanded
     * fraction.
     */
    @Override
    public double measured() {
        return mechanism.getSpeed();
    }

    /**
     * {@inheritDoc}
     *
     * <p>A roller has no zeroing concept: there is no home position, no absolute reference, and
     * nothing that could be un-homed. Always {@code true}, so a roller never gates a state with
     * {@code NOT_ZEROED}.
     */
    @Override
    public boolean zeroed() {
        return true;
    }

    /** {@inheritDoc} */
    @Override
    public Set<Subsystem> requirements() {
        return Set.of(mechanism);
    }

    // =========================================================================
    //                              ACTUATION
    // =========================================================================

    /**
     * {@inheritDoc}
     *
     * <p>Every branch calls a {@code RollerMechanism} command factory, and every one of those
     * factories builds a new decorated command each call, so the fresh-instance requirement holds
     * without any caching on this side.
     *
     * <p>{@link RollerGoal.IntakeUntilPiece} maps to {@code intake()} rather than
     * {@code intakeContinuous()} for a reason that is invisible from the method names:
     * {@code intake()} runs {@code updateStallDetection()} in its body and
     * {@code intakeContinuous()} does not. Substituting the continuous variant here would silently
     * disable stall-based pickup on every robot that relies on it.
     *
     * <p>This method never throws. An exception escaping here would propagate out of
     * {@code CommandScheduler.run()} and take the robot loop with it, so an unrecognised or
     * {@code null} goal degrades to stopping the rollers — the one behaviour that is safe under
     * every circumstance.
     */
    @Override
    public Command pursueCommand(RollerGoal goal) {
        if (goal instanceof RollerGoal.IntakeUntilPiece) {
            return mechanism.intake();
        }
        if (goal instanceof RollerGoal.IntakeContinuous) {
            return mechanism.intakeContinuous();
        }
        if (goal instanceof RollerGoal.Eject) {
            return mechanism.eject();
        }
        if (goal instanceof RollerGoal.Speed) {
            RollerGoal.Speed speed = (RollerGoal.Speed) goal;
            return mechanism.runAtSpeed(resolvedOr(speed, sanitise(speed.dutyCycle(), MAX_DUTY_CYCLE)));
        }
        if (goal instanceof RollerGoal.FeedVolts) {
            RollerGoal.FeedVolts feed = (RollerGoal.FeedVolts) goal;
            return mechanism.feedVoltage(resolvedOr(feed, sanitise(feed.volts(), MAX_FEED_VOLTS)));
        }
        return mechanism.stopCommand();
    }

    /**
     * {@inheritDoc}
     *
     * <p>The three open-loop goals — eject, duty cycle, feed voltage — stop the motor on arrival.
     * Their pursue commands never end on their own, and a roller that keeps being driven after the
     * state machine considers it finished is both a waste of current and, on an intake pressed
     * against a hard stop, a thermal problem.
     *
     * <p>{@link RollerGoal.IntakeUntilPiece} also stops on arrival — and this covers a case the
     * {@code intake()} command's own {@code finallyDo} does not. That command ends and stops the
     * motor only on the {@code hasPiece()} path; when the goal instead "arrives" by hitting its
     * {@code maxSeconds} timeout, {@code hasPiece()} is still false, the {@code intake()} command is
     * still running, and without a hold command the runner would keep it running and the intake would
     * keep spinning against nothing. Returning a stop here ends that. On the piece-detected path the
     * pursue command has already stopped the motor, so the hold is a harmless re-stop.
     *
     * <p>Everything else returns {@code null}, which keeps the pursue command hosted. That is correct
     * for {@link RollerGoal.Idle} (whose pursue command is already a stop) and
     * {@link RollerGoal.IntakeContinuous} (which is explicitly meant to keep running until the state
     * is left).
     */
    @Override
    public Command holdCommand(RollerGoal goal) {
        if (goal instanceof RollerGoal.Eject
                || goal instanceof RollerGoal.Speed
                || goal instanceof RollerGoal.FeedVolts
                || goal instanceof RollerGoal.IntakeUntilPiece) {
            return mechanism.stopCommand();
        }
        return null;
    }

    // =========================================================================
    //                               ARRIVAL
    // =========================================================================

    /**
     * {@inheritDoc}
     *
     * <p>Pure in the required sense: it reads {@code hasPiece()}, the goal, and
     * {@code secondsSinceApplied}, and nothing else. No field records what was last applied, which
     * is what lets {@code isAt} interrogate this binding about goals belonging to states the
     * machine is not in.
     *
     * <p>Per goal:
     * <ul>
     *   <li>{@link RollerGoal.IntakeUntilPiece} arrives when {@code hasPiece()} is true, or when
     *       its timeout expires. The timeout half is not decoration — on a roller whose detection
     *       is unconfigured or miswired, {@code hasPiece()} is permanently false, and without the
     *       expiry the state machine would sit in the intaking state for the rest of the match.
     *       {@link #note(RollerGoal)} distinguishes the two outcomes for whoever reads the log.</li>
     *   <li>{@link RollerGoal.Eject}, {@link RollerGoal.Speed} and {@link RollerGoal.FeedVolts}
     *       arrive on their own dwell. Nothing observes a piece leaving or a roller reaching a
     *       commanded fraction, so these are timers and say so through {@link #observable}.</li>
     *   <li>{@link RollerGoal.Idle} and {@link RollerGoal.IntakeContinuous} arrive immediately.
     *       Both are assertions about what the roller should be doing rather than states it has to
     *       reach, and neither has anything to wait for.</li>
     * </ul>
     *
     * <p>Like {@link #pursueCommand}, this never throws; an unrecognised or {@code null} goal
     * reports arrival rather than wedging the machine in a state it can never leave.
     */
    @Override
    public boolean atGoal(RollerGoal goal, double secondsSinceApplied) {
        if (goal instanceof RollerGoal.IntakeUntilPiece) {
            RollerGoal.IntakeUntilPiece intake = (RollerGoal.IntakeUntilPiece) goal;
            return mechanism.hasPiece() || secondsSinceApplied >= intake.maxSeconds();
        }
        if (goal instanceof RollerGoal.Eject) {
            return secondsSinceApplied >= ((RollerGoal.Eject) goal).seconds();
        }
        if (goal instanceof RollerGoal.Speed) {
            return secondsSinceApplied >= ((RollerGoal.Speed) goal).settleSeconds();
        }
        if (goal instanceof RollerGoal.FeedVolts) {
            return secondsSinceApplied >= ((RollerGoal.FeedVolts) goal).settleSeconds();
        }
        return true;
    }

    /**
     * {@inheritDoc}
     *
     * <p>{@code true} only for {@link RollerGoal.IntakeUntilPiece}, whose arrival consults a real
     * sensor through {@code hasPiece()}. Every other goal reports {@code false}, and the
     * distinction is worth being pedantic about: the eject, duty-cycle and feed goals arrive on a
     * dwell that measures nothing, and idle and continuous-intake arrive by definition rather than
     * by observation. Reporting any of them as observable would put a value in
     * {@code Bindings/<key>/Observable} that invites somebody debugging a match log to believe a
     * sensor confirmed something it never saw.
     *
     * <p>Note that {@link RollerGoal.IntakeUntilPiece} is only honestly observable when the
     * mechanism actually carries detection. Because that cannot be read back — see the class
     * javadoc — this reports {@code true} on the assumption that a team writing an intake-until
     * goal has the hardware for it, and {@link #note(RollerGoal)} carries the caveat.
     */
    @Override
    public boolean observable(RollerGoal goal) {
        return goal instanceof RollerGoal.IntakeUntilPiece;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Signed difference between the commanded duty cycle and the applied one, in
     * {@link #unit()}. Feed goals are converted from volts to a fraction of nominal so that the
     * number stays comparable to {@link #measured()} rather than switching axes mid-log.
     *
     * <p>The intake and eject goals return {@code NaN}, and not because nothing is commanded:
     * their duty cycles live in {@code RollerMechanism.Config.intakeSpeed} and
     * {@code ejectSpeed}, both package-private with no accessor. Reporting a guess would be worse
     * than reporting nothing.
     */
    @Override
    public double error(RollerGoal goal) {
        if (goal instanceof RollerGoal.Speed) {
            RollerGoal.Speed speed = (RollerGoal.Speed) goal;
            return resolvedOr(speed, sanitise(speed.dutyCycle(), MAX_DUTY_CYCLE)) - mechanism.getSpeed();
        }
        if (goal instanceof RollerGoal.FeedVolts) {
            RollerGoal.FeedVolts feed = (RollerGoal.FeedVolts) goal;
            double commandedFraction =
                    resolvedOr(feed, sanitise(feed.volts(), MAX_FEED_VOLTS)) / MAX_FEED_VOLTS;
            return commandedFraction - mechanism.getSpeed();
        }
        if (goal instanceof RollerGoal.Idle) {
            return -mechanism.getSpeed();
        }
        return Double.NaN;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Always {@code NaN}. A roller runs no closed loop and has no tolerance band in any unit
     * this binding reports: arrival is either a boolean sensor or a stopwatch, and neither has a
     * width. Publishing the settle time here would be a category error, since the field is
     * documented in {@link #unit()} terms and a reader would see seconds plotted as a duty cycle.
     */
    @Override
    public double tolerance(RollerGoal goal) {
        return Double.NaN;
    }

    // =========================================================================
    //                            HUMAN READABLE
    // =========================================================================

    /**
     * {@inheritDoc}
     *
     * <p>The goal's kind and nothing else. There are exactly six possible return values for any
     * robot, which is what "low cardinality" has to mean for a string that gets edge-detected into
     * a log: folding the duty cycle or the dwell into the label would multiply that six by the
     * number of distinct numbers any state ever names, for no benefit that
     * {@link #detail(RollerGoal)} does not already provide.
     */
    @Override
    public String label(RollerGoal goal) {
        if (goal instanceof RollerGoal.IntakeUntilPiece) {
            return "IntakeUntilPiece";
        }
        if (goal instanceof RollerGoal.IntakeContinuous) {
            return "IntakeContinuous";
        }
        if (goal instanceof RollerGoal.Eject) {
            return "Eject";
        }
        if (goal instanceof RollerGoal.Idle) {
            return "Idle";
        }
        if (goal instanceof RollerGoal.Speed) {
            return "Speed";
        }
        if (goal instanceof RollerGoal.FeedVolts) {
            return "FeedVolts";
        }
        return "Unknown";
    }

    /**
     * {@inheritDoc}
     *
     * <p>The label plus whatever number the goal carries and, where it helps, the live readback.
     * This string is never edge-detected, so interpolating freely here is exactly what it is for.
     */
    @Override
    public String detail(RollerGoal goal) {
        if (goal instanceof RollerGoal.IntakeUntilPiece) {
            RollerGoal.IntakeUntilPiece intake = (RollerGoal.IntakeUntilPiece) goal;
            return String.format("IntakeUntilPiece(timeout %.2fs), hasPiece=%s, %.1fA",
                    intake.maxSeconds(), mechanism.hasPiece(), mechanism.getCurrent());
        }
        if (goal instanceof RollerGoal.IntakeContinuous) {
            return String.format("IntakeContinuous at %.2f frac, %.1fA",
                    mechanism.getSpeed(), mechanism.getCurrent());
        }
        if (goal instanceof RollerGoal.Eject) {
            return String.format("Eject(dwell %.2fs) at %.2f frac",
                    ((RollerGoal.Eject) goal).seconds(), mechanism.getSpeed());
        }
        if (goal instanceof RollerGoal.Idle) {
            return String.format("Idle, applied %.2f frac%s",
                    mechanism.getSpeed(),
                    Math.abs(mechanism.getSpeed()) <= IDLE_DUTY_EPSILON ? "" : " (still coasting down)");
        }
        if (goal instanceof RollerGoal.Speed) {
            RollerGoal.Speed speed = (RollerGoal.Speed) goal;
            return String.format("Speed(%.2f frac, settle %.2fs), applied %.2f frac",
                    resolvedOr(speed, sanitise(speed.dutyCycle(), MAX_DUTY_CYCLE)),
                    speed.settleSeconds(), mechanism.getSpeed());
        }
        if (goal instanceof RollerGoal.FeedVolts) {
            RollerGoal.FeedVolts feed = (RollerGoal.FeedVolts) goal;
            return String.format("FeedVolts(%.2fV, settle %.2fs), applied %.2f frac",
                    resolvedOr(feed, sanitise(feed.volts(), MAX_FEED_VOLTS)),
                    feed.settleSeconds(), mechanism.getSpeed());
        }
        return label(goal);
    }

    /**
     * {@inheritDoc}
     *
     * <p>Surfaces the two things about this mechanism that a stuck state cannot explain by itself.
     *
     * <p>The first is detection precedence. {@code RollerMechanism.hasPiece()} returns the beam
     * break's inverted reading and returns immediately when one is configured; the stall-current
     * latch is only reached when no beam break exists. On a robot configured with both, the beam
     * break wins outright and the stall threshold is dead code — so a team debugging a missed
     * pickup on such a robot should be looking at the sensor and its wiring, never at the stall
     * current or its timing.
     *
     * <p>The second is that this binding cannot verify any detection is configured at all, because
     * {@code RollerMechanism} does not expose it. When nothing was declared through
     * {@link PieceDetection}, an intake-until goal that is sitting at {@code hasPiece() == false}
     * says so here, along with the reminder that an unconfigured roller produces exactly this
     * symptom and will resolve it by timing out rather than by picking anything up.
     */
    @Override
    public String note(RollerGoal goal) {
        if (!(goal instanceof RollerGoal.IntakeUntilPiece)) {
            return "";
        }
        if (mechanism.hasPiece()) {
            return "piece detected; if both a beam break and stall detection are configured, "
                    + "the beam break is what reported this — hasPiece() returns on the beam break "
                    + "before the stall latch is ever consulted.";
        }
        StringBuilder note = new StringBuilder(
                "no piece detected yet; this goal will otherwise expire on its timeout. "
                        + "If both a beam break and stall detection are configured, only the beam "
                        + "break can report a piece — hasPiece() returns on it early and the stall "
                        + "latch is unreachable dead code.");
        if (detection == PieceDetection.UNDECLARED) {
            note.append(" This binding cannot confirm any detection is configured: "
                    + "RollerMechanism.Config keeps beamBreakPort and stallCurrentThreshold "
                    + "package-private with no accessor. A roller with neither configured has a "
                    + "permanently false hasPiece() and looks exactly like this. Declare "
                    + "PieceDetection on the binding to have that checked at build time instead.");
        } else if (detection == PieceDetection.ABSENT) {
            note.append(" This roller was declared to have no detection at all, so hasPiece() "
                    + "can never become true and only the timeout will end this goal.");
        }
        return note.toString();
    }

    // =========================================================================
    //                             VALIDATION
    // =========================================================================

    /**
     * {@inheritDoc}
     *
     * <p>Catches three classes of problem on a laptop rather than in a pit.
     *
     * <p>Out-of-range and non-finite outputs come first. {@link RollerGoal.Speed} and
     * {@link RollerGoal.FeedVolts} deliberately pass their components through unclamped so that
     * this hook can name the offending state, which is far more useful than a 4.0 quietly becoming
     * a 1.0 and appearing to work. Having reported the problem, this method also stores a
     * sanitised value in {@link #resolvedOutput} so the pursue path has something safe to hand a
     * motor even if a caller chooses to continue past the errors.
     *
     * <p>A confusion between the two goal types is worth calling out separately: a
     * {@code FeedVolts} goal holding a value inside {@code [-1, 1]} is much more likely to be a
     * duty cycle that reached the wrong factory than a genuine request for two-thirds of a volt,
     * which would barely turn the rollers.
     *
     * <p>Finally, an {@link RollerGoal.IntakeUntilPiece} goal aimed at a roller declared to have
     * no detection is rejected outright. That check only runs when the team declared
     * {@link PieceDetection#ABSENT}, because the mechanism itself will not say — see the class
     * javadoc for why this is a declaration rather than an inspection.
     */
    @Override
    public void validate(RollerGoal goal, Consumer<String> problems) {
        if (goal == null) {
            problems.accept("roller '" + key + "' was given a null goal.");
            return;
        }

        if (goal instanceof RollerGoal.IntakeUntilPiece && detection == PieceDetection.ABSENT) {
            problems.accept("roller '" + key + "' has neither a beam break nor stall detection "
                    + "configured; IntakeUntilPiece can only ever time out. Use "
                    + "RollerGoal.intakeContinuous(), or configure detection on the mechanism.");
        }

        if (goal instanceof RollerGoal.Speed) {
            RollerGoal.Speed speed = (RollerGoal.Speed) goal;
            double duty = speed.dutyCycle();
            if (!Double.isFinite(duty)) {
                problems.accept("roller '" + key + "' Speed goal commands a non-finite duty cycle ("
                        + duty + "); duty cycle must be a finite value in [-1, 1].");
            } else if (Math.abs(duty) > MAX_DUTY_CYCLE) {
                problems.accept("roller '" + key + "' Speed goal commands duty cycle " + duty
                        + ", outside [-1, 1]. If this was meant to be a voltage, use "
                        + "RollerGoal.feed(volts, settleSeconds) instead.");
            }
            resolvedOutput.put(speed, sanitise(duty, MAX_DUTY_CYCLE));
        }

        if (goal instanceof RollerGoal.FeedVolts) {
            RollerGoal.FeedVolts feed = (RollerGoal.FeedVolts) goal;
            double volts = feed.volts();
            if (!Double.isFinite(volts)) {
                problems.accept("roller '" + key + "' FeedVolts goal commands a non-finite voltage ("
                        + volts + "); voltage must be a finite value in [-12, 12].");
            } else if (Math.abs(volts) > MAX_FEED_VOLTS) {
                problems.accept("roller '" + key + "' FeedVolts goal commands " + volts
                        + " V, outside [-12, 12]; the battery cannot supply it.");
            } else if (volts != 0.0 && Math.abs(volts) <= MAX_DUTY_CYCLE) {
                problems.accept("roller '" + key + "' FeedVolts goal commands " + volts
                        + " V, which is almost certainly a duty cycle that reached the wrong "
                        + "factory — that voltage would barely turn the rollers. Use "
                        + "RollerGoal.speed(dutyCycle, settleSeconds) for duty cycle.");
            }
            resolvedOutput.put(feed, sanitise(volts, MAX_FEED_VOLTS));
        }
    }

    // =========================================================================
    //                             INTERNALS
    // =========================================================================

    /**
     * Returns the output resolved for {@code goal} at build time, or {@code fallback} when this
     * goal never passed through {@link #validate}.
     *
     * <p>A goal reaching the pursue path unvalidated is unusual but not impossible — an override
     * or a hand-built runner can bypass the builder — so this deliberately falls back instead of
     * failing. The fallback is the same sanitisation validate would have applied, which keeps the
     * two paths behaviourally identical and stops a stray {@code NaN} from reaching a motor.
     *
     * @param goal     the goal whose resolved output is wanted
     * @param fallback value to use when the goal was never validated
     * @return the resolved output, always finite
     */
    private double resolvedOr(RollerGoal goal, double fallback) {
        Double resolved = resolvedOutput.get(goal);
        return resolved != null ? resolved : fallback;
    }

    /**
     * Clamps {@code value} into {@code [-limit, limit]}, mapping non-finite input to zero.
     *
     * <p>Zero is the right answer for a {@code NaN}: it is the only output that is safe on every
     * mechanism, and the condition that produced it has already been reported by
     * {@link #validate}. Silently running a roller at full speed because a unit conversion divided
     * by zero somewhere upstream would be considerably worse than not running it at all.
     *
     * @param value the commanded value
     * @param limit the symmetric magnitude limit; must be positive
     * @return a finite value within {@code [-limit, limit]}
     */
    private static double sanitise(double value, double limit) {
        if (!Double.isFinite(value)) {
            return 0.0;
        }
        return Math.max(-limit, Math.min(limit, value));
    }
}
