package frc.lib.catalyst.statemachine.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.statemachine.Binding;
import frc.lib.catalyst.statemachine.FaultPolicy;
import frc.lib.catalyst.statemachine.Handle;
import frc.lib.catalyst.statemachine.Phase;
import frc.lib.catalyst.statemachine.Route;
import frc.lib.catalyst.statemachine.Routing;
import frc.lib.catalyst.statemachine.Snapshot;
import frc.lib.catalyst.statemachine.StateGraph;
import frc.lib.catalyst.statemachine.StateMachineCore;
import frc.lib.catalyst.statemachine.StateMachineTelemetry;
import frc.lib.catalyst.statemachine.StateSpec;
import frc.lib.catalyst.statemachine.EdgeSpec;
import frc.lib.catalyst.statemachine.TransitionRecord;
import frc.lib.catalyst.statemachine.TransitionResult;
import frc.lib.catalyst.statemachine.ValidationReport;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

/**
 * A real, fully-logged state machine for an entire robot superstructure.
 *
 * <p>Unlike {@link frc.lib.catalyst.mechanisms.SuperstructureCoordinator}, which only understood
 * {@code LinearMechanism} and {@code RotationalMechanism} positions, this accepts <b>every</b>
 * Catalyst mechanism type — elevator, arm, wrist, differential wrist, turret, flywheel, roller,
 * claw, winch, pneumatic — and any subsystem you wrote yourself, through
 * {@link frc.lib.catalyst.statemachine.mech.Mechanisms}.
 *
 * <h2>What makes it a state machine rather than a preset applier</h2>
 *
 * <ul>
 *   <li><b>A legal-transition graph.</b> An edge you did not declare is a transition the robot will
 *       not make. Requests that violate it are refused, with a reason, and logged.</li>
 *   <li><b>Arrival is proven, not assumed.</b> {@link #current()} is only ever a state whose every
 *       gating mechanism was measured to be at its goal. A timeout leaves the machine where it
 *       actually is, so the next transition plans from the truth.</li>
 *   <li><b>Guards, entry guards and interlocks</b> — declarative preconditions, each with a reason
 *       string that shows up in the log when it blocks something.</li>
 *   <li><b>Staged actuation</b> per edge: raise the elevator, <em>then</em> deploy the arm, without
 *       hand-writing a command group whose sequencing nothing can inspect.</li>
 *   <li><b>Everything is logged</b>, on a disciplined cadence, under {@code /Catalyst/&lt;prefix&gt;/}.</li>
 * </ul>
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * public enum SuperState { STOW, INTAKE, CARRY, AIM, SCORE, CLIMB }
 *
 * var b = Superstructure.builder(SuperState.class, "Superstructure");
 *
 * var elevator = b.bind("elevator", Mechanisms.linear(elevatorMech));
 * var arm      = b.bind("arm",      Mechanisms.rotational(armMech));
 * var wrist    = b.bind("wrist",    Mechanisms.rotational(wristMech));
 * var claw     = b.bind("claw",     Mechanisms.claw(clawMech));
 * var intake   = b.bind("intake",   Mechanisms.roller(intakeMech));
 * var shooter  = b.bind("shooter",  Mechanisms.flywheel(shooterMech));
 * var turret   = b.bind("turret",   Mechanisms.turret(turretMech));
 * var climber  = b.bind("climber",  Mechanisms.winch(climberMech));
 * var funnel   = b.bind("funnel",   Mechanisms.pneumatic(funnelMech));
 *
 * superstructure = b
 *     // The safe posture, stated once. Every state inherits it unless it says otherwise,
 *     // so the mechanism you forget in one state does not stay where the last state parked it.
 *     .defaults(s -> s
 *         .set(claw,    ClawGoal.hold())
 *         .set(intake,  RollerGoal.idle())
 *         .set(shooter, FlywheelGoal.idle())
 *         .set(turret,  TurretGoal.forward())
 *         .set(climber, WinchGoal.stop())
 *         .set(funnel,  PneumaticGoal.retracted()))
 *
 *     .state(SuperState.STOW, s -> s
 *         .set(elevator, LinearGoal.meters(0.0))
 *         .set(arm,      RotationalGoal.degrees(0))
 *         .set(wrist,    RotationalGoal.degrees(0)))
 *
 *     .state(SuperState.INTAKE, s -> s
 *         .set(elevator, LinearGoal.meters(0.05))
 *         .set(arm,      RotationalGoal.degrees(-20))
 *         .set(wrist,    RotationalGoal.degrees(15))
 *         .set(intake,   RollerGoal.intakeUntilPiece(3.0))
 *         .set(claw,     ClawGoal.open()))
 *
 *     .state(SuperState.CARRY, s -> s
 *         .set(elevator, LinearGoal.meters(0.15))
 *         .set(arm,      RotationalGoal.degrees(10))
 *         .set(wrist,    RotationalGoal.degrees(0))
 *         .set(claw,     ClawGoal.grip(1.5)))
 *
 *     .state(SuperState.AIM, s -> s
 *         .set(elevator, LinearGoal.meters(0.30))
 *         .set(arm,      RotationalGoal.degrees(35))
 *         .set(wrist,    RotationalGoal.degrees(-10))
 *         .set(shooter,  FlywheelGoal.rpm(4200))
 *         .set(turret,   TurretGoal.at(drive::getPose, SPEAKER, "speaker"))
 *         .entryGuard(clawMech::hasPiece, "no piece"))
 *
 *     .state(SuperState.SCORE, s -> s
 *         .set(elevator, LinearGoal.meters(0.30))
 *         .set(arm,      RotationalGoal.degrees(35))
 *         .set(shooter,  FlywheelGoal.rpm(4200))
 *         .set(claw,     ClawGoal.open())
 *         .settleFor(0.2))
 *
 *     .state(SuperState.CLIMB, s -> s
 *         .set(elevator, LinearGoal.meters(0.0))
 *         .set(arm,      RotationalGoal.degrees(0))
 *         .set(climber,  WinchGoal.extend())
 *         .release(turret)                     // the driver owns the turret while climbing
 *         .entryGuard(RobotState::isEndgame, "not endgame"))
 *
 *     .hub(SuperState.STOW)                    // STOW connects to everything
 *     .allowBoth(SuperState.CARRY, SuperState.AIM)
 *     .allow(SuperState.AIM, SuperState.SCORE)
 *     .allow(SuperState.INTAKE, SuperState.CARRY)
 *
 *     // Raise the elevator BEFORE the arm swings out, or the arm hits the chassis.
 *     .edge(SuperState.STOW, SuperState.AIM, e -> e.stage(elevator).stage(arm, wrist))
 *
 *     // Once the climber is out, nothing but CLIMB and STOW is safe.
 *     .interlock("climberStowed", () -> !climberMech.isFullyExtended(),
 *                s -> s != SuperState.CLIMB && s != SuperState.STOW)
 *
 *     .build();
 *
 * // in robotInit, tell it where the robot is physically built:
 * superstructure.engine().seed(SuperState.STOW);
 *
 * // bindings
 * operator.a().onTrue(superstructure.goTo(SuperState.INTAKE, "op.a"));
 * operator.y().onTrue(superstructure.goTo(SuperState.AIM,    "op.y"));
 * superstructure.arrivedAt(SuperState.CARRY).onTrue(leds.flash(Color.kGreen));
 * }</pre>
 *
 * @param <S> the enum of superstructure states
 * @since 1.2.0
 */
public final class Superstructure<S extends Enum<S>> extends SubsystemBase implements SuperstructureLike {

    private final StateMachineCore<S> engine;
    private final Class<S> stateType;
    private final List<Command> runners;

    private Superstructure(String name, StateMachineCore<S> engine, Class<S> stateType,
                           List<Command> runners) {
        super(name);
        this.engine = engine;
        this.stateType = stateType;
        this.runners = runners;
    }

    /** Start building a superstructure over {@code stateType}. */
    public static <S extends Enum<S>> Builder<S> builder(Class<S> stateType, String name) {
        return new Builder<>(stateType, name);
    }

    /**
     * Steps the engine exactly once per scheduler loop.
     *
     * <p>{@code CommandScheduler.run()} runs every {@code Subsystem.periodic()} before it polls
     * triggers and before it runs any command, so the engine always evaluates arrival against
     * inputs that every mechanism has already refreshed this loop.
     */
    @Override
    public void periodic() {
        engine.setEnabled(edu.wpi.first.wpilibj.DriverStation.isEnabled());
        engine.step();
    }

    /** The underlying engine, for seeding, snapshots and anything not surfaced here. */
    public StateMachineCore<S> engine() {
        return engine;
    }

    // ==================================================================
    // Commands
    // ==================================================================

    /** {@link #goTo(Enum, String)} attributed to {@code "code"}. */
    public Command goTo(S target) {
        return goTo(target, "code");
    }

    /**
     * Request {@code target} and end when the superstructure has settled there.
     *
     * <p>Ends immediately if the request is refused — the button visibly does nothing, the reason
     * is logged and announced, and the requirement is released so nothing is left holding. Also
     * ends on a fault, and when a later request supersedes this one.
     *
     * <p>Built with {@code Commands.defer}, so the route, the guards and the origin state are all
     * resolved when the command is <em>scheduled</em>, not when it is constructed. Storing the
     * result in a field and re-binding it every match works correctly — which is precisely what
     * the old coordinator got wrong.
     *
     * <p><b>In autonomous, do not write {@code goTo(A).andThen(next)}</b>: a self-detected fault
     * cannot make a command end "interrupted", so {@code next} would run anyway. Use
     * {@link #onlyIfSettled(Enum, Command)}.
     *
     * @param triggerSource recorded in the transition history, so the log says who asked
     */
    public Command goTo(S target, String triggerSource) {
        return Commands.defer(() -> {
            TransitionResult<S> result = engine.request(target, triggerSource);
            if (result.rejected()) return Commands.none();
            final long seq = result.seq();
            return Commands.idle(this)
                    .until(() -> engine.isSettledAt(target)
                            || engine.isFaulted()
                            || engine.lastAcceptedSeq() != seq)
                    .finallyDo(interrupted -> {
                        if (interrupted && engine.activeSeq() == seq) {
                            engine.abort("driving command interrupted");
                        }
                    });
        }, Set.of(this)).withName(getName() + ".To(" + target.name() + ")");
    }

    /** {@link #goToAndHold(Enum, String)} attributed to {@code "code"}. */
    public Command goToAndHold(S target) {
        return goToAndHold(target, "code");
    }

    /**
     * Like {@link #goTo}, but never ends on arrival — it keeps holding the state until interrupted.
     * The natural shape for {@code whileTrue}.
     */
    public Command goToAndHold(S target, String triggerSource) {
        return Commands.defer(() -> {
            TransitionResult<S> result = engine.request(target, triggerSource);
            if (result.rejected()) return Commands.none();
            final long seq = result.seq();
            return Commands.idle(this)
                    .until(() -> engine.isFaulted() || engine.lastAcceptedSeq() != seq)
                    .finallyDo(interrupted -> {
                        if (interrupted && engine.activeSeq() == seq) {
                            engine.abort("driving command interrupted");
                        }
                    });
        }, Set.of(this)).withName(getName() + ".Hold(" + target.name() + ")");
    }

    /** Request {@code target} and return immediately, without waiting or holding a requirement. */
    public Command requestOnly(S target) {
        return Commands.runOnce(() -> engine.request(target, "requestOnly"))
                .withName(getName() + ".Request(" + target.name() + ")");
    }

    /** Wait until settled at {@code target}. Holds no requirement, so it composes freely. */
    public Command waitUntilSettled(S target) {
        return Commands.waitUntil(() -> engine.isSettledAt(target))
                .withName(getName() + ".WaitFor(" + target.name() + ")");
    }

    /**
     * Run {@code next} only if the machine is settled at {@code state} when this is reached.
     *
     * <p>The safe autonomous idiom: {@code goTo(AIM).andThen(sm.onlyIfSettled(AIM, shoot()))} will
     * not shoot into the floor because the arm never made it.
     */
    public Command onlyIfSettled(S state, Command next) {
        return Commands.either(next, Commands.none(), () -> engine.isSettledAt(state))
                .withName(getName() + ".IfSettled(" + state.name() + ")");
    }

    /** Tell the machine where the robot physically is. Call once at robot init. */
    public Command seed(S assumedState) {
        return Commands.runOnce(() -> engine.seed(assumedState))
                .ignoringDisable(true)
                .withName(getName() + ".Seed(" + assumedState.name() + ")");
    }

    /** Cancel any in-flight transition. */
    public Command abort(String reason) {
        return Commands.runOnce(() -> engine.abort(reason)).withName(getName() + ".Abort");
    }

    /** Clear a latched fault. */
    public Command clearFault() {
        return Commands.runOnce(engine::clearFault).ignoringDisable(true)
                .withName(getName() + ".ClearFault");
    }

    // ==================================================================
    // Triggers
    // ==================================================================

    /** The machine believes it is in {@code state} (not necessarily confirmed). */
    public Trigger in(S state) {
        return new Trigger(engine.inState(state));
    }

    /** In {@code state}, confirmed, and still measuring true. */
    public Trigger settledIn(S state) {
        return new Trigger(engine.settledIn(state));
    }

    /** Rising edge of {@link #settledIn(Enum)} — fires once each time the state is genuinely reached. */
    public Trigger arrivedAt(S state) {
        return settledIn(state);
    }

    /** A transition is in flight. */
    public Trigger transitioning() {
        return new Trigger(engine.transitioning());
    }

    /** A fault is latched. */
    public Trigger faulted() {
        return new Trigger(engine.faultedSupplier());
    }

    /** Pulses for one loop on each refused request — bind it to a rumble so refusals are felt. */
    public Trigger rejected() {
        return new Trigger(engine.rejectedSupplier());
    }

    /** A bound mechanism is currently held by another command, usually a driver override. */
    public Trigger overridden() {
        return new Trigger(engine.overridden());
    }

    // ==================================================================
    // Queries
    // ==================================================================

    /** Last proven state. */
    public S current() { return engine.current(); }
    /** Whether that belief is currently proven. */
    public boolean stateConfirmed() { return engine.stateConfirmed(); }
    /** Requested destination. */
    public S target() { return engine.target(); }
    /** What the machine is doing. */
    public Phase phase() { return engine.phase(); }
    /** Measured: is every gating mechanism at {@code state}'s goal right now? */
    public boolean isAt(S state) { return engine.isAt(state); }
    /** In {@code state}, confirmed, and measuring true. */
    public boolean isSettledAt(S state) { return engine.isSettledAt(state); }
    /** Whether a fault is latched. */
    public boolean isFaulted() { return engine.isFaulted(); }
    /** Fraction of the active hop's gating mechanisms at goal. */
    public double progress() { return engine.progress(); }
    /** Stable one-line holdup summary. */
    public String blocker() { return engine.blocker(); }
    /** One-line human summary of everything. */
    public String summary() { return engine.summary(); }
    /** Gating mechanisms not yet at goal. */
    public List<String> waitingOn() { return engine.waitingOn(); }
    /** States that would be accepted right now. */
    public List<S> legalTargets() { return new ArrayList<>(engine.legalTargets()); }
    /** Route a request would take, with no side effects. */
    public Route<S> plan(S target) { return engine.plan(target); }
    /** Newest-first transition history. */
    public List<TransitionRecord<S>> history() { return engine.history(); }
    /** Everything, in one immutable value. */
    public Snapshot<S> snapshot() { return engine.snapshot(); }
    /** The legal-transition graph. */
    public StateGraph<S> graph() { return engine.graph(); }

    // ==================================================================
    // SuperstructureLike — the GoalDirector seam
    // ==================================================================

    @Override
    public Command transitionTo(String stateName) {
        // Resolution happens inside the deferred supplier, and a bad name is logged as a rejection
        // rather than thrown — an exception escaping here would propagate out of
        // CommandScheduler.run() and take the robot loop with it.
        return Commands.defer(() -> {
            S target;
            try {
                target = Enum.valueOf(stateType, stateName);
            } catch (IllegalArgumentException | NullPointerException ex) {
                edu.wpi.first.wpilibj.DriverStation.reportWarning(
                        "[" + getName() + "] unknown state '" + stateName + "'; known states are "
                                + getStateNames(), false);
                return Commands.none();
            }
            return goTo(target, "goal");
        }, Set.of(this)).withName(getName() + ".To(" + stateName + ")");
    }

    @Override
    public boolean isAtState(String stateName) {
        try {
            return engine.isSettledAt(Enum.valueOf(stateType, stateName));
        } catch (IllegalArgumentException | NullPointerException ex) {
            return false;
        }
    }

    @Override
    public String getCurrentState() {
        S s = engine.current();
        return s == null ? "" : s.name();
    }

    @Override
    public boolean isTransitioning() {
        return engine.isTransitioning();
    }

    @Override
    public List<String> getStateNames() {
        List<String> out = new ArrayList<>();
        for (S s : stateType.getEnumConstants()) out.add(s.name());
        return out;
    }

    // ==================================================================
    // Dashboard
    // ==================================================================

    /**
     * Publish state, phase, blocker, summary and progress as a single {@link Sendable}, so one drag
     * onto Elastic or Shuffleboard gives a working pit display with no layout work.
     */
    public void addToDashboard(String tab) {
        Shuffleboard.getTab(tab).add(getName(), (Sendable) builder -> {
            builder.setSmartDashboardType("Superstructure");
            builder.addStringProperty("State", this::getCurrentState, null);
            builder.addBooleanProperty("Confirmed", this::stateConfirmed, null);
            builder.addStringProperty("Phase", () -> phase().name(), null);
            builder.addStringProperty("Blocker", this::blocker, null);
            builder.addStringProperty("Summary", this::summary, null);
            builder.addDoubleProperty("Progress", this::progress, null);
            builder.addBooleanProperty("Faulted", this::isFaulted, null);
        });
    }

    /** The default commands installed on each bound mechanism, for teams managing them by hand. */
    public List<Command> goalRunners() {
        return List.copyOf(runners);
    }

    // ==================================================================
    // Builder
    // ==================================================================

    /**
     * Assembles a superstructure. Mirrors {@link StateMachineCore.Builder} and adds the robot-side
     * concerns: logging, the clock, and installing the per-mechanism default commands.
     *
     * @param <S> the enum of superstructure states
     */
    public static final class Builder<S extends Enum<S>> {

        private final Class<S> stateType;
        private final String name;
        private final StateMachineCore.Builder<S> core;
        private final Map<Handle<?>, Actuator<?>> actuators = new LinkedHashMap<>();
        private DoubleSupplier clock = Timer::getFPGATimestamp;
        private String logPrefix;
        private String alertSubsystem;
        private boolean logging = true;
        private boolean driverStationMessages = true;
        private boolean manageDefaults = true;

        Builder(Class<S> stateType, String name) {
            this.stateType = stateType;
            this.name = name == null ? "Superstructure" : name;
            this.logPrefix = this.name;
            this.core = StateMachineCore.builder(stateType, this.name);
        }

        /** Register a mechanism and get back a type-carrying handle. */
        public <G> Handle<G> bind(String key, Actuator<G> actuator) {
            Handle<G> h = core.bind(key, actuator);
            actuators.put(h, actuator);
            return h;
        }

        /** Register a mechanism that is driven but never gates arrival — LEDs, dashboards. */
        public <G> Handle<G> bindAdvisory(String key, Actuator<G> actuator) {
            Handle<G> h = core.bindAdvisory(key, actuator);
            actuators.put(h, actuator);
            return h;
        }

        /**
         * Register something that is not actuated at all — a sensor or a computed value that a
         * state's goals should wait on. It gates arrival but no command is ever run for it.
         */
        public <G> Handle<G> bindSensor(String key, Binding<G> binding) {
            return core.bind(key, binding);
        }

        /** Time source. Defaults to the FPGA timestamp. */
        public Builder<S> clock(DoubleSupplier clock) {
            this.clock = clock == null ? Timer::getFPGATimestamp : clock;
            return this;
        }

        /** Log prefix under {@code Catalyst/}. Defaults to the machine name. */
        public Builder<S> logPrefix(String prefix) {
            this.logPrefix = prefix;
            return this;
        }

        /** Turn structured logging off entirely. On by default — it is the point of the package. */
        public Builder<S> logging(boolean enabled) {
            this.logging = enabled;
            return this;
        }

        /** Subsystem name used for {@link frc.lib.catalyst.util.AlertManager} entries. */
        public Builder<S> alertSubsystem(String subsystem) {
            this.alertSubsystem = subsystem;
            return this;
        }

        /** Whether to emit one-shot Driver Station warnings on faults and rejections. */
        public Builder<S> driverStationMessages(boolean enabled) {
            this.driverStationMessages = enabled;
            return this;
        }

        /**
         * Whether to install a {@link GoalRunner} as each bound mechanism's default command.
         *
         * <p>On by default, because it is what makes driver override work without any extra code.
         * If a bound mechanism already has a default command, {@code build()} fails rather than
         * silently replacing it — a silently stomped {@code setDefaultCommand(holdPosition())} is
         * a very confusing afternoon.
         */
        public Builder<S> manageDefaults(boolean manage) {
            this.manageDefaults = manage;
            return this;
        }

        // --- mirrored core builder options ---

        /** Deadline for any hop with no more specific timeout. Default 4.0 s. */
        public Builder<S> defaultTimeout(double seconds) { core.defaultTimeout(seconds); return this; }
        /** Transition records to retain. Default 50. */
        public Builder<S> historyCapacity(int entries) { core.historyCapacity(entries); return this; }
        /** State assumed before seeding. */
        public Builder<S> initialState(S state) { core.initialState(state); return this; }
        /** Direct-only (default) or automatic multi-hop routing. */
        public Builder<S> routing(Routing routing) { core.routing(routing); return this; }
        /** Fault policy for states that do not declare their own. */
        public Builder<S> defaultFaultPolicy(FaultPolicy p) { core.defaultFaultPolicy(p); return this; }
        /** When false, deadlines warn instead of faulting. The migration setting. */
        public Builder<S> strict(boolean strict) { core.strict(strict); return this; }
        /** Abort the transition when any mechanism is taken over by another command. */
        public Builder<S> abortOnOverride(boolean abort) { core.abortOnOverride(abort); return this; }
        /** The safe posture, inherited by every state that does not override it. */
        public Builder<S> defaults(Consumer<StateSpec<S>> spec) { core.defaults(spec); return this; }
        /** Declare one state. Every enum constant must be declared exactly once. */
        public Builder<S> state(S state, Consumer<StateSpec<S>> spec) { core.state(state, spec); return this; }
        /** Declare edges from {@code from} to each listed state. */
        @SafeVarargs
        public final Builder<S> allow(S from, S first, S... rest) { core.allow(from, first, rest); return this; }
        /** Declare edges in both directions. */
        public Builder<S> allowBoth(S a, S b) { core.allowBoth(a, b); return this; }
        /** Connect {@code hub} to and from every other state. */
        public Builder<S> hub(S hub) { core.hub(hub); return this; }
        /** Declare one edge with its guard, deadline, cost and actuation stages. */
        public Builder<S> edge(S from, S to, Consumer<EdgeSpec<S>> spec) { core.edge(from, to, spec); return this; }
        /** Pin a route instead of trusting breadth-first search. */
        @SafeVarargs
        public final Builder<S> via(S from, S to, S... waypoints) { core.via(from, to, waypoints); return this; }
        /** Suppress the unreachable-state error for these states. */
        @SafeVarargs
        public final Builder<S> allowUnreachable(S... states) { core.allowUnreachable(states); return this; }
        /** A global, positively-named precondition. */
        public Builder<S> interlock(String n, BooleanSupplier ok, Predicate<S> blocks) {
            core.interlock(n, ok, blocks);
            return this;
        }

        /**
         * Check the configuration without building or touching hardware. The unit-test hook, and
         * worth calling from a test even for a robot you never simulate.
         */
        public ValidationReport validate() {
            core.clock(clock);
            return core.validate();
        }

        /**
         * Validate, construct, and install the per-mechanism default commands.
         *
         * @throws frc.lib.catalyst.statemachine.StateMachineConfigException listing every problem
         */
        public Superstructure<S> build() {
            core.clock(clock);
            if (logging) {
                core.telemetry(new CatalystStateMachineLog<S>(
                        logPrefix, alertSubsystem == null ? logPrefix : alertSubsystem,
                        driverStationMessages));
            } else {
                core.telemetry(StateMachineTelemetry.<S>noop());
            }

            List<String> extra = new ArrayList<>();
            if (manageDefaults) {
                checkExistingDefaults(extra);
                checkDefaultManageable(extra);
            }
            checkRequirements(extra);
            if (!extra.isEmpty()) {
                throw new frc.lib.catalyst.statemachine.StateMachineConfigException(name, extra);
            }

            StateMachineCore<S> engine = core.build();
            List<Command> runners = new ArrayList<>();
            Superstructure<S> superstructure = new Superstructure<>(name, engine, stateType, runners);

            if (manageDefaults) {
                for (Map.Entry<Handle<?>, Actuator<?>> e : actuators.entrySet()) {
                    Command runner = makeRunner(engine, e.getKey(), e.getValue());
                    runners.add(runner);
                    for (Subsystem s : e.getValue().requirements()) {
                        s.setDefaultCommand(runner);
                    }
                }
            }
            return superstructure;
        }

        @SuppressWarnings("unchecked")
        private <G> Command makeRunner(StateMachineCore<S> engine, Handle<?> handle, Actuator<?> actuator) {
            return new GoalRunner<>(engine, (Handle<G>) handle, (Actuator<G>) actuator, clock);
        }

        private void checkExistingDefaults(List<String> problems) {
            for (Map.Entry<Handle<?>, Actuator<?>> e : actuators.entrySet()) {
                Set<Subsystem> required = e.getValue().requirements();
                if (required == null) continue;
                for (Subsystem s : required) {
                    Command existing = s.getDefaultCommand();
                    if (existing != null) {
                        problems.add("'" + e.getKey().key() + "' already has a default command ("
                                + existing.getName() + "). The state machine provides hold behaviour "
                                + "itself — remove that setDefaultCommand(...) call, or use "
                                + ".manageDefaults(false) and schedule the GoalRunners yourself.");
                    }
                }
            }
        }

        /**
         * When the state machine installs the default commands, every managed binding must own
         * exactly one subsystem, and no two bindings may share one.
         *
         * <p>A binding that owns several subsystems would have its single {@link GoalRunner}
         * installed as the default of each, so a driver command interrupting one would drag the
         * others off their goals — and re-scheduling that shared default would steal all of them
         * back the moment the driver released one. Two bindings on the same subsystem is worse still:
         * {@code setDefaultCommand} silently overwrites, so the second binding wins and the first
         * never actuates. Both are refused here rather than discovered on the field; a team that
         * genuinely needs either arrangement drives the runners itself with
         * {@code .manageDefaults(false)}.
         */
        private void checkDefaultManageable(List<String> problems) {
            Map<Subsystem, String> owner = new java.util.HashMap<>();
            for (Map.Entry<Handle<?>, Actuator<?>> e : actuators.entrySet()) {
                String key = e.getKey().key();
                Set<Subsystem> required = e.getValue().requirements();
                if (required == null || required.isEmpty()) continue;   // reported by checkRequirements
                if (required.size() != 1) {
                    problems.add("'" + key + "' owns " + required.size() + " subsystems, but a "
                            + "state-machine-managed default command must own exactly one. Split it "
                            + "into one binding per subsystem, or use .manageDefaults(false) and drive "
                            + "the GoalRunners yourself.");
                }
                for (Subsystem s : required) {
                    String prior = owner.putIfAbsent(s, key);
                    if (prior != null) {
                        problems.add("bindings '" + prior + "' and '" + key + "' both own the same "
                                + "subsystem; the second would silently replace the first as its "
                                + "default command. Bind the subsystem once, or use "
                                + ".manageDefaults(false).");
                    }
                }
            }
        }

        /**
         * Every actuator must own at least one subsystem, or it has nothing to drive and no
         * requirement to hold against a driver override.
         *
         * <p>The full requirement <em>probe</em> promised by {@link Actuator#pursueCommand} — building
         * each command once and asserting its requirements are a subset of the declared set — is done
         * lazily by {@link GoalRunner} at construction rather than here, because it needs a concrete
         * goal instance and those are not resolved until the engine is built.
         */
        private void checkRequirements(List<String> problems) {
            for (Map.Entry<Handle<?>, Actuator<?>> e : actuators.entrySet()) {
                Actuator<?> a = e.getValue();
                Set<Subsystem> declared = a.requirements();
                if (declared == null || declared.isEmpty()) {
                    problems.add("'" + e.getKey().key() + "' declares no requirements; a binding must "
                            + "own the subsystem it drives");
                }
            }
        }
    }
}
