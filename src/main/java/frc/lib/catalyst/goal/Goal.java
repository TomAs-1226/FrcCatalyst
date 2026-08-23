package frc.lib.catalyst.goal;

import frc.lib.catalyst.command.CatalystCommand;
import org.wpilib.command3.Command;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A single, named <em>intent</em> the robot can pursue — "score high",
 * "intake from the ground", "aim and shoot". A {@code Goal} is the unit the
 * optional {@link GoalDirector} works with.
 *
 * <p>This is the "software-defined robot" layer: instead of the driver
 * commanding every mechanism by hand, they hand the robot an intent and the
 * director figures out the details — reach the right superstructure state,
 * run the setup work (spin a shooter, aim a turret), report when it's ready,
 * and hand control back the instant the driver wants it.
 *
 * <p><b>This whole package is optional.</b> Nothing else in Catalyst depends
 * on it. A goal composes pieces you already have:
 * <ul>
 *   <li>a {@link frc.lib.catalyst.mechanisms.SuperstructureCoordinator}
 *       state to reach (optional), and/or</li>
 *   <li>arbitrary <em>setup</em> commands to run alongside the transition
 *       (optional) — e.g. spin a flywheel, track with a turret, auto-align
 *       the drivetrain, and/or</li>
 *   <li>a <em>readiness</em> test that decides when the goal is "good to go"
 *       (optional).</li>
 * </ul>
 *
 * <p>A goal with no superstructure state and no coordinator is perfectly
 * valid: it's just "run this setup command, with intent telemetry and driver
 * override for free." So teams that never touch the coordinator can still use
 * goals as a thin intent layer.
 *
 * <p>Example:
 * <pre>{@code
 * Goal SCORE_HIGH = Goal.named("ScoreHigh")
 *     .superstructureState("SCORE_HIGH")            // coordinator state to reach
 *     .with(() -> shooter.spinUp(4300))             // setup, runs alongside
 *     .readyWhen(() -> superstructure.isAtState("SCORE_HIGH")
 *                       && shooter.atSpeed())        // when is it good to fire?
 *     .build();
 *
 * Goal STOW = Goal.named("Stow").superstructureState("STOW").build();
 * }</pre>
 *
 * @see GoalDirector
 */
public final class Goal {

    private final String name;
    private final String superstructureState;     // nullable
    private final Supplier<Command> setup;         // nullable
    private final BooleanSupplier ready;           // nullable

    private Goal(String name, String superstructureState,
                 Supplier<Command> setup, BooleanSupplier ready) {
        this.name = name;
        this.superstructureState = superstructureState;
        this.setup = setup;
        this.ready = ready;
    }

    /** The display name (also the NetworkTables key). */
    public String name() {
        return name;
    }

    /** The {@code SuperstructureCoordinator} state to reach, or {@code null}. */
    public String superstructureState() {
        return superstructureState;
    }

    /** Whether this goal targets a superstructure state. */
    public boolean hasSuperstructureState() {
        return superstructureState != null && !superstructureState.isEmpty();
    }

    /** A fresh setup command for this goal, or {@code null} if none. */
    public CatalystCommand newSetupCommand() {
        return setup == null ? null : CatalystCommand.of(setup.get());
    }

    /** Whether this goal has setup work. */
    public boolean hasSetup() {
        return setup != null;
    }

    /**
     * Whether this goal is "ready" right now. If no readiness test was given,
     * a goal is considered ready as soon as it has been requested (the
     * director treats reaching the superstructure state, if any, as readiness;
     * see {@link GoalDirector}).
     */
    public boolean hasReadinessTest() {
        return ready != null;
    }

    /** Evaluate the user-supplied readiness test (only call if it exists). */
    boolean readyNow() {
        return ready != null && ready.getAsBoolean();
    }

    /** Start building a goal with the given name. */
    public static Builder named(String name) {
        return new Builder(name);
    }

    /** Fluent builder for {@link Goal}. */
    public static final class Builder {
        private final String name;
        private String superstructureState;
        private Supplier<Command> setup;
        private BooleanSupplier ready;

        private Builder(String name) {
            this.name = name;
        }

        /**
         * The {@link frc.lib.catalyst.mechanisms.SuperstructureCoordinator}
         * state this goal drives toward. Optional — omit it for goals that are
         * pure setup work (e.g. "spin up and aim" with no superstructure move).
         */
        public Builder superstructureState(String stateName) {
            this.superstructureState = stateName;
            return this;
        }

        /**
         * Setup work to run <em>alongside</em> the superstructure transition.
         * Supplied as a factory so the goal can be pursued repeatedly (WPILib
         * commands can't be rescheduled while running). If this command holds
         * (never finishes — e.g. "keep the shooter spinning"), the goal is
         * held until the driver releases it.
         */
        public Builder with(Supplier<Command> setupFactory) {
            this.setup = setupFactory;
            return this;
        }

        /**
         * Convenience overload for a setup command you can hand over directly.
         * <p>Commands v2 needed {@code asProxy()} here so the command could be re-scheduled on
         * each pursuit. v3 has no such restriction and no such method, so the supplier hands back
         * the command directly.
         */
        public Builder with(Command setupCommand) {
            this.setup = setupCommand == null ? null : () -> setupCommand;
            return this;
        }

        /**
         * When is this goal "ready"? For a scoring goal this is typically
         * "superstructure in position AND shooter at speed AND aligned." Used
         * for driver feedback (rumble, LEDs) and the {@code Ready} telemetry.
         * Optional.
         */
        public Builder readyWhen(BooleanSupplier readinessTest) {
            this.ready = readinessTest;
            return this;
        }

        /** Build the immutable goal. */
        public Goal build() {
            if (name == null || name.isEmpty()) {
                throw new IllegalArgumentException("Goal needs a name");
            }
            return new Goal(name, superstructureState, setup, ready);
        }
    }
}
