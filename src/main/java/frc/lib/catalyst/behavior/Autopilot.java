package frc.lib.catalyst.behavior;

import frc.lib.catalyst.command.CatalystCommand;
import frc.lib.catalyst.logging.CatalystLog;
import org.wpilib.command3.Command;
import frc.lib.catalyst.command.Commands;
import org.wpilib.command3.Mechanism;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntSupplier;
import java.util.concurrent.atomic.AtomicBoolean;
import frc.lib.catalyst.autonomy.CycleCore;
import org.wpilib.system.Timer;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import frc.lib.catalyst.identity.CatalystFeatures;

/**
 * Teleop cycle co-pilot — the driver holds one button and the robot runs
 * the acquire → score → acquire loop on its own, releasing instantly when
 * the button comes up so the driver can take over.
 *
 * <p>It's a thin, game-agnostic loop over two {@link Action}s and a "do we
 * have a game piece?" supplier:
 * <ul>
 *   <li>no piece → run the <b>acquire</b> action (drive to + intake the
 *       nearest detected piece),</li>
 *   <li>have a piece → run the <b>score</b> action (drive to a scoring spot
 *       and score),</li>
 *   <li>repeat.</li>
 * </ul>
 *
 * <p>The decision is re-evaluated every time the loop comes around, so a
 * piece picked up mid-acquire flips it to scoring on the next pass.
 *
 * <pre>{@code
 * Autopilot copilot = Autopilot.builder()
 *     .name("Cycle")
 *     .acquire(grabNearestPiece)   // Action: pathfind to detected piece + intake
 *     .score(driveAndScore)        // Action: pathfind to open node + score
 *     .hasPiece(claw::hasPiece)
 *     .build();
 *
 * // Hold to engage, release to take over:
 * driver.rightTrigger().whileTrue(copilot.run());
 * }</pre>
 *
 * <p>While engaged, the co-pilot owns the subsystems its actions require
 * (typically the drivetrain + intake/scorer), so the driver's steering is
 * suspended until they release. Pairs naturally with
 * {@link frc.lib.catalyst.util.RumbleEvents} — buzz the driver when a piece
 * is acquired or a score completes.
 *
 * <p>Publishes the current phase to {@code /Catalyst/Behavior/<name>/Phase}.
 */
public final class Autopilot {

    private final String name;
    /** The phases in order. The two-phase sugar builds {@code [acquire, score]}. */
    private final List<Action> phases;
    /** Which phase the robot's own logic wants. For the sugar: holding a piece means score. */
    private final IntSupplier selector;
    private final CycleCore cycle;
    private final boolean handsBack;
    private final Set<Mechanism> requirements;
    /** Key prefix under CatalystLog's root, e.g. {@code Behavior/Cycle/} - the path is unchanged. */
    private final String key;
    /** Last value written to Phase. Skipping unchanged writes keeps this off the loop's budget. */
    private String lastPhase;

    private Autopilot(Builder b) {
        this.name = b.name;
        this.phases = List.copyOf(b.phases);
        this.selector = b.selector;
        this.handsBack = b.stallHandbackSeconds > 0;
        this.cycle = new CycleCore(phases.size(), b.dwellSeconds, b.stallHandbackSeconds);
        this.requirements = new HashSet<>();
        for (Action phase : phases) {
            this.requirements.addAll(phase.requirements());
        }
        this.key = "Behavior/" + name + "/";
    }

    /**
     * The co-pilot command. Bind with {@code whileTrue(...)} so it engages
     * while a button is held and cancels cleanly on release.
     */
    public CatalystCommand run() {
        // Set when the cycle gives up, which is the only way the repeating sequence ends other than
        // the driver releasing the button.
        AtomicBoolean handedBack = new AtomicBoolean(false);

        Command step = Commands.defer(() -> {
            double now = Timer.getTimestamp();
            int desired = safeSelect();
            Action wanted = phases.get(Math.floorMod(desired, phases.size()));

            // The precondition is evaluated here, guarded, and handed to CycleCore as a plain
            // boolean - so the decision stays a pure function and a team's lambda that throws
            // cannot take the co-pilot down with it.
            CycleCore.Decision decision = cycle.decide(now, desired, safeCanStart(wanted));
            Action chosen = phases.get(decision.phase());

            switch (decision.act()) {
                case RUN -> {
                    publishPhase(chosen.name());
                    return chosen.toCommand();
                }
                case HAND_BACK -> {
                    publishPhase("HandingBack: " + chosen.name() + " " + decision.reason());
                    handedBack.set(true);
                    return Commands.none();
                }
                default -> {
                    publishPhase("Stalled: " + chosen.name() + " cannot start");
                    // Hold the requirements and wait rather than returning a finished command,
                    // which would let repeatingSequence re-defer at loop rate and allocate a fresh
                    // command every 20 ms. Wakes as soon as the picture changes in any way that
                    // could matter, so the poll costs one guarded call per loop and nothing else.
                    return Commands.waitUntil(() -> safeCanStart(chosen) || safeSelect() != decision.phase()
                                    || (handsBack && cycle.stalledSeconds(Timer.getTimestamp()) > 0))
                            .withName("Autopilot:" + name + "/stalled");
                }
            }
        }, requirements);

        return Commands.repeatingSequence(step)
                .untilTrue(handedBack::get)
                .beforeStarting(() -> {
                    cycle.reset();
                    handedBack.set(false);
                    publishPhase("Engaged");
                })
                .finallyDo(interrupted -> publishPhase("DriverControl"))
                .withName("Autopilot:" + name);
    }

    /** The team's phase choice. A selector that throws leaves the cycle where it is. */
    private int safeSelect() {
        try {
            return selector.getAsInt();
        } catch (Throwable t) {
            return cycle.phase();
        }
    }

    /** Publish only on change: Phase is stable for seconds at a time and NT writes are not free. */
    private void publishPhase(String phase) {
        if (phase.equals(lastPhase)) {
            return;
        }
        lastPhase = phase;
        CatalystLog.log(key + "Phase", phase);
    }

    /** A precondition is team code. It failing must not take the co-pilot down with it. */
    private boolean safeCanStart(Action action) {
        try {
            return action.canStart();
        } catch (Throwable t) {
            return false;
        }
    }

    // ============================================================
    //                        BUILDER
    // ============================================================

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private String name = "Autopilot";
        private Action acquire;
        private Action score;
        private BooleanSupplier hasPiece = () -> false;
        private final List<Action> phases = new ArrayList<>();
        private IntSupplier selector;
        private double dwellSeconds;
        private double stallHandbackSeconds;

        /** NT subtable name under /Catalyst/Behavior/. Default "Autopilot". */
        public Builder name(String name) {
            this.name = name;
            return this;
        }

        /** Action that gets a game piece (drive to + intake the nearest one). */
        public Builder acquire(Action acquire) {
            this.acquire = acquire;
            return this;
        }

        /** Action that scores a held piece (drive to a scoring spot + score). */
        public Builder score(Action score) {
            this.score = score;
            return this;
        }

        /** How to tell whether the robot currently holds a piece. */
        public Builder hasPiece(BooleanSupplier hasPiece) {
            this.hasPiece = hasPiece;
            return this;
        }

        /**
         * How long a phase change must be asked for before it happens, in seconds. Default 0, which
         * is what this has always done.
         *
         * <p>Worth setting to something like 0.15 whenever the phase is chosen by a sensor with a
         * threshold. A beam break or a current-spike detector sitting right on its trip point flips
         * the answer as fast as the loop runs, and without dwell that alternates the two phases
         * against each other so neither ever gets anywhere.
         */
        public Builder dwellSeconds(double seconds) {
            this.dwellSeconds = seconds;
            return this;
        }

        /**
         * Give the robot back to the driver after the cycle has been unable to start for this long,
         * in seconds. Default 0, meaning never - the behaviour every existing robot has.
         *
         * <p>The case for setting it: the co-pilot holds the drivetrain while it is engaged, so a
         * phase whose precondition can never be met is a driver who has lost their robot and has to
         * work out why. With this set they get it back and the dashboard says what happened.
         */
        public Builder handBackAfterStalled(double seconds) {
            this.stallHandbackSeconds = seconds;
            return this;
        }

        /**
         * A cycle of more than two phases, run in the order given, with {@code selector} choosing
         * which one the robot wants right now.
         *
         * <p>Separate from {@link #acquire}/{@link #score} rather than replacing them: the two-phase
         * form is most of what teams need and its build-time check that both are present is worth
         * keeping. Using this form replaces that pair.
         *
         * @param selector returns the index of the wanted phase; out-of-range values wrap
         * @param phases   at least one action, in cycle order
         */
        public Builder phases(IntSupplier selector, Action... phases) {
            if (phases == null || phases.length == 0) {
                throw new IllegalArgumentException("a cycle needs at least one phase");
            }
            this.selector = selector;
            this.phases.clear();
            this.phases.addAll(List.of(phases));
            return this;
        }

        public Autopilot build() {
            if (phases.isEmpty()) {
                // The two-phase form. Its check stays exactly as strict as it was.
                if (acquire == null || score == null) {
                    throw new IllegalStateException("Autopilot needs both an acquire and a score action");
                }
                phases.add(acquire);
                phases.add(score);
                BooleanSupplier holding = hasPiece;
                // Holding a piece means score; otherwise go and get one. The guard lives in
                // Autopilot.safeSelect, so a supplier that throws leaves the cycle where it is.
                selector = () -> holding.getAsBoolean() ? 1 : 0;
            } else if (selector == null) {
                throw new IllegalStateException("a multi-phase Autopilot needs a selector");
            }
            CatalystFeatures.record(CatalystFeatures.AUTOPILOT, name);
            return new Autopilot(this);
        }
    }
}
