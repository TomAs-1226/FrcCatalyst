package frc.lib.catalyst.behavior;

import frc.lib.catalyst.command.CatalystCommand;
import frc.lib.catalyst.logging.CatalystLog;
import org.wpilib.command3.Command;
import frc.lib.catalyst.command.Commands;
import org.wpilib.command3.Mechanism;

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
    private final Action acquire;
    private final Action score;
    private final BooleanSupplier hasPiece;
    private final Set<Mechanism> requirements;
    /** Key prefix under CatalystLog's root, e.g. {@code Behavior/Cycle/} - the path is unchanged. */
    private final String key;
    /** Last value written to Phase. Skipping unchanged writes keeps this off the loop's budget. */
    private String lastPhase;

    private Autopilot(Builder b) {
        this.name = b.name;
        this.acquire = b.acquire;
        this.score = b.score;
        this.hasPiece = b.hasPiece;
        this.requirements = new HashSet<>();
        this.requirements.addAll(acquire.requirements());
        this.requirements.addAll(score.requirements());
        this.key = "Behavior/" + name + "/";
    }

    /**
     * The co-pilot command. Bind with {@code whileTrue(...)} so it engages
     * while a button is held and cancels cleanly on release.
     */
    public CatalystCommand run() {
        Command step = Commands.defer(() -> {
            boolean holding = safeHasPiece();
            Action chosen = holding ? score : acquire;

            // An Action carries a precondition and this used to ignore it, alone among everything
            // that consumes one. An acquire that cannot succeed - no piece in view, a camera down -
            // was scheduled anyway and the repeating sequence ran it forever, so the driver was
            // locked out of the drivetrain until they noticed and released the button, with nothing
            // on the dashboard saying why.
            if (!safeCanStart(chosen)) {
                publishPhase("Stalled: " + chosen.name() + " cannot start");
                // Hold the requirements and wait. Returning a finished command instead would let
                // repeatingSequence re-defer at loop rate, allocating a command every 20 ms; and
                // dropping the requirements here would hand the drivetrain back mid-cycle, which
                // is a different surprise. Resumes the moment the precondition clears, or the
                // moment the piece state flips and the other phase becomes the right one.
                return Commands.waitUntil(() -> safeCanStart(chosen) || safeHasPiece() != holding)
                        .withName("Autopilot:" + name + "/stalled");
            }

            publishPhase(holding ? "Score" : "Acquire");
            return chosen.toCommand();
        }, requirements);

        return Commands.repeatingSequence(step)
                .beforeStarting(() -> publishPhase("Engaged"))
                .finallyDo(interrupted -> publishPhase("DriverControl"))
                .withName("Autopilot:" + name);
    }

    /** Publish only on change: Phase is stable for seconds at a time and NT writes are not free. */
    private void publishPhase(String phase) {
        if (phase.equals(lastPhase)) {
            return;
        }
        lastPhase = phase;
        CatalystLog.log(key + "Phase", phase);
    }

    private boolean safeHasPiece() {
        try {
            return hasPiece.getAsBoolean();
        } catch (Throwable t) {
            return false;
        }
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

        public Autopilot build() {
            if (acquire == null || score == null) {
                throw new IllegalStateException("Autopilot needs both an acquire and a score action");
            }
            CatalystFeatures.record(CatalystFeatures.AUTOPILOT, name);
            return new Autopilot(this);
        }
    }
}
