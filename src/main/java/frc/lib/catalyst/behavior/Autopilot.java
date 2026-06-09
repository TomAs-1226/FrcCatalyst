package frc.lib.catalyst.behavior;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

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
    private final Set<Subsystem> requirements;
    private final NetworkTable nt;

    private Autopilot(Builder b) {
        this.name = b.name;
        this.acquire = b.acquire;
        this.score = b.score;
        this.hasPiece = b.hasPiece;
        this.requirements = new HashSet<>();
        this.requirements.addAll(acquire.requirements());
        this.requirements.addAll(score.requirements());
        this.nt = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("Behavior").getSubTable(name);
    }

    /**
     * The co-pilot command. Bind with {@code whileTrue(...)} so it engages
     * while a button is held and cancels cleanly on release.
     */
    public Command run() {
        Command step = Commands.defer(() -> {
            boolean holding = safeHasPiece();
            if (holding) {
                nt.getEntry("Phase").setString("Score");
                return score.toCommand();
            } else {
                nt.getEntry("Phase").setString("Acquire");
                return acquire.toCommand();
            }
        }, requirements);

        return Commands.repeatingSequence(step)
                .beforeStarting(() -> nt.getEntry("Phase").setString("Engaged"))
                .finallyDo(interrupted -> nt.getEntry("Phase").setString("DriverControl"))
                .withName("Autopilot:" + name);
    }

    private boolean safeHasPiece() {
        try {
            return hasPiece.getAsBoolean();
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
            return new Autopilot(this);
        }
    }
}
