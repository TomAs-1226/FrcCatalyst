package frc.lib.catalyst.physics.diagnostics;

import java.util.Locale;
import java.util.function.BooleanSupplier;

/**
 * Tells a stalled mechanism from a mechanism that just did its job.
 *
 * <p>The raw signature of a jam is easy: the motor is being told to move, it is pulling current, and
 * nothing is turning. The hard part is that <em>a successful intake looks exactly the same</em>. A
 * roller that grabs a game piece stalls against it, draws current, and stops — and burning it out
 * because the code called that a jam is as bad as ignoring a real one.
 *
 * <p>The two are not distinguishable from motor signals alone, and this class does not pretend
 * otherwise. It separates them the only honest way: ask whether the piece is there.
 *
 * <ul>
 *   <li>Stalled, and the piece sensor says a piece is held → <b>acquired</b>. Expected, benign.</li>
 *   <li>Stalled, and no piece → <b>jammed</b>. Something is wrong.</li>
 *   <li>No piece sensor configured → every stall reports as {@link State#STALLED}, and the caller
 *       decides. Better an honest "it is stalled" than a guess dressed up as a diagnosis.</li>
 * </ul>
 *
 * <pre>{@code
 * JamDetector intake = JamDetector.builder("Intake")
 *     .stallCurrentAmps(35)
 *     .stallVelocity(2.0)               // rot/s below which it counts as stopped
 *     .holdSeconds(0.25)
 *     .pieceHeld(intakeMech::hasPiece)  // the sensor that tells the two apart
 *     .build();
 *
 * intake.update(now, roller.getStatorCurrent(), roller.getVelocity(), roller.isCommanded());
 *
 * if (intake.state() == JamDetector.State.JAMMED) { intake.reverse().schedule(); }
 * }</pre>
 *
 * <p>All three conditions must hold together for {@link Builder#holdSeconds(double)}, so a momentary
 * current spike on start-up does not trip it. The clock is a parameter, so this is fully testable with
 * no HAL and no robot.
 *
 * @since 1.6.0
 */
public final class JamDetector {

    /** What the mechanism appears to be doing. */
    public enum State {
        /** Moving, or not being asked to move. Nothing to report. */
        OK,
        /** Stalled against something, and a piece is held — an intake that succeeded. */
        ACQUIRED,
        /** Stalled against something, and no piece is held — a jam. */
        JAMMED,
        /** Stalled, with no piece sensor configured to say which of the two it is. */
        STALLED
    }

    private final String name;
    private final double stallCurrentAmps;
    private final double stallVelocity;
    private final double holdSeconds;
    private final BooleanSupplier pieceHeld;

    private double stallStartedAt = Double.NaN;
    private State state = State.OK;
    private double lastTimestamp = Double.NaN;

    private JamDetector(Builder builder) {
        this.name = builder.name;
        this.stallCurrentAmps = builder.stallCurrentAmps;
        this.stallVelocity = builder.stallVelocity;
        this.holdSeconds = builder.holdSeconds;
        this.pieceHeld = builder.pieceHeld;
    }

    /**
     * Evaluate one loop's signals.
     *
     * @param timestampSeconds current FPGA-clock time
     * @param statorCurrentAmps current the motor is drawing
     * @param velocity          measured mechanism velocity, in whatever units {@code stallVelocity} used
     * @param commanded         whether the mechanism is actually being told to move. Without this a
     *                          mechanism resting at zero output reads as stalled forever
     * @return the state after this update
     */
    public State update(double timestampSeconds, double statorCurrentAmps, double velocity,
                        boolean commanded) {
        lastTimestamp = timestampSeconds;
        boolean stalling = commanded
                && Math.abs(statorCurrentAmps) >= stallCurrentAmps
                && Math.abs(velocity) <= stallVelocity;

        if (!stalling) {
            stallStartedAt = Double.NaN;
            state = State.OK;
            return state;
        }

        if (Double.isNaN(stallStartedAt)) stallStartedAt = timestampSeconds;
        if (timestampSeconds - stallStartedAt < holdSeconds) {
            // Stalling, but not for long enough to call it — a start-up current spike looks like this.
            state = State.OK;
            return state;
        }

        if (pieceHeld == null) {
            state = State.STALLED;
        } else {
            state = pieceHeld.getAsBoolean() ? State.ACQUIRED : State.JAMMED;
        }
        return state;
    }

    /** The current verdict. */
    public State state() {
        return state;
    }

    /** True only for a genuine jam — never for a successful acquisition. */
    public boolean isJammed() {
        return state == State.JAMMED;
    }

    /** True when the mechanism stalled with a piece held: the intake worked. */
    public boolean hasAcquired() {
        return state == State.ACQUIRED;
    }

    /** How long the current stall has lasted, in seconds. Zero when not stalling. */
    public double stallDurationSeconds() {
        if (Double.isNaN(stallStartedAt) || Double.isNaN(lastTimestamp)) return 0.0;
        return Math.max(0.0, lastTimestamp - stallStartedAt);
    }

    /** The mechanism this detector watches. */
    public String name() {
        return name;
    }

    /** Clear the detector back to {@link State#OK}. */
    public void reset() {
        stallStartedAt = Double.NaN;
        state = State.OK;
    }

    /** One line for a log or an alert. */
    public String describe() {
        return switch (state) {
            case OK -> name + ": running normally";
            case ACQUIRED -> String.format(Locale.ROOT, "%s: piece acquired (stalled %.2f s)",
                    name, stallDurationSeconds());
            case JAMMED -> String.format(Locale.ROOT, "%s: JAMMED - stalled %.2f s with no piece",
                    name, stallDurationSeconds());
            case STALLED -> String.format(Locale.ROOT,
                    "%s: stalled %.2f s - no piece sensor configured, cannot tell jam from acquisition",
                    name, stallDurationSeconds());
        };
    }

    /** Start configuring a detector for a named mechanism. */
    public static Builder builder(String name) {
        return new Builder(name);
    }

    /** Builder for {@link JamDetector}. */
    public static final class Builder {
        private final String name;
        private double stallCurrentAmps = 30.0;
        private double stallVelocity = 1.0;
        private double holdSeconds = 0.25;
        private BooleanSupplier pieceHeld;

        Builder(String name) {
            this.name = name;
        }

        /**
         * Current at or above which the motor counts as pushing hard, in amps. Defaults to 30. Set it
         * above what the mechanism draws while moving freely and below its stall current.
         */
        public Builder stallCurrentAmps(double stallCurrentAmps) {
            this.stallCurrentAmps = stallCurrentAmps;
            return this;
        }

        /**
         * Velocity at or below which the mechanism counts as stopped, in its own units. Defaults to
         * 1.0. Leave a little room for encoder noise and a slipping-but-turning roller.
         */
        public Builder stallVelocity(double stallVelocity) {
            this.stallVelocity = stallVelocity;
            return this;
        }

        /**
         * How long all three conditions must hold before reporting, in seconds. Defaults to 0.25 —
         * long enough to ride out the current spike every mechanism draws when it starts moving.
         */
        public Builder holdSeconds(double holdSeconds) {
            this.holdSeconds = holdSeconds;
            return this;
        }

        /**
         * The piece sensor that separates a successful intake from a jam. Without it, every stall
         * reports as {@link State#STALLED} and the caller decides.
         */
        public Builder pieceHeld(BooleanSupplier pieceHeld) {
            this.pieceHeld = pieceHeld;
            return this;
        }

        /** Validate and build. */
        public JamDetector build() {
            if (name == null || name.isBlank()) {
                throw new IllegalStateException("a mechanism name is required");
            }
            if (!(stallCurrentAmps > 0)) {
                throw new IllegalStateException("stallCurrentAmps must be > 0 (got "
                        + stallCurrentAmps + ")");
            }
            if (stallVelocity < 0) {
                throw new IllegalStateException("stallVelocity must be >= 0 (got " + stallVelocity + ")");
            }
            if (holdSeconds < 0) {
                throw new IllegalStateException("holdSeconds must be >= 0 (got " + holdSeconds + ")");
            }
            return new JamDetector(this);
        }
    }
}
