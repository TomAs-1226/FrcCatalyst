package frc.lib.catalyst.util;

import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.system.Timer;

import java.util.function.BooleanSupplier;

/**
 * Whether the robot is currently a sniper or a spray-and-pray.
 *
 * <p>Four postures, named after the guns that behave the way each one does:
 *
 * <ul>
 *   <li><b>AWP</b> — lining a shot up. Something is being aimed, the robot cares more about where
 *       the shot goes than about how soon it goes, and the interesting question on a dashboard is
 *       "are we locked yet".
 *   <li><b>MAC-10</b> — shots are leaving faster than anybody is aiming them. Volume over placement.
 *   <li><b>RELOADING</b> — taking a game piece in. Not a threat right now, and the driver coach
 *       wants to know how long it lasts.
 *   <li><b>HOLSTERED</b> — none of the above. Not shooting, not aiming, not intaking.
 * </ul>
 *
 * <p><b>It is a joke that turned out to be useful.</b> The name is the fun part; what it publishes
 * is a genuine answer to a question a driver coach asks out loud every match — <em>are we taking
 * careful shots or dumping them?</em> — and which otherwise has to be reconstructed by watching a
 * turret angle and a feed motor at the same time. One string on a dashboard says it, and the
 * transition between the two is exactly the moment a match strategy changed.
 *
 * <p>Nothing here controls anything. It observes, classifies and publishes, so wiring it up
 * wrongly cannot make the robot behave differently — the worst case is a dashboard that lies about
 * a mode, and it is deliberately built so that the classification degrades to {@code HOLSTERED}
 * rather than to a confident wrong answer.
 *
 * <h2>Wiring it up</h2>
 *
 * <pre>{@code
 * private final FireMode fire = FireMode.builder()
 *         .aiming(turret::isTracking)                       // something is being pointed
 *         .locked(() -> turret.isOnTarget(solution, heading, 2.0))
 *         .intaking(() -> intakeTrigger.getAsBoolean())     // going to get one
 *         .build();
 *
 * // In the shot command, wherever a game piece actually leaves:
 * feeder.feedVoltage(8.0).beforeStarting(fire::recordShot);
 *
 * // Once a loop:
 * fire.update();
 * }</pre>
 *
 * <p>Every supplier is optional. With none of them wired the robot can still reach MAC-10 from shot
 * cadence alone, which is the mode that matters most anyway.
 *
 * @since 2.0.0
 */
public final class FireMode {

    /** The posture a robot is currently shooting in. */
    public enum Posture {
        /** Not shooting, not aiming, not intaking. */
        HOLSTERED("HOLSTERED"),

        /** Lining one up. Precision matters more than cadence. */
        AWP("AWP"),

        /** Shots leaving faster than anyone is aiming them. */
        MAC_10("MAC-10"),

        /** Taking a piece in. Empty, and doing something about it. */
        RELOADING("RELOADING");

        private final String display;

        Posture(String display) {
            this.display = display;
        }

        /** The name to put in front of a human. {@code MAC-10}, not {@code MAC_10}. */
        public String display() {
            return display;
        }
    }

    /** Key everything here is published under. */
    private static final String KEY = "FireMode/";

    /**
     * How recently a shot has to have happened for the robot to still count as shooting.
     *
     * <p>Two seconds rather than something tighter because the posture is about intent, not about
     * the instant: a robot taking one careful shot every three seconds is not spraying, and a
     * dashboard that flickered back to HOLSTERED between two rapid shots would be unreadable.
     */
    private static final double SHOOTING_WINDOW_SECONDS = 2.0;

    /**
     * Shots per second at or above which the posture is MAC-10 regardless of what is being aimed.
     *
     * <p>Two per second. Below that a robot is placing shots even if it never stops; above it,
     * nobody is aiming anything, whatever the turret happens to be doing. The rate is measured
     * across the whole window rather than from the last gap, so a single fast pair does not flip
     * the mode.
     */
    private static final double SPRAY_SHOTS_PER_SECOND = 2.0;

    /** How many shot timestamps to keep. One window at spray rate, with room to spare. */
    private static final int HISTORY = 16;

    private final BooleanSupplier aiming;
    private final BooleanSupplier locked;
    private final BooleanSupplier intaking;

    private final double[] shots = new double[HISTORY];
    private int shotCount = 0;
    private int next = 0;

    private Posture posture = Posture.HOLSTERED;
    private Posture previous = Posture.HOLSTERED;

    private FireMode(BooleanSupplier aiming, BooleanSupplier locked, BooleanSupplier intaking) {
        this.aiming = aiming;
        this.locked = locked;
        this.intaking = intaking;
    }

    /** Start building one. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builds a {@link FireMode}. Every signal is optional. */
    public static final class Builder {
        private BooleanSupplier aiming = () -> false;
        private BooleanSupplier locked = () -> false;
        private BooleanSupplier intaking = () -> false;

        /** True while something is being deliberately pointed at a target. */
        public Builder aiming(BooleanSupplier aiming) {
            this.aiming = aiming == null ? () -> false : aiming;
            return this;
        }

        /** True while the aim is good enough to take the shot. */
        public Builder locked(BooleanSupplier locked) {
            this.locked = locked == null ? () -> false : locked;
            return this;
        }

        /**
         * True while a game piece is being taken in.
         *
         * <p>{@code roller::isIntaking} if the mechanism exposes it, otherwise the trigger the
         * driver holds. Note that {@code RollerMechanism.hasPiece()} is the wrong signal — that is
         * true once the piece is <em>in</em>, which is the moment reloading stops.
         */
        public Builder intaking(BooleanSupplier intaking) {
            this.intaking = intaking == null ? () -> false : intaking;
            return this;
        }

        /** Build it. */
        public FireMode build() {
            return new FireMode(aiming, locked, intaking);
        }
    }

    /**
     * Record that a game piece just left the robot.
     *
     * <p>Call this where the piece actually goes, not where the button is pressed — a held trigger
     * is one press and many shots, and the cadence is the whole input to the classification.
     */
    public synchronized void recordShot() {
        shots[next] = Timer.getTimestamp();
        next = (next + 1) % HISTORY;
        if (shotCount < HISTORY) shotCount++;
    }

    /**
     * Reclassify and publish. Call once a loop.
     *
     * <p>Safe to call when nothing is wired up, in simulation, and before any shot has been
     * recorded; it answers {@code HOLSTERED} and publishes that.
     */
    public synchronized void update() {
        previous = posture;
        posture = classify();

        CatalystLog.log(KEY + "Mode", posture.display());
        CatalystLog.log(KEY + "ShotsPerSecond", shotsPerSecond());
        CatalystLog.log(KEY + "Locked", posture == Posture.AWP && safely(locked));
        CatalystLog.log(KEY + "Reloading", posture == Posture.RELOADING);
    }

    /** The current posture. */
    public synchronized Posture current() {
        return posture;
    }

    /** True on the loop the posture changed — the moment worth noticing. */
    public synchronized boolean justChanged() {
        return posture != previous;
    }

    /** Shots per second over the recent window. Zero when nothing has been fired. */
    public synchronized double shotsPerSecond() {
        double now = Timer.getTimestamp();
        int recent = 0;
        for (int i = 0; i < shotCount; i++) {
            if (now - shots[i] <= SHOOTING_WINDOW_SECONDS) recent++;
        }
        return recent / SHOOTING_WINDOW_SECONDS;
    }

    /** Forget the shot history. For a mode change, or between matches. */
    public synchronized void reset() {
        shotCount = 0;
        next = 0;
        posture = Posture.HOLSTERED;
        previous = Posture.HOLSTERED;
    }

    private Posture classify() {
        // Cadence wins. A robot putting shots out at spray rate is spraying whatever its turret is
        // nominally doing - and a turret that is still "tracking" while four shots a second leave
        // is describing an intention, not the behaviour anyone is watching.
        if (shotsPerSecond() >= SPRAY_SHOTS_PER_SECOND) {
            return Posture.MAC_10;
        }
        // Reloading sits under spraying and above everything else, which is the ordering a
        // through-path robot needs: one that intakes and feeds continuously is spraying, and the
        // cadence says so, but a robot that has stopped shooting to go and collect is reloading
        // even if the turret is still nominally pointed at something. Intent beats aim here in the
        // same way cadence beats intent above it.
        if (safely(intaking)) {
            return Posture.RELOADING;
        }
        if (safely(aiming)) {
            return Posture.AWP;
        }
        // Shooting slowly without an aiming signal wired up is still deliberate shooting, and AWP is
        // the honest description of it. Without a recent shot either, there is nothing happening.
        return hasShotRecently() ? Posture.AWP : Posture.HOLSTERED;
    }

    private boolean hasShotRecently() {
        double now = Timer.getTimestamp();
        for (int i = 0; i < shotCount; i++) {
            if (now - shots[i] <= SHOOTING_WINDOW_SECONDS) return true;
        }
        return false;
    }

    /**
     * Read a supplier without letting it take the robot down.
     *
     * <p>These come from team code and typically reach into a turret and a vision solution. This is
     * a dashboard readout; a null pose inside somebody's {@code isOnTarget} must not be able to
     * throw out of a periodic loop from here.
     */
    private static boolean safely(BooleanSupplier s) {
        try {
            return s.getAsBoolean();
        } catch (Throwable ignored) {
            return false;
        }
    }
}
