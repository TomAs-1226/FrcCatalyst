package frc.lib.catalyst.autonomy;

import java.util.List;
import java.util.Optional;

/**
 * Shared control of a swerve drive: the driver drives, and assists help with the parts a person
 * does badly under pressure - holding a heading, lining up on something - without taking the robot
 * away. New in Autonomy 2.1.
 *
 * <p>Like every Autonomy core it decides and never commands. Each loop it takes the driver's
 * command and the assists' <em>proposals</em>, and returns one {@link Decision}: the velocity to
 * send, who owns the heading, how much authority they have, and why. Sending it is the robot's
 * code.
 *
 * <h2>The rules it enforces</h2>
 *
 * These are what "not intrusive" means here, and each has a test:
 *
 * <ol>
 *   <li><b>The driver's stick always wins.</b> While the driver turns, the rotation is theirs, the
 *       same loop. A heading assist that {@linkplain HeadingProposal#waitsForRest() waits for
 *       rest} comes back only after the stick has rested {@link Config#restSeconds} <em>and</em>
 *       the robot has stopped rotating - so it never undoes the end of the driver's turn.
 *   <li><b>An assist never makes the robot faster</b> than the driver asked, and never sends it
 *       against the driver's direction. A translation proposal that would is cut down or dropped.
 *   <li><b>No steps.</b> A heading assist's authority ramps from zero over {@link
 *       Config#rampSeconds} each time it takes over.
 *   <li><b>One owner of the heading at a time</b>: the first proposal in the list that may act.
 *       The order of the list is the priority.
 *   <li><b>Bounded.</b> A heading assist's turn rate is capped by its own proposal.
 * </ol>
 *
 * <p>Thresholds that switch a proposal on and off belong to whoever makes the proposal, and should
 * have hysteresis. Smoothing the resulting velocity is {@link VelocityLimiter}'s job, after this.
 *
 * <p>Angles are radians, counter-clockwise positive; velocities are on the field, m/s.
 */
public final class SharedControl {

    /** The timing of the rules. */
    public static final class Config {
        /** How long the turn stick must rest before a waiting heading assist may act, s. */
        public double restSeconds = 0.3;
        /** The yaw rate under which the robot counts as no longer turning, rad/s. */
        public double settledRadps = 0.3;
        /** How long a heading assist takes to reach full authority after taking over, s. */
        public double rampSeconds = 0.25;
    }

    /**
     * What the driver asks this loop.
     *
     * @param vx field velocity, m/s
     * @param vy field velocity, m/s
     * @param omega the turn the driver asks, rad/s; used only while {@code turning}
     * @param turning whether the turn stick is off its deadband
     */
    public record Driver(double vx, double vy, double omega, boolean turning) {}

    /**
     * An assist's offer to hold the heading.
     *
     * @param source its name, for the board ("HOLD", "BUMP", "AIM")
     * @param targetRad the heading to hold; NaN means "the heading the robot has when I take over"
     * @param kP rad/s of turn per radian of error
     * @param maxRateRadps the fastest it may turn the robot
     * @param waitsForRest whether it waits for the stick to rest and the robot to stop rotating
     *     after the driver turns (a hold should; an aim the driver asked for need not)
     * @param external whether the robot's own code drives the heading when this owns it (an aim
     *     with its own controller); the decision then names it and commands no turn itself
     * @param reason why it is offered, for the board
     */
    public record HeadingProposal(String source, double targetRad, double kP, double maxRateRadps,
                                  boolean waitsForRest, boolean external, String reason) {

        /** A plain hold of the heading the robot has when the assist takes over. */
        public static HeadingProposal hold(String source, double kP, double maxRateRadps, String reason) {
            return new HeadingProposal(source, Double.NaN, kP, maxRateRadps, true, false, reason);
        }

        /** A turn to a given heading, once the stick has rested. */
        public static HeadingProposal to(String source, double targetRad, double kP, double maxRateRadps,
                                         String reason) {
            return new HeadingProposal(source, targetRad, kP, maxRateRadps, true, false, reason);
        }

        /** An owner that drives the heading itself, the moment the stick is at rest. */
        public static HeadingProposal external(String source, String reason) {
            return new HeadingProposal(source, Double.NaN, 0.0, 0.0, false, true, reason);
        }
    }

    /**
     * An assist's offer to adjust the direction of travel.
     *
     * @param source its name, for the board
     * @param vx the field velocity it would rather the robot had, m/s
     * @param vy the field velocity it would rather the robot had, m/s
     * @param reason why, for the board
     */
    public record TranslationProposal(String source, double vx, double vy, String reason) {}

    /**
     * What to send this loop.
     *
     * @param vx field velocity, m/s
     * @param vy field velocity, m/s
     * @param omega turn rate, rad/s; zero when {@code external}
     * @param headingOwner "DRIVER" while the driver turns, an assist's source, or "NONE"
     * @param external the heading owner drives the heading itself; send nothing for the turn
     * @param authority the heading owner's authority, 0 to 1 (1 for the driver)
     * @param headingTargetRad the heading being held, NaN when there is none
     * @param translationOwner "DRIVER", or the source of the translation proposal applied
     * @param reason one line, for the board and the log
     */
    public record Decision(double vx, double vy, double omega, String headingOwner, boolean external,
                           double authority, double headingTargetRad, String translationOwner,
                           String reason) {}

    /** No heading owner. */
    public static final String NONE = "NONE";

    /** The driver. */
    public static final String DRIVER = "DRIVER";

    private final Config config;
    private double lastNow = Double.NaN;
    private double restSince = Double.NaN;
    private boolean rested;
    private String owner = NONE;
    private double authority;
    private double heldTarget = Double.NaN;

    public SharedControl(Config config) {
        this.config = config;
    }

    public Config config() {
        return config;
    }

    /**
     * Forget everything held: call when the pose's heading is reset, and when the drive command
     * starts. A heading held across a reset is a target in a frame that no longer exists.
     */
    public void reset() {
        lastNow = Double.NaN;
        restSince = Double.NaN;
        rested = false;
        owner = NONE;
        authority = 0.0;
        heldTarget = Double.NaN;
    }

    /**
     * One loop.
     *
     * @param now the clock, s
     * @param driver what the driver asks
     * @param headingRad the robot's heading
     * @param yawRateRadps the robot's measured yaw rate
     * @param headings the heading assists that offer this loop, highest priority first
     * @param translation the translation assist that offers this loop, if any
     */
    public Decision decide(double now, Driver driver, double headingRad, double yawRateRadps,
                           List<HeadingProposal> headings, Optional<TranslationProposal> translation) {
        double dt = Double.isNaN(lastNow) ? 0.0 : Math.clamp(now - lastNow, 0.0, 0.1);
        lastNow = now;

        // ---- translation: the driver's, or an assist's within rule 2
        double vx = driver.vx();
        double vy = driver.vy();
        String translationOwner = DRIVER;
        String translationReason = "";
        if (translation.isPresent()) {
            TranslationProposal p = translation.get();
            double asked = Math.hypot(driver.vx(), driver.vy());
            double offered = Math.hypot(p.vx(), p.vy());
            boolean finite = Double.isFinite(p.vx()) && Double.isFinite(p.vy());
            boolean sameWay = p.vx() * driver.vx() + p.vy() * driver.vy() > 0.0;
            if (finite && asked > 0.0 && offered > 0.0 && sameWay) {
                double scale = Math.min(1.0, asked / offered);
                vx = p.vx() * scale;
                vy = p.vy() * scale;
                translationOwner = p.source();
                translationReason = p.reason();
            }
        }

        // ---- heading, rule 1: the driver's stick wins, the same loop
        if (driver.turning()) {
            restSince = Double.NaN;
            rested = false;
            owner = DRIVER;
            authority = 1.0;
            heldTarget = Double.NaN;
            return new Decision(vx, vy, driver.omega(), DRIVER, false, 1.0, Double.NaN, translationOwner,
                    join("driver turning", translationReason));
        }
        if (Double.isNaN(restSince)) {
            restSince = now;
        }
        if (!rested && now - restSince >= config.restSeconds && Math.abs(yawRateRadps) <= config.settledRadps) {
            rested = true;
        }

        // ---- rule 4: the first proposal that may act owns the heading
        HeadingProposal chosen = null;
        for (HeadingProposal p : headings) {
            if (p != null && (!p.waitsForRest() || rested)) {
                chosen = p;
                break;
            }
        }
        if (chosen == null) {
            owner = NONE;
            authority = 0.0;
            heldTarget = Double.NaN;
            String why = headings.isEmpty() ? "no heading assist" : "waiting for the turn to settle";
            return new Decision(vx, vy, 0.0, NONE, false, 0.0, Double.NaN, translationOwner,
                    join(why, translationReason));
        }

        // ---- rule 3: a new owner starts from nothing and ramps in
        if (!chosen.source().equals(owner)) {
            owner = chosen.source();
            authority = 0.0;
            heldTarget = Double.isNaN(chosen.targetRad()) ? headingRad : chosen.targetRad();
        } else if (!Double.isNaN(chosen.targetRad())) {
            heldTarget = chosen.targetRad();
        }
        authority = config.rampSeconds <= 0.0 ? 1.0 : Math.min(1.0, authority + dt / config.rampSeconds);

        if (chosen.external()) {
            return new Decision(vx, vy, 0.0, owner, true, 1.0, Double.NaN, translationOwner,
                    join(chosen.reason(), translationReason));
        }

        // ---- rule 5: bounded
        double error = wrap(heldTarget - headingRad);
        double limit = Math.abs(chosen.maxRateRadps());
        double omega = Math.clamp(chosen.kP() * error, -limit, limit) * authority;
        if (!Double.isFinite(omega)) {
            omega = 0.0;
        }
        return new Decision(vx, vy, omega, owner, false, authority, heldTarget, translationOwner,
                join(chosen.reason(), translationReason));
    }

    private static String join(String a, String b) {
        return b == null || b.isEmpty() ? a : a + "; " + b;
    }

    static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
