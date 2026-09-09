package frc.lib.catalyst.hardware;

import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

import java.util.OptionalDouble;

/**
 * Two IMUs at known points on the robot, read as one.
 *
 * <p>Systemcore has an IMU built in, and most robots already carry a Pigeon. That is two sensors at
 * two <em>different places</em> on the same rigid body, which is worth more than two sensors in one
 * place: a rigid body constrains how their readings must relate, so disagreement is measurable
 * rather than merely suspected.
 *
 * <h2>What the second sensor actually buys</h2>
 *
 * <p><b>A second opinion on yaw rate.</b> Not, by default, an average. Averaging two gyros only
 * beats the better one when they are of <em>similar</em> quality: the optimal blend weights each by
 * the inverse of its variance, and a plain mean is that formula with the noise assumed equal. A
 * Pigeon 2 and a board-mounted IMU are not equal, so averaging them drags a good rate toward a worse
 * one — which is the opposite of the intent.
 *
 * <p>So {@link #getYawRate()} reports the primary, exactly as heading does, and a team that has
 * <em>measured</em> both sensors can opt into a proper weighting with
 * {@link #withYawRateNoise(double, double)}. The second sensor still earns its place: it shares no
 * failure mode with the first — the Pigeon is on the CAN bus and the Systemcore IMU is not — so the
 * disagreement below catches a failure that a single gyro reports as truth.
 *
 * <p><b>Angular acceleration, measured rather than differentiated.</b> This is the part that is only
 * possible with two. For a rigid body rotating in the plane, a point at offset {@code r} from
 * another sees an extra acceleration:
 *
 * <pre>
 *   a₁ - a₂ = α × r  +  ω × (ω × r)
 * </pre>
 *
 * <p>where {@code r} is the vector between the two sensors. The second term is centripetal and is
 * known once {@code ω} is known, so subtracting it leaves {@code α × r} — and {@code α} follows
 * directly. Differentiating a gyro to get {@code α} amplifies noise badly; this measures it from a
 * difference of two accelerometers instead, which does not.
 *
 * <p><b>A disagreement signal.</b> Two gyros on one rigid body must report the same yaw rate. When
 * they stop agreeing, something is wrong — a sensor has failed, one has come loose, or a mount has
 * shifted. Physics Core is built around exactly this idea, and until now it had no independent
 * heading source to disagree with.
 *
 * <h2>What it does not do</h2>
 *
 * <p>It does not average headings. Yaw is an integrated quantity and the two sensors drift apart at
 * different rates; averaging two drifting integrals produces a third drifting integral with no
 * clearly better error, and hides which one moved. {@link #getYaw()} reports the primary, and
 * {@link #yawDisagreementDegrees()} reports the gap so a caller can decide what it means.
 *
 * <p>The lever arm has to be right. It is measured on the robot, in metres, in robot coordinates
 * (X forward, Y left). A wrong offset produces a confident, wrong angular acceleration — so
 * {@link #angularAccelerationRadPerSecSq()} returns empty rather than guessing when the offset is
 * too small for the difference to mean anything.
 *
 * @since 2.0.0
 */
public final class DualIMU implements CatalystIMU {

    /**
     * Below this separation the two accelerometers see effectively the same motion.
     *
     * <p>Five centimetres is a floor on <em>degeneracy</em>, not a recommendation. The measurement
     * divides the difference of two accelerometers by the lever arm, so the noise it reports is
     * roughly {@code sigma * sqrt(2) / r}: at 5 cm that is about twenty-eight times each sensor's
     * own noise, which is no longer an angular acceleration in any useful sense. Above it the answer
     * degrades smoothly rather than becoming meaningless, so this refuses only the cases where the
     * difference cannot carry signal at all — and {@link #angularAccelerationNoiseGain()} reports
     * what any particular geometry costs.
     *
     * <p>An earlier comment here claimed half a metre. That would have refused a Pigeon and a
     * Systemcore mounted 20 cm apart, which is an entirely ordinary layout and gives a perfectly
     * usable reading.
     */
    private static final double MIN_LEVER_ARM_METERS = 0.05;

    private final CatalystIMU primary;
    private final CatalystIMU secondary;
    private final Translation2d leverArm;

    /**
     * How much of the yaw rate comes from the primary, 0–1.
     *
     * <p>One by default: the primary alone. Anything else has to be earned by measuring both
     * sensors, because a blend chosen by feel is a blend that can only make the better sensor worse.
     */
    private double primaryRateWeight = 1.0;

    /**
     * Combine two IMUs.
     *
     * @param primary       the sensor whose heading is authoritative — normally the Pigeon, which is
     *                      better placed and drifts less
     * @param secondary     the second sensor, normally Systemcore's
     * @param primaryPos    where the primary sits, in robot coordinates, metres
     * @param secondaryPos  where the secondary sits, in robot coordinates, metres
     */
    public DualIMU(CatalystIMU primary, CatalystIMU secondary,
                   Translation2d primaryPos, Translation2d secondaryPos) {
        this.primary = primary;
        this.secondary = secondary;
        this.leverArm = primaryPos.minus(secondaryPos);
    }

    // --- CatalystIMU: the primary is authoritative ---------------------------

    @Override public Rotation2d getHeading() { return primary.getHeading(); }
    @Override public double getYaw()         { return primary.getYaw(); }
    @Override public double getPitch()       { return primary.getPitch(); }
    @Override public double getRoll()        { return primary.getRoll(); }
    @Override public void zeroYaw()          { primary.zeroYaw(); secondary.zeroYaw(); }

    @Override
    public void setYaw(double degrees) {
        primary.setYaw(degrees);
        secondary.setYaw(degrees);
    }

    /**
     * Yaw rate.
     *
     * <p>The primary's, unless {@link #withYawRateNoise(double, double)} has been told what the two
     * sensors' noise actually is. See the class notes: averaging sensors of unequal quality produces
     * something worse than the better one, and the Pigeon is the better one here.
     */
    @Override
    public double getYawRate() {
        if (primaryRateWeight >= 1.0) {
            return primary.getYawRate();
        }
        return primaryRateWeight * primary.getYawRate()
                + (1.0 - primaryRateWeight) * secondary.getYawRate();
    }

    /**
     * Blend the two yaw rates, weighted by how noisy each sensor actually is.
     *
     * <p>For two independent measurements of the same quantity, the lowest-variance combination
     * weights each by the inverse of its variance:
     *
     * <pre>
     *   w₁ = (1/σ₁²) / (1/σ₁² + 1/σ₂²)
     * </pre>
     *
     * <p>A plain average is that with σ₁ = σ₂ assumed. When they differ it is strictly worse than
     * using the better sensor alone, which is why it is not the default.
     *
     * <p>The gain is also smaller than it looks. Two equal sensors buy a factor of {@code 1/√2}; a
     * sensor three times noisier than its partner moves the combined figure by about 5%. If you have
     * not measured, do not set this — the primary alone is the right answer and costs nothing.
     *
     * <p>Measure by holding the robot still and taking the standard deviation of each sensor's
     * reported rate over a few seconds, in degrees per second.
     *
     * @param primaryNoiseDegPerSec   standard deviation of the primary at rest
     * @param secondaryNoiseDegPerSec standard deviation of the secondary at rest
     * @throws IllegalArgumentException if either figure is not positive — a noiseless sensor does not
     *                                  exist, and treating one as noiseless silently discards the other
     */
    public DualIMU withYawRateNoise(double primaryNoiseDegPerSec, double secondaryNoiseDegPerSec) {
        if (!(primaryNoiseDegPerSec > 0) || !(secondaryNoiseDegPerSec > 0)) {
            throw new IllegalArgumentException(
                    "noise must be positive, got " + primaryNoiseDegPerSec
                            + " and " + secondaryNoiseDegPerSec);
        }
        double p = 1.0 / (primaryNoiseDegPerSec * primaryNoiseDegPerSec);
        double q = 1.0 / (secondaryNoiseDegPerSec * secondaryNoiseDegPerSec);
        this.primaryRateWeight = p / (p + q);
        return this;
    }

    /** How much of the yaw rate is coming from the primary, 0–1. One unless you changed it. */
    public double primaryYawRateWeight() {
        return primaryRateWeight;
    }

    @Override
    public String deviceType() {
        return primary.deviceType() + " + " + secondary.deviceType();
    }

    // --- what only two sensors can tell you ----------------------------------

    /**
     * How much this geometry multiplies accelerometer noise, in the angular acceleration it reports.
     *
     * <p>The measurement is a difference of two accelerometers divided by the lever arm, so if each
     * sensor has noise {@code sigma}, the reported angular acceleration carries about
     * {@code sigma * sqrt(2) / r} of it. This is that factor.
     *
     * <p>Worth looking at once, when the sensors are mounted. A robot that can pull 20 rad/s² and a
     * gain of 3 is fine; the same robot with the two IMUs 6 cm apart has a gain near 24, and the
     * reading is mostly noise. Nothing enforces a limit here — the geometry is a fact about the
     * robot, and this is how to find out what it bought.
     *
     * @return the multiplier, or positive infinity when the sensors are too close to measure at all
     */
    public double angularAccelerationNoiseGain() {
        double r = leverArm.getNorm();
        return r < MIN_LEVER_ARM_METERS ? Double.POSITIVE_INFINITY : Math.sqrt(2.0) / r;
    }

    /** How far apart the two sensors are, in metres. */
    public double leverArmMeters() {
        return leverArm.getNorm();
    }

    /**
     * How much the two gyros disagree about yaw rate, in degrees per second.
     *
     * <p>On a rigid body this should sit near zero whatever the robot is doing. A persistent gap
     * means a sensor has failed, drifted badly, or physically moved — and it is visible long before
     * the resulting pose error is.
     */
    public double yawRateDisagreementDegPerSec() {
        return primary.getYawRate() - secondary.getYawRate();
    }

    /**
     * How far the two headings have drifted apart, in degrees.
     *
     * <p>Grows slowly and steadily in normal use, because integrating two slightly different rates
     * produces two slightly different angles. A sudden jump is an impact or a sensor fault, and is
     * the more interesting signal of the two.
     */
    public double yawDisagreementDegrees() {
        return Rotation2d.fromDegrees(primary.getYaw() - secondary.getYaw()).getDegrees();
    }

    /**
     * Angular acceleration about the robot's vertical axis, in rad/s².
     *
     * <p>Solved from the difference between the two accelerometers rather than by differentiating a
     * gyro, which is the whole point of having two: differentiation amplifies noise, a difference of
     * two measurements does not.
     *
     * <p>For a planar rigid body, the acceleration difference between two points separated by
     * {@code r} is {@code α × r + ω × (ω × r)}. The centripetal term is {@code -ω²r} and is known
     * from the gyros, so removing it leaves {@code α × r}, whose magnitude is {@code |α||r|}
     * perpendicular to {@code r}. Projecting the residual onto that perpendicular direction gives
     * {@code α} with the right sign.
     *
     * @param primaryAccel   primary sensor's measured acceleration, robot frame, m/s²
     * @param secondaryAccel secondary sensor's measured acceleration, robot frame, m/s²
     * @return angular acceleration, or empty when the sensors are too close together for the
     *         difference to carry signal
     */
    public OptionalDouble angularAccelerationRadPerSecSq(Translation2d primaryAccel,
                                                        Translation2d secondaryAccel) {
        double r = leverArm.getNorm();
        if (r < MIN_LEVER_ARM_METERS) {
            // Two sensors this close see nearly identical motion, so their difference is noise
            // divided by a small number. Returning a number here would be inventing one.
            return OptionalDouble.empty();
        }

        double omega = Math.toRadians(getYawRate());

        // Remove the centripetal component, which points from the primary back toward the secondary
        // and has magnitude omega^2 * r.
        Translation2d centripetal = leverArm.times(-omega * omega);
        Translation2d residual = primaryAccel.minus(secondaryAccel).minus(centripetal);

        // What is left is alpha x r: perpendicular to the lever arm, magnitude |alpha| * r.
        // Rotating the lever arm 90 degrees CCW gives the direction a positive alpha produces.
        Translation2d perpendicular = new Translation2d(-leverArm.getY(), leverArm.getX());
        double projected = (residual.getX() * perpendicular.getX()
                + residual.getY() * perpendicular.getY()) / (r * r);

        return OptionalDouble.of(projected);
    }

    /**
     * Angular acceleration, read from both sensors directly.
     *
     * <p>The form worth using. The two-argument version exists for callers holding accelerations
     * from somewhere else, but fetching them by hand means going through a Pigeon's API and
     * Systemcore's onboard IMU separately, in two unit conventions, and getting the frames right —
     * which is enough work that the measurement does not get taken.
     *
     * @return angular acceleration in rad/s², or empty if either sensor is not reporting
     *         acceleration or the two sit too close together to measure it
     */
    public OptionalDouble angularAccelerationRadPerSecSq() {
        var a = primary.getAcceleration();
        var b = secondary.getAcceleration();
        if (a.isEmpty() || b.isEmpty()) {
            // One sensor silent is no measurement. Substituting zero for the missing one would
            // produce a confident number from a single accelerometer, which is exactly the thing
            // this class exists to avoid.
            return OptionalDouble.empty();
        }
        return angularAccelerationRadPerSecSq(a.get().minus(restBias), b.get());
    }

    /**
     * Learn what the two sensors read while the robot is standing still, and subtract it from then
     * on.
     *
     * <p>Without this the measurement has a constant offset that has nothing to do with rotation.
     * Two accelerometers a few degrees out of plane with each other each read a different slice of
     * gravity, and this method divides their difference by the lever arm — so a small tilt becomes a
     * large angular acceleration. Measured on a bench: a Pigeon sitting at roll -2.24° read
     * -0.474 m/s² in Y where the Systemcore's own IMU read +0.086, and that 0.56 m/s² of gravity
     * across a 0.30 m arm came out as <b>-1.87 rad/s² on a robot that was not moving</b>. The
     * arithmetic was right; the inputs were tilted.
     *
     * <p>Call it once with the robot at rest and level — {@code robotInit} is the natural place, or
     * a button in the pit. Calling it while the robot is moving bakes that motion in as the new
     * zero, so it is deliberately explicit rather than automatic.
     *
     * @return whether a bias was captured; false when either sensor had nothing to report
     */
    public boolean calibrateAtRest() {
        var a = primary.getAcceleration();
        var b = secondary.getAcceleration();
        if (a.isEmpty() || b.isEmpty()) {
            return false;
        }
        // The whole difference, not a modelled part of it. Whatever the two sensors disagree about
        // while nothing is happening is by definition not rotation.
        restBias = a.get().minus(b.get());
        return true;
    }

    /** Forget any rest calibration, so readings are raw again. */
    public void clearRestCalibration() {
        restBias = Translation2d.ZERO;
    }

    /**
     * The offset learned by {@link #calibrateAtRest()}, in m/s².
     *
     * <p>Worth publishing. A large value means the two sensors are further out of plane than
     * intended, and that is a mounting problem the number will otherwise hide.
     */
    public Translation2d restBias() {
        return restBias;
    }

    /**
     * What the sensors disagree about at rest, removed from every reading. Zero until
     * {@link #calibrateAtRest()} is called.
     */
    private Translation2d restBias = Translation2d.ZERO;

    /** Whether both sensors are reporting acceleration, so the reading above is available. */
    public boolean canMeasureAngularAcceleration() {
        return leverArm.getNorm() >= MIN_LEVER_ARM_METERS
                && primary.getAcceleration().isPresent()
                && secondary.getAcceleration().isPresent();
    }

    /** The authoritative sensor. */
    public CatalystIMU primary() {
        return primary;
    }

    /** The second sensor, for reading it directly. */
    public CatalystIMU secondary() {
        return secondary;
    }

    /** Publish both sensors and their disagreement under {@code /Catalyst/IMU/}. */
    public void publish() {
        CatalystLog.log("IMU/Yaw", getYaw());
        CatalystLog.log("IMU/YawRate", getYawRate());
        CatalystLog.log("IMU/Primary/Yaw", primary.getYaw());
        CatalystLog.log("IMU/Secondary/Yaw", secondary.getYaw());
        CatalystLog.log("IMU/YawDisagreementDeg", yawDisagreementDegrees());
        CatalystLog.log("IMU/YawRateDisagreementDegPerSec", yawRateDisagreementDegPerSec());
        CatalystLog.log("IMU/LeverArmMeters", leverArmMeters());
        CatalystLog.log("IMU/PrimaryRateWeight", primaryRateWeight);
        angularAccelerationRadPerSecSq()
                .ifPresent(a -> CatalystLog.log("IMU/AngularAccelRadPerSecSq", a));
    }
}
