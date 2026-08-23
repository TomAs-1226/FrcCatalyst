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
 * <p><b>A better yaw rate.</b> Two independent gyros averaged have roughly {@code 1/sqrt(2)} the
 * noise of one, and — more usefully — no shared failure mode. The Pigeon lives on the CAN bus and
 * the Systemcore IMU does not, so a bus problem cannot take both.
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
     * Below this separation the two accelerometers see nearly the same motion.
     *
     * <p>The difference between them is then mostly sensor noise divided by a small number, which
     * is a large noisy number rather than an angular acceleration. Half a metre is about the
     * shortest lever arm on a typical robot that carries useful signal.
     */
    private static final double MIN_LEVER_ARM_METERS = 0.05;

    private final CatalystIMU primary;
    private final CatalystIMU secondary;
    private final Translation2d leverArm;

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
     * Yaw rate, averaged across both gyros.
     *
     * <p>Unlike yaw, rate is a direct measurement rather than an integral, so averaging is
     * straightforwardly better: uncorrelated noise falls by about {@code 1/sqrt(2)} and neither
     * sensor's failure takes the reading out on its own.
     */
    @Override
    public double getYawRate() {
        return (primary.getYawRate() + secondary.getYawRate()) / 2.0;
    }

    @Override
    public String deviceType() {
        return primary.deviceType() + " + " + secondary.deviceType();
    }

    // --- what only two sensors can tell you ----------------------------------

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
        return angularAccelerationRadPerSecSq(a.get(), b.get());
    }

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
        angularAccelerationRadPerSecSq()
                .ifPresent(a -> CatalystLog.log("IMU/AngularAccelRadPerSecSq", a));
    }
}
