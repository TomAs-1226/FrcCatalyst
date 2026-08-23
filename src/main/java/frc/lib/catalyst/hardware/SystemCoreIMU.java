package frc.lib.catalyst.hardware;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Systemcore's built-in IMU, as a Catalyst heading source.
 *
 * <p>It costs nothing — no CAN id, no wiring, no bus bandwidth — which makes it worth having even
 * on a robot with a Pigeon. Two uses it is genuinely good for:
 *
 * <ul>
 *   <li><b>A second opinion.</b> Physics Core's whole premise is that disagreement between sensors
 *       is information. A free, independent heading source that shares no wiring, no bus and no
 *       failure mode with the Pigeon is the cheapest disagreement Catalyst can buy.</li>
 *   <li><b>A fallback.</b> A Pigeon that drops off the CAN bus mid-match takes odometry with it.
 *       This one cannot drop off the bus, because it is not on one.</li>
 * </ul>
 *
 * <p><b>It is not a Pigeon replacement for swerve.</b> The Pigeon is the better sensor for
 * odometry — better noise, better drift, and mounted where the drivetrain is rather than wherever
 * the Systemcore happened to fit. Use {@link CatalystGyro} for the drivetrain and this alongside it.
 *
 * <p><b>Mount orientation matters and is not guessable.</b> Systemcore has to be told which way up
 * it is, in its own web UI under the IMU settings; the value passed here must match. Getting it
 * wrong does not fail loudly — it silently swaps which physical axis reports as yaw, which looks
 * like a robot that turns when it should pitch.
 *
 * <p>WPILib's onboard IMU reports radians and has no absolute yaw setter, only a reset. Both are
 * handled here: angles are converted to degrees to match every other Catalyst heading source, and
 * {@link #setYaw(double)} is implemented as a reset plus a remembered offset.
 *
 * @since 2.0.0
 */
public final class SystemCoreIMU implements CatalystIMU {

    /** Device label used in logs and on the robot's spec sheet. */
    public static final String DEVICE_TYPE = "Systemcore IMU";

    private final OnboardIMU imu;

    /** Offset applied on top of the hardware yaw, so setYaw() can work without a hardware setter. */
    private double yawOffsetDegrees;

    /**
     * Use the IMU with the given mount orientation.
     *
     * @param orientation how the Systemcore is physically mounted; must match the orientation
     *                    configured in the Systemcore web UI
     */
    public SystemCoreIMU(OnboardIMU.MountOrientation orientation) {
        this.imu = new OnboardIMU(orientation);
    }

    /** Use the IMU mounted flat — the common case, Systemcore lying on a horizontal surface. */
    public SystemCoreIMU() {
        this(OnboardIMU.MountOrientation.FLAT);
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }

    @Override
    public double getYaw() {
        return Math.toDegrees(imu.getYawRadians()) + yawOffsetDegrees;
    }

    @Override
    public double getPitch() {
        return Math.toDegrees(imu.getAngleY());
    }

    @Override
    public double getRoll() {
        return Math.toDegrees(imu.getAngleX());
    }

    @Override
    public double getYawRate() {
        return Math.toDegrees(imu.getGyroRateZ());
    }

    @Override
    public void zeroYaw() {
        imu.resetYaw();
        yawOffsetDegrees = 0;
    }

    /**
     * {@inheritDoc}
     *
     * <p>The hardware only offers a reset, so this resets and then remembers the difference. That
     * is behaviourally the same as the Pigeon's absolute setter for everything Catalyst does with
     * it, but it is worth knowing the offset lives in this object: construct a second
     * {@code SystemCoreIMU} and it will not share the offset.
     */
    @Override
    public void setYaw(double degrees) {
        imu.resetYaw();
        yawOffsetDegrees = degrees;
    }

    @Override
    public String deviceType() {
        return DEVICE_TYPE;
    }

    /** Full 3D attitude, which the Pigeon wrapper does not expose. Useful for a 3D robot view. */
    public org.wpilib.math.geometry.Rotation3d getRotation3d() {
        return imu.getRotation3d();
    }

    /**
     * Acceleration in the robot's XY plane, straight off the onboard accelerometer.
     *
     * <p>Always present: the sensor is on the board and there is nothing to be absent. Reported in
     * m/s² with gravity still in it, per the interface.
     */
    @Override
    public java.util.Optional<Translation2d> getAcceleration() {
        return java.util.Optional.of(new Translation2d(imu.getAccelX(), imu.getAccelY()));
    }

    /** The underlying WPILib IMU, for acceleration and per-axis rates. */
    public OnboardIMU getOnboardIMU() {
        return imu;
    }
}
