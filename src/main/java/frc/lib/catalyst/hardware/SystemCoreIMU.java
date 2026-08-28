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

    /** How the board is mounted, kept because the rate and acceleration paths need to know. */
    private final OnboardIMU.MountOrientation orientation;

    /**
     * Sensor frame to robot frame, or null when nobody has said.
     *
     * <p>WPILib applies mount orientation to yaw and to the Euler angles, and to nothing else — one
     * {@code rawgyro} topic and one {@code rawaccel} topic exist, with no per-orientation form.
     * Confirmed twice: in the alpha-6 bytecode, and by dumping the topics a board actually
     * publishes. So on anything but a FLAT mount the rates and accelerations below are in the
     * sensor's frame, and only the caller knows how that relates to the robot.
     */
    private final Rotation2d sensorToRobot;

    /** Offset applied on top of the hardware yaw, so setYaw() can work without a hardware setter. */
    private double yawOffsetDegrees;

    /**
     * Use the IMU with the given mount orientation.
     *
     * @param orientation how the Systemcore is physically mounted; must match the orientation
     *                    configured in the Systemcore web UI
     */
    public SystemCoreIMU(OnboardIMU.MountOrientation orientation) {
        this(orientation, null);
    }

    /**
     * Use the IMU with a mount orientation and an explicit sensor-to-robot rotation.
     *
     * <p>Supplying the rotation is what makes acceleration available on a board that is not mounted
     * FLAT. Without it, {@link #getAcceleration()} returns empty for any other orientation rather
     * than handing back a vector in the wrong frame — because that vector still contains gravity,
     * and {@link DualIMU} differences accelerations expecting gravity to cancel. It does not cancel,
     * and the leftover becomes a large constant angular acceleration that is not happening. On a
     * bench with a stationary robot that was measured at -1.89 rad/s².
     *
     * <p>Pass {@link Rotation2d#kZero} to say "the sensor frame is the robot frame" — which is what
     * a FLAT board aligned with robot-forward means, and is also the way to keep readings flowing
     * while deliberately tilting the hardware to test something.
     *
     * @param orientation   how the Systemcore is physically mounted; must match the Systemcore web
     *                      UI's setting
     * @param sensorToRobot rotation from the sensor's frame into the robot's, or null to have
     *                      acceleration reported only when the mount is FLAT
     */
    public SystemCoreIMU(OnboardIMU.MountOrientation orientation, Rotation2d sensorToRobot) {
        this.imu = new OnboardIMU(orientation);
        this.orientation = orientation;
        this.sensorToRobot = sensorToRobot;
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

    /**
     * {@inheritDoc}
     *
     * <p>Read off the sensor's Z axis, which is the robot's yaw axis only when the board is FLAT or
     * a sensor-to-robot rotation has been given. On any other mount with no rotation supplied this
     * is a pitch or roll rate wearing a yaw label, so it reports zero instead: this returns a
     * primitive and has no way to say "unknown", and a plausible wrong rate is worse than a still
     * one. {@link #isYawRateTrustworthy()} says which of the two is being returned.
     */
    @Override
    public double getYawRate() {
        return isFrameKnown() ? Math.toDegrees(imu.getGyroRateZ()) : 0.0;
    }

    /**
     * Whether {@link #getYawRate()} is measuring the robot's yaw axis.
     *
     * <p>False when the board is mounted other than FLAT and nobody has said how it sits relative to
     * the robot. {@link DualIMU} consults this before weighting this sensor's rate.
     */
    public boolean isYawRateTrustworthy() {
        return isFrameKnown();
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
    // CONFIRMED ON HARDWARE, 2026-08-26. The system server publishes yaw and Euler angles once per
    // mount orientation - /imu/yaw_flat, /imu/yaw_landscape, /imu/yaw_portrait, and the three
    // matching /imu/euler_* topics - but exactly ONE /imu/rawgyro and ONE /imu/rawaccel. There is no
    // per-orientation form of either.
    //
    // That is the board agreeing with what the jars said: mount orientation is applied to yaw and
    // Euler angles and to nothing else. On a Systemcore mounted LANDSCAPE or PORTRAIT, the rates and
    // accelerations read here are in the sensor's frame, not the robot's - so this returns a vector
    // containing gravity rather than the robot's XY plane, and DualIMU's differencing then produces
    // a confident wrong angular acceleration rather than an empty Optional.
    //
    // Unresolved, and it needs a decision rather than more measurement: either take a robot-relative
    // rotation here and apply it, or return empty for any orientation other than FLAT.
    @Override
    public java.util.Optional<Translation2d> getAcceleration() {
        if (!isFrameKnown()) {
            // Empty, not a guess. The alternative is a plane containing gravity presented as the
            // robot's XY, which reads as a real measurement all the way into Physics Core.
            return java.util.Optional.empty();
        }
        Translation2d sensor = new Translation2d(imu.getAccelX(), imu.getAccelY());
        return java.util.Optional.of(
                sensorToRobot == null ? sensor : sensor.rotateBy(sensorToRobot));
    }

    /**
     * Whether the readings below can be put in the robot's frame at all.
     *
     * <p>True when the board is FLAT — sensor and robot frames agree — or when a caller has supplied
     * the rotation between them.
     */
    private boolean isFrameKnown() {
        return sensorToRobot != null || orientation == OnboardIMU.MountOrientation.FLAT;
    }

    /** The underlying WPILib IMU, for acceleration and per-axis rates. */
    public OnboardIMU getOnboardIMU() {
        return imu;
    }
}
