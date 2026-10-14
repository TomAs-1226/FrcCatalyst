package frc.lib.catalyst.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.LinearAcceleration;

import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Simple Pigeon2 IMU wrapper for heading, pitch, and roll.
 */
public class CatalystGyro implements CatalystIMU {

    /**
     * Device type label used in the CAN registry and on the robot's spec sheet.
     *
     * <p>Spelled the way Phoenix names the class, because the spec sheet's hardware inventory falls
     * back to a device's own class name for anything Catalyst did not register — and one device
     * appearing under two spellings would read as two devices.
     */
    public static final String DEVICE_TYPE = "Pigeon2";

    /**
     * How often the signals this class reads every loop are asked to arrive, in hertz.
     *
     * <p>Phoenix's own defaults are not uniform, and the two that matter most here are the slow
     * ones. Read from CTRE's source for the Pigeon 2:
     *
     * <ul>
     *   <li>{@code Yaw}, {@code Pitch}, {@code Roll} — 100 Hz on CAN 2.0
     *   <li>{@code AngularVelocityZWorld} — <b>10 Hz</b>
     *   <li>{@code AccelerationX/Y} — <b>10 Hz</b>
     * </ul>
     *
     * <p>Every Systemcore bus is CAN 2.0, and so is the roboRIO's, so 10 Hz is what a robot actually
     * gets. {@link #getYawRate()} is read once a loop at 50 Hz for heading control, which means its
     * value can be 100 ms old — five loops of a derivative term acting on a number that has not
     * changed. Acceleration is worse: {@link DualIMU} differences it against Systemcore's own IMU
     * sampled now, so under a hard turn the difference is not two sensors disagreeing, it is one
     * sensor being a tenth of a second behind the other.
     *
     * <p>100 Hz is twice the loop rate, which is the least that makes a per-loop read meaningful.
     * Gravity X/Y/Z are requested at the same rate as acceleration so gravity removal does not
     * combine current motion with an old tilt estimate. Extra signals consume bus bandwidth;
     * {@link #withSignalRate(double)} lets a team choose the rate for its bus.
     */
    public static final double DEFAULT_SIGNAL_HZ = 100.0;

    /** Default maximum age of a gravity-removed acceleration reading, seconds. */
    public static final double DEFAULT_LINEAR_ACCELERATION_MAX_AGE_SECONDS = 0.10;
    /** Default maximum timestamp spread across acceleration and gravity, seconds. */
    public static final double DEFAULT_LINEAR_ACCELERATION_MAX_SKEW_SECONDS = 0.02;

    private final Pigeon2 pigeon;
    private final int canId;
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;
    private final StatusSignal<LinearAcceleration> accelerationZ;
    private final StatusSignal<Double> gravityX;
    private final StatusSignal<Double> gravityY;
    private final StatusSignal<Double> gravityZ;
    private final BaseStatusSignal[] linearAccelerationSignals;

    public CatalystGyro(int canId) {
        this(canId, "");
    }

    public CatalystGyro(int canId, String canBus) {
        this.canId = canId;
        this.pigeon = new Pigeon2(canId, CatalystCANBus.of(canBus).phoenix());
        this.accelerationX = pigeon.getAccelerationX(false);
        this.accelerationY = pigeon.getAccelerationY(false);
        this.accelerationZ = pigeon.getAccelerationZ(false);
        this.gravityX = pigeon.getGravityVectorX(false);
        this.gravityY = pigeon.getGravityVectorY(false);
        this.gravityZ = pigeon.getGravityVectorZ(false);
        this.linearAccelerationSignals = new BaseStatusSignal[] {
                accelerationX, accelerationY, accelerationZ, gravityX, gravityY, gravityZ };
        claimCanId(canId, canBus);
        applySignalRates(DEFAULT_SIGNAL_HZ);
        // Intentionally do NOT apply a configuration here. Applying a default
        // Pigeon2Configuration would erase whatever is on the device — most
        // importantly the mount-pose offset teams set in Tuner X. Use the
        // config-taking constructor if you want Catalyst to own the config.
    }

    /**
     * Construct with an explicit {@link Pigeon2Configuration} to apply. Use this
     * only when you want Catalyst (not Tuner X) to own the device configuration;
     * it overwrites the whole config, so include your mount pose in {@code config}.
     */
    public CatalystGyro(int canId, String canBus, Pigeon2Configuration config) {
        this.canId = canId;
        this.pigeon = new Pigeon2(canId, CatalystCANBus.of(canBus).phoenix());
        this.accelerationX = pigeon.getAccelerationX(false);
        this.accelerationY = pigeon.getAccelerationY(false);
        this.accelerationZ = pigeon.getAccelerationZ(false);
        this.gravityX = pigeon.getGravityVectorX(false);
        this.gravityY = pigeon.getGravityVectorY(false);
        this.gravityZ = pigeon.getGravityVectorZ(false);
        this.linearAccelerationSignals = new BaseStatusSignal[] {
                accelerationX, accelerationY, accelerationZ, gravityX, gravityY, gravityZ };
        claimCanId(canId, canBus);
        for (int i = 0; i < 5; i++) {
            var status = pigeon.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        applySignalRates(DEFAULT_SIGNAL_HZ);
    }

    /**
     * Ask for the signals this class reads at a different rate.
     *
     * <p>Returns {@code this}, so it reads as part of construction. Lower it on a crowded bus, or
     * raise it on CAN FD where the ceiling is higher.
     *
     * <p>Phoenix stores a period rather than a frequency, in whole milliseconds, so the rate that
     * comes back is the nearest one a whole-millisecond period can express. 100, 50 and 20 Hz land
     * exactly; asking for 37 gets 37.037. Worth knowing before wondering why a requested rate reads
     * back slightly different.
     *
     * @param hz how often to send yaw rate and acceleration; see {@link #DEFAULT_SIGNAL_HZ}
     */
    public CatalystGyro withSignalRate(double hz) {
        applySignalRates(hz);
        return this;
    }

    /**
     * Raise the signals Catalyst reads every loop off their 10 Hz defaults.
     *
     * <p>Only the ones this class reads, and deliberately no {@code optimizeBusUtilization()} to go
     * with it. That call silences every signal nobody has explicitly asked for, which would be a
     * quiet trap for a team reading something else off {@link #getPigeon()} — their reading would
     * stop updating with nothing to say why. A motor is a closed enough object for Catalyst to make
     * that call on its behalf; a gyro a team also talks to directly is not.
     */
    private void applySignalRates(double hz) {
        com.ctre.phoenix6.BaseStatusSignal.setUpdateFrequencyForAll(hz,
                pigeon.getAngularVelocityZWorld(),
                accelerationX, accelerationY, accelerationZ, gravityX, gravityY, gravityZ);
    }

    /**
     * Claim the gyro's id like every motor does.
     *
     * <p>Two reasons this is worth doing. A Pigeon sharing an id with a TalonFX is a wiring fault
     * that used to go unreported because nothing registered the gyro, and the robot's spec sheet
     * lists a gyro only if something told the registry there was one.
     */
    private static void claimCanId(int canId, String canBus) {
        CANRegistry.register("Gyro", canId, canBus, DEVICE_TYPE);
    }

    /** Get heading as Rotation2d (yaw, CCW positive). */
    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }

    /** Get yaw in degrees (CCW positive, continuous). */
    @Override
    public double getYaw() {
        return pigeon.getYaw().getValueAsDouble();
    }

    /** Get pitch in degrees. */
    @Override
    public double getPitch() {
        return pigeon.getPitch().getValueAsDouble();
    }

    /** Get roll in degrees. */
    @Override
    public double getRoll() {
        return pigeon.getRoll().getValueAsDouble();
    }

    /** Get yaw angular velocity in degrees per second. */
    @Override
    public double getYawRate() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    /** Reset yaw to zero. */
    @Override
    public void zeroYaw() {
        pigeon.setYaw(0);
    }

    /** Set yaw to a specific value in degrees. */
    @Override
    public void setYaw(double degrees) {
        pigeon.setYaw(degrees);
    }

    /** Get the underlying Pigeon2. */
    /**
     * Acceleration in the robot's XY plane, from the Pigeon's accelerometer.
     *
     * <p>Phoenix reports it as a typed {@code LinearAcceleration}, so this converts once here rather
     * than leaving every caller to remember whether the signal is in g or m/s².
     *
     * <p>Empty if the signal has not arrived. A Pigeon that has just powered up, or one on a bus
     * that has dropped, reports nothing rather than zero - and zero acceleration is a specific,
     * wrong claim about a robot that might be accelerating hard.
     */
    @Override
    public java.util.Optional<Translation2d> getAcceleration() {
        var x = pigeon.getAccelerationX();
        var y = pigeon.getAccelerationY();
        if (!x.getStatus().isOK() || !y.getStatus().isOK()) {
            return java.util.Optional.empty();
        }
        return java.util.Optional.of(new Translation2d(
                x.getValue().in(MetersPerSecondPerSecond),
                y.getValue().in(MetersPerSecondPerSecond)));
    }

    /**
     * Fresh gravity-removed acceleration in the Pigeon's reported XYZ frame, m/s².
     * Uses the default age and timestamp-spread limits. No wait for CAN frames is performed.
     * This is distinct from the raw {@link #getAcceleration()} used by {@link DualIMU}.
     */
    public java.util.Optional<Translation2d> getLinearAcceleration() {
        return getLinearAcceleration(DEFAULT_LINEAR_ACCELERATION_MAX_AGE_SECONDS,
                DEFAULT_LINEAR_ACCELERATION_MAX_SKEW_SECONDS);
    }

    /**
     * Gravity-removed acceleration with explicit freshness limits, seconds.
     *
     * <p>Refreshes the cached signals with Phoenix's zero-timeout, non-blocking refresh. Bad status,
     * absent timestamp, non-finite data, future/old data, or an excessive timestamp spread returns
     * empty. The timestamp spread covers all four signals, rather than only each axis pair.
     *
     * <p>Phoenix acceleration includes gravity; its dimensionless gravity vector represents the
     * gravity contribution in units of standard g. Subtract that contribution after converting
     * acceleration to m/s². No robot pitch, mounting offset, or stationary bias is guessed here.
     *
     * @param maxAgeSeconds maximum age of every signal; finite and nonnegative
     * @param maxSkewSeconds maximum timestamp spread; finite and nonnegative
     */
    public java.util.Optional<Translation2d> getLinearAcceleration(
            double maxAgeSeconds, double maxSkewSeconds) {
        BaseStatusSignal.refreshAll(false, linearAccelerationSignals);
        double now = com.ctre.phoenix6.Utils.getCurrentTimeSeconds();
        return LinearAccelerationSample.from(
                reading(accelerationX, accelerationX.getValue().in(MetersPerSecondPerSecond)),
                reading(accelerationY, accelerationY.getValue().in(MetersPerSecondPerSecond)),
                reading(gravityX, gravityX.getValueAsDouble()),
                reading(gravityY, gravityY.getValueAsDouble()),
                now, maxAgeSeconds, maxSkewSeconds);
    }

    /**
     * Fresh gravity-removed acceleration leveled into the reported yaw frame, m/s².
     * Uses the default age/spread limits and an externally configured upright mounting frame.
     */
    public java.util.Optional<Translation2d> getLevelLinearAcceleration() {
        return getLevelLinearAcceleration(DEFAULT_LINEAR_ACCELERATION_MAX_AGE_SECONDS,
                DEFAULT_LINEAR_ACCELERATION_MAX_SKEW_SECONDS);
    }

    /**
     * As {@link #getLevelLinearAcceleration()}, with explicit freshness limits in seconds.
     *
     * <p>All six signals must be valid, fresh and time-aligned. The gravity vector must have norm
     * within 10% of one, with |Z|/norm at least 0.5 (no more than 60 degrees from vertical).
     * The minimal tilt rotation takes gravity to the same-sign Z axis, accepting either reported
     * polarity and leaving a flat frame unchanged. Near-horizontal frames return empty; polarity
     * cannot diagnose an inverted mounting. Correct upright mount configuration is the caller's
     * job. Uses zero-timeout refresh, never waits for a new CAN frame.
     */
    public java.util.Optional<Translation2d> getLevelLinearAcceleration(
            double maxAgeSeconds, double maxSkewSeconds) {
        BaseStatusSignal.refreshAll(false, linearAccelerationSignals);
        double now = com.ctre.phoenix6.Utils.getCurrentTimeSeconds();
        return LinearAccelerationSample.level(
                reading(accelerationX, accelerationX.getValue().in(MetersPerSecondPerSecond)),
                reading(accelerationY, accelerationY.getValue().in(MetersPerSecondPerSecond)),
                reading(accelerationZ, accelerationZ.getValue().in(MetersPerSecondPerSecond)),
                reading(gravityX, gravityX.getValueAsDouble()),
                reading(gravityY, gravityY.getValueAsDouble()),
                reading(gravityZ, gravityZ.getValueAsDouble()),
                now, maxAgeSeconds, maxSkewSeconds);
    }

    private static LinearAccelerationSample.Reading reading(BaseStatusSignal signal, double value) {
        var timestamp = signal.getTimestamp();
        return new LinearAccelerationSample.Reading(value, timestamp.getTime(),
                signal.getStatus().isOK() && timestamp.isValid());
    }

    public Pigeon2 getPigeon() {
        return pigeon;
    }

    @Override
    public String deviceType() {
        return DEVICE_TYPE;
    }

    public int getCanId() {
        return canId;
    }
}
