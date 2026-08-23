package frc.lib.catalyst.hardware;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
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

    private final Pigeon2 pigeon;
    private final int canId;

    public CatalystGyro(int canId) {
        this(canId, "");
    }

    public CatalystGyro(int canId, String canBus) {
        this.canId = canId;
        this.pigeon = new Pigeon2(canId, CatalystCANBus.of(canBus).phoenix());
        claimCanId(canId, canBus);
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
        claimCanId(canId, canBus);
        for (int i = 0; i < 5; i++) {
            var status = pigeon.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
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
