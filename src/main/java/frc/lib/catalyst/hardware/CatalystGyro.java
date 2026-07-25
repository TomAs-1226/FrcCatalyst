package frc.lib.catalyst.hardware;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Simple Pigeon2 IMU wrapper for heading, pitch, and roll.
 */
public class CatalystGyro {

    private final Pigeon2 pigeon;
    private final int canId;

    public CatalystGyro(int canId) {
        this(canId, "");
    }

    public CatalystGyro(int canId, String canBus) {
        this.canId = canId;
        this.pigeon = new Pigeon2(canId, canBus);
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
        this.pigeon = new Pigeon2(canId, canBus);
        for (int i = 0; i < 5; i++) {
            var status = pigeon.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
    }

    /** Get heading as Rotation2d (yaw, CCW positive). */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }

    /** Get yaw in degrees (CCW positive, continuous). */
    public double getYaw() {
        return pigeon.getYaw().getValueAsDouble();
    }

    /** Get pitch in degrees. */
    public double getPitch() {
        return pigeon.getPitch().getValueAsDouble();
    }

    /** Get roll in degrees. */
    public double getRoll() {
        return pigeon.getRoll().getValueAsDouble();
    }

    /** Get yaw angular velocity in degrees per second. */
    public double getYawRate() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    /** Reset yaw to zero. */
    public void zeroYaw() {
        pigeon.setYaw(0);
    }

    /** Set yaw to a specific value in degrees. */
    public void setYaw(double degrees) {
        pigeon.setYaw(degrees);
    }

    /** Get the underlying Pigeon2. */
    public Pigeon2 getPigeon() {
        return pigeon;
    }

    public int getCanId() {
        return canId;
    }
}
