package frc.lib.catalyst.hardware;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * Simple CANcoder wrapper for absolute position feedback.
 */
public class CatalystEncoder {

    private final CANcoder encoder;
    private final int canId;

    public CatalystEncoder(int canId) {
        this(canId, "", 0.0, false);
    }

    public CatalystEncoder(int canId, String canBus, double magnetOffset, boolean inverted) {
        this.canId = canId;
        this.encoder = new CANcoder(canId, canBus);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = magnetOffset;
        config.MagnetSensor.SensorDirection = inverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;

        for (int i = 0; i < 5; i++) {
            var status = encoder.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
    }

    /** Get absolute position in rotations [-0.5, 0.5]. */
    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    /** Get position in rotations (continuous). */
    public double getPosition() {
        return encoder.getPosition().getValueAsDouble();
    }

    /** Get velocity in rotations per second. */
    public double getVelocity() {
        return encoder.getVelocity().getValueAsDouble();
    }

    /** Get the underlying CANcoder. */
    public CANcoder getCANcoder() {
        return encoder;
    }

    public int getCanId() {
        return canId;
    }
}
