package frc.lib.catalyst.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Configuration for the LED subsystem.
 *
 * <p>Example:
 * <pre>{@code
 * LEDConfig config = LEDConfig.builder()
 *     .pwmPort(0)
 *     .ledCount(60)
 *     .defaultColor(Color.kGreen)
 *     .build();
 * }</pre>
 */
public class LEDConfig {

    final int pwmPort;
    final int ledCount;
    final Color defaultColor;

    private LEDConfig(Builder b) {
        this.pwmPort = b.pwmPort;
        this.ledCount = b.ledCount;
        this.defaultColor = b.defaultColor;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private int pwmPort = 0;
        private int ledCount = 60;
        private Color defaultColor = Color.kBlack;

        /** PWM port the LED strip is connected to. */
        public Builder pwmPort(int port) { this.pwmPort = port; return this; }

        /** Number of LEDs in the strip. */
        public Builder ledCount(int count) { this.ledCount = count; return this; }

        /** Default color when no command is running. */
        public Builder defaultColor(Color color) { this.defaultColor = color; return this; }

        public LEDConfig build() {
            return new LEDConfig(this);
        }
    }
}
