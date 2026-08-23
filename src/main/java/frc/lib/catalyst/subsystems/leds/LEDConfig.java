package frc.lib.catalyst.subsystems.leds;

import org.wpilib.util.Color;

/**
 * Configuration for the LED subsystem.
 *
 * <p>Example:
 * <pre>{@code
 * LEDConfig config = LEDConfig.builder()
 *     .pwmPort(0)
 *     .ledCount(60)
 *     .defaultColor(Color.GREEN)
 *     .build();
 * }</pre>
 */
public class LEDConfig {

    /** Systemcore drives at most this many unique LEDs in total, across all IO pins. */
    public static final int MAX_LEDS = 1024;

    final int pwmPort;
    final int ledCount;
    final Color defaultColor;
    /** Index of the first LED this strip owns within the controller's shared space. */
    final int startIndex;

    private LEDConfig(Builder b) {
        this.pwmPort = b.pwmPort;
        this.ledCount = b.ledCount;
        this.defaultColor = b.defaultColor;
        this.startIndex = b.startIndex;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private int pwmPort = 0;
        private int ledCount = 60;
        private Color defaultColor = Color.BLACK;
        private int startIndex = 0;

        /**
         * IO pin the LED strip is connected to.
         *
         * <p>The name is historical. On a roboRIO this was a PWM port; Systemcore drives WS2812 from
         * <em>any</em> of its IO pins, so this is really "which pin". Kept as {@code pwmPort} so
         * existing robot code compiles unchanged — {@link #ioPin(int)} is the same setter under a
         * name that describes the hardware.
         */
        public Builder pwmPort(int port) { this.pwmPort = port; return this; }

        /** IO pin the LED strip is connected to. Reads better than {@link #pwmPort(int)}. */
        public Builder ioPin(int pin) { return pwmPort(pin); }

        /**
         * Index of this strip's first LED within the controller's shared space.
         *
         * <p>Systemcore lets several strips share one controller by giving each an offset and a
         * length — the OS calls these {@code ledoffset} and {@code ledcount}. Two strips on one pin
         * therefore need the second one's start set past the end of the first, or they will write
         * over each other and the symptom is one strip mirroring the other.
         *
         * <p>Defaults to 0, which is right for the single-strip case.
         */
        public Builder startIndex(int index) { this.startIndex = index; return this; }

        /** Number of LEDs in the strip. */
        public Builder ledCount(int count) { this.ledCount = count; return this; }

        /** Default color when no command is running. */
        public Builder defaultColor(Color color) { this.defaultColor = color; return this; }

        public LEDConfig build() {
            if (ledCount <= 0) {
                throw new IllegalArgumentException("ledCount must be positive, got " + ledCount);
            }
            if (startIndex < 0) {
                throw new IllegalArgumentException("startIndex must not be negative, got " + startIndex);
            }
            // Systemcore's firmware caps the total at 1024 unique LEDs across every pin. Exceeding
            // it does not fail cleanly on the device, so it is worth catching here where the number
            // is still attached to a line of code.
            if (startIndex + ledCount > MAX_LEDS) {
                throw new IllegalArgumentException(
                        "startIndex + ledCount is " + (startIndex + ledCount) + ", over Systemcore's "
                                + MAX_LEDS + "-LED limit");
            }
            return new LEDConfig(this);
        }
    }
}
