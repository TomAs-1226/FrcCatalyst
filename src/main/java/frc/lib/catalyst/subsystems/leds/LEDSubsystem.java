package frc.lib.catalyst.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LED subsystem for addressable LEDs with pre-built patterns.
 *
 * <p>Provides solid colors, blinking, rainbow, progress bar, and
 * alliance-aware default colors.
 *
 * <p>Example:
 * <pre>{@code
 * LEDSubsystem leds = new LEDSubsystem(LEDConfig.builder()
 *     .pwmPort(0)
 *     .ledCount(60)
 *     .build());
 *
 * // Set default command to alliance color
 * leds.setDefaultCommand(leds.allianceColor());
 *
 * // Flash green when intake has a piece
 * intake.hasPieceTrigger().whileTrue(leds.blink(Color.kGreen, 5));
 * }</pre>
 */
public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final int ledCount;
    private int rainbowOffset = 0;

    public LEDSubsystem(LEDConfig config) {
        this.ledCount = config.ledCount;
        this.led = new AddressableLED(config.pwmPort);
        this.buffer = new AddressableLEDBuffer(config.ledCount);

        led.setLength(config.ledCount);
        led.start();

        // Set initial color
        setSolidColor(config.defaultColor);
    }

    // --- Direct Control ---

    /** Set all LEDs to a solid color. */
    public void setSolidColor(Color color) {
        for (int i = 0; i < ledCount; i++) {
            buffer.setLED(i, color);
        }
        led.setData(buffer);
    }

    /** Set a specific LED to a color. */
    public void setLED(int index, Color color) {
        if (index >= 0 && index < ledCount) {
            buffer.setLED(index, color);
        }
    }

    /** Push the current buffer to the LEDs. */
    public void update() {
        led.setData(buffer);
    }

    // --- Command Factories ---

    /** Command to set all LEDs to a solid color. */
    public Command solid(Color color) {
        return run(() -> setSolidColor(color))
                .withName("LED.Solid");
    }

    /** Command to blink all LEDs at a given frequency. */
    public Command blink(Color color, double frequencyHz) {
        return run(() -> {
            double period = 1.0 / frequencyHz;
            boolean on = (Timer.getFPGATimestamp() % period) < (period / 2.0);
            setSolidColor(on ? color : Color.kBlack);
        }).withName("LED.Blink");
    }

    /** Command to display a rainbow pattern. */
    public Command rainbow() {
        return run(() -> {
            for (int i = 0; i < ledCount; i++) {
                int hue = (rainbowOffset + (i * 180 / ledCount)) % 180;
                buffer.setHSV(i, hue, 255, 128);
            }
            rainbowOffset = (rainbowOffset + 3) % 180;
            led.setData(buffer);
        }).withName("LED.Rainbow");
    }

    /** Command to display a progress bar (0.0 to 1.0). */
    public Command progress(Color color, double percent) {
        return run(() -> {
            int litCount = (int) (ledCount * Math.max(0, Math.min(1, percent)));
            for (int i = 0; i < ledCount; i++) {
                buffer.setLED(i, i < litCount ? color : Color.kBlack);
            }
            led.setData(buffer);
        }).withName("LED.Progress");
    }

    /** Command to display alliance color (red or blue, white if unknown). */
    public Command allianceColor() {
        return run(() -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                setSolidColor(alliance.get() == DriverStation.Alliance.Red
                        ? Color.kRed : Color.kBlue);
            } else {
                setSolidColor(Color.kWhite);
            }
        }).withName("LED.Alliance");
    }

    /** Command to turn off all LEDs. */
    public Command off() {
        return run(() -> setSolidColor(Color.kBlack))
                .withName("LED.Off");
    }

    /** Command for a scrolling/chase pattern. */
    public Command chase(Color color, int width) {
        return run(() -> {
            int offset = (int) ((Timer.getFPGATimestamp() * 20) % ledCount);
            for (int i = 0; i < ledCount; i++) {
                boolean lit = ((i + offset) % (width * 2)) < width;
                buffer.setLED(i, lit ? color : Color.kBlack);
            }
            led.setData(buffer);
        }).withName("LED.Chase");
    }

    /** Command for alternating two colors. */
    public Command alternating(Color color1, Color color2) {
        return run(() -> {
            for (int i = 0; i < ledCount; i++) {
                buffer.setLED(i, i % 2 == 0 ? color1 : color2);
            }
            led.setData(buffer);
        }).withName("LED.Alternating");
    }

    /** Command for a breathing/fade effect. */
    public Command breathe(Color color, double periodSeconds) {
        return run(() -> {
            double t = (Timer.getFPGATimestamp() % periodSeconds) / periodSeconds;
            double brightness = (Math.sin(t * 2 * Math.PI) + 1.0) / 2.0;
            Color dimmed = new Color(
                    color.red * brightness,
                    color.green * brightness,
                    color.blue * brightness);
            setSolidColor(dimmed);
        }).withName("LED.Breathe");
    }

    /**
     * Command for a fire/flame effect.
     * Simulates flickering fire with random heat values.
     * Commonly used by top teams for aesthetic effects.
     */
    public Command fire() {
        final double[] heat = new double[ledCount];
        return run(() -> {
            // Cool down each cell
            for (int i = 0; i < ledCount; i++) {
                heat[i] = Math.max(0, heat[i] - (Math.random() * 0.15));
            }
            // Spark randomly at the bottom
            for (int i = 0; i < 3; i++) {
                int idx = (int) (Math.random() * Math.min(7, ledCount));
                heat[idx] = Math.min(1.0, heat[idx] + 0.6 + Math.random() * 0.4);
            }
            // Diffuse heat upward
            for (int i = ledCount - 1; i >= 2; i--) {
                heat[i] = (heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3.0;
            }
            // Map heat to color (black -> red -> yellow -> white)
            for (int i = 0; i < ledCount; i++) {
                double h = heat[i];
                double r = Math.min(1.0, h * 3.0);
                double g = Math.min(1.0, Math.max(0, (h - 0.33) * 3.0));
                double b = Math.min(1.0, Math.max(0, (h - 0.66) * 3.0));
                buffer.setLED(i, new Color(r, g, b));
            }
            led.setData(buffer);
        }).withName("LED.Fire");
    }

    /**
     * Command for a smooth gradient between two colors.
     *
     * @param color1 start color
     * @param color2 end color
     */
    public Command gradient(Color color1, Color color2) {
        return run(() -> {
            for (int i = 0; i < ledCount; i++) {
                double t = (double) i / Math.max(1, ledCount - 1);
                Color blended = new Color(
                        color1.red + (color2.red - color1.red) * t,
                        color1.green + (color2.green - color1.green) * t,
                        color1.blue + (color2.blue - color1.blue) * t);
                buffer.setLED(i, blended);
            }
            led.setData(buffer);
        }).withName("LED.Gradient");
    }

    /**
     * Command for a scrolling gradient effect.
     *
     * @param color1 first color
     * @param color2 second color
     * @param speedHz scroll speed in cycles per second
     */
    public Command scrollingGradient(Color color1, Color color2, double speedHz) {
        return run(() -> {
            double offset = (Timer.getFPGATimestamp() * speedHz) % 1.0;
            for (int i = 0; i < ledCount; i++) {
                double t = ((double) i / ledCount + offset) % 1.0;
                // Ping-pong: 0->1->0
                double blend = t < 0.5 ? t * 2.0 : 2.0 - t * 2.0;
                Color blended = new Color(
                        color1.red + (color2.red - color1.red) * blend,
                        color1.green + (color2.green - color1.green) * blend,
                        color1.blue + (color2.blue - color1.blue) * blend);
                buffer.setLED(i, blended);
            }
            led.setData(buffer);
        }).withName("LED.ScrollingGradient");
    }

    /**
     * Command for a strobe effect (fast flashing).
     *
     * @param color strobe color
     * @param frequencyHz flash frequency (10-20 Hz is typical)
     */
    public Command strobe(Color color, double frequencyHz) {
        return run(() -> {
            double period = 1.0 / frequencyHz;
            boolean on = (Timer.getFPGATimestamp() % period) < (period * 0.1); // 10% duty cycle
            setSolidColor(on ? color : Color.kBlack);
        }).withName("LED.Strobe");
    }

    /**
     * Command for a larson scanner (Knight Rider / Cylon) effect.
     *
     * @param color scanner color
     * @param width width of the scanning dot
     * @param speedHz back-and-forth cycles per second
     */
    public Command larsonScanner(Color color, int width, double speedHz) {
        return run(() -> {
            double t = (Timer.getFPGATimestamp() * speedHz) % 1.0;
            // Ping-pong position
            double pos = t < 0.5 ? t * 2.0 : 2.0 - t * 2.0;
            int center = (int) (pos * (ledCount - 1));

            for (int i = 0; i < ledCount; i++) {
                int dist = Math.abs(i - center);
                if (dist < width) {
                    double brightness = 1.0 - (double) dist / width;
                    buffer.setLED(i, new Color(
                            color.red * brightness,
                            color.green * brightness,
                            color.blue * brightness));
                } else {
                    buffer.setLED(i, Color.kBlack);
                }
            }
            led.setData(buffer);
        }).withName("LED.LarsonScanner");
    }

    /**
     * Command for a dynamic progress bar (for climb, scoring, etc.).
     * Takes a supplier so the progress updates in real-time.
     *
     * @param color bar color
     * @param progressSupplier supplies progress value 0.0 to 1.0
     */
    public Command dynamicProgress(Color color, java.util.function.DoubleSupplier progressSupplier) {
        return run(() -> {
            double pct = Math.max(0, Math.min(1, progressSupplier.getAsDouble()));
            int litCount = (int) (ledCount * pct);
            for (int i = 0; i < ledCount; i++) {
                buffer.setLED(i, i < litCount ? color : Color.kBlack);
            }
            led.setData(buffer);
        }).withName("LED.DynamicProgress");
    }

    /**
     * Command that shows a status color based on a condition supplier.
     * Useful for game piece status, alignment feedback, etc.
     *
     * @param readyColor color when condition is true
     * @param notReadyColor color when condition is false
     * @param condition the condition to check
     */
    public Command statusIndicator(Color readyColor, Color notReadyColor,
                                    java.util.function.BooleanSupplier condition) {
        return run(() -> {
            setSolidColor(condition.getAsBoolean() ? readyColor : notReadyColor);
        }).withName("LED.StatusIndicator");
    }

    /**
     * Command that shows alignment progress.
     * Green center expands outward as alignment improves.
     *
     * @param alignedColor color when aligned
     * @param progressSupplier 0.0 = not aligned, 1.0 = fully aligned
     */
    public Command alignmentIndicator(Color alignedColor,
                                       java.util.function.DoubleSupplier progressSupplier) {
        return run(() -> {
            double progress = Math.max(0, Math.min(1, progressSupplier.getAsDouble()));
            int litFromCenter = (int) (ledCount / 2.0 * progress);
            int center = ledCount / 2;

            for (int i = 0; i < ledCount; i++) {
                int distFromCenter = Math.abs(i - center);
                if (distFromCenter <= litFromCenter) {
                    double brightness = 1.0 - (double) distFromCenter / (ledCount / 2.0);
                    buffer.setLED(i, new Color(
                            alignedColor.red * brightness,
                            alignedColor.green * brightness,
                            alignedColor.blue * brightness));
                } else {
                    buffer.setLED(i, Color.kBlack);
                }
            }
            led.setData(buffer);
        }).withName("LED.AlignmentIndicator");
    }

    /** Get the LED buffer for custom patterns. */
    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }

    /** Get the number of LEDs. */
    public int getLedCount() {
        return ledCount;
    }
}
