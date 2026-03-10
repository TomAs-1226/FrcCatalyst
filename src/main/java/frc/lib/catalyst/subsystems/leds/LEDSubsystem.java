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

    /** Get the LED buffer for custom patterns. */
    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }

    /** Get the number of LEDs. */
    public int getLedCount() {
        return ledCount;
    }
}
