package frc.lib.catalyst.mechanisms;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A PWM hobby/RC servo — a hood, a ratchet release, a funnel flapper, a camera-tilt, any small
 * position-controlled actuator wired to a PWM channel rather than a CAN motor.
 *
 * <p>A servo is <b>open-loop</b>: you command an angle and the servo's own internal controller holds
 * it, but there is no encoder feeding a position back. So this mechanism is deliberately simpler than
 * the CAN mechanisms — no PID, no Motion Magic, no gravity feedforward, no SysId. What it does give
 * you is the same ergonomics as the rest of Catalyst: a builder with sensible defaults, named
 * positions you can {@code goTo("DEPLOYED")}, angle clamping to the mechanism's real travel, live
 * telemetry, and a {@link #describe()} view for the sim dashboard.
 *
 * <p>Because there is no feedback, "measured" reads back the <em>commanded</em> angle. A short settle
 * time is the honest way to know a servo has physically finished moving; the state-machine binding
 * ({@link frc.lib.catalyst.statemachine.mech.Mechanisms#servo}) uses exactly that.
 *
 * <pre>{@code
 * ServoMechanism hood = new ServoMechanism(
 *     ServoMechanism.Config.builder()
 *         .name("Hood")
 *         .channel(0)                    // PWM port
 *         .angleRange(20, 60)            // physical travel, degrees
 *         .position("CLOSE", 20)
 *         .position("FAR", 55)
 *         .build());
 *
 * operator.a().onTrue(hood.goTo("FAR"));
 * operator.b().onTrue(hood.goTo("CLOSE"));
 * }</pre>
 *
 * @since 1.3.0
 */
public class ServoMechanism extends CatalystMechanism {

    private final Config config;
    private final Servo servo;

    /** Last angle we commanded, in degrees. Reported as the measured value (open-loop). */
    private double commandedAngleDeg;

    /**
     * Build a servo mechanism from a validated {@link Config}.
     *
     * @param config the configuration built with {@link Config#builder()}
     */
    public ServoMechanism(Config config) {
        super(config.name);
        this.config = config;
        this.servo = new Servo(config.channel);
        if (config.hasCustomBounds) {
            // Non-standard PWM pulse bounds (µs) for servos that don't use the 1.0–2.0 ms default.
            servo.setBoundsMicroseconds(config.maxUs, config.deadbandMaxUs, config.centerUs,
                    config.deadbandMinUs, config.minUs);
        }
        this.commandedAngleDeg = config.startAngleDeg;
        applyAngle(config.startAngleDeg);
    }

    // =========================================================================
    //                              CONTROL
    // =========================================================================

    /**
     * Command an angle in degrees, clamped to the configured {@code angleRange}. Open-loop: the
     * servo drives itself there and holds. Prefer the {@link #goTo(double)} command in bindings.
     *
     * @param degrees target angle; values outside the range are clamped
     */
    public void setAngle(double degrees) {
        applyAngle(degrees);
    }

    /**
     * Command a raw position in {@code [0, 1]} across the servo's full mechanical travel, ignoring the
     * configured angle range. For servos you'd rather think of as "0 to 1" than in degrees.
     */
    public void setPosition(double fraction) {
        double f = Math.max(0.0, Math.min(1.0, fraction));
        servo.set(f);
        commandedAngleDeg = config.minAngleDeg + f * (config.maxAngleDeg - config.minAngleDeg);
    }

    private void applyAngle(double degrees) {
        double clamped = Math.max(config.minAngleDeg, Math.min(config.maxAngleDeg, degrees));
        commandedAngleDeg = clamped;
        double span = config.maxAngleDeg - config.minAngleDeg;
        double fraction = span <= 0 ? 0.0 : (clamped - config.minAngleDeg) / span;
        servo.set(fraction);
    }

    // =========================================================================
    //                            COMMAND FACTORIES
    // =========================================================================

    /**
     * Drive to {@code degrees} and hold. The command never ends on its own (a servo holds its own
     * position, so this simply keeps the requirement and re-asserts the setpoint), so bind it with
     * {@code onTrue} for a latch or compose it in a sequence.
     */
    public Command goTo(double degrees) {
        return run(() -> applyAngle(degrees)).withName(name + ".goTo(" + fmt(degrees) + ")");
    }

    /**
     * Drive to a named position (configured with {@code Config.Builder.position(name, degrees)}).
     *
     * @throws IllegalArgumentException if {@code positionName} was never configured
     */
    public Command goTo(String positionName) {
        Double target = config.namedPositions.get(positionName);
        if (target == null) {
            throw new IllegalArgumentException("Unknown servo position '" + positionName
                    + "'. Available: " + config.namedPositions.keySet());
        }
        return goTo(target).withName(name + ".goTo(" + positionName + ")");
    }

    /** Re-assert the last commanded angle. A no-op mover that just holds where you are. */
    public Command hold() {
        return run(() -> applyAngle(commandedAngleDeg)).withName(name + ".hold");
    }

    // =========================================================================
    //                              STATE / TELEMETRY
    // =========================================================================

    /** The last commanded angle in degrees. There is no encoder, so this is the best "position" available. */
    public double getAngle() {
        return commandedAngleDeg;
    }

    /** The commanded position as a fraction of travel, {@code [0, 1]}. */
    public double getPosition() {
        return servo.get();
    }

    /** Minimum configured angle in degrees. */
    public double getMinAngle() {
        return config.minAngleDeg;
    }

    /** Maximum configured angle in degrees. */
    public double getMaxAngle() {
        return config.maxAngleDeg;
    }

    /** The named position presets, as an immutable map (used by the state-machine binding). */
    public Map<String, Double> getNamedPositions() {
        return config.namedPositions;
    }

    /** The underlying WPILib {@link Servo}, for advanced use. */
    public Servo getServo() {
        return servo;
    }

    @Override
    public MechanismView describe() {
        return MechanismView.of(name, "servo")
                .value(commandedAngleDeg, "deg")
                .setpoint(commandedAngleDeg)
                .range(config.minAngleDeg, config.maxAngleDeg)
                .extra("openLoop", true)
                .build();
    }

    @Override
    protected void stop() {
        // Cut the PWM signal. The servo goes limp rather than fighting to hold — the safe "off"
        // for a mechanism a driver has finished with.
        servo.setDisabled();
    }

    @Override
    protected void updateTelemetry() {
        log("Angle", commandedAngleDeg);
        log("Position", servo.get());
    }

    private static String fmt(double v) {
        return String.format("%.1f", v);
    }

    // =========================================================================
    //                                CONFIG
    // =========================================================================

    /** Immutable, validated configuration for a {@link ServoMechanism}. */
    public static class Config {
        final String name;
        final int channel;
        final double minAngleDeg;
        final double maxAngleDeg;
        final double startAngleDeg;
        final Map<String, Double> namedPositions;
        final boolean hasCustomBounds;
        final int maxUs, deadbandMaxUs, centerUs, deadbandMinUs, minUs;

        private Config(Builder b) {
            this.name = b.name;
            this.channel = b.channel;
            this.minAngleDeg = b.minAngleDeg;
            this.maxAngleDeg = b.maxAngleDeg;
            this.startAngleDeg = b.startAngleDeg;
            this.namedPositions = Map.copyOf(b.namedPositions);
            this.hasCustomBounds = b.hasCustomBounds;
            this.maxUs = b.maxUs;
            this.deadbandMaxUs = b.deadbandMaxUs;
            this.centerUs = b.centerUs;
            this.deadbandMinUs = b.deadbandMinUs;
            this.minUs = b.minUs;
        }

        /** Start a new configuration. */
        public static Builder builder() {
            return new Builder();
        }

        /** Fluent builder for {@link ServoMechanism} configuration. */
        public static class Builder {
            private String name = "Servo";
            private int channel = -1;
            private double minAngleDeg = 0.0;
            private double maxAngleDeg = 180.0;
            private double startAngleDeg = Double.NaN;
            private final Map<String, Double> namedPositions = new LinkedHashMap<>();
            private boolean hasCustomBounds = false;
            private int maxUs = 2000, deadbandMaxUs = 1800, centerUs = 1500, deadbandMinUs = 1200, minUs = 1000;

            /** Display name — also the telemetry/log key. */
            public Builder name(String name) {
                this.name = name;
                return this;
            }

            /** PWM channel the servo is wired to (required). */
            public Builder channel(int channel) {
                this.channel = channel;
                return this;
            }

            /**
             * The physical travel of the mechanism in degrees. {@code goTo(...)} clamps to this and
             * maps it across the servo's full stroke. Defaults to {@code 0..180}.
             */
            public Builder angleRange(double minDegrees, double maxDegrees) {
                this.minAngleDeg = minDegrees;
                this.maxAngleDeg = maxDegrees;
                return this;
            }

            /** Angle the servo is driven to at construction. Defaults to the minimum. */
            public Builder startAngle(double degrees) {
                this.startAngleDeg = degrees;
                return this;
            }

            /** Add a named position preset you can drive to with {@code goTo("NAME")}. */
            public Builder position(String name, double degrees) {
                this.namedPositions.put(name, degrees);
                return this;
            }

            /**
             * Custom PWM pulse bounds in microseconds, for servos that don't use the standard
             * 1.0–2.0 ms range (e.g. some linear actuators or continuous-rotation servos). Order
             * matches WPILib's {@code Servo.setBoundsMicroseconds}: max, deadband-max, center,
             * deadband-min, min.
             */
            public Builder pulseBoundsMicroseconds(int max, int deadbandMax, int center,
                                                   int deadbandMin, int min) {
                this.hasCustomBounds = true;
                this.maxUs = max;
                this.deadbandMaxUs = deadbandMax;
                this.centerUs = center;
                this.deadbandMinUs = deadbandMin;
                this.minUs = min;
                return this;
            }

            /** Validate and build. */
            public Config build() {
                if (channel < 0) {
                    throw new IllegalArgumentException("ServoMechanism '" + name
                            + "' requires a PWM channel — call .channel(port)");
                }
                if (maxAngleDeg <= minAngleDeg) {
                    throw new IllegalArgumentException("ServoMechanism '" + name
                            + "' angleRange max (" + maxAngleDeg + ") must exceed min (" + minAngleDeg + ")");
                }
                if (Double.isNaN(startAngleDeg)) {
                    startAngleDeg = minAngleDeg;
                }
                for (var e : namedPositions.entrySet()) {
                    double d = e.getValue();
                    if (d < minAngleDeg || d > maxAngleDeg) {
                        throw new IllegalArgumentException("ServoMechanism '" + name + "' position '"
                                + e.getKey() + "' (" + d + " deg) is outside angleRange ["
                                + minAngleDeg + ", " + maxAngleDeg + "]");
                    }
                }
                return new Config(this);
            }
        }
    }
}
