package frc.lib.catalyst.mechanisms;

import frc.lib.catalyst.command.CatalystCommand;
import com.ctre.phoenix6.signals.GravityTypeValue;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.hardware.CatalystMotor.FollowerSpec;
import frc.lib.catalyst.io.RollerMechanismInputs;
import frc.lib.catalyst.util.HealthMonitor;
import frc.lib.catalyst.util.RumbleEvents;

import java.util.ArrayList;
import java.util.List;

/**
 * Generic roller mechanism. Use for intakes, conveyors, indexers,
 * or any mechanism that spins rollers at a set speed.
 *
 * <p>Supports game piece detection via:
 * <ul>
 *   <li>Stall current detection (current spike = game piece acquired)</li>
 *   <li>Beam break sensor (DIO)</li>
 * </ul>
 *
 * <p>Example usage:
 * <pre>{@code
 * RollerMechanism intake = new RollerMechanism(
 *     RollerMechanism.Config.builder()
 *         .name("Intake")
 *         .motor(16)
 *         .intakeSpeed(0.8)
 *         .ejectSpeed(-0.6)
 *         .currentLimit(30)
 *         .stallDetection(25, 0.2)
 *         .build());
 * }</pre>
 */
public class RollerMechanism extends CatalystMechanism {

    private final Config config;
    private final CatalystMotor motor;
    private final DigitalInput beamBreak;

    // Stall detection state
    private final Timer stallTimer = new Timer();
    private boolean hasPiece = false;
    private boolean stallTimerStarted = false;

    private final RollerMechanismInputs inputs = new RollerMechanismInputs();

    // Simulation (null on a real robot). A roller has no positional state; we
    // model spin-up so the motor velocity + current are realistic in sim.
    private FlywheelSim sim;

    public RollerMechanism(Config config) {
        super(config.name);
        this.config = config;

        CatalystMotor.Builder motorBuilder = CatalystMotor.builder(config.motorCanId)
                .name(config.name + "Motor")
                .canBus(config.canBus)
                .inverted(config.inverted)
                .brakeMode(config.brakeMode)
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit);
        for (FollowerSpec spec : config.followers) {
            motorBuilder.withFollower(spec.canId(), spec.oppose());
        }
        this.motor = motorBuilder.build();

        // Beam break sensor
        if (config.beamBreakPort >= 0) {
            beamBreak = new DigitalInput(config.beamBreakPort);
        } else {
            beamBreak = null;
        }

        HealthMonitor.standardMotorChecks(name, motor, config.statorCurrentLimit, 70);

        if (RobotBase.isSimulation()) {
            // Rollers have no gear/MOI config; a light direct-drive Kraken model
            // is plenty to make the wheel spin and draw current in sim.
            DCMotor model = DCMotor.getKrakenX60(1 + config.followers.size());
            sim = new FlywheelSim(Models.flywheelFromPhysicalConstants(model, 0.0008, 1.0), model);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (sim != null) {
            var simState = motor.getTalonFX().getSimState();
            sim.setInput(simState.getMotorVoltage());
            sim.update(0.02);
            simState.setRotorVelocity((sim.getAngularVelocity() / (2 * Math.PI)));
        }
    }

    @Override
    public MechanismView describe() {
        return MechanismView.of(name, "roller")
                .value(getSpeed(), "frac")
                .velocity(motor.getVelocity())
                .current(getCurrent())
                .extra("hasPiece", hasPiece())
                .build();
    }

    // --- Getters ---

    /** Check if a game piece is detected (via beam break or stall detection). */
    public boolean hasPiece() {
        if (beamBreak != null) {
            return !beamBreak.get(); // beam break is normally open, false = broken = has piece
        }
        return hasPiece;
    }

    /**
     * Force the simulated game-piece state. Sim only — ignored on a real robot,
     * where detection comes from the beam break or stall logic. Lets the
     * dashboard demo intake/handoff sequencing without a physical sensor.
     */
    public void setSimHasPiece(boolean present) {
        if (RobotBase.isSimulation()) {
            hasPiece = present;
        }
    }

    /** Trigger that fires when a game piece is detected. */
    public Trigger hasPieceTrigger() {
        return new Trigger(this::hasPiece);
    }

    /** Get current motor speed as a fraction [-1, 1]. */
    public double getSpeed() {
        return motor.getAppliedVoltage() / 12.0;
    }

    /** Get current draw in amps. */
    public double getCurrent() {
        return motor.getStatorCurrent();
    }

    // --- Command Factories ---

    /**
     * Command to run intake rollers.
     * If stall detection or beam break is configured, ends when game piece is detected.
     * Otherwise runs indefinitely (use whileTrue).
     */
    public CatalystCommand intake() {
        return run(() -> {
            motor.setPercent(config.intakeSpeed);
            updateStallDetection();
            setState("Intaking");
        }).untilTrue(this::hasPiece)
                .finallyDo(() -> {
                    motor.stop();
                    setState("Idle");
                })
                .withName(name + ".Intake");
    }

    /**
     * Command to intake without auto-stopping.
     * Useful when you want manual control over when to stop.
     */
    public CatalystCommand intakeContinuous() {
        return run(() -> {
            motor.setPercent(config.intakeSpeed);
            setState("Intaking");
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".IntakeContinuous");
    }

    /** Command to eject game pieces. */
    public CatalystCommand eject() {
        return run(() -> {
            motor.setPercent(config.ejectSpeed);
            hasPiece = false;
            setState("Ejecting");
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".Eject");
    }

    /** Command to run rollers at a custom speed [-1, 1]. */
    public CatalystCommand runAtSpeed(double speed) {
        return run(() -> {
            motor.setPercent(speed);
            setState("Running " + String.format("%.0f%%", speed * 100));
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".RunAt(" + String.format("%.0f%%", speed * 100) + ")");
    }

    /** Command to run rollers at a custom voltage [-12, 12]. */
    public CatalystCommand runAtVoltage(double volts) {
        return run(() -> {
            motor.setVoltage(volts);
            setState("Running " + String.format("%.1fV", volts));
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".RunAt(" + String.format("%.1fV", volts) + ")");
    }

    /** Reset the has-piece state. */
    public CatalystCommand resetPieceDetection() {
        return runOnce(() -> {
            hasPiece = false;
            stallTimerStarted = false;
            setState("Reset");
        }).withName(name + ".ResetDetection");
    }

    /**
     * Command to intake with speed ramping.
     * Gradually increases speed to prevent wheel slip at initial contact.
     * @param rampTimeSeconds time to ramp from 0 to full intake speed
     */
    public CatalystCommand intakeWithRamp(double rampTimeSeconds) {
        final Timer rampTimer = new Timer();
        return run(() -> {
            double elapsed = rampTimer.get();
            double rampFraction = Math.min(1.0, elapsed / rampTimeSeconds);
            motor.setPercent(config.intakeSpeed * rampFraction);
            updateStallDetection();
            setState("IntakeRamp " + String.format("%.0f%%", rampFraction * 100));
        }).untilTrue(this::hasPiece)
                .beforeStarting(rampTimer::restart)
                .finallyDo(() -> {
                    motor.stop();
                    setState("Idle");
                })
                .withName(name + ".IntakeWithRamp");
    }

    /**
     * Command to pulse the rollers — short bursts for unjamming.
     * @param onTimeSeconds time rollers run per pulse
     * @param offTimeSeconds time between pulses
     * @param speed speed during pulse [-1, 1]
     */
    public CatalystCommand pulse(double onTimeSeconds, double offTimeSeconds, double speed) {
        return run(() -> {
            double period = onTimeSeconds + offTimeSeconds;
            double phase = Timer.getTimestamp() % period;
            if (phase < onTimeSeconds) {
                motor.setPercent(speed);
            } else {
                motor.stop();
            }
            setState("Pulsing");
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".Pulse");
    }

    /**
     * Command to feed a game piece at a specific voltage.
     * Better for consistent feed regardless of battery voltage.
     *
     * @param volts feed voltage
     */
    public CatalystCommand feedVoltage(double volts) {
        return run(() -> {
            motor.setVoltage(volts);
            setState("Feeding " + String.format("%.1fV", volts));
        }).finallyDo(() -> {
            motor.stop();
            setState("Idle");
        }).withName(name + ".Feed");
    }

    // --- Stall Detection ---

    private void updateStallDetection() {
        if (config.stallCurrentThreshold <= 0) return;

        double current = motor.getStatorCurrent();
        if (current >= config.stallCurrentThreshold) {
            if (!stallTimerStarted) {
                stallTimer.restart();
                stallTimerStarted = true;
            }
            if (stallTimer.get() >= config.stallTimeThreshold) {
                hasPiece = true;
            }
        } else {
            stallTimerStarted = false;
        }
    }

    // --- Internals ---

    @Override
    protected void stop() {
        motor.stop();
        setState("Stopped");
    }

    @Override
    protected void updateTelemetry() {
        motor.updateTelemetry();

        inputs.dutyCycle = getSpeed();
        inputs.statorCurrentAmps = motor.getStatorCurrent();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent();
        inputs.appliedVolts = motor.getAppliedVoltage();
        inputs.temperatureC = motor.getTemperature();
        inputs.motorRotations = motor.getPosition();
        inputs.hasPiece = hasPiece();
        inputs.beamBreakBroken = beamBreak != null && !beamBreak.get();
        inputs.stalled = hasPiece;
        processInputs(inputs);

        // Per-key telemetry for v0.2 dashboard compatibility.
        log("Speed", inputs.dutyCycle);
        log("CurrentAmps", inputs.statorCurrentAmps);
        log("HasPiece", inputs.hasPiece);
        log("MotorRotations", inputs.motorRotations);

        HealthMonitor.getInstance().update();
    }

    /** Get the underlying motor for advanced use. */
    @Override
    protected CatalystMotor primaryMotorForSysId() {
        return motor;
    }

    /** Rumble on each piece pickup. */
    @Override
    public void bindRumble(RumbleEvents events,
                           RumbleEvents.Pattern pattern, RumbleEvents.Channel channel) {
        bindRumble(events, hasPieceTrigger(), pattern, channel);
    }

    public CatalystMotor getMotor() {
        return motor;
    }

    // ===========================================
    //                  CONFIG
    // ===========================================

    public static class Config {
        final String name;
        final int motorCanId;
        final List<FollowerSpec> followers;
        final String canBus;
        final boolean inverted;
        final boolean brakeMode;
        final double currentLimit;
        final double statorCurrentLimit;
        final double intakeSpeed;
        final double ejectSpeed;
        final double stallCurrentThreshold;
        final double stallTimeThreshold;
        final int beamBreakPort;

        private Config(Builder b) {
            this.name = b.name;
            this.motorCanId = b.motorCanId;
            this.followers = List.copyOf(b.followers);
            this.canBus = b.canBus;
            this.inverted = b.inverted;
            this.brakeMode = b.brakeMode;
            this.currentLimit = b.currentLimit;
            this.statorCurrentLimit = b.statorCurrentLimit;
            this.intakeSpeed = b.intakeSpeed;
            this.ejectSpeed = b.ejectSpeed;
            this.stallCurrentThreshold = b.stallCurrentThreshold;
            this.stallTimeThreshold = b.stallTimeThreshold;
            this.beamBreakPort = b.beamBreakPort;
        }

        public static Builder builder() {
            return new Builder();
        }

        public static class Builder {
            private String name = "RollerMechanism";
            private int motorCanId = 0;
            private final List<FollowerSpec> followers = new ArrayList<>();
            private String canBus = "";
            private boolean inverted = false;
            private boolean brakeMode = false;
            private double currentLimit = 30;
            private double statorCurrentLimit = 60;
            private double intakeSpeed = 0.8;
            private double ejectSpeed = -0.6;
            private double stallCurrentThreshold = -1; // disabled by default
            private double stallTimeThreshold = 0.2;
            private int beamBreakPort = -1; // disabled by default

            public Builder name(String name) { this.name = name; return this; }
            public Builder motor(int canId) { this.motorCanId = canId; return this; }

            /**
             * Attach a follower roller. Additive — call once per follower
             * for two-roller intakes with mirrored motors.
             */
            public Builder follower(int canId, boolean oppose) {
                this.followers.add(new FollowerSpec(canId, oppose));
                return this;
            }

            /** Convenience: follower with {@code oppose = false}. */
            public Builder follower(int canId) { return follower(canId, false); }

            /** Add several followers in one call. */
            public Builder followers(FollowerSpec... specs) {
                for (FollowerSpec s : specs) this.followers.add(s);
                return this;
            }

            public Builder canBus(String canBus) { this.canBus = canBus; return this; }
            public Builder inverted(boolean inverted) { this.inverted = inverted; return this; }
            public Builder brakeMode(boolean brakeMode) { this.brakeMode = brakeMode; return this; }
            public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
            public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }

            /** Intake speed as duty cycle [-1, 1]. Positive = intake direction. */
            public Builder intakeSpeed(double speed) { this.intakeSpeed = speed; return this; }

            /** Eject speed as duty cycle [-1, 1]. Should be opposite sign of intakeSpeed. */
            public Builder ejectSpeed(double speed) { this.ejectSpeed = speed; return this; }

            /**
             * Enable stall-based game piece detection.
             * @param currentAmps current threshold in amps
             * @param timeSeconds how long current must exceed threshold
             */
            public Builder stallDetection(double currentAmps, double timeSeconds) {
                this.stallCurrentThreshold = currentAmps;
                this.stallTimeThreshold = timeSeconds;
                return this;
            }

            /** Enable beam break sensor for game piece detection. */
            public Builder beamBreak(int dioPort) {
                this.beamBreakPort = dioPort;
                return this;
            }

            public Config build() {
                if (motorCanId == 0) {
                    throw new IllegalStateException("Motor CAN ID must be set");
                }
                return new Config(this);
            }
        }
    }
}
