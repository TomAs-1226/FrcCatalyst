package frc.lib.catalyst.mechanisms;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.hardware.MotorType;

/**
 * Generic flywheel mechanism. Use for shooters, accelerator wheels,
 * or any mechanism that requires precise velocity control.
 *
 * <p>Supports single or dual motor configurations. Dual motors can
 * spin at different speeds for spin control.
 *
 * <p>Example usage:
 * <pre>{@code
 * // Single flywheel
 * FlywheelMechanism shooter = new FlywheelMechanism(
 *     FlywheelMechanism.Config.builder()
 *         .name("Shooter")
 *         .motor(20)
 *         .gearRatio(1.5)
 *         .pid(0.3, 0, 0)
 *         .feedforward(0.12, 0.11)
 *         .velocityTolerance(3.0)
 *         .currentLimit(60)
 *         .build());
 *
 * // Dual flywheel (top/bottom for spin)
 * FlywheelMechanism dualShooter = new FlywheelMechanism(
 *     FlywheelMechanism.Config.builder()
 *         .name("Shooter")
 *         .motor(20)
 *         .secondMotor(21)
 *         .gearRatio(1.5)
 *         .pid(0.3, 0, 0)
 *         .feedforward(0.12, 0.11)
 *         .velocityTolerance(3.0)
 *         .build());
 * }</pre>
 */
public class FlywheelMechanism extends CatalystMechanism {

    private final Config config;
    private final CatalystMotor primaryMotor;
    private final CatalystMotor secondaryMotor;

    // Simulation
    private FlywheelSim primarySim;
    private FlywheelSim secondarySim;

    // State
    private double primarySetpointRPS = 0;
    private double secondarySetpointRPS = 0;

    public FlywheelMechanism(Config config) {
        super(config.name);
        this.config = config;

        // Primary motor
        this.primaryMotor = CatalystMotor.builder(config.primaryMotorCanId)
                .name(config.name + "Primary")
                .canBus(config.canBus)
                .inverted(config.primaryInverted)
                .brakeMode(false) // flywheels usually coast
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit)
                .gearRatio(config.gearRatio)
                .pid(config.kP, config.kI, config.kD)
                .feedforward(config.kS, config.kV, config.kA)
                .build();

        // Secondary motor (independent, not a follower)
        if (config.secondaryMotorCanId >= 0) {
            this.secondaryMotor = CatalystMotor.builder(config.secondaryMotorCanId)
                    .name(config.name + "Secondary")
                    .canBus(config.canBus)
                    .inverted(config.secondaryInverted)
                    .brakeMode(false)
                    .currentLimit(config.currentLimit)
                    .statorCurrentLimit(config.statorCurrentLimit)
                    .gearRatio(config.gearRatio)
                    .pid(config.kP, config.kI, config.kD)
                    .feedforward(config.kS, config.kV, config.kA)
                    .build();
        } else {
            this.secondaryMotor = null;
        }

        // Simulation
        if (RobotBase.isSimulation()) {
            DCMotor motorModel = config.motorType.getDCMotor(1);
            primarySim = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(motorModel, config.moi, config.gearRatio),
                    motorModel);
            if (secondaryMotor != null) {
                secondarySim = new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(motorModel, config.moi, config.gearRatio),
                        motorModel);
            }
        }
    }

    // --- Getters ---

    /** Get primary flywheel velocity in rotations per second. */
    public double getVelocity() {
        return primaryMotor.getVelocity();
    }

    /** Get secondary flywheel velocity in RPS (0 if no secondary). */
    public double getSecondaryVelocity() {
        return secondaryMotor != null ? secondaryMotor.getVelocity() : 0;
    }

    /** Get primary setpoint in RPS. */
    public double getSetpoint() {
        return primarySetpointRPS;
    }

    /** Get secondary setpoint in RPS. */
    public double getSecondarySetpoint() {
        return secondarySetpointRPS;
    }

    /** Check if primary flywheel is at target velocity within tolerance. */
    public boolean atSpeed() {
        if (primarySetpointRPS == 0) return false;
        boolean primaryOk = Math.abs(getVelocity() - primarySetpointRPS) < config.velocityTolerance;
        if (secondaryMotor == null) return primaryOk;
        boolean secondaryOk = Math.abs(getSecondaryVelocity() - secondarySetpointRPS) < config.velocityTolerance;
        return primaryOk && secondaryOk;
    }

    /** Trigger that fires when flywheel is at target speed. */
    public Trigger atSpeedTrigger() {
        return new Trigger(this::atSpeed);
    }

    // --- Command Factories ---

    /** Command to spin up to a target velocity (rotations per second). */
    public Command spinUp(double velocityRPS) {
        return run(() -> {
            primarySetpointRPS = velocityRPS;
            secondarySetpointRPS = velocityRPS;
            primaryMotor.setVelocity(velocityRPS);
            if (secondaryMotor != null) {
                secondaryMotor.setVelocity(velocityRPS);
            }
            setState("SpinUp " + String.format("%.0f", velocityRPS) + " RPS");
        }).finallyDo(() -> {
            primaryMotor.stop();
            if (secondaryMotor != null) secondaryMotor.stop();
            primarySetpointRPS = 0;
            secondarySetpointRPS = 0;
            setState("Idle");
        }).withName(name + ".SpinUp(" + String.format("%.0f", velocityRPS) + ")");
    }

    /**
     * Command to spin up dual flywheels at different speeds.
     * Useful for spin control (backspin/topspin).
     */
    public Command spinUp(double primaryRPS, double secondaryRPS) {
        if (secondaryMotor == null) {
            return spinUp(primaryRPS);
        }
        return run(() -> {
            primarySetpointRPS = primaryRPS;
            secondarySetpointRPS = secondaryRPS;
            primaryMotor.setVelocity(primaryRPS);
            secondaryMotor.setVelocity(secondaryRPS);
            setState("SpinUp " + String.format("%.0f/%.0f", primaryRPS, secondaryRPS) + " RPS");
        }).finallyDo(() -> {
            primaryMotor.stop();
            secondaryMotor.stop();
            primarySetpointRPS = 0;
            secondarySetpointRPS = 0;
            setState("Idle");
        }).withName(name + ".SpinUp(" + String.format("%.0f/%.0f", primaryRPS, secondaryRPS) + ")");
    }

    /** Command to spin up and wait until at speed. */
    public Command spinUpAndWait(double velocityRPS) {
        return spinUp(velocityRPS).until(this::atSpeed)
                .withName(name + ".SpinUpAndWait(" + String.format("%.0f", velocityRPS) + ")");
    }

    /** Command to run at a set voltage (open loop). */
    public Command runVoltage(double volts) {
        return run(() -> {
            primaryMotor.setVoltage(volts);
            if (secondaryMotor != null) secondaryMotor.setVoltage(volts);
            setState("Voltage " + String.format("%.1fV", volts));
        }).finallyDo(() -> {
            primaryMotor.stop();
            if (secondaryMotor != null) secondaryMotor.stop();
            setState("Idle");
        }).withName(name + ".RunVoltage(" + String.format("%.1f", volts) + ")");
    }

    // --- Internals ---

    @Override
    protected void stop() {
        primaryMotor.stop();
        if (secondaryMotor != null) secondaryMotor.stop();
        primarySetpointRPS = 0;
        secondarySetpointRPS = 0;
        setState("Stopped");
    }

    @Override
    protected void updateTelemetry() {
        primaryMotor.updateTelemetry();
        log("VelocityRPS", getVelocity());
        log("SetpointRPS", primarySetpointRPS);
        log("AtSpeed", atSpeed());
        if (secondaryMotor != null) {
            secondaryMotor.updateTelemetry();
            log("SecondaryVelocityRPS", getSecondaryVelocity());
            log("SecondarySetpointRPS", secondarySetpointRPS);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (primarySim != null) {
            var simState = primaryMotor.getTalonFX().getSimState();
            primarySim.setInput(simState.getMotorVoltage());
            primarySim.update(0.02);
            simState.setRotorVelocity(primarySim.getAngularVelocityRPM() / 60.0 * config.gearRatio);
        }
        if (secondarySim != null && secondaryMotor != null) {
            var simState = secondaryMotor.getTalonFX().getSimState();
            secondarySim.setInput(simState.getMotorVoltage());
            secondarySim.update(0.02);
            simState.setRotorVelocity(secondarySim.getAngularVelocityRPM() / 60.0 * config.gearRatio);
        }
    }

    public CatalystMotor getPrimaryMotor() { return primaryMotor; }
    public CatalystMotor getSecondaryMotor() { return secondaryMotor; }

    // ===========================================
    //                  CONFIG
    // ===========================================

    public static class Config {
        final String name;
        final int primaryMotorCanId;
        final int secondaryMotorCanId;
        final String canBus;
        final boolean primaryInverted;
        final boolean secondaryInverted;
        final MotorType motorType;
        final double gearRatio;
        final double moi;
        final double currentLimit;
        final double statorCurrentLimit;
        final double velocityTolerance;
        final double kP, kI, kD;
        final double kS, kV, kA;

        private Config(Builder b) {
            this.name = b.name;
            this.primaryMotorCanId = b.primaryMotorCanId;
            this.secondaryMotorCanId = b.secondaryMotorCanId;
            this.canBus = b.canBus;
            this.primaryInverted = b.primaryInverted;
            this.secondaryInverted = b.secondaryInverted;
            this.motorType = b.motorType;
            this.gearRatio = b.gearRatio;
            this.moi = b.moi;
            this.currentLimit = b.currentLimit;
            this.statorCurrentLimit = b.statorCurrentLimit;
            this.velocityTolerance = b.velocityTolerance;
            this.kP = b.kP; this.kI = b.kI; this.kD = b.kD;
            this.kS = b.kS; this.kV = b.kV; this.kA = b.kA;
        }

        public static Builder builder() {
            return new Builder();
        }

        public static class Builder {
            private String name = "FlywheelMechanism";
            private int primaryMotorCanId = 0;
            private int secondaryMotorCanId = -1;
            private String canBus = "";
            private boolean primaryInverted = false;
            private boolean secondaryInverted = false;
            private MotorType motorType = MotorType.KRAKEN_X60;
            private double gearRatio = 1.0;
            private double moi = 0.01; // kg*m^2
            private double currentLimit = 60;
            private double statorCurrentLimit = 120;
            private double velocityTolerance = 3.0; // RPS
            private double kP = 0, kI = 0, kD = 0;
            private double kS = 0, kV = 0, kA = 0;

            public Builder name(String name) { this.name = name; return this; }
            public Builder motor(int canId) { this.primaryMotorCanId = canId; return this; }

            /** Add a second motor for dual-flywheel setups (independent, NOT a follower). */
            public Builder secondMotor(int canId) { this.secondaryMotorCanId = canId; return this; }

            public Builder canBus(String canBus) { this.canBus = canBus; return this; }
            public Builder primaryInverted(boolean inv) { this.primaryInverted = inv; return this; }
            public Builder secondaryInverted(boolean inv) { this.secondaryInverted = inv; return this; }
            /** Set the motor type for accurate simulation (default: Kraken X60). */
            public Builder motorType(MotorType type) { this.motorType = type; return this; }
            public Builder gearRatio(double ratio) { this.gearRatio = ratio; return this; }

            /** Moment of inertia in kg*m^2 (for simulation). */
            public Builder moi(double kgm2) { this.moi = kgm2; return this; }

            public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
            public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }

            /** How close to target speed (in RPS) counts as "at speed". */
            public Builder velocityTolerance(double rps) { this.velocityTolerance = rps; return this; }

            public Builder pid(double kP, double kI, double kD) {
                this.kP = kP; this.kI = kI; this.kD = kD; return this;
            }

            public Builder feedforward(double kS, double kV) {
                this.kS = kS; this.kV = kV; return this;
            }

            public Builder feedforward(double kS, double kV, double kA) {
                this.kS = kS; this.kV = kV; this.kA = kA; return this;
            }

            public Config build() {
                if (primaryMotorCanId == 0) {
                    throw new IllegalStateException("Motor CAN ID must be set");
                }
                return new Config(this);
            }
        }
    }
}
