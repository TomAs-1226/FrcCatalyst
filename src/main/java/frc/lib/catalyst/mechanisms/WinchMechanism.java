package frc.lib.catalyst.mechanisms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.io.WinchMechanismInputs;
import frc.lib.catalyst.util.HealthMonitor;

import java.util.function.DoubleSupplier;

/**
 * Generic winch mechanism. Use for climbers, deployment mechanisms,
 * or any mechanism that extends/retracts using a spool or winch.
 *
 * <p>Provides simple extend/retract commands with position limits
 * and current monitoring for safety.
 *
 * <p>Example usage:
 * <pre>{@code
 * WinchMechanism climber = new WinchMechanism(
 *     WinchMechanism.Config.builder()
 *         .name("Climber")
 *         .motor(25)
 *         .gearRatio(25.0)
 *         .spoolRadius(0.02)
 *         .range(0.0, 0.6)
 *         .extendSpeed(0.8)
 *         .retractSpeed(-1.0)
 *         .currentLimit(80)
 *         .build());
 * }</pre>
 */
public class WinchMechanism extends CatalystMechanism {

    private final Config config;
    private final CatalystMotor motor;
    private final CatalystMotor secondMotor;

    private final WinchMechanismInputs inputs = new WinchMechanismInputs();
    private boolean hasBeenZeroed = false;

    // Simulation (created only in sim). Linear model when a spool radius is set,
    // otherwise a rotational model so rotation-only winches still move in sim.
    private ElevatorSim elevatorSim;
    private DCMotorSim rotationSim;

    public WinchMechanism(Config config) {
        super(config.name);
        this.config = config;

        CatalystMotor.Builder motorBuilder = CatalystMotor.builder(config.motorCanId)
                .name(config.name + "Motor")
                .canBus(config.canBus)
                .inverted(config.inverted)
                .brakeMode(true) // always brake for safety
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit)
                .gearRatio(config.gearRatio)
                .useFocControl(config.useFOC);

        // Soft limits in mechanism rotations
        if (config.spoolRadius > 0) {
            double minRot = config.minPosition / (2.0 * Math.PI * config.spoolRadius);
            double maxRot = config.maxPosition / (2.0 * Math.PI * config.spoolRadius);
            motorBuilder.softLimits(minRot, maxRot);
        }

        this.motor = motorBuilder.build();

        // Second motor (for dual climber arms)
        if (config.secondMotorCanId >= 0) {
            this.secondMotor = CatalystMotor.builder(config.secondMotorCanId)
                    .name(config.name + "Motor2")
                    .canBus(config.canBus)
                    .inverted(config.secondInverted)
                    .brakeMode(true)
                    .currentLimit(config.currentLimit)
                    .statorCurrentLimit(config.statorCurrentLimit)
                    .gearRatio(config.gearRatio)
                    .build();
        } else {
            this.secondMotor = null;
        }

        HealthMonitor.standardMotorChecks(name, motor, config.statorCurrentLimit, 70);
        if (secondMotor != null) {
            HealthMonitor.standardMotorChecks(name, "Sec", secondMotor, config.statorCurrentLimit, 70);
        }

        if (RobotBase.isSimulation()) {
            DCMotor model = DCMotor.getKrakenX60(secondMotor != null ? 2 : 1);
            if (config.spoolRadius > 0) {
                elevatorSim = new ElevatorSim(model, config.gearRatio, config.loadMass,
                        config.spoolRadius, config.minPosition, config.maxPosition,
                        false, config.minPosition);
            } else {
                rotationSim = new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(model, 0.01, config.gearRatio), model);
            }
        }
    }

    // --- Simulation ---

    @Override
    public void simulationPeriodic() {
        if (elevatorSim != null) {
            var simState = motor.getTalonFX().getSimState();
            elevatorSim.setInputVoltage(simState.getMotorVoltage());
            elevatorSim.update(0.02);
            double circ = 2.0 * Math.PI * config.spoolRadius;
            double spoolRot = elevatorSim.getPositionMeters() / circ;
            double spoolRps = elevatorSim.getVelocityMetersPerSecond() / circ;
            applyRotorSim(spoolRot, spoolRps);
        } else if (rotationSim != null) {
            var simState = motor.getTalonFX().getSimState();
            rotationSim.setInputVoltage(simState.getMotorVoltage());
            rotationSim.update(0.02);
            applyRotorSim(rotationSim.getAngularPositionRotations(),
                    rotationSim.getAngularVelocityRPM() / 60.0);
        }
    }

    /** Push a spool position (rotations) and velocity (rps) onto both motors' rotor sim state. */
    private void applyRotorSim(double spoolRotations, double spoolRps) {
        double rotorPos = spoolRotations * config.gearRatio;
        double rotorVel = spoolRps * config.gearRatio;
        var simState = motor.getTalonFX().getSimState();
        simState.setRawRotorPosition(rotorPos);
        simState.setRotorVelocity(rotorVel);
        if (secondMotor != null) {
            var s2 = secondMotor.getTalonFX().getSimState();
            s2.setRawRotorPosition(rotorPos);
            s2.setRotorVelocity(rotorVel);
        }
    }

    // --- Conversions ---

    private double getPositionMeters() {
        if (config.spoolRadius <= 0) return motor.getPosition();
        return motor.getPosition() * (2.0 * Math.PI * config.spoolRadius);
    }

    // --- Getters ---

    /** Get current position in meters (if spool radius set) or rotations. */
    public double getPosition() {
        return getPositionMeters();
    }

    /** Get current draw in amps. */
    public double getCurrent() {
        return motor.getStatorCurrent();
    }

    /** Check if fully extended. */
    public boolean isFullyExtended() {
        return getPosition() >= config.maxPosition - 0.01;
    }

    /** Check if fully retracted. */
    public boolean isFullyRetracted() {
        return getPosition() <= config.minPosition + 0.01;
    }

    /** Min travel (meters if a spool radius is set, otherwise rotations). */
    public double getMinPosition() { return config.minPosition; }

    /** Max travel (meters if a spool radius is set, otherwise rotations). */
    public double getMaxPosition() { return config.maxPosition; }

    @Override
    public MechanismView describe() {
        String unit = config.spoolRadius > 0 ? "m" : "rot";
        return MechanismView.of(name, "winch")
                .value(getPosition(), unit)
                .range(config.minPosition, config.maxPosition)
                .current(getCurrent())
                .extra("extended", isFullyExtended())
                .extra("retracted", isFullyRetracted())
                .extra("dualMotor", secondMotor != null)
                .build();
    }

    // --- Triggers ---

    public Trigger fullyExtendedTrigger() {
        return new Trigger(this::isFullyExtended);
    }

    public Trigger fullyRetractedTrigger() {
        return new Trigger(this::isFullyRetracted);
    }

    // --- Command Factories ---

    /** Command to extend the mechanism at the configured extend speed. */
    public Command extend() {
        return run(() -> {
            setMotors(config.extendSpeed);
            setState("Extending");
        }).finallyDo(() -> {
            stopMotors();
            setState("Idle");
        }).withName(name + ".Extend");
    }

    /** Command to retract the mechanism at the configured retract speed. */
    public Command retract() {
        return run(() -> {
            setMotors(config.retractSpeed);
            setState("Retracting");
        }).finallyDo(() -> {
            stopMotors();
            setState("Idle");
        }).withName(name + ".Retract");
    }

    /** Command to run at a custom speed [-1, 1]. */
    public Command runAtSpeed(double speed) {
        return run(() -> {
            setMotors(speed);
            setState("Running " + String.format("%.0f%%", speed * 100));
        }).finallyDo(() -> {
            stopMotors();
            setState("Idle");
        }).withName(name + ".RunAt(" + String.format("%.0f%%", speed * 100) + ")");
    }

    /** Command to control with a joystick axis. */
    public Command manualControl(DoubleSupplier speedSupplier) {
        return run(() -> {
            double speed = speedSupplier.getAsDouble();
            if (Math.abs(speed) < 0.05) {
                stopMotors();
                setState("Hold");
            } else {
                setMotors(speed);
                setState("Manual " + String.format("%.0f%%", speed * 100));
            }
        }).finallyDo(() -> {
            stopMotors();
            setState("Idle");
        }).withName(name + ".ManualControl");
    }

    /** Command to zero the encoder. */
    public Command zero() {
        return runOnce(() -> {
            motor.zeroEncoder();
            if (secondMotor != null) secondMotor.zeroEncoder();
            hasBeenZeroed = true;
            setState("Zeroed");
        }).withName(name + ".Zero");
    }

    // --- Helpers ---

    private void setMotors(double speed) {
        motor.setPercent(speed);
        if (secondMotor != null) secondMotor.setPercent(speed);
    }

    private void stopMotors() {
        motor.stop();
        if (secondMotor != null) secondMotor.stop();
    }

    // --- Internals ---

    @Override
    protected void stop() {
        stopMotors();
        setState("Stopped");
    }

    @Override
    protected void updateTelemetry() {
        motor.updateTelemetry();
        if (secondMotor != null) secondMotor.updateTelemetry();

        double circumference = config.spoolRadius > 0 ? 2.0 * Math.PI * config.spoolRadius : 1.0;
        inputs.extensionMeters = getPosition();
        inputs.velocityMPS = motor.getVelocity() * circumference;
        inputs.statorCurrentAmps = motor.getStatorCurrent();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent();
        inputs.appliedVolts = motor.getAppliedVoltage();
        inputs.temperatureC = motor.getTemperature();
        if (secondMotor != null) {
            inputs.followerStatorCurrentAmps = new double[] { secondMotor.getStatorCurrent() };
            inputs.followerTemperatureC = new double[] { secondMotor.getTemperature() };
        } else {
            inputs.followerStatorCurrentAmps = new double[0];
            inputs.followerTemperatureC = new double[0];
        }
        inputs.hasBeenZeroed = hasBeenZeroed;
        processInputs(inputs);

        // Per-key telemetry for v0.2 dashboard compatibility.
        log("Position", inputs.extensionMeters);
        log("CurrentAmps", inputs.statorCurrentAmps);
        log("FullyExtended", isFullyExtended());
        log("FullyRetracted", isFullyRetracted());

        HealthMonitor.getInstance().update();
    }

    public CatalystMotor getMotor() { return motor; }
    public CatalystMotor getSecondMotor() { return secondMotor; }

    @Override
    protected CatalystMotor primaryMotorForSysId() {
        return motor;
    }

    // ===========================================
    //                  CONFIG
    // ===========================================

    public static class Config {
        final String name;
        final int motorCanId;
        final int secondMotorCanId;
        final String canBus;
        final boolean inverted;
        final boolean secondInverted;
        final double gearRatio;
        final boolean useFOC;
        final double spoolRadius;
        final double minPosition;
        final double maxPosition;
        final double currentLimit;
        final double statorCurrentLimit;
        final double extendSpeed;
        final double retractSpeed;
        final double loadMass;

        private Config(Builder b) {
            this.name = b.name;
            this.motorCanId = b.motorCanId;
            this.secondMotorCanId = b.secondMotorCanId;
            this.canBus = b.canBus;
            this.inverted = b.inverted;
            this.secondInverted = b.secondInverted;
            this.gearRatio = b.gearRatio;
            this.useFOC = b.useFOC;
            this.spoolRadius = b.spoolRadius;
            this.minPosition = b.minPosition;
            this.maxPosition = b.maxPosition;
            this.currentLimit = b.currentLimit;
            this.statorCurrentLimit = b.statorCurrentLimit;
            this.extendSpeed = b.extendSpeed;
            this.retractSpeed = b.retractSpeed;
            this.loadMass = b.loadMass;
        }

        public static Builder builder() {
            return new Builder();
        }

        public static class Builder {
            private String name = "WinchMechanism";
            private int motorCanId = 0;
            private int secondMotorCanId = -1;
            private String canBus = "";
            private boolean inverted = false;
            private boolean secondInverted = false;
            private double gearRatio = 25.0;
            private boolean useFOC = true;
            private double spoolRadius = 0.02; // meters
            private double minPosition = 0;
            private double maxPosition = 0.6;
            private double currentLimit = 80;
            private double statorCurrentLimit = 120;
            private double extendSpeed = 0.8;
            private double retractSpeed = -1.0;
            private double loadMass = 6.0; // kg, used only for simulation

            public Builder name(String name) { this.name = name; return this; }
            public Builder motor(int canId) { this.motorCanId = canId; return this; }

            /** Add a second motor (independent, for dual-arm climbers). */
            public Builder secondMotor(int canId) { this.secondMotorCanId = canId; return this; }

            public Builder canBus(String canBus) { this.canBus = canBus; return this; }
            public Builder inverted(boolean inverted) { this.inverted = inverted; return this; }
            public Builder secondInverted(boolean inverted) { this.secondInverted = inverted; return this; }
            public Builder gearRatio(double ratio) { this.gearRatio = ratio; return this; }
            public Builder useFOC(boolean useFOC) { this.useFOC = useFOC; return this; }

            /** Spool/drum radius in meters. Used for position-to-rotation conversion. */
            public Builder spoolRadius(double meters) { this.spoolRadius = meters; return this; }

            /** Min and max position in meters. */
            public Builder range(double minMeters, double maxMeters) {
                this.minPosition = minMeters;
                this.maxPosition = maxMeters;
                return this;
            }

            public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
            public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }

            /** Extend speed as duty cycle [0, 1]. */
            public Builder extendSpeed(double speed) { this.extendSpeed = speed; return this; }

            /** Retract speed as duty cycle [-1, 0]. */
            public Builder retractSpeed(double speed) { this.retractSpeed = speed; return this; }

            /** Load mass in kg the winch lifts. Used only for the simulation model. */
            public Builder loadMass(double kg) { this.loadMass = kg; return this; }

            public Config build() {
                if (motorCanId == 0) {
                    throw new IllegalStateException("Motor CAN ID must be set");
                }
                return new Config(this);
            }
        }
    }
}
