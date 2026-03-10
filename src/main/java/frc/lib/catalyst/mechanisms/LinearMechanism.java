package frc.lib.catalyst.mechanisms;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.util.AlertManager;
import frc.lib.catalyst.util.FeedforwardGains;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Generic linear motion mechanism. Use for elevators, linear slides,
 * telescoping arms, or any mechanism that moves in a straight line.
 *
 * <p>Features:
 * <ul>
 *   <li>Motion Magic position control with gravity compensation</li>
 *   <li>Named position presets</li>
 *   <li>Built-in simulation</li>
 *   <li>Automatic telemetry</li>
 *   <li>Pre-built command factories</li>
 * </ul>
 *
 * <p>Example usage:
 * <pre>{@code
 * LinearMechanism elevator = new LinearMechanism(
 *     LinearMechanism.Config.builder()
 *         .name("Elevator")
 *         .motor(13)
 *         .follower(14, true)
 *         .gearRatio(10.0)
 *         .drumRadius(0.0254)
 *         .range(0.0, 1.2)
 *         .mass(5.0)
 *         .pid(50, 0, 0.5)
 *         .motionMagic(2.0, 4.0, 20.0)
 *         .currentLimit(40)
 *         .position("STOW", 0.0)
 *         .position("HIGH", 1.1)
 *         .build());
 * }</pre>
 */
public class LinearMechanism extends CatalystMechanism {

    private final Config config;
    private final CatalystMotor motor;
    private final DigitalInput forwardLimitSwitch;
    private final DigitalInput reverseLimitSwitch;

    // Simulation
    private ElevatorSim sim;

    // WPILib ProfiledPID (alternative to Motion Magic)
    private final ProfiledPIDController profiledPID;
    private final FeedforwardGains feedforwardGains;
    private final boolean useWPILibProfile;

    // State
    private double setpointMeters = 0;
    private boolean hasBeenZeroed = false;

    // Fault monitoring
    private int consecutiveHighTempCycles = 0;

    public LinearMechanism(Config config) {
        super(config.name);
        this.config = config;

        int motorCount = (config.followerCanId >= 0 ? 2 : 1) + config.additionalFollowerCanIds.length;

        // Build motor with appropriate configuration
        CatalystMotor.Builder motorBuilder = CatalystMotor.builder(config.motorCanId)
                .name(config.name + "Motor")
                .canBus(config.canBus)
                .inverted(config.inverted)
                .brakeMode(true)
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit)
                .gearRatio(config.gearRatio)
                .pid(config.kP, config.kI, config.kD)
                .feedforward(config.kS, config.kV, config.kA)
                .gravityGain(config.kG, GravityTypeValue.Elevator_Static)
                .motionMagic(config.motionMagicCruiseVelocity,
                        config.motionMagicAcceleration,
                        config.motionMagicJerk);

        // Convert range from meters to rotations for soft limits
        double minRotations = metersToRotations(config.minPosition);
        double maxRotations = metersToRotations(config.maxPosition);
        motorBuilder.softLimits(minRotations, maxRotations);

        if (config.followerCanId >= 0) {
            motorBuilder.withFollower(config.followerCanId, config.followerOppose);
        }

        this.motor = motorBuilder.build();

        // Set starting position
        if (config.startingPosition != 0) {
            motor.setEncoderPosition(metersToRotations(config.startingPosition));
            setpointMeters = config.startingPosition;
        }

        // Limit switches
        forwardLimitSwitch = config.forwardLimitPort >= 0 ? new DigitalInput(config.forwardLimitPort) : null;
        reverseLimitSwitch = config.reverseLimitPort >= 0 ? new DigitalInput(config.reverseLimitPort) : null;

        // WPILib ProfiledPID setup (alternative to Motion Magic)
        this.useWPILibProfile = config.useWPILibProfile;
        if (config.useWPILibProfile && config.profileMaxVelocity > 0) {
            profiledPID = new ProfiledPIDController(
                    config.profileKP, config.profileKI, config.profileKD,
                    new TrapezoidProfile.Constraints(config.profileMaxVelocity, config.profileMaxAcceleration));
            feedforwardGains = FeedforwardGains.elevator(config.kS, config.kV, config.kA, config.kG);
        } else {
            profiledPID = null;
            feedforwardGains = null;
        }

        // Set up simulation with proper motor model
        if (RobotBase.isSimulation()) {
            DCMotor motorModel = config.motorType.getDCMotor(motorCount);
            sim = new ElevatorSim(
                    motorModel,
                    config.gearRatio,
                    config.mass,
                    config.drumRadius,
                    config.minPosition,
                    config.maxPosition,
                    true,
                    config.minPosition);
        }
    }

    // --- Position Conversions ---
    // For multi-stage elevators, each stage multiplies the carriage travel.
    // effectiveDrumCircumference = 2*PI*drumRadius * stages

    private double metersToRotations(double meters) {
        return meters / (2.0 * Math.PI * config.drumRadius * config.stages);
    }

    private double rotationsToMeters(double rotations) {
        return rotations * (2.0 * Math.PI * config.drumRadius * config.stages);
    }

    // --- Getters ---

    /** Get current position in meters. */
    public double getPosition() {
        return rotationsToMeters(motor.getPosition());
    }

    /** Get current velocity in meters per second. */
    public double getVelocity() {
        return rotationsToMeters(motor.getVelocity());
    }

    /** Get the current setpoint in meters. */
    public double getSetpoint() {
        return setpointMeters;
    }

    /** Get current draw in amps. */
    public double getCurrent() {
        return motor.getStatorCurrent();
    }

    /** Check if the mechanism is at a given position within tolerance. */
    public boolean atPosition(double meters, double toleranceMeters) {
        return Math.abs(getPosition() - meters) < toleranceMeters;
    }

    /** Check if the mechanism is at its setpoint within tolerance. */
    public boolean atSetpoint(double toleranceMeters) {
        return atPosition(setpointMeters, toleranceMeters);
    }

    /** Check if at a named position within default tolerance (2 cm). */
    public boolean atPosition(String positionName) {
        Double target = config.namedPositions.get(positionName);
        if (target == null) return false;
        return atPosition(target, 0.02);
    }

    // --- Limit Switches ---

    /** Check if the forward (top) limit switch is pressed. */
    public boolean isForwardLimitPressed() {
        return forwardLimitSwitch != null && !forwardLimitSwitch.get();
    }

    /** Check if the reverse (bottom) limit switch is pressed. */
    public boolean isReverseLimitPressed() {
        return reverseLimitSwitch != null && !reverseLimitSwitch.get();
    }

    /** Check if the mechanism has been zeroed (either manually or via limit switch). */
    public boolean hasBeenZeroed() {
        return hasBeenZeroed;
    }

    // --- Triggers ---

    /** Trigger that fires when at the given position (2 cm tolerance). */
    public Trigger atPositionTrigger(double meters) {
        return atPositionTrigger(meters, 0.02);
    }

    /** Trigger that fires when at the given position within tolerance. */
    public Trigger atPositionTrigger(double meters, double toleranceMeters) {
        return new Trigger(() -> atPosition(meters, toleranceMeters));
    }

    /** Trigger that fires when at a named position (2 cm tolerance). */
    public Trigger atPositionTrigger(String positionName) {
        return new Trigger(() -> atPosition(positionName));
    }

    /** Trigger that fires when the forward limit switch is pressed. */
    public Trigger forwardLimitTrigger() {
        return new Trigger(this::isForwardLimitPressed);
    }

    /** Trigger that fires when the reverse limit switch is pressed. */
    public Trigger reverseLimitTrigger() {
        return new Trigger(this::isReverseLimitPressed);
    }

    // --- Command Factories ---

    /**
     * Command to move to a position in meters using Motion Magic.
     * Ends immediately after setting the target (use .until() or atPositionTrigger for waiting).
     */
    public Command goTo(double meters) {
        return runOnce(() -> {
            setpointMeters = MathUtil.clamp(meters, config.minPosition, config.maxPosition);
            motor.setMotionMagicPosition(metersToRotations(setpointMeters));
            setState("GoTo " + String.format("%.2f", setpointMeters) + "m");
        }).withName(name + ".GoTo(" + String.format("%.2f", meters) + ")");
    }

    /**
     * Command to move to a named position using Motion Magic.
     * @throws IllegalArgumentException if the position name is not defined
     */
    public Command goTo(String positionName) {
        Double target = config.namedPositions.get(positionName);
        if (target == null) {
            throw new IllegalArgumentException(
                    "Unknown position '" + positionName + "' for " + name
                            + ". Available: " + config.namedPositions.keySet());
        }
        return goTo(target).withName(name + ".GoTo(" + positionName + ")");
    }

    /**
     * Command to move to a position and wait until it arrives (within tolerance).
     */
    public Command goToAndWait(double meters, double toleranceMeters) {
        return run(() -> {
            setpointMeters = MathUtil.clamp(meters, config.minPosition, config.maxPosition);
            motor.setMotionMagicPosition(metersToRotations(setpointMeters));
            setState("GoTo " + String.format("%.2f", setpointMeters) + "m");
        }).until(() -> atPosition(meters, toleranceMeters))
                .withName(name + ".GoToAndWait(" + String.format("%.2f", meters) + ")");
    }

    /**
     * Command to move to a named position and wait until it arrives.
     */
    public Command goToAndWait(String positionName, double toleranceMeters) {
        Double target = config.namedPositions.get(positionName);
        if (target == null) {
            throw new IllegalArgumentException(
                    "Unknown position '" + positionName + "' for " + name);
        }
        return goToAndWait(target, toleranceMeters)
                .withName(name + ".GoToAndWait(" + positionName + ")");
    }

    /**
     * Command that continuously holds the current position.
     * Good as a default command.
     */
    public Command holdPosition() {
        return run(() -> {
            motor.setMotionMagicPosition(metersToRotations(setpointMeters));
            setState("Hold " + String.format("%.2f", setpointMeters) + "m");
        }).withName(name + ".HoldPosition");
    }

    /** Command to jog upward at a given voltage. */
    public Command jogUp(double volts) {
        return run(() -> {
            motor.setVoltage(Math.abs(volts));
            setpointMeters = getPosition();
            setState("JogUp");
        }).finallyDo(() -> {
            setpointMeters = getPosition();
            motor.setMotionMagicPosition(metersToRotations(setpointMeters));
        }).withName(name + ".JogUp");
    }

    /** Command to jog downward at a given voltage. */
    public Command jogDown(double volts) {
        return run(() -> {
            motor.setVoltage(-Math.abs(volts));
            setpointMeters = getPosition();
            setState("JogDown");
        }).finallyDo(() -> {
            setpointMeters = getPosition();
            motor.setMotionMagicPosition(metersToRotations(setpointMeters));
        }).withName(name + ".JogDown");
    }

    /** Command to jog with a dynamic speed supplier (e.g., joystick). */
    public Command jog(DoubleSupplier voltsSupplier) {
        return run(() -> {
            double volts = voltsSupplier.getAsDouble();
            if (Math.abs(volts) < 0.1) {
                motor.setMotionMagicPosition(metersToRotations(setpointMeters));
            } else {
                motor.setVoltage(volts);
                setpointMeters = getPosition();
            }
            setState("Jog");
        }).finallyDo(() -> {
            setpointMeters = getPosition();
            motor.setMotionMagicPosition(metersToRotations(setpointMeters));
        }).withName(name + ".Jog");
    }

    /**
     * Command to move to a position using WPILib ProfiledPID + feedforward.
     * Alternative to Motion Magic — runs the trapezoidal profile on the roboRIO.
     * Requires {@code useWPILibProfile(true)} in config.
     * Runs continuously until cancelled or another command takes over.
     */
    public Command goToProfiled(double meters) {
        if (profiledPID == null) {
            throw new IllegalStateException(
                    name + ": WPILib profile not configured. Use .useWPILibProfile(true) in config.");
        }
        return run(() -> {
            setpointMeters = MathUtil.clamp(meters, config.minPosition, config.maxPosition);
            double output = profiledPID.calculate(getPosition(), setpointMeters);
            double ff = feedforwardGains.calculateElevator(profiledPID.getSetpoint().velocity);
            motor.setVoltage(output + ff);
            setState("ProfiledGoTo " + String.format("%.2f", setpointMeters) + "m");
        }).beforeStarting(() -> profiledPID.reset(getPosition(), getVelocity()))
                .withName(name + ".ProfiledGoTo(" + String.format("%.2f", meters) + ")");
    }

    /**
     * Command to move to a named position using WPILib ProfiledPID.
     */
    public Command goToProfiled(String positionName) {
        Double target = config.namedPositions.get(positionName);
        if (target == null) {
            throw new IllegalArgumentException(
                    "Unknown position '" + positionName + "' for " + name);
        }
        return goToProfiled(target).withName(name + ".ProfiledGoTo(" + positionName + ")");
    }

    /**
     * Command that holds position using WPILib ProfiledPID.
     * Alternative to the Motion Magic holdPosition() command.
     */
    public Command holdPositionProfiled() {
        if (profiledPID == null) {
            throw new IllegalStateException(
                    name + ": WPILib profile not configured. Use .useWPILibProfile(true) in config.");
        }
        return run(() -> {
            double output = profiledPID.calculate(getPosition(), setpointMeters);
            double ff = feedforwardGains.calculateElevator(profiledPID.getSetpoint().velocity);
            motor.setVoltage(output + ff);
            setState("ProfiledHold " + String.format("%.2f", setpointMeters) + "m");
        }).beforeStarting(() -> profiledPID.reset(getPosition(), getVelocity()))
                .withName(name + ".ProfiledHoldPosition");
    }

    /** Command to zero the encoder at the current position. */
    public Command zero() {
        return runOnce(() -> {
            motor.zeroEncoder();
            setpointMeters = 0;
            setState("Zeroed");
        }).withName(name + ".Zero");
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
        log("PositionMeters", getPosition());
        log("VelocityMPS", getVelocity());
        log("SetpointMeters", setpointMeters);
        log("CurrentAmps", getCurrent());
        log("AtSetpoint", atSetpoint(0.02));
        log("HasBeenZeroed", hasBeenZeroed);
        if (forwardLimitSwitch != null) log("ForwardLimit", isForwardLimitPressed());
        if (reverseLimitSwitch != null) log("ReverseLimit", isReverseLimitPressed());

        // Auto-zero on reverse limit switch
        if (config.autoZeroOnReverseLimit && isReverseLimitPressed()) {
            motor.zeroEncoder();
            setpointMeters = config.minPosition;
            hasBeenZeroed = true;
        }

        // Fault detection
        checkFaults();
    }

    private void checkFaults() {
        AlertManager alerts = AlertManager.getInstance();
        double temp = motor.getTemperature();
        if (temp > config.maxTemperatureC) {
            consecutiveHighTempCycles++;
            if (consecutiveHighTempCycles > 50) { // ~1 second
                alerts.warning(name, "Motor temperature high: " + String.format("%.0f", temp) + "C");
                // Safety cutoff: zero output to let motor cool down
                if (temp > config.maxTemperatureC + 10) {
                    motor.stop();
                    alerts.error(name, "Motor OVERTEMP cutoff at " + String.format("%.0f", temp) + "C!");
                }
            }
        } else {
            consecutiveHighTempCycles = 0;
        }
    }

    @Override
    public void simulationPeriodic() {
        if (sim != null) {
            var simState = motor.getTalonFX().getSimState();
            sim.setInput(simState.getMotorVoltage());
            sim.update(0.02);
            simState.setRawRotorPosition(metersToRotations(sim.getPositionMeters()) * config.gearRatio);
            simState.setRotorVelocity(metersToRotations(sim.getVelocityMetersPerSecond()) * config.gearRatio);
        }
    }

    /** Get the underlying motor for advanced use. */
    public CatalystMotor getMotor() {
        return motor;
    }

    // ===========================================
    //                  CONFIG
    // ===========================================

    public static class Config {
        final String name;
        final int motorCanId;
        final String canBus;
        final boolean inverted;
        final int followerCanId;
        final boolean followerOppose;
        final int[] additionalFollowerCanIds;
        final boolean[] additionalFollowerOppose;
        final MotorType motorType;
        final double gearRatio;
        final int stages;
        final double drumRadius;
        final double minPosition;
        final double maxPosition;
        final double startingPosition;
        final double mass;
        final double currentLimit;
        final double statorCurrentLimit;
        final double kP, kI, kD;
        final double kS, kV, kA, kG;
        final double motionMagicCruiseVelocity;
        final double motionMagicAcceleration;
        final double motionMagicJerk;
        final Map<String, Double> namedPositions;
        final int forwardLimitPort;
        final int reverseLimitPort;
        final boolean autoZeroOnReverseLimit;
        final double maxTemperatureC;
        final double positionToleranceMeters;

        // WPILib ProfiledPID (alternative to Motion Magic)
        final boolean useWPILibProfile;
        final double profileKP, profileKI, profileKD;
        final double profileMaxVelocity;     // m/s
        final double profileMaxAcceleration; // m/s^2

        private Config(Builder b) {
            this.name = b.name;
            this.motorCanId = b.motorCanId;
            this.canBus = b.canBus;
            this.inverted = b.inverted;
            this.followerCanId = b.followerCanId;
            this.followerOppose = b.followerOppose;
            this.additionalFollowerCanIds = b.additionalFollowerCanIds;
            this.additionalFollowerOppose = b.additionalFollowerOppose;
            this.motorType = b.motorType;
            this.gearRatio = b.gearRatio;
            this.stages = b.stages;
            this.drumRadius = b.drumRadius;
            this.minPosition = b.minPosition;
            this.maxPosition = b.maxPosition;
            this.startingPosition = b.startingPosition;
            this.mass = b.mass;
            this.currentLimit = b.currentLimit;
            this.statorCurrentLimit = b.statorCurrentLimit;
            this.kP = b.kP; this.kI = b.kI; this.kD = b.kD;
            this.kS = b.kS; this.kV = b.kV; this.kA = b.kA; this.kG = b.kG;
            this.motionMagicCruiseVelocity = b.motionMagicCruiseVelocity;
            this.motionMagicAcceleration = b.motionMagicAcceleration;
            this.motionMagicJerk = b.motionMagicJerk;
            this.namedPositions = Map.copyOf(b.namedPositions);
            this.forwardLimitPort = b.forwardLimitPort;
            this.reverseLimitPort = b.reverseLimitPort;
            this.autoZeroOnReverseLimit = b.autoZeroOnReverseLimit;
            this.maxTemperatureC = b.maxTemperatureC;
            this.positionToleranceMeters = b.positionToleranceMeters;
            this.useWPILibProfile = b.useWPILibProfile;
            this.profileKP = b.profileKP;
            this.profileKI = b.profileKI;
            this.profileKD = b.profileKD;
            this.profileMaxVelocity = b.profileMaxVelocity;
            this.profileMaxAcceleration = b.profileMaxAcceleration;
        }

        /** Get the total travel distance. */
        public double getTravelDistance() {
            return maxPosition - minPosition;
        }

        /**
         * Estimate the gravity feedforward voltage needed to hold the mechanism.
         * Accounts for multi-stage factor, motor type, gear ratio, drum radius, and mass.
         */
        public double estimateGravityFF() {
            double force = mass * 9.81; // N
            // Multi-stage: the force at the drum is divided by stages
            double torqueAtDrum = force * drumRadius / stages; // Nm
            return motorType.holdingVoltage(torqueAtDrum, gearRatio);
        }

        /**
         * Estimate max mechanism speed in meters per second.
         * Accounts for multi-stage multiplication.
         */
        public double estimateMaxSpeed() {
            double maxMotorRPS = motorType.freeSpeedRPS();
            double maxMechanismRPS = maxMotorRPS / gearRatio;
            return maxMechanismRPS * (2.0 * Math.PI * drumRadius * stages);
        }

        public static Builder builder() {
            return new Builder();
        }

        public static class Builder {
            private String name = "LinearMechanism";
            private int motorCanId = 0;
            private String canBus = "";
            private boolean inverted = false;
            private int followerCanId = -1;
            private boolean followerOppose = false;
            private int[] additionalFollowerCanIds = new int[0];
            private boolean[] additionalFollowerOppose = new boolean[0];
            private MotorType motorType = MotorType.KRAKEN_X60;
            private double gearRatio = 1.0;
            private int stages = 1;
            private double drumRadius = 0.0254; // 1 inch default
            private double minPosition = 0;
            private double maxPosition = 1.0;
            private double startingPosition = 0;
            private double mass = 5.0;
            private double currentLimit = 40;
            private double statorCurrentLimit = 80;
            private double kP = 0, kI = 0, kD = 0;
            private double kS = 0, kV = 0, kA = 0, kG = 0;
            private double motionMagicCruiseVelocity = 0;
            private double motionMagicAcceleration = 0;
            private double motionMagicJerk = 0;
            private final Map<String, Double> namedPositions = new HashMap<>();
            private int forwardLimitPort = -1;
            private int reverseLimitPort = -1;
            private boolean autoZeroOnReverseLimit = false;
            private double maxTemperatureC = 70;
            private double positionToleranceMeters = 0.02;
            private boolean useWPILibProfile = false;
            private double profileKP = 0, profileKI = 0, profileKD = 0;
            private double profileMaxVelocity = 0;
            private double profileMaxAcceleration = 0;

            public Builder name(String name) { this.name = name; return this; }
            public Builder motor(int canId) { this.motorCanId = canId; return this; }
            public Builder canBus(String canBus) { this.canBus = canBus; return this; }
            public Builder inverted(boolean inverted) { this.inverted = inverted; return this; }

            public Builder follower(int canId, boolean oppose) {
                this.followerCanId = canId;
                this.followerOppose = oppose;
                return this;
            }

            /** Set the motor type for accurate simulation and physics calculations. */
            public Builder motorType(MotorType type) { this.motorType = type; return this; }

            public Builder gearRatio(double ratio) { this.gearRatio = ratio; return this; }
            public Builder drumRadius(double meters) { this.drumRadius = meters; return this; }

            /**
             * Number of cascading elevator stages (default 1).
             * Multi-stage elevators multiply the carriage travel per spool rotation.
             * A 2-stage elevator moves the carriage twice as far per motor rotation.
             */
            public Builder stages(int numStages) { this.stages = numStages; return this; }

            /** Set min and max position in meters. */
            public Builder range(double minMeters, double maxMeters) {
                this.minPosition = minMeters;
                this.maxPosition = maxMeters;
                return this;
            }

            /**
             * Starting position of the mechanism in meters (default 0).
             * The encoder is seeded to this position on construction.
             */
            public Builder startingPosition(double meters) { this.startingPosition = meters; return this; }

            /** Mass of the moving carriage/stage in kg. Used for gravity FF and simulation. */
            public Builder mass(double kg) { this.mass = kg; return this; }

            public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
            public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }

            public Builder pid(double kP, double kI, double kD) {
                this.kP = kP; this.kI = kI; this.kD = kD; return this;
            }

            public Builder feedforward(double kS, double kV) {
                this.kS = kS; this.kV = kV; return this;
            }

            public Builder feedforward(double kS, double kV, double kA) {
                this.kS = kS; this.kV = kV; this.kA = kA; return this;
            }

            /** Gravity compensation gain. */
            public Builder gravityGain(double kG) { this.kG = kG; return this; }

            /** Motion Magic cruise velocity (mechanism rot/s), acceleration (rot/s^2), jerk (rot/s^3). */
            public Builder motionMagic(double cruiseVelocity, double acceleration, double jerk) {
                this.motionMagicCruiseVelocity = cruiseVelocity;
                this.motionMagicAcceleration = acceleration;
                this.motionMagicJerk = jerk;
                return this;
            }

            /** Add a named position preset in meters. */
            public Builder position(String name, double meters) {
                this.namedPositions.put(name, meters);
                return this;
            }

            /**
             * Add a forward (top) limit switch on a DIO port.
             * The mechanism will stop moving forward when pressed.
             */
            public Builder forwardLimitSwitch(int dioPort) {
                this.forwardLimitPort = dioPort;
                return this;
            }

            /**
             * Add a reverse (bottom) limit switch on a DIO port.
             * Optionally auto-zeros the encoder when triggered.
             */
            public Builder reverseLimitSwitch(int dioPort, boolean autoZero) {
                this.reverseLimitPort = dioPort;
                this.autoZeroOnReverseLimit = autoZero;
                return this;
            }

            /** Set the temperature threshold for fault alerts (default 70C). */
            public Builder maxTemperature(double celsius) { this.maxTemperatureC = celsius; return this; }

            /** Set the position tolerance for atPosition checks (default 0.02m / 2cm). */
            public Builder positionTolerance(double meters) { this.positionToleranceMeters = meters; return this; }

            /**
             * Enable WPILib ProfiledPID as an alternative to CTRE Motion Magic.
             * This runs the trapezoidal profile on the roboRIO instead of on the TalonFX.
             * Useful for non-CTRE motors or when you want WPILib-native control.
             * Use the goToProfiled() and holdPositionProfiled() commands when enabled.
             *
             * @param kP proportional gain (volts per meter of error)
             * @param kI integral gain
             * @param kD derivative gain
             * @param maxVelocityMPS max velocity in meters per second
             * @param maxAccelerationMPSS max acceleration in meters per second squared
             */
            public Builder useWPILibProfile(double kP, double kI, double kD,
                                             double maxVelocityMPS, double maxAccelerationMPSS) {
                this.useWPILibProfile = true;
                this.profileKP = kP;
                this.profileKI = kI;
                this.profileKD = kD;
                this.profileMaxVelocity = maxVelocityMPS;
                this.profileMaxAcceleration = maxAccelerationMPSS;
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
