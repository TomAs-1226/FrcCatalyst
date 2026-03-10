package frc.lib.catalyst.mechanisms;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
 * Generic rotational mechanism. Use for arms, wrists, turrets, hoods,
 * or any mechanism that rotates around an axis.
 *
 * <p>Provides cosine-based gravity compensation (configurable), Motion Magic
 * position control, named presets, simulation, and telemetry.
 *
 * <p>Positions are in degrees. Zero is typically horizontal or stowed position.
 *
 * <p>Example usage:
 * <pre>{@code
 * RotationalMechanism arm = new RotationalMechanism(
 *     RotationalMechanism.Config.builder()
 *         .name("Arm")
 *         .motor(15)
 *         .gearRatio(50.0)
 *         .length(0.5)
 *         .mass(3.0)
 *         .range(-10, 120)
 *         .pid(80, 0, 1.0)
 *         .motionMagic(200, 400, 2000)
 *         .currentLimit(30)
 *         .position("STOW", 0)
 *         .position("SCORE", 100)
 *         .build());
 * }</pre>
 */
public class RotationalMechanism extends CatalystMechanism {

    private final Config config;
    private final CatalystMotor motor;
    private final DigitalInput hardStop;

    // Simulation
    private SingleJointedArmSim sim;

    // WPILib ProfiledPID (alternative to Motion Magic)
    private final ProfiledPIDController profiledPID;
    private final FeedforwardGains feedforwardGains;
    private final boolean useWPILibProfile;

    // State
    private double setpointDegrees = 0;
    private boolean hasBeenZeroed = false;
    private int consecutiveHighTempCycles = 0;

    public RotationalMechanism(Config config) {
        super(config.name);
        this.config = config;

        int motorCount = config.followerCanId >= 0 ? 2 : 1;

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
                .gravityGain(config.kG,
                        config.useCosineGravity
                                ? GravityTypeValue.Arm_Cosine
                                : GravityTypeValue.Elevator_Static)
                .motionMagic(config.motionMagicCruiseVelocity,
                        config.motionMagicAcceleration,
                        config.motionMagicJerk);

        // Soft limits in mechanism rotations
        double minRotations = config.minAngle / 360.0;
        double maxRotations = config.maxAngle / 360.0;
        motorBuilder.softLimits(minRotations, maxRotations);

        if (config.followerCanId >= 0) {
            motorBuilder.withFollower(config.followerCanId, config.followerOppose);
        }

        this.motor = motorBuilder.build();

        // Set starting position
        if (config.startingAngle != 0) {
            motor.setEncoderPosition(degreesToRotations(config.startingAngle));
            setpointDegrees = config.startingAngle;
        }

        // Hard stop / home switch
        hardStop = config.hardStopPort >= 0 ? new DigitalInput(config.hardStopPort) : null;

        // WPILib ProfiledPID setup
        this.useWPILibProfile = config.useWPILibProfile;
        if (config.useWPILibProfile && config.profileMaxVelocity > 0) {
            // Profile uses rotations as the unit (matching CTRE mechanism units)
            profiledPID = new ProfiledPIDController(
                    config.profileKP, config.profileKI, config.profileKD,
                    new TrapezoidProfile.Constraints(
                            config.profileMaxVelocity / 360.0,   // deg/s -> rot/s
                            config.profileMaxAcceleration / 360.0)); // deg/s^2 -> rot/s^2
            feedforwardGains = FeedforwardGains.arm(config.kS, config.kV, config.kA, config.kG);
        } else {
            profiledPID = null;
            feedforwardGains = null;
        }

        // Simulation with proper motor model
        if (RobotBase.isSimulation()) {
            DCMotor motorModel = config.motorType.getDCMotor(motorCount);
            double moi = config.customMOI > 0
                    ? config.customMOI
                    : SingleJointedArmSim.estimateMOI(config.length, config.mass);
            sim = new SingleJointedArmSim(
                    motorModel,
                    config.gearRatio,
                    moi,
                    config.length,
                    Math.toRadians(config.minAngle),
                    Math.toRadians(config.maxAngle),
                    true,
                    Math.toRadians(config.minAngle));
        }
    }

    // --- Conversions ---

    private double degreesToRotations(double degrees) {
        return degrees / 360.0;
    }

    private double rotationsToDegrees(double rotations) {
        return rotations * 360.0;
    }

    // --- Getters ---

    /** Get current angle in degrees. */
    public double getAngle() {
        return rotationsToDegrees(motor.getPosition());
    }

    /** Get angular velocity in degrees per second. */
    public double getAngularVelocity() {
        return rotationsToDegrees(motor.getVelocity());
    }

    /** Get current setpoint in degrees. */
    public double getSetpoint() {
        return setpointDegrees;
    }

    /** Get current draw in amps. */
    public double getCurrent() {
        return motor.getStatorCurrent();
    }

    /** Check if at a given angle within tolerance (degrees). */
    public boolean atAngle(double degrees, double toleranceDegrees) {
        return Math.abs(getAngle() - degrees) < toleranceDegrees;
    }

    /** Check if at the current setpoint within tolerance. */
    public boolean atSetpoint(double toleranceDegrees) {
        return atAngle(setpointDegrees, toleranceDegrees);
    }

    /** Check if at a named position within default tolerance (2 degrees). */
    public boolean atPosition(String positionName) {
        Double target = config.namedPositions.get(positionName);
        if (target == null) return false;
        return atAngle(target, 2.0);
    }

    // --- Triggers ---

    /** Trigger that fires when at the given angle (2 degree tolerance). */
    public Trigger atAngleTrigger(double degrees) {
        return atAngleTrigger(degrees, 2.0);
    }

    /** Trigger that fires when at the given angle within tolerance. */
    public Trigger atAngleTrigger(double degrees, double toleranceDegrees) {
        return new Trigger(() -> atAngle(degrees, toleranceDegrees));
    }

    /** Trigger that fires when at a named position (2 degree tolerance). */
    public Trigger atPositionTrigger(String positionName) {
        return new Trigger(() -> atPosition(positionName));
    }

    // --- Command Factories ---

    /** Command to move to an angle in degrees using Motion Magic. */
    public Command goTo(double degrees) {
        return runOnce(() -> {
            setpointDegrees = MathUtil.clamp(degrees, config.minAngle, config.maxAngle);
            motor.setMotionMagicPosition(degreesToRotations(setpointDegrees));
            setState("GoTo " + String.format("%.1f", setpointDegrees) + "deg");
        }).withName(name + ".GoTo(" + String.format("%.1f", degrees) + ")");
    }

    /** Command to move to a named position. */
    public Command goTo(String positionName) {
        Double target = config.namedPositions.get(positionName);
        if (target == null) {
            throw new IllegalArgumentException(
                    "Unknown position '" + positionName + "' for " + name
                            + ". Available: " + config.namedPositions.keySet());
        }
        return goTo(target).withName(name + ".GoTo(" + positionName + ")");
    }

    /** Command to move to an angle and wait until it arrives. */
    public Command goToAndWait(double degrees, double toleranceDegrees) {
        return run(() -> {
            setpointDegrees = MathUtil.clamp(degrees, config.minAngle, config.maxAngle);
            motor.setMotionMagicPosition(degreesToRotations(setpointDegrees));
            setState("GoTo " + String.format("%.1f", setpointDegrees) + "deg");
        }).until(() -> atAngle(degrees, toleranceDegrees))
                .withName(name + ".GoToAndWait(" + String.format("%.1f", degrees) + ")");
    }

    /** Command to move to a named position and wait until it arrives. */
    public Command goToAndWait(String positionName, double toleranceDegrees) {
        Double target = config.namedPositions.get(positionName);
        if (target == null) {
            throw new IllegalArgumentException(
                    "Unknown position '" + positionName + "' for " + name);
        }
        return goToAndWait(target, toleranceDegrees)
                .withName(name + ".GoToAndWait(" + positionName + ")");
    }

    /** Command that continuously holds the current setpoint. Good as default command. */
    public Command holdPosition() {
        return run(() -> {
            motor.setMotionMagicPosition(degreesToRotations(setpointDegrees));
            setState("Hold " + String.format("%.1f", setpointDegrees) + "deg");
        }).withName(name + ".HoldPosition");
    }

    /** Command to jog clockwise at a given voltage. */
    public Command jogCW(double volts) {
        return run(() -> {
            motor.setVoltage(Math.abs(volts));
            setpointDegrees = getAngle();
            setState("JogCW");
        }).finallyDo(() -> {
            setpointDegrees = getAngle();
            motor.setMotionMagicPosition(degreesToRotations(setpointDegrees));
        }).withName(name + ".JogCW");
    }

    /** Command to jog counter-clockwise at a given voltage. */
    public Command jogCCW(double volts) {
        return run(() -> {
            motor.setVoltage(-Math.abs(volts));
            setpointDegrees = getAngle();
            setState("JogCCW");
        }).finallyDo(() -> {
            setpointDegrees = getAngle();
            motor.setMotionMagicPosition(degreesToRotations(setpointDegrees));
        }).withName(name + ".JogCCW");
    }

    /** Command to jog with a dynamic speed supplier (e.g., joystick). */
    public Command jog(DoubleSupplier voltsSupplier) {
        return run(() -> {
            double volts = voltsSupplier.getAsDouble();
            if (Math.abs(volts) < 0.1) {
                motor.setMotionMagicPosition(degreesToRotations(setpointDegrees));
            } else {
                motor.setVoltage(volts);
                setpointDegrees = getAngle();
            }
            setState("Jog");
        }).finallyDo(() -> {
            setpointDegrees = getAngle();
            motor.setMotionMagicPosition(degreesToRotations(setpointDegrees));
        }).withName(name + ".Jog");
    }

    /**
     * Command to move to an angle using WPILib ProfiledPID + feedforward.
     * Alternative to Motion Magic — runs the trapezoidal profile on the roboRIO.
     * Requires {@code useWPILibProfile(true)} in config.
     */
    public Command goToProfiled(double degrees) {
        if (profiledPID == null) {
            throw new IllegalStateException(
                    name + ": WPILib profile not configured. Use .useWPILibProfile() in config.");
        }
        return run(() -> {
            setpointDegrees = MathUtil.clamp(degrees, config.minAngle, config.maxAngle);
            double posRotations = degreesToRotations(getAngle());
            double targetRotations = degreesToRotations(setpointDegrees);
            double output = profiledPID.calculate(posRotations, targetRotations);
            double ff = config.useCosineGravity
                    ? feedforwardGains.calculateArm(Math.toRadians(getAngle()),
                            profiledPID.getSetpoint().velocity * 2 * Math.PI)
                    : feedforwardGains.calculateSimple(profiledPID.getSetpoint().velocity * 2 * Math.PI);
            motor.setVoltage(output + ff);
            setState("ProfiledGoTo " + String.format("%.1f", setpointDegrees) + "deg");
        }).beforeStarting(() -> profiledPID.reset(
                degreesToRotations(getAngle()),
                degreesToRotations(getAngularVelocity())))
                .withName(name + ".ProfiledGoTo(" + String.format("%.1f", degrees) + ")");
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
     */
    public Command holdPositionProfiled() {
        if (profiledPID == null) {
            throw new IllegalStateException(
                    name + ": WPILib profile not configured. Use .useWPILibProfile() in config.");
        }
        return run(() -> {
            double posRotations = degreesToRotations(getAngle());
            double targetRotations = degreesToRotations(setpointDegrees);
            double output = profiledPID.calculate(posRotations, targetRotations);
            double ff = config.useCosineGravity
                    ? feedforwardGains.calculateArm(Math.toRadians(getAngle()),
                            profiledPID.getSetpoint().velocity * 2 * Math.PI)
                    : feedforwardGains.calculateSimple(profiledPID.getSetpoint().velocity * 2 * Math.PI);
            motor.setVoltage(output + ff);
            setState("ProfiledHold " + String.format("%.1f", setpointDegrees) + "deg");
        }).beforeStarting(() -> profiledPID.reset(
                degreesToRotations(getAngle()),
                degreesToRotations(getAngularVelocity())))
                .withName(name + ".ProfiledHoldPosition");
    }

    /** Command to zero the encoder at the current position. */
    public Command zero() {
        return runOnce(() -> {
            motor.zeroEncoder();
            setpointDegrees = 0;
            setState("Zeroed");
        }).withName(name + ".Zero");
    }

    // --- Internals ---

    @Override
    protected void stop() {
        motor.stop();
        setState("Stopped");
    }

    /** Check if the hard stop / home switch is pressed. */
    public boolean isHardStopPressed() {
        return hardStop != null && !hardStop.get();
    }

    /** Trigger for the hard stop switch. */
    public Trigger hardStopTrigger() {
        return new Trigger(this::isHardStopPressed);
    }

    @Override
    protected void updateTelemetry() {
        motor.updateTelemetry();
        log("AngleDegrees", getAngle());
        log("AngularVelocityDPS", getAngularVelocity());
        log("SetpointDegrees", setpointDegrees);
        log("CurrentAmps", getCurrent());
        log("AtSetpoint", atSetpoint(2.0));
        if (hardStop != null) log("HardStop", isHardStopPressed());

        // Auto-zero on hard stop
        if (config.autoZeroOnHardStop && isHardStopPressed()) {
            motor.setEncoderPosition(degreesToRotations(config.hardStopAngle));
            setpointDegrees = config.hardStopAngle;
            hasBeenZeroed = true;
        }

        // Fault detection with temperature cutoff
        double temp = motor.getTemperature();
        if (temp > config.maxTemperatureC) {
            consecutiveHighTempCycles++;
            if (consecutiveHighTempCycles > 50) {
                AlertManager.getInstance().warning(name,
                        "Motor temperature high: " + String.format("%.0f", temp) + "C");
                if (temp > config.maxTemperatureC + 10) {
                    motor.stop();
                    AlertManager.getInstance().error(name,
                            "Motor OVERTEMP cutoff at " + String.format("%.0f", temp) + "C!");
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
            double mechanismRotations = Math.toDegrees(sim.getAngleRads()) / 360.0;
            simState.setRawRotorPosition(mechanismRotations * config.gearRatio);
            double mechanismVelRPS = Math.toDegrees(sim.getVelocityRadPerSec()) / 360.0;
            simState.setRotorVelocity(mechanismVelRPS * config.gearRatio);
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
        final MotorType motorType;
        final double gearRatio;
        final double length;
        final double mass;
        final double customMOI;
        final double minAngle;
        final double maxAngle;
        final double currentLimit;
        final double statorCurrentLimit;
        final boolean useCosineGravity;
        final double kP, kI, kD;
        final double kS, kV, kA, kG;
        final double motionMagicCruiseVelocity;
        final double motionMagicAcceleration;
        final double motionMagicJerk;
        final Map<String, Double> namedPositions;
        final double startingAngle;
        final int hardStopPort;
        final boolean autoZeroOnHardStop;
        final double hardStopAngle;
        final double maxTemperatureC;

        // WPILib ProfiledPID
        final boolean useWPILibProfile;
        final double profileKP, profileKI, profileKD;
        final double profileMaxVelocity;     // deg/s
        final double profileMaxAcceleration; // deg/s^2

        private Config(Builder b) {
            this.name = b.name;
            this.motorCanId = b.motorCanId;
            this.canBus = b.canBus;
            this.inverted = b.inverted;
            this.followerCanId = b.followerCanId;
            this.followerOppose = b.followerOppose;
            this.motorType = b.motorType;
            this.gearRatio = b.gearRatio;
            this.length = b.length;
            this.mass = b.mass;
            this.customMOI = b.customMOI;
            this.minAngle = b.minAngle;
            this.maxAngle = b.maxAngle;
            this.currentLimit = b.currentLimit;
            this.statorCurrentLimit = b.statorCurrentLimit;
            this.useCosineGravity = b.useCosineGravity;
            this.kP = b.kP; this.kI = b.kI; this.kD = b.kD;
            this.kS = b.kS; this.kV = b.kV; this.kA = b.kA; this.kG = b.kG;
            this.motionMagicCruiseVelocity = b.motionMagicCruiseVelocity;
            this.motionMagicAcceleration = b.motionMagicAcceleration;
            this.motionMagicJerk = b.motionMagicJerk;
            this.namedPositions = Map.copyOf(b.namedPositions);
            this.startingAngle = b.startingAngle;
            this.hardStopPort = b.hardStopPort;
            this.autoZeroOnHardStop = b.autoZeroOnHardStop;
            this.hardStopAngle = b.hardStopAngle;
            this.maxTemperatureC = b.maxTemperatureC;
            this.useWPILibProfile = b.useWPILibProfile;
            this.profileKP = b.profileKP;
            this.profileKI = b.profileKI;
            this.profileKD = b.profileKD;
            this.profileMaxVelocity = b.profileMaxVelocity;
            this.profileMaxAcceleration = b.profileMaxAcceleration;
        }

        /**
         * Estimate the gravity feedforward voltage at a given angle.
         * Uses configured motor type, gear ratio, arm length, and mass.
         */
        public double estimateGravityFF(double angleDegrees) {
            double torque = mass * 9.81 * length * Math.cos(Math.toRadians(angleDegrees));
            return motorType.holdingVoltage(torque, gearRatio);
        }

        /**
         * Estimate max angular speed in degrees per second.
         */
        public double estimateMaxSpeed() {
            double maxMotorRPS = motorType.freeSpeedRPS();
            return (maxMotorRPS / gearRatio) * 360.0;
        }

        public static Builder builder() {
            return new Builder();
        }

        public static class Builder {
            private String name = "RotationalMechanism";
            private int motorCanId = 0;
            private String canBus = "";
            private boolean inverted = false;
            private int followerCanId = -1;
            private boolean followerOppose = false;
            private MotorType motorType = MotorType.KRAKEN_X60;
            private double gearRatio = 1.0;
            private double length = 0.5; // meters
            private double mass = 3.0; // kg
            private double customMOI = -1; // -1 = auto-estimate
            private double minAngle = -180;
            private double maxAngle = 180;
            private double currentLimit = 40;
            private double statorCurrentLimit = 80;
            private boolean useCosineGravity = true;
            private double kP = 0, kI = 0, kD = 0;
            private double kS = 0, kV = 0, kA = 0, kG = 0;
            private double motionMagicCruiseVelocity = 0;
            private double motionMagicAcceleration = 0;
            private double motionMagicJerk = 0;
            private final Map<String, Double> namedPositions = new HashMap<>();
            private double startingAngle = 0;
            private int hardStopPort = -1;
            private boolean autoZeroOnHardStop = false;
            private double hardStopAngle = 0;
            private double maxTemperatureC = 70;
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

            /** Motor-to-mechanism gear ratio (motor rotations per mechanism rotation). */
            public Builder gearRatio(double ratio) { this.gearRatio = ratio; return this; }

            /** Arm length in meters (pivot to center of mass, for simulation). */
            public Builder length(double meters) { this.length = meters; return this; }

            /** Mass of the arm in kg (for simulation and gravity FF estimation). */
            public Builder mass(double kg) { this.mass = kg; return this; }

            /**
             * Override the auto-calculated moment of inertia (kg*m^2).
             * Use this for non-uniform arm geometries.
             */
            public Builder moi(double kgm2) { this.customMOI = kgm2; return this; }

            /** Set min and max angle in degrees. */
            public Builder range(double minDegrees, double maxDegrees) {
                this.minAngle = minDegrees;
                this.maxAngle = maxDegrees;
                return this;
            }

            public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
            public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }

            /** Use cosine gravity compensation (true for arms, false for turrets/continuous rotation). */
            public Builder useCosineGravity(boolean use) { this.useCosineGravity = use; return this; }

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

            /** Motion Magic parameters in mechanism units (rot/s, rot/s^2, rot/s^3). */
            public Builder motionMagic(double cruiseVelocity, double acceleration, double jerk) {
                this.motionMagicCruiseVelocity = cruiseVelocity;
                this.motionMagicAcceleration = acceleration;
                this.motionMagicJerk = jerk;
                return this;
            }

            /** Add a named position preset in degrees. */
            public Builder position(String name, double degrees) {
                this.namedPositions.put(name, degrees);
                return this;
            }

            /**
             * Starting angle of the mechanism in degrees (default 0).
             * The encoder is seeded to this position on construction.
             */
            public Builder startingAngle(double degrees) { this.startingAngle = degrees; return this; }

            /**
             * Add a hard stop / home switch on a DIO port.
             * @param dioPort DIO port number
             * @param autoZero if true, auto-zeros the encoder when switch is triggered
             * @param homeAngle the angle in degrees at the hard stop position
             */
            public Builder hardStop(int dioPort, boolean autoZero, double homeAngle) {
                this.hardStopPort = dioPort;
                this.autoZeroOnHardStop = autoZero;
                this.hardStopAngle = homeAngle;
                return this;
            }

            /** Set the temperature threshold for fault alerts (default 70C). */
            public Builder maxTemperature(double celsius) { this.maxTemperatureC = celsius; return this; }

            /**
             * Enable WPILib ProfiledPID as an alternative to CTRE Motion Magic.
             * Runs the trapezoidal profile on the roboRIO.
             * Use goToProfiled() and holdPositionProfiled() commands when enabled.
             *
             * @param kP proportional gain (volts per rotation of error)
             * @param kI integral gain
             * @param kD derivative gain
             * @param maxVelocityDPS max velocity in degrees per second
             * @param maxAccelerationDPSS max acceleration in degrees per second squared
             */
            public Builder useWPILibProfile(double kP, double kI, double kD,
                                             double maxVelocityDPS, double maxAccelerationDPSS) {
                this.useWPILibProfile = true;
                this.profileKP = kP;
                this.profileKI = kI;
                this.profileKD = kD;
                this.profileMaxVelocity = maxVelocityDPS;
                this.profileMaxAcceleration = maxAccelerationDPSS;
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
