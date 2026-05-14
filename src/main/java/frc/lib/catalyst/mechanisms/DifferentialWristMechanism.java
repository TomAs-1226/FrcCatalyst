package frc.lib.catalyst.mechanisms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.io.DifferentialWristMechanismInputs;

import java.util.HashMap;
import java.util.Map;

/**
 * Two-motor differential wrist mechanism (a.k.a. "diffy wrist").
 *
 * <p>A diffy wrist couples two coaxial motors through a bevel-gear differential
 * so that:
 * <ul>
 *   <li>Sum of motor rotations → pitch axis</li>
 *   <li>Difference of motor rotations → roll axis</li>
 * </ul>
 *
 * <p>The mechanism converts between (pitch, roll) and (leftRotations, rightRotations)
 * using:
 * <pre>{@code
 * leftRotations  = (pitch + roll) / 360.0
 * rightRotations = (pitch - roll) / 360.0
 * }</pre>
 *
 * <p>Both motors are commanded via independent Motion Magic. Inputs are
 * snapshotted each loop into {@link DifferentialWristMechanismInputs} and
 * published through {@link frc.lib.catalyst.logging.CatalystLog}.
 *
 * <p>Example usage:
 * <pre>{@code
 * DifferentialWristMechanism wrist = new DifferentialWristMechanism(
 *     DifferentialWristMechanism.Config.builder()
 *         .name("Wrist")
 *         .leftMotor(40)
 *         .rightMotor(41)
 *         .gearRatio(20.0)
 *         .pitchRange(-90, 90)
 *         .rollRange(-180, 180)
 *         .pid(40, 0, 0.5)
 *         .motionMagic(50, 100, 500)
 *         .currentLimit(40)
 *         .position("STOW", 0, 0)
 *         .position("SCORE", 60, 90)
 *         .build());
 * }</pre>
 */
public class DifferentialWristMechanism extends CatalystMechanism {

    private final Config config;
    private final CatalystMotor leftMotor;
    private final CatalystMotor rightMotor;

    private double pitchSetpointDegrees = 0;
    private double rollSetpointDegrees = 0;
    private boolean hasBeenZeroed = false;

    private final DifferentialWristMechanismInputs inputs = new DifferentialWristMechanismInputs();

    public DifferentialWristMechanism(Config config) {
        super(config.name);
        this.config = config;

        this.leftMotor = CatalystMotor.builder(config.leftMotorCanId)
                .name(config.name + "Left")
                .canBus(config.canBus)
                .inverted(config.leftInverted)
                .brakeMode(true)
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit)
                .gearRatio(config.gearRatio)
                .pid(config.kP, config.kI, config.kD)
                .feedforward(config.kS, config.kV, config.kA)
                .motionMagic(config.motionMagicCruiseVelocity,
                        config.motionMagicAcceleration,
                        config.motionMagicJerk)
                .build();

        this.rightMotor = CatalystMotor.builder(config.rightMotorCanId)
                .name(config.name + "Right")
                .canBus(config.canBus)
                .inverted(config.rightInverted)
                .brakeMode(true)
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit)
                .gearRatio(config.gearRatio)
                .pid(config.kP, config.kI, config.kD)
                .feedforward(config.kS, config.kV, config.kA)
                .motionMagic(config.motionMagicCruiseVelocity,
                        config.motionMagicAcceleration,
                        config.motionMagicJerk)
                .build();
    }

    // --- Conversions ---

    private static double degreesToRotations(double degrees) {
        return degrees / 360.0;
    }

    private static double rotationsToDegrees(double rotations) {
        return rotations * 360.0;
    }

    // --- Resolved state ---

    /** Resolved pitch axis position in degrees. */
    public double getPitch() {
        return rotationsToDegrees(leftMotor.getPosition() + rightMotor.getPosition()) / 2.0;
    }

    /** Resolved roll axis position in degrees. */
    public double getRoll() {
        return rotationsToDegrees(leftMotor.getPosition() - rightMotor.getPosition()) / 2.0;
    }

    /** Resolved pitch axis velocity in degrees per second. */
    public double getPitchVelocity() {
        return rotationsToDegrees(leftMotor.getVelocity() + rightMotor.getVelocity()) / 2.0;
    }

    /** Resolved roll axis velocity in degrees per second. */
    public double getRollVelocity() {
        return rotationsToDegrees(leftMotor.getVelocity() - rightMotor.getVelocity()) / 2.0;
    }

    /** Last commanded pitch setpoint in degrees. */
    public double getPitchSetpoint() {
        return pitchSetpointDegrees;
    }

    /** Last commanded roll setpoint in degrees. */
    public double getRollSetpoint() {
        return rollSetpointDegrees;
    }

    /** True when both pitch and roll are within configured tolerance of their setpoints. */
    public boolean atSetpoint() {
        return Math.abs(getPitch() - pitchSetpointDegrees) < config.toleranceDegrees
                && Math.abs(getRoll() - rollSetpointDegrees) < config.toleranceDegrees;
    }

    /** Trigger for {@link #atSetpoint()}. */
    public Trigger atSetpointTrigger() {
        return new Trigger(this::atSetpoint);
    }

    // --- Command Factories ---

    /** Command both axes to a (pitch, roll) target via Motion Magic. */
    public Command goTo(double pitchDegrees, double rollDegrees) {
        return runOnce(() -> {
            applyTargets(pitchDegrees, rollDegrees);
            setState(String.format("GoTo p=%.1f r=%.1f", pitchSetpointDegrees, rollSetpointDegrees));
        }).withName(name + String.format(".GoTo(%.1f, %.1f)", pitchDegrees, rollDegrees));
    }

    /** Command both axes to a named preset. */
    public Command goTo(String positionName) {
        double[] target = config.namedPositions.get(positionName);
        if (target == null) {
            throw new IllegalArgumentException("Unknown position '" + positionName + "' for " + name
                    + ". Available: " + config.namedPositions.keySet());
        }
        return goTo(target[0], target[1]).withName(name + ".GoTo(" + positionName + ")");
    }

    /** Command both axes to a target and end when both are within tolerance. */
    public Command goToAndWait(double pitchDegrees, double rollDegrees) {
        return run(() -> {
            applyTargets(pitchDegrees, rollDegrees);
            setState(String.format("GoTo p=%.1f r=%.1f", pitchSetpointDegrees, rollSetpointDegrees));
        }).until(this::atSetpoint)
                .withName(name + String.format(".GoToAndWait(%.1f, %.1f)", pitchDegrees, rollDegrees));
    }

    /** Command that continuously holds the current setpoints. Good as a default command. */
    public Command holdPosition() {
        return run(() -> {
            applyTargets(pitchSetpointDegrees, rollSetpointDegrees);
            setState("Hold");
        }).withName(name + ".HoldPosition");
    }

    /** Command to seed both motor encoders so that {@code (pitch=0, roll=0)} corresponds to the current position. */
    public Command zero() {
        return runOnce(() -> {
            leftMotor.zeroEncoder();
            rightMotor.zeroEncoder();
            pitchSetpointDegrees = 0;
            rollSetpointDegrees = 0;
            hasBeenZeroed = true;
            setState("Zeroed");
        }).withName(name + ".Zero");
    }

    // --- Helpers ---

    private void applyTargets(double pitchDegrees, double rollDegrees) {
        pitchSetpointDegrees = MathUtil.clamp(pitchDegrees, config.minPitch, config.maxPitch);
        rollSetpointDegrees = MathUtil.clamp(rollDegrees, config.minRoll, config.maxRoll);
        double leftRotations = degreesToRotations(pitchSetpointDegrees + rollSetpointDegrees);
        double rightRotations = degreesToRotations(pitchSetpointDegrees - rollSetpointDegrees);
        leftMotor.setMotionMagicPosition(leftRotations);
        rightMotor.setMotionMagicPosition(rightRotations);
    }

    // --- Internals ---

    @Override
    protected void stop() {
        leftMotor.stop();
        rightMotor.stop();
        setState("Stopped");
    }

    @Override
    protected void updateTelemetry() {
        leftMotor.updateTelemetry();
        rightMotor.updateTelemetry();

        inputs.pitchDegrees = getPitch();
        inputs.rollDegrees = getRoll();
        inputs.pitchVelocityDPS = getPitchVelocity();
        inputs.rollVelocityDPS = getRollVelocity();
        inputs.pitchSetpointDegrees = pitchSetpointDegrees;
        inputs.rollSetpointDegrees = rollSetpointDegrees;
        inputs.atSetpoint = atSetpoint();
        inputs.leftStatorCurrentAmps = leftMotor.getStatorCurrent();
        inputs.rightStatorCurrentAmps = rightMotor.getStatorCurrent();
        inputs.leftAppliedVolts = leftMotor.getAppliedVoltage();
        inputs.rightAppliedVolts = rightMotor.getAppliedVoltage();
        inputs.leftTemperatureC = leftMotor.getTemperature();
        inputs.rightTemperatureC = rightMotor.getTemperature();
        inputs.hasBeenZeroed = hasBeenZeroed;
        processInputs(inputs);

        log("PitchDegrees", inputs.pitchDegrees);
        log("RollDegrees", inputs.rollDegrees);
        log("PitchSetpointDegrees", inputs.pitchSetpointDegrees);
        log("RollSetpointDegrees", inputs.rollSetpointDegrees);
        log("AtSetpoint", inputs.atSetpoint);
    }

    public CatalystMotor getLeftMotor() { return leftMotor; }
    public CatalystMotor getRightMotor() { return rightMotor; }

    // ===========================================
    //                  CONFIG
    // ===========================================

    public static class Config {
        final String name;
        final int leftMotorCanId;
        final int rightMotorCanId;
        final String canBus;
        final boolean leftInverted;
        final boolean rightInverted;
        final double gearRatio;
        final double minPitch, maxPitch;
        final double minRoll, maxRoll;
        final double currentLimit;
        final double statorCurrentLimit;
        final double kP, kI, kD;
        final double kS, kV, kA;
        final double motionMagicCruiseVelocity;
        final double motionMagicAcceleration;
        final double motionMagicJerk;
        final double toleranceDegrees;
        final Map<String, double[]> namedPositions;

        private Config(Builder b) {
            this.name = b.name;
            this.leftMotorCanId = b.leftMotorCanId;
            this.rightMotorCanId = b.rightMotorCanId;
            this.canBus = b.canBus;
            this.leftInverted = b.leftInverted;
            this.rightInverted = b.rightInverted;
            this.gearRatio = b.gearRatio;
            this.minPitch = b.minPitch; this.maxPitch = b.maxPitch;
            this.minRoll = b.minRoll; this.maxRoll = b.maxRoll;
            this.currentLimit = b.currentLimit;
            this.statorCurrentLimit = b.statorCurrentLimit;
            this.kP = b.kP; this.kI = b.kI; this.kD = b.kD;
            this.kS = b.kS; this.kV = b.kV; this.kA = b.kA;
            this.motionMagicCruiseVelocity = b.motionMagicCruiseVelocity;
            this.motionMagicAcceleration = b.motionMagicAcceleration;
            this.motionMagicJerk = b.motionMagicJerk;
            this.toleranceDegrees = b.toleranceDegrees;
            this.namedPositions = Map.copyOf(b.namedPositions);
        }

        public static Builder builder() { return new Builder(); }

        public static class Builder {
            private String name = "DifferentialWristMechanism";
            private int leftMotorCanId = 0;
            private int rightMotorCanId = 0;
            private String canBus = "";
            private boolean leftInverted = false;
            private boolean rightInverted = false;
            private double gearRatio = 1.0;
            private double minPitch = -180, maxPitch = 180;
            private double minRoll = -180, maxRoll = 180;
            private double currentLimit = 40;
            private double statorCurrentLimit = 80;
            private double kP = 0, kI = 0, kD = 0;
            private double kS = 0, kV = 0, kA = 0;
            private double motionMagicCruiseVelocity = 0;
            private double motionMagicAcceleration = 0;
            private double motionMagicJerk = 0;
            private double toleranceDegrees = 2.0;
            private final Map<String, double[]> namedPositions = new HashMap<>();

            public Builder name(String name) { this.name = name; return this; }
            public Builder leftMotor(int canId) { this.leftMotorCanId = canId; return this; }
            public Builder rightMotor(int canId) { this.rightMotorCanId = canId; return this; }
            public Builder canBus(String canBus) { this.canBus = canBus; return this; }
            public Builder leftInverted(boolean inverted) { this.leftInverted = inverted; return this; }
            public Builder rightInverted(boolean inverted) { this.rightInverted = inverted; return this; }

            /** Motor-to-mechanism gear ratio (motor rotations per mechanism rotation). */
            public Builder gearRatio(double ratio) { this.gearRatio = ratio; return this; }

            /** Pitch axis software limits in degrees. */
            public Builder pitchRange(double minDegrees, double maxDegrees) {
                this.minPitch = minDegrees;
                this.maxPitch = maxDegrees;
                return this;
            }

            /** Roll axis software limits in degrees. */
            public Builder rollRange(double minDegrees, double maxDegrees) {
                this.minRoll = minDegrees;
                this.maxRoll = maxDegrees;
                return this;
            }

            public Builder currentLimit(double amps) { this.currentLimit = amps; return this; }
            public Builder statorCurrentLimit(double amps) { this.statorCurrentLimit = amps; return this; }

            public Builder pid(double kP, double kI, double kD) {
                this.kP = kP; this.kI = kI; this.kD = kD; return this;
            }

            public Builder feedforward(double kS, double kV) { this.kS = kS; this.kV = kV; return this; }
            public Builder feedforward(double kS, double kV, double kA) {
                this.kS = kS; this.kV = kV; this.kA = kA; return this;
            }

            /** Motion Magic parameters applied to both motors (mechanism rot/s, rot/s^2, rot/s^3). */
            public Builder motionMagic(double cruiseVelocity, double acceleration, double jerk) {
                this.motionMagicCruiseVelocity = cruiseVelocity;
                this.motionMagicAcceleration = acceleration;
                this.motionMagicJerk = jerk;
                return this;
            }

            /** Default tolerance for {@link DifferentialWristMechanism#atSetpoint()} in degrees. */
            public Builder tolerance(double degrees) { this.toleranceDegrees = degrees; return this; }

            /** Add a named (pitch, roll) preset in degrees. */
            public Builder position(String name, double pitchDegrees, double rollDegrees) {
                this.namedPositions.put(name, new double[] { pitchDegrees, rollDegrees });
                return this;
            }

            public Config build() {
                if (leftMotorCanId == 0 || rightMotorCanId == 0) {
                    throw new IllegalStateException("Both left and right motor CAN IDs must be set");
                }
                return new Config(this);
            }
        }
    }
}
