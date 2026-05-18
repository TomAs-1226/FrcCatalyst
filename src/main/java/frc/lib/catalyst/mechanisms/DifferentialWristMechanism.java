package frc.lib.catalyst.mechanisms;

import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.io.DifferentialWristMechanismInputs;
import frc.lib.catalyst.util.HealthMonitor;
import frc.lib.catalyst.util.TunableGains;
import frc.lib.catalyst.util.TunableNumber;

import java.util.HashMap;
import java.util.Map;

/**
 * Two-motor differential wrist mechanism (a.k.a. "diffy wrist") driven by
 * <b>CTRE's native differential control</b>.
 *
 * <p>A diffy wrist couples two coaxial motors through a bevel-gear differential
 * so that:
 * <ul>
 *   <li>Sum of motor rotations → pitch axis</li>
 *   <li>Difference of motor rotations → roll axis</li>
 * </ul>
 *
 * <p>Internally the <b>left motor is the master</b> and runs
 * {@link DifferentialMotionMagicVoltage} with the right motor configured as a
 * {@link DifferentialFollower}. Both targets are sent in one CAN frame and
 * Phoenix-6 keeps them coordinated at firmware level — much tighter than the
 * old "two independent Motion Magic loops" approach (and the right thing for
 * any 2-motor differential mechanism). Slot 0 holds the <b>average / pitch</b>
 * gains; Slot 1 holds the <b>differential / roll</b> gains. By default the two
 * slots share gains; call {@link Config.Builder#differentialPid(double, double, double)}
 * to tune them separately.
 *
 * <p>Inputs are snapshotted each loop into
 * {@link DifferentialWristMechanismInputs} and published through
 * {@link frc.lib.catalyst.logging.CatalystLog}.
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
 *         .pid(40, 0, 0.5)               // applies to pitch (Slot 0) and roll (Slot 1)
 *         .differentialPid(30, 0, 0.3)   // optional: separate roll gains
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

    // Reused control requests — avoid per-loop GC.
    private final DifferentialMotionMagicVoltage diffMMRequest =
            new DifferentialMotionMagicVoltage(0, 0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private double pitchSetpointDegrees = 0;
    private double rollSetpointDegrees = 0;
    private boolean hasBeenZeroed = false;

    private final DifferentialWristMechanismInputs inputs =
            new DifferentialWristMechanismInputs();

    // Live-tunable gains. Slot 0 (avg/pitch) is hot-reloaded by TunableGains as
    // for every other mechanism. Slot 1 (diff/roll) has its own four tunables
    // since it's specific to differential mechanisms.
    private final TunableGains tunableGains;
    private final TunableNumber diffKP, diffKI, diffKD;
    private final TunableNumber diffKS, diffKV, diffKA;

    public DifferentialWristMechanism(Config config) {
        super(config.name);
        this.config = config;

        // Master (LEFT): full PID + Motion Magic configuration. Slot 0 = pitch gains.
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

        // Slave (RIGHT): minimal config. PID gains aren't used — the slave
        // receives a coordinated voltage command from the master each frame.
        // We still keep gearRatio so the slave's encoder reports mechanism
        // rotations, which the master's DifferentialSensors block subtracts
        // against to compute the differential feedback.
        this.rightMotor = CatalystMotor.builder(config.rightMotorCanId)
                .name(config.name + "Right")
                .canBus(config.canBus)
                .inverted(config.rightInverted)
                .brakeMode(true)
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit)
                .gearRatio(config.gearRatio)
                .build();

        // Wire master → slave differential pairing. Two pieces:
        //   (1) Master's DifferentialSensors points at the slave's encoder so
        //       avg/diff feedback are computed at firmware level.
        //   (2) Master's Slot 1 holds the differential PID gains.
        //   (3) Slave is put into DifferentialFollower mode; it relays the
        //       master's coordinated output without running its own PID.
        DifferentialSensorsConfigs diffSensors = new DifferentialSensorsConfigs();
        diffSensors.DifferentialSensorSource = DifferentialSensorSourceValue.RemoteTalonFX_HalfDiff;
        diffSensors.DifferentialTalonFXSensorID = config.rightMotorCanId;
        leftMotor.getTalonFX().getConfigurator().apply(diffSensors);

        Slot1Configs s1 = new Slot1Configs();
        s1.kP = config.diffKP;
        s1.kI = config.diffKI;
        s1.kD = config.diffKD;
        s1.kS = config.diffKS;
        s1.kV = config.diffKV;
        s1.kA = config.diffKA;
        leftMotor.getTalonFX().getConfigurator().apply(s1);

        rightMotor.getTalonFX().setControl(
                new DifferentialFollower(config.leftMotorCanId, MotorAlignmentValue.Aligned));

        diffMMRequest.AverageSlot = 0;
        diffMMRequest.DifferentialSlot = 1;

        // Live tuning wiring.
        this.tunableGains = new TunableGains(
                config.name,
                config.kP, config.kI, config.kD,
                config.kS, config.kV, config.kA, 0,
                config.motionMagicCruiseVelocity,
                config.motionMagicAcceleration,
                config.motionMagicJerk);
        String diffPath = "Catalyst/Tuning/" + config.name + "/Diff";
        this.diffKP = new TunableNumber(diffPath + "/kP", config.diffKP);
        this.diffKI = new TunableNumber(diffPath + "/kI", config.diffKI);
        this.diffKD = new TunableNumber(diffPath + "/kD", config.diffKD);
        this.diffKS = new TunableNumber(diffPath + "/kS", config.diffKS);
        this.diffKV = new TunableNumber(diffPath + "/kV", config.diffKV);
        this.diffKA = new TunableNumber(diffPath + "/kA", config.diffKA);

        HealthMonitor.standardMotorChecks(name, "Left", leftMotor, config.statorCurrentLimit, 70);
        HealthMonitor.standardMotorChecks(name, "Right", rightMotor, config.statorCurrentLimit, 70);
    }

    // --- Conversions ---
    //
    // Math note: with the convention
    //     leftRot  = (pitch + roll) / 360
    //     rightRot = (pitch - roll) / 360
    // and Phoenix-6 `RemoteTalonFX_HalfDiff` feedback defined as
    //     avg_feedback  = (master + remote) / 2  = pitch / 360
    //     diff_feedback = (master - remote) / 2  = roll  / 360
    // both commanded targets are simply `degrees / 360` in mechanism rotations.

    private static double pitchDegreesToAvgRotations(double pitchDeg) {
        return pitchDeg / 360.0;
    }

    private static double rollDegreesToDiffRotations(double rollDeg) {
        return rollDeg / 360.0;
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

    /** Command both axes to a (pitch, roll) target via the native differential Motion Magic. */
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
        double avgRot = pitchDegreesToAvgRotations(pitchSetpointDegrees);
        double diffRot = rollDegreesToDiffRotations(rollSetpointDegrees);
        leftMotor.getTalonFX().setControl(diffMMRequest
                .withAveragePosition(avgRot)
                .withDifferentialPosition(diffRot));
    }

    private boolean diffGainsChanged() {
        return diffKP.hasChanged() || diffKI.hasChanged() || diffKD.hasChanged()
                || diffKS.hasChanged() || diffKV.hasChanged() || diffKA.hasChanged();
    }

    // --- Internals ---

    @Override
    protected void stop() {
        leftMotor.getTalonFX().setControl(neutralRequest);
        // Slave neutrals automatically when the master neutrals; no need to
        // override its DifferentialFollower mode.
        setState("Stopped");
    }

    @Override
    protected void updateTelemetry() {
        leftMotor.updateTelemetry();
        rightMotor.updateTelemetry();
        tunableGains.checkAndApply(leftMotor);
        if (diffGainsChanged()) {
            leftMotor.updateSlot1(
                    diffKP.get(), diffKI.get(), diffKD.get(),
                    diffKS.get(), diffKV.get(), diffKA.get());
        }

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

        HealthMonitor.getInstance().update();
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
        final double diffKP, diffKI, diffKD;
        final double diffKS, diffKV, diffKA;
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
            // Differential gains default to the average gains when the user
            // doesn't override — works for symmetric wrists where pitch and
            // roll have similar dynamics.
            this.diffKP = b.diffKP != null ? b.diffKP : b.kP;
            this.diffKI = b.diffKI != null ? b.diffKI : b.kI;
            this.diffKD = b.diffKD != null ? b.diffKD : b.kD;
            this.diffKS = b.diffKS != null ? b.diffKS : b.kS;
            this.diffKV = b.diffKV != null ? b.diffKV : b.kV;
            this.diffKA = b.diffKA != null ? b.diffKA : b.kA;
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
            // Boxed so we can distinguish "unset → fall back to avg gains"
            // from "set to 0 explicitly".
            private Double diffKP, diffKI, diffKD;
            private Double diffKS, diffKV, diffKA;
            private double motionMagicCruiseVelocity = 0;
            private double motionMagicAcceleration = 0;
            private double motionMagicJerk = 0;
            private double toleranceDegrees = 2.0;
            private final Map<String, double[]> namedPositions = new HashMap<>();

            public Builder name(String name) { this.name = name; return this; }

            /** CAN ID of the left motor, which acts as the differential master. */
            public Builder leftMotor(int canId) { this.leftMotorCanId = canId; return this; }

            /** CAN ID of the right motor, which acts as the differential follower. */
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

            /**
             * Slot 0 (average / pitch-axis) PID gains. Also seeds the differential
             * gains if {@link #differentialPid(double, double, double)} isn't
             * called.
             */
            public Builder pid(double kP, double kI, double kD) {
                this.kP = kP; this.kI = kI; this.kD = kD; return this;
            }

            public Builder feedforward(double kS, double kV) { this.kS = kS; this.kV = kV; return this; }
            public Builder feedforward(double kS, double kV, double kA) {
                this.kS = kS; this.kV = kV; this.kA = kA; return this;
            }

            /**
             * Slot 1 (differential / roll-axis) PID gains. When not set, the
             * differential controller uses the same gains as the average controller.
             */
            public Builder differentialPid(double kP, double kI, double kD) {
                this.diffKP = kP; this.diffKI = kI; this.diffKD = kD;
                return this;
            }

            /** Slot 1 (differential / roll-axis) feedforward gains. */
            public Builder differentialFeedforward(double kS, double kV) {
                this.diffKS = kS; this.diffKV = kV;
                return this;
            }

            /** Slot 1 (differential / roll-axis) feedforward gains with kA. */
            public Builder differentialFeedforward(double kS, double kV, double kA) {
                this.diffKS = kS; this.diffKV = kV; this.diffKA = kA;
                return this;
            }

            /** Motion Magic parameters shared by both axes (mechanism rot/s, rot/s^2, rot/s^3). */
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
                if (leftMotorCanId == rightMotorCanId) {
                    throw new IllegalStateException("Left and right motors must have different CAN IDs");
                }
                return new Config(this);
            }
        }
    }

}
