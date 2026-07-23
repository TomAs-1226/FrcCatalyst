package frc.lib.catalyst.mechanisms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.io.TurretMechanismInputs;
import frc.lib.catalyst.util.AimingSolverTOF;
import frc.lib.catalyst.util.HealthCheck;
import frc.lib.catalyst.util.HealthMonitor;
import frc.lib.catalyst.util.TunableGains;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Single-axis turret with continuous-angle resolution and field-relative
 * target tracking.
 *
 * <p>The turret stores its angle <b>relative to the chassis</b> in degrees,
 * with mechanical soft limits ({@code range(min, max)}). The hard problem
 * a turret introduces — staying within those limits while a target sweeps
 * across the robot, without spinning the wires off — is handled by
 * {@link #resolveTurretAngle}: given a desired robot-relative angle, it
 * picks the reachable representation ({@code desired + 360·k}) closest to
 * the current position, and only "unwraps" the long way around when the
 * short way would hit a hard stop.
 *
 * <p>For aiming at a fixed field point (a goal) while driving, pair this
 * with {@link AimingSolver}, which does the Shoot-On-The-Fly motion
 * compensation and hands back a field-relative bearing:
 *
 * <pre>{@code
 * TurretMechanism turret = new TurretMechanism(
 *     TurretMechanism.Config.builder()
 *         .name("Turret")
 *         .motor(15)
 *         .gearRatio(40.0)               // motor rotations per turret rotation
 *         .range(-200, 200)              // mechanical travel, degrees
 *         .pid(40, 0, 0.5)
 *         .feedforward(0.15, 0.0)
 *         .motionMagic(8, 16, 80)        // rps, rps^2, rps^3 (turret rotations)
 *         .tolerance(1.0)
 *         .build());
 *
 * AimingSolver solver = AimingSolver.builder()
 *     .target(FieldConstants.GOAL)
 *     .shotTime(shotTimeTable)
 *     .build();
 *
 * turret.setDefaultCommand(turret.track(
 *     () -> solver.solve(drive.getPose(), drive.getFieldRelativeSpeeds()),
 *     () -> drive.getHeading().getDegrees()));
 * }</pre>
 */
public class TurretMechanism extends CatalystMechanism {

    private final Config config;
    private final CatalystMotor motor;
    private final TunableGains tunableGains;

    private final TurretMechanismInputs inputs = new TurretMechanismInputs();

    // Simulation (null on a real robot).
    private DCMotorSim sim;

    private double setpointDegrees = 0.0;
    private double aimFieldAngleDegrees = Double.NaN;
    private boolean unwrapping = false;
    private boolean hasBeenZeroed = false;

    // For velocity feedforward while tracking a moving goal: differentiate
    // the resolved command across loops.
    private double lastResolvedDeg = Double.NaN;
    private double lastCommandTs = 0.0;

    public TurretMechanism(Config config) {
        super(config.name);
        this.config = config;

        CatalystMotor.Builder motorBuilder = CatalystMotor.builder(config.motorCanId)
                .name(config.name + "Motor")
                .canBus(config.canBus)
                .inverted(config.inverted)
                .brakeMode(config.brakeMode)
                .currentLimit(config.currentLimit)
                .statorCurrentLimit(config.statorCurrentLimit)
                .gearRatio(config.gearRatio)
                .pid(config.kP, config.kI, config.kD)
                .feedforward(config.kS, config.kV, config.kA)
                .motionMagic(config.motionMagicCruiseVelocity,
                        config.motionMagicAcceleration,
                        config.motionMagicJerk);

        // Mechanical soft limits in turret rotations.
        motorBuilder.softLimits(config.minAngle / 360.0, config.maxAngle / 360.0);

        if (config.cancoderId >= 0) {
            motorBuilder.fusedCANcoder(config.cancoderId, config.cancoderRotorToSensorRatio);
        }

        this.motor = motorBuilder.build();

        if (config.startingAngle != 0 && config.cancoderId < 0) {
            motor.setEncoderPosition(config.startingAngle / 360.0);
            setpointDegrees = config.startingAngle;
        }

        this.tunableGains = new TunableGains(
                config.name,
                config.kP, config.kI, config.kD,
                config.kS, config.kV, config.kA, 0,
                config.motionMagicCruiseVelocity,
                config.motionMagicAcceleration,
                config.motionMagicJerk);

        if (RobotBase.isSimulation()) {
            sim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            config.motorType.getDCMotor(1), config.simMOI, config.gearRatio),
                    config.motorType.getDCMotor(1));
        }

        registerHealthChecks();
    }

    @Override
    public void simulationPeriodic() {
        if (sim != null) {
            var simState = motor.getTalonFX().getSimState();
            sim.setInputVoltage(simState.getMotorVoltage());
            sim.update(0.02);
            simState.setRawRotorPosition(sim.getAngularPositionRotations() * config.gearRatio);
            simState.setRotorVelocity(sim.getAngularVelocityRPM() / 60.0 * config.gearRatio);
        }
    }

    private void registerHealthChecks() {
        HealthMonitor.standardMotorChecks(name, motor, config.statorCurrentLimit, config.maxTemperatureC);

        HealthCheck.builder(name, "Stall")
                .severity(HealthCheck.Severity.WARN)
                .description("Output applied but turret not moving")
                .when(() -> Math.abs(motor.getAppliedVoltage()) > 2.0
                        && Math.abs(getAngularVelocity()) < 0.5
                        && Math.abs(getAngle() - setpointDegrees) > config.toleranceDegrees * 4)
                .detail(() -> String.format("%.1fV, %.2f deg/s", motor.getAppliedVoltage(), getAngularVelocity()))
                .debounce(0.75)
                .clearAfter(0.25)
                .register();

        HealthCheck.builder(name, "NotZeroed")
                .severity(HealthCheck.Severity.INFO)
                .description("Turret has not been homed")
                .when(() -> !hasBeenZeroed && config.cancoderId < 0)
                .debounce(2.0)
                .register();
    }

    // ============================================================
    //              CONTINUOUS-ANGLE RESOLUTION
    // ============================================================

    /**
     * Resolve a desired robot-relative angle to a reachable turret setpoint.
     *
     * <p>Considers every {@code desired + 360·k} representation and returns
     * the in-range one closest to {@code currentDeg}. If none is in range,
     * clamps the nearest representation to the limits (the target is simply
     * unreachable — e.g. directly behind a ±170° turret).
     *
     * <p>Pure function — exposed static so it can be unit-tested.
     */
    public static double resolveTurretAngle(double desiredDeg, double currentDeg,
                                            double minDeg, double maxDeg) {
        double best = Double.NaN;
        double bestErr = Double.POSITIVE_INFINITY;
        for (int k = -3; k <= 3; k++) {
            double cand = desiredDeg + 360.0 * k;
            if (cand < minDeg || cand > maxDeg) continue;
            double err = Math.abs(cand - currentDeg);
            if (err < bestErr) {
                bestErr = err;
                best = cand;
            }
        }
        if (Double.isNaN(best)) {
            // Unreachable — clamp the closest representation to the limits.
            double closest = desiredDeg;
            double cErr = Double.POSITIVE_INFINITY;
            for (int k = -3; k <= 3; k++) {
                double cand = desiredDeg + 360.0 * k;
                double err = Math.abs(cand - currentDeg);
                if (err < cErr) {
                    cErr = err;
                    closest = cand;
                }
            }
            best = MathUtil.clamp(closest, minDeg, maxDeg);
        }
        return best;
    }

    private void commandResolved(double desiredRobotRelativeDeg) {
        double current = getAngle();
        double resolved = resolveTurretAngle(
                desiredRobotRelativeDeg, current, config.minAngle, config.maxAngle);
        // "unwrapping" = the resolved target is more than a half turn from the
        // naive (mod-360) target, i.e. we're taking the long way.
        double naive = MathUtil.inputModulus(desiredRobotRelativeDeg, current - 180, current + 180);
        unwrapping = Math.abs(resolved - naive) > 180.0;
        setpointDegrees = resolved;

        // Velocity feedforward: differentiate the resolved command and convert
        // deg/s → turret rps → volts via kV. This lets the turret lead a moving
        // goal instead of chasing it a loop behind. Skip the loop where an
        // unwrap makes the command jump (the derivative would be garbage), and
        // skip when kV isn't configured.
        double ffVolts = 0.0;
        double now = Timer.getFPGATimestamp();
        if (config.kV > 0 && !Double.isNaN(lastResolvedDeg)) {
            double dt = now - lastCommandTs;
            double delta = resolved - lastResolvedDeg;
            if (dt > 1e-4 && dt < 0.5 && Math.abs(delta) < 180.0) {
                double rps = (delta / dt) / 360.0;
                ffVolts = config.kV * rps;
                ffVolts = MathUtil.clamp(ffVolts, -2.0, 2.0); // FF shouldn't dominate
            }
        }
        lastResolvedDeg = resolved;
        lastCommandTs = now;

        motor.setMotionMagicPosition(resolved / 360.0, ffVolts);
    }

    /**
     * Command a resolved angle with an <em>explicit</em> velocity feedforward
     * (degrees/second, robot-relative) instead of differentiating the command.
     * Preferred for {@link #track}, where {@link AimingSolver} hands back an
     * exact analytic bearing rate — no finite-difference lag or noise.
     */
    private void commandResolved(double desiredRobotRelativeDeg, double ffDegPerSec) {
        double current = getAngle();
        double resolved = resolveTurretAngle(
                desiredRobotRelativeDeg, current, config.minAngle, config.maxAngle);
        double naive = MathUtil.inputModulus(desiredRobotRelativeDeg, current - 180, current + 180);
        unwrapping = Math.abs(resolved - naive) > 180.0;
        setpointDegrees = resolved;

        double ffVolts = 0.0;
        if (config.kV > 0 && !unwrapping && !Double.isNaN(ffDegPerSec)) {
            ffVolts = MathUtil.clamp(config.kV * (ffDegPerSec / 360.0), -2.0, 2.0);
        }
        lastResolvedDeg = resolved;
        lastCommandTs = Timer.getFPGATimestamp();

        motor.setMotionMagicPosition(resolved / 360.0, ffVolts);
    }

    // ============================================================
    //                       GETTERS
    // ============================================================

    /** Turret angle relative to the chassis, in degrees. */
    public double getAngle() {
        return motor.getPosition() * 360.0;
    }

    /** Turret angular velocity in degrees per second. */
    public double getAngularVelocity() {
        return motor.getVelocity() * 360.0;
    }

    /** Last commanded robot-relative setpoint, in degrees. */
    public double getSetpoint() {
        return setpointDegrees;
    }

    /** True when within the configured tolerance of the setpoint. */
    public boolean atSetpoint() {
        return Math.abs(getAngle() - setpointDegrees) < config.toleranceDegrees;
    }

    /** Trigger for {@link #atSetpoint()} — useful as a "ready to shoot" gate. */
    public Trigger atSetpointTrigger() {
        return new Trigger(this::atSetpoint);
    }

    /** True if the last aim resolved by taking the long way around the limits. */
    public boolean isUnwrapping() {
        return unwrapping;
    }

    /**
     * Live aiming error against a Shoot-On-The-Fly {@link AimingSolver.Solution}:
     * how far (degrees) the bore is from the solved field bearing right now.
     * Handy for dashboards and as the basis of a custom "ready" gate.
     *
     * @param solution        the latest solve
     * @param robotHeadingDeg current robot heading (degrees)
     */
    public double aimErrorDeg(AimingSolverTOF.Solution solution, double robotHeadingDeg) {
        if (solution == null) return Double.NaN;
        double targetRobotRel = solution.turretFieldAngleDeg() - robotHeadingDeg;
        return MathUtil.inputModulus(targetRobotRel - getAngle(), -180.0, 180.0);
    }

    /**
     * Whether the turret is aimed at the solution within {@code toleranceDeg} —
     * the "is the barrel on target" half of a shoot-while-moving readiness check
     * (pair with {@code shooter.atSpeed()} and {@code solution.feasible()}).
     */
    public boolean isOnTarget(AimingSolverTOF.Solution solution, double robotHeadingDeg, double toleranceDeg) {
        return solution != null && solution.feasible()
                && Math.abs(aimErrorDeg(solution, robotHeadingDeg)) <= toleranceDeg;
    }

    @Override
    public MechanismView describe() {
        return MechanismView.of(name, "turret")
                .value(getAngle(), "deg").setpoint(getSetpoint())
                .range(config.minAngle, config.maxAngle)
                .velocity(getAngularVelocity()).current(motor.getStatorCurrent())
                .extra("unwrapping", isUnwrapping())
                .extra("atSetpoint", atSetpoint())
                .build();
    }

    public CatalystMotor getMotor() {
        return motor;
    }

    // ============================================================
    //                   COMMAND FACTORIES
    // ============================================================

    /** Drive to a raw robot-relative turret angle (degrees), wrap-resolved. */
    public Command goToAngle(double robotRelativeDeg) {
        return runOnce(() -> {
            aimFieldAngleDegrees = Double.NaN;
            commandResolved(robotRelativeDeg);
            setState(String.format("GoTo %.1f", robotRelativeDeg));
        }).withName(name + ".GoToAngle(" + String.format("%.1f", robotRelativeDeg) + ")");
    }

    /** Point the turret straight forward (robot-relative 0°). */
    public Command lockForward() {
        return goToAngle(0).withName(name + ".LockForward");
    }

    /** Hold the current setpoint. Good as a default command when not tracking. */
    public Command holdAngle() {
        return run(() -> motor.setMotionMagicPosition(setpointDegrees / 360.0))
                .withName(name + ".Hold");
    }

    /**
     * Continuously aim at a field-relative bearing while subtracting the
     * live robot heading, so the turret stays locked on a field direction
     * as the chassis rotates.
     *
     * @param fieldAngleDeg   field-relative bearing supplier (degrees)
     * @param robotHeadingDeg robot heading supplier (degrees, same convention)
     */
    public Command aimAtFieldAngle(DoubleSupplier fieldAngleDeg, DoubleSupplier robotHeadingDeg) {
        return run(() -> {
            double field = fieldAngleDeg.getAsDouble();
            aimFieldAngleDegrees = field;
            commandResolved(field - robotHeadingDeg.getAsDouble());
            setState("AimField");
        }).withName(name + ".AimAtFieldAngle");
    }

    /**
     * Continuously aim at a fixed field point given the live robot pose.
     * Ignores robot velocity — for shoot-while-moving use {@link #track}.
     */
    public Command aimAtTarget(Supplier<Pose2d> robotPose, Translation2d target) {
        return run(() -> {
            Pose2d p = robotPose.get();
            Translation2d v = target.minus(p.getTranslation());
            double field = Math.toDegrees(Math.atan2(v.getY(), v.getX()));
            aimFieldAngleDegrees = field;
            commandResolved(field - p.getRotation().getDegrees());
            setState("AimTarget");
        }).withName(name + ".AimAtTarget");
    }

    /**
     * Track an {@link AimingSolver.Solution} — the Shoot-On-The-Fly path.
     * Holds position when the solution is infeasible.
     *
     * @param solution        supplier of the latest solve (call the solver in the lambda)
     * @param robotHeadingDeg robot heading supplier (degrees)
     */
    public Command track(Supplier<AimingSolverTOF.Solution> solution, DoubleSupplier robotHeadingDeg) {
        return track(solution, robotHeadingDeg, () -> 0.0);
    }

    /**
     * Full Shoot-On-The-Fly tracking for a chassis that may be <b>translating
     * and rotating</b> at once. The turret's robot-relative bearing is
     * {@code fieldAngle - heading}, so its rate is
     * {@code fieldBearingRate - yawRate}. Passing the live yaw rate keeps the
     * velocity feedforward exact while the robot spins; with a swerve this is
     * {@code () -> Math.toDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond)}.
     *
     * @param solution         supplier of the latest solve
     * @param robotHeadingDeg  robot heading supplier (degrees)
     * @param robotYawRateDps  robot yaw rate supplier (degrees/second)
     */
    public Command track(Supplier<AimingSolverTOF.Solution> solution,
                         DoubleSupplier robotHeadingDeg,
                         DoubleSupplier robotYawRateDps) {
        return run(() -> {
            AimingSolverTOF.Solution s = solution.get();
            if (s == null || !s.feasible()) {
                aimFieldAngleDegrees = Double.NaN;
                motor.setMotionMagicPosition(setpointDegrees / 360.0); // hold
                setState("TrackHold");
                return;
            }
            aimFieldAngleDegrees = s.turretFieldAngleDeg();
            // Exact velocity feedforward from the solver's analytic field-bearing
            // rate, converted to the robot-relative frame by removing the yaw
            // rate, so the turret leads a moving goal instead of lagging it.
            double ffRobotRelDps = s.turretFieldRateDps() - robotYawRateDps.getAsDouble();
            commandResolved(s.turretFieldAngleDeg() - robotHeadingDeg.getAsDouble(), ffRobotRelDps);
            setState("Track");
        }).withName(name + ".Track");
    }

    /**
     * Close the loop on a vision horizontal error instead of odometry.
     * Nudges the setpoint by the camera error each loop; when {@code hasTarget}
     * is false it holds the last setpoint.
     *
     * @param hasTarget true when the camera sees the target
     * @param errorDeg  horizontal angle to target (degrees). Positive should
     *                  drive the turret positive — flip with
     *                  {@link Config.Builder#visionInverted(boolean)} if not.
     */
    public Command aimWithVision(BooleanSupplier hasTarget, DoubleSupplier errorDeg) {
        return run(() -> {
            if (!hasTarget.getAsBoolean()) {
                motor.setMotionMagicPosition(setpointDegrees / 360.0);
                setState("VisionNoTarget");
                return;
            }
            double err = errorDeg.getAsDouble() * (config.visionInverted ? -1.0 : 1.0);
            aimFieldAngleDegrees = Double.NaN;
            commandResolved(getAngle() + err);
            setState("Vision");
        }).withName(name + ".AimWithVision");
    }

    /** Seed the encoder so the current physical position reads {@code angleDeg}. */
    public Command zero(double angleDeg) {
        return runOnce(() -> {
            motor.setEncoderPosition(angleDeg / 360.0);
            setpointDegrees = angleDeg;
            hasBeenZeroed = true;
            setState("Zeroed");
        }).withName(name + ".Zero");
    }

    /** Seed the encoder to 0°. */
    public Command zero() {
        return zero(0);
    }

    // ============================================================
    //                       INTERNALS
    // ============================================================

    @Override
    protected void stop() {
        motor.stop();
        setState("Stopped");
    }

    @Override
    protected CatalystMotor primaryMotorForSysId() {
        return motor;
    }

    @Override
    protected void updateTelemetry() {
        motor.updateTelemetry();
        tunableGains.checkAndApply(motor);

        inputs.angleDegrees = getAngle();
        inputs.angularVelocityDPS = getAngularVelocity();
        inputs.setpointDegrees = setpointDegrees;
        inputs.aimFieldAngleDegrees = aimFieldAngleDegrees;
        inputs.atSetpoint = atSetpoint();
        inputs.unwrapping = unwrapping;
        inputs.statorCurrentAmps = motor.getStatorCurrent();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent();
        inputs.appliedVolts = motor.getAppliedVoltage();
        inputs.temperatureC = motor.getTemperature();
        inputs.hasBeenZeroed = hasBeenZeroed;
        processInputs(inputs);

        log("AngleDegrees", inputs.angleDegrees);
        log("SetpointDegrees", inputs.setpointDegrees);
        log("AtSetpoint", inputs.atSetpoint);
        log("Unwrapping", inputs.unwrapping);

        HealthMonitor.getInstance().update();
    }

    // ============================================================
    //                        CONFIG
    // ============================================================

    public static class Config {
        final String name;
        final int motorCanId;
        final String canBus;
        final boolean inverted;
        final boolean brakeMode;
        final double gearRatio;
        final double minAngle;
        final double maxAngle;
        final double currentLimit;
        final double statorCurrentLimit;
        final double kP, kI, kD;
        final double kS, kV, kA;
        final double motionMagicCruiseVelocity;
        final double motionMagicAcceleration;
        final double motionMagicJerk;
        final double toleranceDegrees;
        final double maxTemperatureC;
        final double startingAngle;
        final boolean visionInverted;
        final int cancoderId;
        final double cancoderRotorToSensorRatio;
        final MotorType motorType;
        final double simMOI;

        private Config(Builder b) {
            this.name = b.name;
            this.motorCanId = b.motorCanId;
            this.canBus = b.canBus;
            this.inverted = b.inverted;
            this.brakeMode = b.brakeMode;
            this.gearRatio = b.gearRatio;
            this.minAngle = b.minAngle;
            this.maxAngle = b.maxAngle;
            this.currentLimit = b.currentLimit;
            this.statorCurrentLimit = b.statorCurrentLimit;
            this.kP = b.kP; this.kI = b.kI; this.kD = b.kD;
            this.kS = b.kS; this.kV = b.kV; this.kA = b.kA;
            this.motionMagicCruiseVelocity = b.motionMagicCruiseVelocity;
            this.motionMagicAcceleration = b.motionMagicAcceleration;
            this.motionMagicJerk = b.motionMagicJerk;
            this.toleranceDegrees = b.toleranceDegrees;
            this.maxTemperatureC = b.maxTemperatureC;
            this.startingAngle = b.startingAngle;
            this.visionInverted = b.visionInverted;
            this.cancoderId = b.cancoderId;
            this.cancoderRotorToSensorRatio = b.cancoderRotorToSensorRatio;
            this.motorType = b.motorType;
            this.simMOI = b.simMOI;
        }

        public static Builder builder() { return new Builder(); }

        public static class Builder {
            private String name = "Turret";
            private int motorCanId = 0;
            private String canBus = "";
            private boolean inverted = false;
            private boolean brakeMode = true;   // hold aim on disable
            private double gearRatio = 1.0;
            private double minAngle = -180;
            private double maxAngle = 180;
            private double currentLimit = 30;
            private double statorCurrentLimit = 40;
            private double kP = 0, kI = 0, kD = 0;
            private double kS = 0, kV = 0, kA = 0;
            private double motionMagicCruiseVelocity = 0;
            private double motionMagicAcceleration = 0;
            private double motionMagicJerk = 0;
            private double toleranceDegrees = 1.0;
            private double maxTemperatureC = 70;
            private double startingAngle = 0;
            private boolean visionInverted = false;
            private int cancoderId = -1;
            private double cancoderRotorToSensorRatio = 1.0;
            private MotorType motorType = MotorType.KRAKEN_X60;
            private double simMOI = 0.02;

            public Builder name(String name) { this.name = name; return this; }
            public Builder motor(int canId) { this.motorCanId = canId; return this; }
            public Builder canBus(String canBus) { this.canBus = canBus; return this; }
            public Builder inverted(boolean inverted) { this.inverted = inverted; return this; }
            public Builder brakeMode(boolean brake) { this.brakeMode = brake; return this; }

            /** Motor rotations per turret rotation. */
            public Builder gearRatio(double ratio) { this.gearRatio = ratio; return this; }

            /** Mechanical travel limits, in robot-relative degrees. Make this wider than ±180 for a turret with overlap. */
            public Builder range(double minDegrees, double maxDegrees) {
                this.minAngle = minDegrees;
                this.maxAngle = maxDegrees;
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

            /** Motion Magic profile in turret rotations (rps, rps^2, rps^3). */
            public Builder motionMagic(double cruiseVelocity, double acceleration, double jerk) {
                this.motionMagicCruiseVelocity = cruiseVelocity;
                this.motionMagicAcceleration = acceleration;
                this.motionMagicJerk = jerk;
                return this;
            }

            public Builder tolerance(double degrees) { this.toleranceDegrees = degrees; return this; }
            public Builder maxTemperature(double celsius) { this.maxTemperatureC = celsius; return this; }

            /** Angle (degrees) the turret is physically at on boot, when not using a CANcoder. */
            public Builder startingAngle(double degrees) { this.startingAngle = degrees; return this; }

            /** Flip the sign of the vision error in {@link #aimWithVision}. */
            public Builder visionInverted(boolean inverted) { this.visionInverted = inverted; return this; }

            /**
             * Fuse an absolute CANcoder so the turret knows its angle on boot
             * without homing. Requires Phoenix Pro.
             *
             * @param canId               CANcoder CAN id
             * @param rotorToSensorRatio  motor rotor rotations per CANcoder rotation
             */
            /** Motor model used for simulation physics (default Kraken X60). */
            public Builder motorType(MotorType type) { this.motorType = type; return this; }

            /** Rotational inertia of the turret about its axis, kg·m² (sim only, default 0.02). */
            public Builder simMOI(double kgm2) { this.simMOI = kgm2; return this; }

            public Builder cancoder(int canId, double rotorToSensorRatio) {
                this.cancoderId = canId;
                this.cancoderRotorToSensorRatio = rotorToSensorRatio;
                return this;
            }

            public Config build() {
                if (motorCanId == 0) {
                    throw new IllegalStateException("Turret motor CAN id must be set");
                }
                if (minAngle >= maxAngle) {
                    throw new IllegalStateException("Turret range min must be < max");
                }
                return new Config(this);
            }
        }
    }
}
