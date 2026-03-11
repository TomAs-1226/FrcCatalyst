package frc.lib.catalyst.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.catalyst.util.SlewRateLimiter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Swerve drive subsystem wrapper for CTRE Phoenix 6 generated swerve code.
 *
 * <p>Teams generate their swerve project using CTRE Tuner X, which creates
 * {@code TunerConstants} and {@code CommandSwerveDrivetrain}. This class wraps
 * the generated drivetrain to provide:
 * <ul>
 *   <li>Simplified command factories for teleop drive, X-brake, heading lock</li>
 *   <li>Automatic PathPlanner configuration</li>
 *   <li>Vision pose estimation integration</li>
 *   <li>Automatic telemetry to NetworkTables</li>
 * </ul>
 *
 * <p>Example usage:
 * <pre>{@code
 * SwerveSubsystem drive = new SwerveSubsystem(
 *     TunerConstants.createDrivetrain(),
 *     4.5, // max speed m/s
 *     SwerveSubsystem.PathPlannerConfig.builder()
 *         .translationPID(5.0, 0.0, 0.0)
 *         .rotationPID(5.0, 0.0, 0.0)
 *         .build()
 * );
 * }</pre>
 */
public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrivetrain drivetrain;
    private final double maxSpeedMPS;
    private final double maxAngularRate;

    // Heading lock PID
    private final PIDController headingPID = new PIDController(5.0, 0, 0);
    private Rotation2d lockedHeading = null;

    // Pose exponential skew correction
    private boolean skewCorrectionEnabled = true;

    // Slew rate limiters for smooth acceleration
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter rotLimiter;

    // Snap-to-angle presets (e.g., 0, 90, 180, 270 for cardinal directions)
    private double[] snapAngles = null;
    private double snapTolerance = 15.0; // degrees

    // Slow mode
    private double speedMultiplier = 1.0;

    // Control requests (reused to avoid GC)
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    // Telemetry
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePub;

    /**
     * Create a SwerveSubsystem wrapping a CTRE-generated SwerveDrivetrain.
     *
     * @param drivetrain the CTRE SwerveDrivetrain (from TunerConstants.createDrivetrain())
     * @param maxSpeedMPS maximum robot speed in meters per second
     * @param pathPlannerConfig PathPlanner configuration (null to skip auto-config)
     */
    public SwerveSubsystem(SwerveDrivetrain drivetrain, double maxSpeedMPS,
                           PathPlannerConfig pathPlannerConfig) {
        this.drivetrain = drivetrain;
        this.maxSpeedMPS = maxSpeedMPS;
        this.maxAngularRate = maxSpeedMPS / 0.4;

        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(Math.toRadians(1.5));

        telemetryTable = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("Swerve");
        posePub = telemetryTable.getStructTopic("Pose", Pose2d.struct).publish();

        if (pathPlannerConfig != null) {
            configurePathPlanner(pathPlannerConfig);
        }
    }

    public SwerveSubsystem(SwerveDrivetrain drivetrain, double maxSpeedMPS) {
        this(drivetrain, maxSpeedMPS, null);
    }

    // --- PathPlanner ---

    private void configurePathPlanner(PathPlannerConfig config) {
        try {
            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getChassisSpeeds,
                    (speeds, feedforwards) -> driveRobotCentric(speeds),
                    new PPHolonomicDriveController(
                            new PIDConstants(config.translationKP, config.translationKI, config.translationKD),
                            new PIDConstants(config.rotationKP, config.rotationKI, config.rotationKD)),
                    RobotConfig.fromGUISettings(),
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                    },
                    this);
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(), false);
        }
    }

    // --- Pose ---

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return drivetrain.getState().Speeds;
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    // --- Drive Methods ---

    /** Drive field-centric with raw speeds (m/s and rad/s). */
    public void driveFieldCentric(double xSpeedMPS, double ySpeedMPS, double rotSpeedRadPerSec) {
        drivetrain.setControl(
                fieldCentricRequest
                        .withVelocityX(MetersPerSecond.of(xSpeedMPS))
                        .withVelocityY(MetersPerSecond.of(ySpeedMPS))
                        .withRotationalRate(RadiansPerSecond.of(rotSpeedRadPerSec)));
    }

    /** Drive robot-centric with raw speeds (m/s and rad/s). */
    public void driveRobotCentric(double xSpeedMPS, double ySpeedMPS, double rotSpeedRadPerSec) {
        drivetrain.setControl(
                robotCentricRequest
                        .withVelocityX(MetersPerSecond.of(xSpeedMPS))
                        .withVelocityY(MetersPerSecond.of(ySpeedMPS))
                        .withRotationalRate(RadiansPerSecond.of(rotSpeedRadPerSec)));
    }

    /** Drive robot-centric with a ChassisSpeeds object. */
    public void driveRobotCentric(ChassisSpeeds speeds) {
        driveRobotCentric(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /** Set X-brake (wheels pointed inward to resist pushing). */
    public void setBrake() {
        drivetrain.setControl(brakeRequest);
    }

    /** Add a vision measurement for pose estimation. */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds,
                                     edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> stdDevs) {
        drivetrain.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    }

    /** Add a vision measurement with default standard deviations. */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        drivetrain.addVisionMeasurement(visionPose, timestampSeconds);
    }

    // --- Command Factories ---

    /**
     * Field-centric drive command for teleop.
     * Inputs are -1 to 1 (joystick axes). Automatically scales to max speed.
     */
    public Command fieldCentricDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                     DoubleSupplier rotSupplier) {
        return run(() -> {
            double x = xSupplier.getAsDouble() * maxSpeedMPS;
            double y = ySupplier.getAsDouble() * maxSpeedMPS;
            double rot = rotSupplier.getAsDouble() * maxAngularRate;
            driveFieldCentric(x, y, rot);
        }).withName("Swerve.FieldCentric");
    }

    /** Field-centric drive with a deadband applied. */
    public Command fieldCentricDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                     DoubleSupplier rotSupplier, double deadband) {
        return run(() -> {
            double x = applyDeadband(xSupplier.getAsDouble(), deadband) * maxSpeedMPS;
            double y = applyDeadband(ySupplier.getAsDouble(), deadband) * maxSpeedMPS;
            double rot = applyDeadband(rotSupplier.getAsDouble(), deadband) * maxAngularRate;
            driveFieldCentric(x, y, rot);
        }).withName("Swerve.FieldCentric");
    }

    /** Robot-centric drive command for teleop. */
    public Command robotCentricDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                     DoubleSupplier rotSupplier) {
        return run(() -> {
            double x = xSupplier.getAsDouble() * maxSpeedMPS;
            double y = ySupplier.getAsDouble() * maxSpeedMPS;
            double rot = rotSupplier.getAsDouble() * maxAngularRate;
            driveRobotCentric(x, y, rot);
        }).withName("Swerve.RobotCentric");
    }

    /**
     * Field-centric drive with heading lock.
     * When the driver is not rotating (rot input below deadband), the robot
     * automatically holds its current heading using a PID controller.
     * When the driver rotates, the lock releases and updates on release.
     *
     * @param xSupplier X axis input (-1 to 1)
     * @param ySupplier Y axis input (-1 to 1)
     * @param rotSupplier rotation input (-1 to 1)
     * @param deadband deadband applied to all axes
     */
    public Command headingLockDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                     DoubleSupplier rotSupplier, double deadband) {
        return run(() -> {
            double x = applyDeadband(xSupplier.getAsDouble(), deadband) * maxSpeedMPS;
            double y = applyDeadband(ySupplier.getAsDouble(), deadband) * maxSpeedMPS;
            double rotInput = applyDeadband(rotSupplier.getAsDouble(), deadband);

            double rot;
            if (Math.abs(rotInput) > 0.0) {
                // Driver is actively rotating — pass through and unlock heading
                rot = rotInput * maxAngularRate;
                lockedHeading = null;
            } else {
                // Driver released rotation — lock to current heading
                if (lockedHeading == null) {
                    lockedHeading = getHeading();
                }
                rot = headingPID.calculate(
                        getHeading().getRadians(), lockedHeading.getRadians());
            }
            driveFieldCentric(x, y, rot);
        }).beforeStarting(() -> lockedHeading = null)
                .withName("Swerve.HeadingLock");
    }

    /**
     * Field-centric drive that locks to a specific heading.
     * The robot translates based on joystick input but always rotates
     * to face the target heading.
     *
     * @param xSupplier X axis input (-1 to 1)
     * @param ySupplier Y axis input (-1 to 1)
     * @param targetHeading the heading to lock to
     * @param deadband deadband for translation axes
     */
    public Command driveWithHeading(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                     Supplier<Rotation2d> targetHeading, double deadband) {
        return run(() -> {
            double x = applyDeadband(xSupplier.getAsDouble(), deadband) * maxSpeedMPS;
            double y = applyDeadband(ySupplier.getAsDouble(), deadband) * maxSpeedMPS;
            double rot = headingPID.calculate(
                    getHeading().getRadians(), targetHeading.get().getRadians());
            driveFieldCentric(x, y, rot);
        }).withName("Swerve.DriveWithHeading");
    }

    /**
     * Field-centric drive that always points toward a target on the field.
     * The robot translates normally but rotates to face the given field position.
     * Useful for aiming at a scoring target while driving.
     *
     * @param xSupplier X axis input (-1 to 1)
     * @param ySupplier Y axis input (-1 to 1)
     * @param targetPoint field position to point at (e.g., speaker location)
     * @param deadband deadband for translation axes
     */
    public Command pointAtTarget(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                  Supplier<Translation2d> targetPoint, double deadband) {
        return run(() -> {
            double x = applyDeadband(xSupplier.getAsDouble(), deadband) * maxSpeedMPS;
            double y = applyDeadband(ySupplier.getAsDouble(), deadband) * maxSpeedMPS;

            // Calculate angle from robot to target
            Translation2d robotPos = getPose().getTranslation();
            Translation2d toTarget = targetPoint.get().minus(robotPos);
            Rotation2d targetAngle = toTarget.getAngle();

            double rot = headingPID.calculate(
                    getHeading().getRadians(), targetAngle.getRadians());
            driveFieldCentric(x, y, rot);
        }).withName("Swerve.PointAtTarget");
    }

    /**
     * Set the heading lock PID gains.
     * Default is P=5.0, I=0, D=0 which works well for most robots.
     */
    public void setHeadingPIDGains(double kP, double kI, double kD) {
        headingPID.setPID(kP, kI, kD);
    }

    /**
     * Enable/disable pose exponential skew correction.
     * When enabled, corrects for swerve skew during combined translation + rotation
     * by rotating commanded velocities by -omega*dt/2. This is the approach used by
     * Team 1690 and documented in the swerve skew whitepaper.
     * Enabled by default.
     */
    public void setSkewCorrectionEnabled(boolean enabled) {
        this.skewCorrectionEnabled = enabled;
    }

    /**
     * Enable slew rate limiting for smoother acceleration.
     * Prevents sudden speed changes that can cause wheel slip or driver discomfort.
     *
     * @param translationRate max translation change rate in m/s per second
     * @param rotationRate max rotation change rate in rad/s per second
     */
    public void enableSlewRateLimiting(double translationRate, double rotationRate) {
        this.xLimiter = new SlewRateLimiter(translationRate);
        this.yLimiter = new SlewRateLimiter(translationRate);
        this.rotLimiter = new SlewRateLimiter(rotationRate);
    }

    /**
     * Enable asymmetric slew rate limiting.
     * Different rates for acceleration and deceleration — useful for
     * aggressive braking with gentle acceleration.
     *
     * @param accelRate acceleration rate (m/s per second)
     * @param decelRate deceleration rate (m/s per second, should be higher for snappy stops)
     * @param rotRate rotation rate (rad/s per second)
     */
    public void enableSlewRateLimiting(double accelRate, double decelRate, double rotRate) {
        this.xLimiter = new SlewRateLimiter(accelRate, decelRate);
        this.yLimiter = new SlewRateLimiter(accelRate, decelRate);
        this.rotLimiter = new SlewRateLimiter(rotRate);
    }

    /**
     * Set snap-to-angle presets for heading lock.
     * When the driver releases rotation, the robot snaps to the nearest preset angle.
     * Common for 254-style driving where drivers snap to cardinal directions.
     *
     * @param anglesDegrees angles to snap to (e.g., 0, 90, 180, 270)
     * @param toleranceDegrees how close to a snap angle to activate (default 15)
     */
    public void setSnapToAngles(double[] anglesDegrees, double toleranceDegrees) {
        this.snapAngles = anglesDegrees;
        this.snapTolerance = toleranceDegrees;
    }

    /** Set a speed multiplier for slow/turbo mode (0.0 to 1.0). */
    public void setSpeedMultiplier(double multiplier) {
        this.speedMultiplier = MathUtil.clamp(multiplier, 0.0, 1.0);
    }

    /**
     * Advanced field-centric drive with all features:
     * - Deadband
     * - Slew rate limiting (if enabled)
     * - Heading lock with snap-to-angle (if configured)
     * - Skew correction
     * - Speed multiplier
     *
     * This is the recommended default drive command for competition.
     */
    public Command advancedDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                  DoubleSupplier rotSupplier, double deadband) {
        return run(() -> {
            // Apply deadband and scaling
            double rawX = applyDeadband(xSupplier.getAsDouble(), deadband);
            double rawY = applyDeadband(ySupplier.getAsDouble(), deadband);
            double rawRot = applyDeadband(rotSupplier.getAsDouble(), deadband);

            double x = rawX * maxSpeedMPS * speedMultiplier;
            double y = rawY * maxSpeedMPS * speedMultiplier;

            // Apply slew rate limiting
            if (xLimiter != null) {
                x = xLimiter.calculate(x);
                y = yLimiter.calculate(y);
            }

            double rot;
            if (Math.abs(rawRot) > 0.0) {
                // Driver actively rotating
                rot = rawRot * maxAngularRate * speedMultiplier;
                if (rotLimiter != null) rot = rotLimiter.calculate(rot);
                lockedHeading = null;
            } else {
                // Auto-lock heading
                if (lockedHeading == null) {
                    Rotation2d currentHeading = getHeading();
                    // Snap to nearest preset angle if configured
                    if (snapAngles != null) {
                        double bestAngle = currentHeading.getDegrees();
                        double minDiff = Double.MAX_VALUE;
                        for (double snapAngle : snapAngles) {
                            double diff = Math.abs(normalizeAngle(currentHeading.getDegrees() - snapAngle));
                            if (diff < minDiff && diff < snapTolerance) {
                                minDiff = diff;
                                bestAngle = snapAngle;
                            }
                        }
                        lockedHeading = Rotation2d.fromDegrees(bestAngle);
                    } else {
                        lockedHeading = currentHeading;
                    }
                }
                rot = headingPID.calculate(
                        getHeading().getRadians(), lockedHeading.getRadians());
                if (rotLimiter != null) rotLimiter.calculate(rot); // keep limiter in sync
            }

            // Skew correction via pose exponential discretization
            if (skewCorrectionEnabled && rot != 0) {
                double dt = 0.02; // 20ms loop
                double halfAngle = rot * dt / 2.0;
                double cos = Math.cos(halfAngle);
                double sin = Math.sin(halfAngle);
                double correctedX = x * cos - y * sin;
                double correctedY = x * sin + y * cos;
                x = correctedX;
                y = correctedY;
            }

            driveFieldCentric(x, y, rot);
        }).beforeStarting(() -> {
            lockedHeading = null;
            if (xLimiter != null) {
                xLimiter.reset(0);
                yLimiter.reset(0);
                rotLimiter.reset(0);
            }
        }).withName("Swerve.AdvancedDrive");
    }

    /**
     * Command to toggle slow mode while held.
     * @param slowFactor speed multiplier when slow (e.g., 0.3 for 30% speed)
     */
    public Command slowModeWhileHeld(double slowFactor) {
        return startEnd(
                () -> speedMultiplier = slowFactor,
                () -> speedMultiplier = 1.0
        ).withName("Swerve.SlowMode");
    }

    /**
     * Auto-align drive command. Drives normally for translation but
     * automatically aligns rotation to face a target pose.
     * Uses the target's rotation, not the angle toward it.
     * Ideal for pre-aligning for scoring positions.
     *
     * @param xSupplier X axis input
     * @param ySupplier Y axis input
     * @param targetPose target pose (robot aligns to match its rotation)
     * @param deadband input deadband
     */
    public Command autoAlignDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                   Supplier<Pose2d> targetPose, double deadband) {
        return run(() -> {
            double x = applyDeadband(xSupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double y = applyDeadband(ySupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double rot = headingPID.calculate(
                    getHeading().getRadians(), targetPose.get().getRotation().getRadians());
            driveFieldCentric(x, y, rot);
        }).withName("Swerve.AutoAlign");
    }

    /**
     * Drive to a specific pose autonomously.
     * Uses PID on X, Y, and heading simultaneously.
     * Simple approach for short-distance precision alignment.
     *
     * @param targetPose the target field pose
     * @param translationKP proportional gain for XY (try 2.0-5.0)
     * @param toleranceMeters position tolerance for "arrived"
     */
    public Command driveToPose(Supplier<Pose2d> targetPose, double translationKP,
                                double toleranceMeters) {
        PIDController xController = new PIDController(translationKP, 0, 0);
        PIDController yController = new PIDController(translationKP, 0, 0);

        return run(() -> {
            Pose2d target = targetPose.get();
            Pose2d current = getPose();

            double xSpeed = xController.calculate(current.getX(), target.getX());
            double ySpeed = yController.calculate(current.getY(), target.getY());
            double rotSpeed = headingPID.calculate(
                    current.getRotation().getRadians(), target.getRotation().getRadians());

            // Clamp speeds
            double maxTranslation = maxSpeedMPS * 0.5;
            xSpeed = MathUtil.clamp(xSpeed, -maxTranslation, maxTranslation);
            ySpeed = MathUtil.clamp(ySpeed, -maxTranslation, maxTranslation);

            driveFieldCentric(xSpeed, ySpeed, rotSpeed);
        }).until(() -> {
            Pose2d current = getPose();
            Pose2d target = targetPose.get();
            return current.getTranslation().getDistance(target.getTranslation()) < toleranceMeters
                    && Math.abs(normalizeAngle(
                    current.getRotation().getDegrees() - target.getRotation().getDegrees())) < 3.0;
        }).withName("Swerve.DriveToPose");
    }

    /** X-brake command (lock wheels). */
    public Command xBrake() {
        return runOnce(this::setBrake).withName("Swerve.XBrake");
    }

    /** Reset heading command (zero the gyro). */
    public Command resetHeading() {
        return runOnce(() -> {
            resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
        }).withName("Swerve.ResetHeading");
    }

    /** Reset pose command. */
    public Command resetPoseCommand(Supplier<Pose2d> poseSupplier) {
        return runOnce(() -> resetPose(poseSupplier.get())).withName("Swerve.ResetPose");
    }

    // --- Internals ---

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0;
        return (value - Math.copySign(deadband, value)) / (1.0 - deadband);
    }

    private static double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) degrees -= 360;
        if (degrees < -180) degrees += 360;
        return degrees;
    }

    public SwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    /** Get max speed in m/s. */
    public double getMaxSpeed() {
        return maxSpeedMPS;
    }

    /** Get current speed magnitude in m/s. */
    public double getCurrentSpeed() {
        ChassisSpeeds speeds = getChassisSpeeds();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    @Override
    public void periodic() {
        Pose2d pose = getPose();
        posePub.set(pose);
        telemetryTable.getEntry("HeadingDeg").setDouble(pose.getRotation().getDegrees());
        ChassisSpeeds speeds = getChassisSpeeds();
        double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        telemetryTable.getEntry("SpeedMPS").setDouble(speed);
        telemetryTable.getEntry("OmegaRadPerSec").setDouble(speeds.omegaRadiansPerSecond);
        telemetryTable.getEntry("SpeedMultiplier").setDouble(speedMultiplier);
    }

    // ===========================================
    //          PATHPLANNER CONFIG
    // ===========================================

    public static class PathPlannerConfig {
        final double translationKP, translationKI, translationKD;
        final double rotationKP, rotationKI, rotationKD;

        private PathPlannerConfig(Builder b) {
            this.translationKP = b.translationKP;
            this.translationKI = b.translationKI;
            this.translationKD = b.translationKD;
            this.rotationKP = b.rotationKP;
            this.rotationKI = b.rotationKI;
            this.rotationKD = b.rotationKD;
        }

        public static Builder builder() {
            return new Builder();
        }

        public static class Builder {
            private double translationKP = 5.0, translationKI = 0, translationKD = 0;
            private double rotationKP = 5.0, rotationKI = 0, rotationKD = 0;

            public Builder translationPID(double kP, double kI, double kD) {
                this.translationKP = kP; this.translationKI = kI; this.translationKD = kD;
                return this;
            }

            public Builder rotationPID(double kP, double kI, double kD) {
                this.rotationKP = kP; this.rotationKI = kI; this.rotationKD = kD;
                return this;
            }

            public PathPlannerConfig build() {
                return new PathPlannerConfig(this);
            }
        }
    }
}
