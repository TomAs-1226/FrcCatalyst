package frc.lib.catalyst.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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

    public SwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    @Override
    public void periodic() {
        Pose2d pose = getPose();
        posePub.set(pose);
        telemetryTable.getEntry("HeadingDeg").setDouble(pose.getRotation().getDegrees());
        ChassisSpeeds speeds = getChassisSpeeds();
        telemetryTable.getEntry("SpeedMPS").setDouble(
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
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
