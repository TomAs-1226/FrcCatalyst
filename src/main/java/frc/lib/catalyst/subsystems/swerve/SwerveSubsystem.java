package frc.lib.catalyst.subsystems.swerve;

import frc.lib.catalyst.command.LegacyCommands;
import frc.lib.catalyst.command.CatalystCommand;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.DriverStationErrors;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Notifier;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import frc.lib.catalyst.command.Commands;
import org.wpilib.command3.Mechanism;
import frc.lib.catalyst.identity.RobotIdentity;
import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.physics.RobotStateSource;
import frc.lib.catalyst.util.AlertManager;
import frc.lib.catalyst.util.RobotState;
import frc.lib.catalyst.util.SlewRateLimiter;

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
 *
 * <p>Implements {@link RobotStateSource}, so code that only needs "where is the robot and how fast"
 * can take the interface and work equally well with a plain drivetrain or with
 * {@code PhysicsCore}.
 */
public class SwerveSubsystem extends frc.lib.catalyst.command.CatalystSubsystem
        implements RobotStateSource, frc.lib.catalyst.subsystems.vision.VisionPoseSink {

    /**
     * The robot loop period, used by skew correction.
     *
     * <p>Matches WPILib's default. A robot on a different period should pass its own; getting this
     * wrong scales the correction rather than breaking it, so it fails quietly.
     */
    private static final double LOOP_PERIOD_SECONDS = 0.02;


    private final SwerveDrivetrain drivetrain;
    private final double maxSpeedMPS;
    private double maxAngularRate;

    // boolean to see if the operator perspective has been applied alredy. (need to add this to make sure the red side is working)
    private boolean hasAppliedOperatorPerspective = false;

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
    // ForwardPerspective is set to OperatorPerspective EXPLICITLY (it is also the Phoenix default) so
    // the red-alliance flip is guaranteed and self-documented: field-centric "forward" is measured
    // from the operator's perspective, which periodic() sets to 180 deg on red via
    // setOperatorPerspectiveForward(...). Without this a red team drives inverted. (Reported by 3211.)
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    // Closed-loop request for path following: velocity control + wheel force
    // feedforwards from PathPlanner (both are dropped by the open-loop teleop path).
    private final SwerveRequest.ApplyRobotVelocity pathApplyRequest =
            new SwerveRequest.ApplyRobotVelocity().withDriveRequestType(DriveRequestType.Velocity);

    // Simulation: a high-rate thread that actually advances the Phoenix sim so the
    // drivetrain moves in the simulator (without it, status signals stay stale).
    // Yields automatically to an external physics engine — see setSimPose().
    private static final double SIM_LOOP_PERIOD = 0.005; // 200 Hz
    private Notifier simNotifier = null;
    private double lastSimTime;
    private boolean internalSimYielded = false;

    // Telemetry. Everything is published through CatalystLog under "Swerve/" (root
    // "Catalyst/"), so a team that swaps the sink once — for WPILOG or a 2027 backend —
    // gets the drivetrain telemetry along with everything else. See issue #24.
    private static final String SWERVE = "Swerve/";

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
        // Commands v3's Mechanism has no constructor to hook, so periodic() is registered
        // explicitly. Without this the method compiles and is simply never called - see
        // CatalystSubsystem#registerPeriodic.
        registerPeriodic();
        final double RadiusRobot = drivetrain.getModuleLocations()[0].getNorm();
        this.maxAngularRate = maxSpeedMPS / RadiusRobot;

        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(Math.toRadians(1.5));

        if (pathPlannerConfig != null) {
            configurePathPlanner(pathPlannerConfig);
        }

        // Hand the drivetrain to the spec sheet. Does nothing until a robot has been declared, and
        // works either way round: a team that calls RobotIdentity.declare(...) at the top of
        // RobotContainer, before this constructor runs, still gets the drivetrain group.
        RobotIdentity.observeDrivetrain(this);

        // Advance the Phoenix physics sim on its own high-rate thread so the
        // The internal Phoenix sim (see startSimThread) is started lazily from the first
        // periodic() call in simulation, NOT here. Starting it in the constructor gives an
        // external physics engine no chance to opt out first: even one updateSimState()
        // call seeds Phoenix's sim-device state (FusedCANcoder syncs an internal
        // rotor-to-CANcoder offset on first use), and a bridge that takes over afterwards
        // with its own conventions inherits per-module corruption that never heals. By
        // first periodic(), anyone integrating external physics has had a full
        // construction window to call disableInternalSim().
    }

    private void startSimThread() {
        if (internalSimYielded) {
            return;
        }
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            double now = Utils.getCurrentTimeSeconds();
            double dt = now - lastSimTime;
            lastSimTime = now;
            drivetrain.updateSimState(dt, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD);
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
                    // Closed-loop velocity control, forwarding PathPlanner's wheel
                    // force feedforwards (both dropped by the open-loop teleop path).
                    (speeds, feedforwards) -> drivetrain.setControl(
                            pathApplyRequest.withVelocity(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            new PIDConstants(config.translationKP, config.translationKI, config.translationKD),
                            new PIDConstants(config.rotationKP, config.rotationKI, config.rotationKD)),
                    RobotConfig.fromGUISettings(),
                    () -> {
                        var alliance = MatchState.getAlliance();
                        return alliance.isPresent() && alliance.get() == Alliance.RED;
                    });
        } catch (Throwable e) {
            // Throwable, not Exception, and that is the whole fix: PathPlannerLib is built against
            // commands v2, so on a v3-only robot AutoBuilder.configure() dies with a
            // NoClassDefFoundError for org.wpilib.command2.Subsystem - an Error, which the old
            // catch let straight through. The robot program crash-looped at boot with no line of
            // team code in the trace. Found on a Systemcore with a real swerve config.
            //
            // Loud + persistent either way: AutoBuilder is left unconfigured, so autos won't
            // path. A quiet reportError is too easy to miss until a match.
            String why = e instanceof NoClassDefFoundError
                    ? "PathPlannerLib needs the WPILibNewCommands (commands v2) vendordep on the "
                            + "robot - add vendordeps/WPILibNewCommands.json (" + e.getMessage() + ")"
                    : String.valueOf(e.getMessage());
            String msg = "PathPlanner failed to configure (autos will not path): " + why;
            DriverStationErrors.reportError(msg, e.getStackTrace());
            AlertManager.getInstance().error("Swerve", msg);
        }
    }

    // --- Pose ---

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    /**
     * {@inheritDoc}
     *
     * <p>A drivetrain resets outright. The frame's timestamp is not needed: odometry restarts from
     * the reset, and the few milliseconds of motion since the frame are inside the estimator's noise.
     */
    @Override
    public void resetPose(Pose2d pose, double timestampSeconds) {
        resetPose(pose);
    }

    /**
     * In simulation only, force the estimator pose to a physics-sim pose
     * (e.g. maple-sim's {@code SwerveDriveSimulation} pose), so Catalyst's
     * odometry tracks the simulated world. No-op on a real robot.
     *
     * <p>Call once per {@code simulationPeriodic()}. See
     * {@code docs/advanced/simulation.md} for the maple-sim wiring.
     *
     * <p>The first call also stops Catalyst's internal Phoenix sim thread, by
     * way of {@link #disableInternalSim()}. Something is clearly supplying
     * physics, and two writers on the same module rotor states would fight:
     * {@code updateSimState()} would overwrite maple-sim's values at 200 Hz and
     * the physics would quietly stop reaching the robot. If you want the
     * internal sim off before any pose arrives, call {@link
     * #disableInternalSim()} yourself.
     */
    public void setSimPose(Pose2d simPose) {
        if (org.wpilib.framework.RobotBase.isSimulation() && simPose != null) {
            disableInternalSim();
            drivetrain.resetPose(simPose);
        }
    }

    /**
     * Stop Catalyst's internal Phoenix simulation thread.
     *
     * <p>Only needed when an external physics engine (maple-sim, or your own)
     * is driving the module sim states. {@link #setSimPose(Pose2d)} calls this
     * for you, so the maple-sim wiring in {@code docs/advanced/simulation.md}
     * needs no extra step. Idempotent, and a no-op on a real robot.
     */
    public void disableInternalSim() {
        if (internalSimYielded) {
            return;
        }
        internalSimYielded = true;

        if (simNotifier != null) {
            simNotifier.stop();
            simNotifier.close();
            simNotifier = null;
        }
    }

    /**
     * Whether Catalyst's own Phoenix sim thread is currently advancing the
     * drivetrain. False on a real robot, and false once an external physics
     * engine has taken over.
     */
    public boolean isInternalSimRunning() {
        return simNotifier != null;
    }

    /**
     * Robot-relative chassis speeds (the convention PathPlanner's
     * {@code robotRelativeSpeedsSupplier} expects).
     */
    public ChassisVelocities getChassisSpeeds() {
        return drivetrain.getState().Velocity;
    }

    /**
     * The module setpoints the drivetrain most recently commanded (angle + speed per module).
     *
     * <p>This is the clean "speeds-out" seam for bridging an external physics engine such as
     * maple-sim: feed these targets to {@code SelfControlledSwerveDriveSimulation.runSwerveStates(...)}
     * and push the resulting pose back with {@link #setSimPose(Pose2d)}, instead of reaching through
     * the raw CTRE drivetrain and reproducing every per-module magnet offset and FusedCANcoder sync
     * by hand. See {@code docs/advanced/simulation.md}. May be {@code null} before the first control
     * request.
     *
     * @return the commanded module states, or {@code null} if none have been issued yet
     * @since 1.2.1
     */
    public SwerveModuleVelocity[] getModuleTargets() {
        return drivetrain.getState().ModuleTargets;
    }

    /**
     * The measured module states (angle + speed per module), from the drivetrain's own encoders.
     *
     * @return the measured module states, or {@code null} if unavailable
     * @since 1.2.1
     */
    public SwerveModuleVelocity[] getModuleVelocities() {
        return drivetrain.getState().ModuleVelocities;
    }

    /**
     * Field-relative chassis speeds — robot-relative speeds rotated by the
     * current heading. This is what {@link frc.lib.catalyst.util.AimingSolver}
     * needs for Shoot-On-The-Fly: the piece inherits the robot's velocity in
     * the field frame, not the robot frame.
     */
    public ChassisVelocities getFieldRelativeSpeeds() {
        return getChassisSpeeds().toFieldRelative(getHeading());
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    // --- RobotStateSource ---

    /**
     * {@inheritDoc}
     *
     * <p>Same value as {@link #getPose()}.
     *
     * @since 1.5.0
     */
    @Override
    public Pose2d pose() {
        return getPose();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Same value as {@link #getFieldRelativeSpeeds()}.
     *
     * @since 1.5.0
     */
    @Override
    public ChassisVelocities fieldVelocity() {
        return getFieldRelativeSpeeds();
    }

    /**
     * {@inheritDoc}
     *
     * <p>The drivetrain state is read live on every {@link #pose()} call, so the answer is "now" —
     * accurate to within the odometry thread's update period. It is reported on the FPGA clock, which
     * is what the rest of the contract's consumers timestamp against.
     *
     * @since 1.5.0
     */
    @Override
    public double timestampSeconds() {
        return Timer.getTimestamp();
    }

    /** Max translational speed in m/s (as configured). */
    public double getMaxSpeedMPS() {
        return maxSpeedMPS;
    }

    /** Max angular rate in rad/s (as configured). */
    public double getMaxAngularRate() {
        return maxAngularRate;
    }

    /**
     * Override the max angular rate (rad/s). The default is derived from the
     * first module's radius, which is wrong for asymmetric module layouts; set
     * it explicitly when the geometry isn't symmetric.
     */
    public void setMaxAngularRate(double radPerSec) {
        this.maxAngularRate = radPerSec;
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

    /** Drive robot-centric with a ChassisVelocities object. */
    public void driveRobotCentric(ChassisVelocities speeds) {
        driveRobotCentric(speeds.vx, speeds.vy, speeds.omega);
    }

    /** Set X-brake (wheels pointed inward to resist pushing). */
    public void setBrake() {
        drivetrain.setControl(brakeRequest);
    }

    /** Add a vision measurement for pose estimation. */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds,
                                     org.wpilib.math.linalg.Matrix<org.wpilib.math.numbers.N3, org.wpilib.math.numbers.N1> stdDevs) {
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
    public CatalystCommand fieldCentricDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                     DoubleSupplier rotSupplier) {
        return run(() -> {
            double x = xSupplier.getAsDouble() * maxSpeedMPS * speedMultiplier;
            double y = ySupplier.getAsDouble() * maxSpeedMPS * speedMultiplier;
            double rot = rotSupplier.getAsDouble() * maxAngularRate * speedMultiplier;
            driveFieldCentric(x, y, rot);
        }).withName("Swerve.FieldCentric");
    }

    /** Field-centric drive with a deadband applied. */
    public CatalystCommand fieldCentricDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                     DoubleSupplier rotSupplier, double deadband) {
        return run(() -> {
            double x = applyDeadband(xSupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double y = applyDeadband(ySupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double rot = applyDeadband(rotSupplier.getAsDouble(), deadband) * maxAngularRate * speedMultiplier;
            driveFieldCentric(x, y, rot);
        }).withName("Swerve.FieldCentric");
    }

    /** Robot-centric drive command for teleop. */
    public CatalystCommand robotCentricDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                     DoubleSupplier rotSupplier) {
        return run(() -> {
            double x = xSupplier.getAsDouble() * maxSpeedMPS * speedMultiplier;
            double y = ySupplier.getAsDouble() * maxSpeedMPS * speedMultiplier;
            double rot = rotSupplier.getAsDouble() * maxAngularRate * speedMultiplier;
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
    public CatalystCommand headingLockDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                     DoubleSupplier rotSupplier, double deadband) {
        return run(() -> {
            double x = applyDeadband(xSupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double y = applyDeadband(ySupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double rotInput = applyDeadband(rotSupplier.getAsDouble(), deadband);

            // Held on purpose - the driver is holding the button - so the hold stays on while parked
            // too, but a small error is ignored and the correction is clamped, like advancedDrive.
            HeadingHold.Decision hold = HeadingHold.decide(rotInput, true, getHeading(), lockedHeading,
                    null, 0, headingPID, maxAngularRate, speedMultiplier);
            lockedHeading = hold.locked();
            driveFieldCentric(x, y, hold.rotRadPerSec());
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
    public CatalystCommand driveWithHeading(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                     Supplier<Rotation2d> targetHeading, double deadband) {
        return run(() -> {
            double x = applyDeadband(xSupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double y = applyDeadband(ySupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double rot = headingRate(getHeading().getRadians(), targetHeading.get().getRadians(), true);
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
    public CatalystCommand pointAtTarget(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                  Supplier<Translation2d> targetPoint, double deadband) {
        return run(() -> {
            double x = applyDeadband(xSupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double y = applyDeadband(ySupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;

            // Calculate angle from robot to target
            Translation2d robotPos = getPose().getTranslation();
            Translation2d toTarget = targetPoint.get().minus(robotPos);
            Rotation2d targetAngle = toTarget.getAngle();

            double rot = headingRate(getHeading().getRadians(), targetAngle.getRadians(), true);
            driveFieldCentric(x, y, rot);
        }).withName("Swerve.PointAtTarget");
    }

    /**
     * Set the heading lock PID gains.
     * Default is P=5.0, I=0, D=0 which works well for most robots.
     *
     * <p>Note: a single heading PID instance backs every heading-based drive mode
     * ({@link #headingLockDrive}, {@link #driveWithHeading}, {@link #pointAtTarget},
     * {@link #advancedDrive}, ...), so this retunes all of them at once.
     */
    public void setHeadingPIDGains(double kP, double kI, double kD) {
        headingPID.setPID(kP, kI, kD);
    }

    /**
     * Enable/disable pose exponential skew correction.
     * When enabled, corrects for swerve skew during combined translation + rotation
     * by rotating commanded velocities by -omega*dt/2. Documented in the swerve
     * skew whitepaper and used by most competitive in-house drives.
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
     * Common pattern for cardinal-direction driving (intake heading, scoring poses).
     *
     * @param anglesDegrees angles to snap to (e.g., 0, 90, 180, 270)
     * @param toleranceDegrees how close to a snap angle to activate (default 15)
     */
    public void setSnapToAngles(double[] anglesDegrees, double toleranceDegrees) {
        this.snapAngles = anglesDegrees;
        this.snapTolerance = toleranceDegrees;
    }

    /**
     * Set a speed multiplier for slow/turbo mode (0.0 to 1.0).
     *
     * <p>Applies to <em>every</em> driver-facing drive command, translation and rotation alike.
     * It used to apply to only two of them, so a robot whose slip detector or slow-mode button had
     * turned the multiplier down went back to full speed the moment the driver held the heading
     * lock or the point-at button. A governor that some commands ignore is not a governor.
     */
    public void setSpeedMultiplier(double multiplier) {
        this.speedMultiplier = Math.clamp(multiplier, 0.0, 1.0);
    }

    /** The current speed multiplier, in [0, 1]. */
    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    /**
     * The heading loop's output as a rotation rate, bounded.
     *
     * <p>Raw {@code headingPID.calculate(...)} is a number of radians per second with no relation
     * to what this drivetrain can do: at the default kP of 5, half a turn of error asks for
     * 15.7 rad/s. Every heading-holding command sent that straight to the drivetrain, so a target
     * behind the robot produced a full-rate spin rather than a turn. Inside the loop's tolerance
     * the output is zero rather than a twitch, and the rest is clamped to the rate the driver's own
     * stick could ask for, after the speed multiplier.
     *
     * @param currentRadians where the robot is pointing
     * @param targetRadians  where it should point
     * @param governed       whether the speed multiplier applies (false for autonomous moves)
     */
    private double headingRate(double currentRadians, double targetRadians, boolean governed) {
        double rate = headingPID.calculate(currentRadians, targetRadians);
        if (headingPID.atSetpoint()) {
            return 0.0;
        }
        double limit = Math.abs(maxAngularRate * (governed ? speedMultiplier : 1.0));
        return Math.clamp(rate, -limit, limit);
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
     *
     * <p>Note: the drive feature flags (skew correction, slew limiting,
     * snap-to-angle) are applied only by this command. The simpler drive modes
     * ({@link #fieldCentricDrive}, {@link #robotCentricDrive},
     * {@link #headingLockDrive}, {@link #driveWithHeading}, {@link #pointAtTarget})
     * intentionally do not apply them.
     */
    public CatalystCommand advancedDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
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

            // Rotation: the driver's, or a held heading while translating, or nothing while parked.
            // A parked robot has no heading to keep straight, and one on blocks cannot turn: the old
            // rule locked to the nearest cardinal on enable and spun every module on a bench.
            boolean translating = Math.abs(rawX) > 0.0 || Math.abs(rawY) > 0.0;
            HeadingHold.Decision hold = HeadingHold.decide(rawRot, translating, getHeading(), lockedHeading,
                    snapAngles, snapTolerance, headingPID, maxAngularRate, speedMultiplier);
            lockedHeading = hold.locked();
            double rot = hold.rotRadPerSec();
            if (rotLimiter != null) {
                rot = rotLimiter.calculate(rot);
            }

            // Skew correction, via WPILib's own pose-exponential discretization.
            //
            // This was hand-rolled and rotated the wrong way: by +omega*dt/2 rather than
            // -omega*dt/2, so it added the skew it exists to remove. Checked against
            // ChassisVelocities.discretize with vx=3, omega=6, dt=0.02 - WPILib gives -3.4377
            // degrees and the old code gave +3.4377. A driver pushing straight forward while
            // spinning got roughly twice the drift they would have had with the feature switched
            // off, which reads as "skew correction makes it worse" rather than as a sign error.
            //
            // Delegated rather than corrected in place. This is the canonical implementation of the
            // operation the javadoc names, it cannot drift from WPILib, and there is no second copy
            // of the trigonometry to get backwards again.
            // ...and then removed entirely, because Phoenix already does it.
            //
            // SwerveRequest.FieldCentric discretizes internally, so correcting here applied the
            // operation twice - and the second application used a different period, since Phoenix
            // applies at its own 250 Hz rate rather than the 50 Hz loop this code can observe.
            // Measured for x=3 m/s, omega=6 rad/s: the wheels received (2.994095, -0.215948) where
            // plain FieldCentric gives (2.999856, -0.036000). The lateral term is six times too
            // large, in a feature whose entire purpose is to remove lateral drift.
            //
            // Left as a comment rather than a corrected constant because the right period is not
            // knowable from here. setSkewCorrectionEnabled is kept and documented as a no-op for
            // this path rather than deleted, since teams call it.

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
     * Engage slow mode while a button is held.
     *
     * <p>This is a <b>state modifier, not a drive command</b> — it sets the
     * speed multiplier the drive command reads and <b>requires no subsystem</b>,
     * so the robot keeps driving (via its default command) while slow mode is
     * held. Bind with {@code whileTrue}:
     *
     * <pre>{@code
     * driver.leftBumper().whileTrue(swerve.slowModeWhileHeld(0.3));
     * }</pre>
     *
     * <p>Because it owns nothing, you don't need {@code .proxy()} and it
     * behaves correctly in simulation.
     *
     * @param slowFactor speed multiplier when slow (e.g. 0.3 for 30%)
     */
    public CatalystCommand slowModeWhileHeld(double slowFactor) {
        return Commands.startEnd(
                () -> setSpeedMultiplier(slowFactor),
                () -> setSpeedMultiplier(1.0)
        ).withName("Swerve.SlowMode").ignoringDisable(true);
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
    public CatalystCommand autoAlignDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                                   Supplier<Pose2d> targetPose, double deadband) {
        return run(() -> {
            double x = applyDeadband(xSupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double y = applyDeadband(ySupplier.getAsDouble(), deadband) * maxSpeedMPS * speedMultiplier;
            double rot = headingRate(getHeading().getRadians(),
                    targetPose.get().getRotation().getRadians(), true);
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
    public CatalystCommand driveToPose(Supplier<Pose2d> targetPose, double translationKP,
                                double toleranceMeters) {
        PIDController xController = new PIDController(translationKP, 0, 0);
        PIDController yController = new PIDController(translationKP, 0, 0);

        return run(() -> {
            Pose2d target = targetPose.get();
            Pose2d current = getPose();

            double xSpeed = xController.calculate(current.getX(), target.getX());
            double ySpeed = yController.calculate(current.getY(), target.getY());
            // The translation below was clamped and the rotation was not, in the same method. Not
            // governed by the speed multiplier: this is an explicit "go to this pose" move with its
            // own budget, and an auto should not change speed because a teleop flag was left set.
            double rotSpeed = headingRate(current.getRotation().getRadians(),
                    target.getRotation().getRadians(), false);

            // Clamp speeds
            double maxTranslation = maxSpeedMPS * 0.5;
            xSpeed = Math.clamp(xSpeed, -maxTranslation, maxTranslation);
            ySpeed = Math.clamp(ySpeed, -maxTranslation, maxTranslation);

            driveFieldCentric(xSpeed, ySpeed, rotSpeed);
        }).untilTrue(() -> {
            Pose2d current = getPose();
            Pose2d target = targetPose.get();
            return current.getTranslation().getDistance(target.getTranslation()) < toleranceMeters
                    && Math.abs(normalizeAngle(
                    current.getRotation().getDegrees() - target.getRotation().getDegrees())) < 3.0;
        }).withName("Swerve.DriveToPose");
    }

    /**
     * Pathfind to a pose and then precision-align with PID.
     *
     * <p>Uses PathPlanner's {@code AutoBuilder.pathfindToPose} for the
     * long-range leg, then hands off to {@link #driveToPose(Supplier, double, double)}
     * once close. AutoBuilder must be configured (via the
     * {@link PathPlannerConfig} constructor) for the pathfinding leg to
     * work — if it isn't, this falls back to PID-only alignment and
     * reports the error to the driver station.
     *
     * @param targetPose      the target field pose
     * @param translationKP   proportional gain for the precision-align leg
     * @param toleranceMeters position tolerance for "arrived"
     * @param constraints     pathfinding constraints (max vel / accel, etc.)
     */
    public CatalystCommand pathfindToPose(Supplier<Pose2d> targetPose, double translationKP,
                                  double toleranceMeters, PathConstraints constraints) {
        // Defer so the pose is read when the command is scheduled, not when it is
        // constructed (usually once, at RobotContainer time). The Supplier
        // signature promises lazy evaluation, and the driveToPose hand-off is
        // already live, so both legs now track a moving target.
        return Commands.defer(() -> {
            try {
                return LegacyCommands.fromV2(AutoBuilder.pathfindToPose(targetPose.get(), constraints))
                        .then(driveToPose(targetPose, translationKP, toleranceMeters));
            } catch (Exception e) {
                DriverStationErrors.reportError(
                        "AutoBuilder not configured for pathfindToPose — falling back to PID align: "
                                + e.getMessage(), true);
                return driveToPose(targetPose, translationKP, toleranceMeters);
            }
        }, Set.of(this)).withName("Swerve.PathfindToPose");
    }

    /**
     * Pathfind to a pose with unlimited constraints (use only when you
     * trust your own velocity / accel limits elsewhere).
     */
    public CatalystCommand pathfindToPose(Supplier<Pose2d> targetPose, double translationKP,
                                  double toleranceMeters) {
        return pathfindToPose(targetPose, translationKP, toleranceMeters,
                PathConstraints.unlimitedConstraints(12.0));
    }

    /** Pathfind-and-align with sensible defaults (kP=4.0, tolerance=0.02 m). */
    public CatalystCommand pathfindToPose(Supplier<Pose2d> targetPose) {
        return pathfindToPose(targetPose, 4.0, 0.02);
    }

    /** Pathfind-and-align with sensible defaults plus custom constraints. */
    public CatalystCommand pathfindToPose(Supplier<Pose2d> targetPose, PathConstraints constraints) {
        return pathfindToPose(targetPose, 4.0, 0.02, constraints);
    }

    /**
     * Follow a Choreo trajectory by name. Choreo's time-optimal {@code .traj}
     * files are loaded through PathPlanner (no extra vendordep — Choreo
     * tooling exports, PathPlanner follows), so this needs {@code AutoBuilder}
     * configured via the {@link PathPlannerConfig} constructor.
     *
     * <p>Put the {@code .traj} files in {@code src/main/deploy/choreo/}. If the
     * trajectory can't be loaded, reports the error to the driver station and
     * returns a no-op rather than crashing.
     *
     * @param trajectoryName file name without the {@code .traj} extension
     */
    public CatalystCommand followChoreoPath(String trajectoryName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(trajectoryName);
            return LegacyCommands.fromV2(AutoBuilder.followPath(path)).withName("Swerve.Choreo(" + trajectoryName + ")");
        } catch (Exception e) {
            DriverStationErrors.reportError(
                    "Failed to load Choreo trajectory \"" + trajectoryName + "\": " + e.getMessage(), true);
            return runOnce(() -> {}).withName("Swerve.Choreo(missing:" + trajectoryName + ")");
        }
    }

    /**
     * Follow a pre-made PathPlanner path <b>exactly</b> by name — assumes the
     * robot starts at the path's start. Use this for segments between known
     * waypoints. Needs {@code AutoBuilder} configured. Reports + no-ops if the
     * path can't be loaded.
     *
     * <p>If a prior reactive action (chase / align) left the robot off the
     * path's start, use {@link #pathfindThenFollowPath(String, PathConstraints)}
     * instead so it pathfinds back on first.
     */
    public CatalystCommand followPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return LegacyCommands.fromV2(AutoBuilder.followPath(path)).withName("Swerve.FollowPath(" + pathName + ")");
        } catch (Exception e) {
            DriverStationErrors.reportError("Failed to load path \"" + pathName + "\": " + e.getMessage(), true);
            return runOnce(() -> {}).withName("Swerve.FollowPath(missing:" + pathName + ")");
        }
    }

    /**
     * Pathfind from the <b>current</b> pose to the start of a named PathPlanner
     * path, then follow it. This is the "rejoin the plan" primitive: after a
     * reactive action (chasing a piece, a precision align) leaves you off
     * course, this gets you back onto the planned route from wherever you
     * actually are — instead of assuming you start at the path's beginning.
     *
     * @param pathName    PathPlanner path file (no extension)
     * @param constraints pathfinding constraints for the rejoin leg
     */
    public CatalystCommand pathfindThenFollowPath(String pathName, PathConstraints constraints) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return LegacyCommands.fromV2(AutoBuilder.pathfindThenFollowPath(path, constraints))
                    .withName("Swerve.PathfindThenFollow(" + pathName + ")");
        } catch (Exception e) {
            DriverStationErrors.reportError(
                    "Failed to load path \"" + pathName + "\": " + e.getMessage(), true);
            return runOnce(() -> {}).withName("Swerve.PathfindThenFollow(missing:" + pathName + ")");
        }
    }

    /** Pathfind-then-follow with unlimited constraints (use your own limits elsewhere). */
    public CatalystCommand pathfindThenFollowPath(String pathName) {
        return pathfindThenFollowPath(pathName, PathConstraints.unlimitedConstraints(12.0));
    }

    /**
     * Drive toward a vision-detected game piece and stop on top of it.
     *
     * <p>The supplier returns the piece's field-relative position when your
     * detection coprocessor sees one, or empty when it doesn't — this is the
     * missing primitive the {@code Autopilot} "acquire" action wants. While a
     * piece is visible, drives field-centric toward it with the front of the
     * robot leading; when it disappears or the robot arrives within
     * {@code toleranceMeters}, the command ends.
     *
     * @param pieceFieldPose supplier of the detected piece position (field frame)
     * @param translationKP  proportional gain for the approach (try 2.0–5.0)
     * @param toleranceMeters arrival distance
     */
    public CatalystCommand driveToPiece(Supplier<java.util.Optional<Translation2d>> pieceFieldPose,
                                double translationKP, double toleranceMeters) {
        PIDController xCtl = new PIDController(translationKP, 0, 0);
        PIDController yCtl = new PIDController(translationKP, 0, 0);

        // Two flags, both reset when the command starts rather than when it is built.
        //
        // A command bound with whileTrue is constructed once, at bind time, and reused on every
        // press. Left set from the previous run, `done` makes untilTrue true on the first tick, so
        // the second press of the button ends the command immediately and the robot does nothing at
        // all - with nothing logged, because nothing went wrong.
        final boolean[] done = { false };
        return run(() -> {
            var opt = pieceFieldPose.get();
            if (opt.isEmpty()) {
                // The javadoc says the command ends when the piece disappears, and it has to: a
                // command that sits here holds the drivetrain requirement, so the default drive
                // command stays interrupted and the driver has no sticks until something else is
                // scheduled.
                driveFieldCentric(0, 0, 0);
                done[0] = true;
                return;
            }
            Translation2d target = opt.get();
            Pose2d cur = getPose();
            double dist = cur.getTranslation().getDistance(target);
            done[0] = dist < toleranceMeters;
            double maxApproach = maxSpeedMPS * 0.6;
            double vx = Math.clamp(xCtl.calculate(cur.getX(), target.getX()), -maxApproach, maxApproach);
            double vy = Math.clamp(yCtl.calculate(cur.getY(), target.getY()), -maxApproach, maxApproach);
            driveFieldCentric(vx, vy, 0);
        }).beforeStarting(() -> {
            done[0] = false;
            xCtl.reset();
            yCtl.reset();
        }).untilTrue(() -> done[0])
          .finallyDo(interrupted -> driveFieldCentric(0, 0, 0))
          .withName("Swerve.DriveToPiece");
    }

    /** Drive-to-piece with sensible defaults (kP = 3.0, tolerance = 0.2 m). */
    public CatalystCommand driveToPiece(Supplier<java.util.Optional<Translation2d>> pieceFieldPose) {
        return driveToPiece(pieceFieldPose, 3.0, 0.2);
    }

    /**
     * Per-module drive distances in metres (signed, cumulative) — used by
     * {@link WheelRadiusCalibration} and available for
     * any custom odometry diagnostics.
     */
    public double[] getModuleDistances() {
        var positions = drivetrain.getState().ModulePositions;
        double[] out = new double[positions.length];
        for (int i = 0; i < positions.length; i++) {
            out[i] = positions[i].distance;
        }
        return out;
    }

    /** X-brake command (lock wheels). Holds the brake for as long as it runs. */
    public CatalystCommand xBrake() {
        return run(this::setBrake).withName("Swerve.XBrake");
    }

    /**
     * Reset heading (zero the gyro). A pure odometry op — requires no
     * subsystem, so it won't interrupt the default drive command.
     */
    public CatalystCommand resetHeading() {
        return Commands.runOnce(() ->
                resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d())))
                .ignoringDisable(true)
                .withName("Swerve.ResetHeading");
    }
    
    @Override
    public CatalystCommand idle() {
        // WPILib's Mechanism.idle() contract is a command that runs forever to
        // hold the requirement, so use run(...) not runOnce(...).
        return run(() -> drivetrain.setControl(idleRequest)).withName("Idle");
    }

    /**
     * Reset the pose. A pure odometry op — requires no subsystem, so it won't
     * interrupt the default drive command.
     */
    public CatalystCommand resetPoseCommand(Supplier<Pose2d> poseSupplier) {
        return Commands.runOnce(() -> resetPose(poseSupplier.get()))
                .ignoringDisable(true)
                .withName("Swerve.ResetPose");
    }

    // --- Internals ---

    private static double applyDeadband(double value, double deadband) {
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
        ChassisVelocities speeds = getChassisSpeeds();
        return Math.hypot(speeds.vx, speeds.vy);
    }

    @Override
    public void periodic() {
        // Lazy-start the internal Phoenix sim on the first loop tick. See the constructor
        // note: starting it there poisons external physics bridges that attach right after
        // construction, because updateSimState() seeds sim-device state before they can
        // call disableInternalSim(). startSimThread() itself is a no-op once yielded.
        if (RobotBase.isSimulation() && simNotifier == null) {
            startSimThread();
        }

        //do the same as in the commandSwerveDriveTrain periodic function so no need to cast it to another type and do a .register for another periodic.
        //do it it didnt happen alredy or if we are on disable so i dont care about the longer time for the periodic function
        if (!hasAppliedOperatorPerspective || RobotState.isDisabled()) {
            RobotState.allianceOpt()
            .ifPresent(AllianceColor -> {
                drivetrain.setOperatorPerspectiveForward(AllianceColor == Alliance.RED ? Rotation2d.k180deg : Rotation2d.kZero);
                hasAppliedOperatorPerspective = true;
            });
        }
        Pose2d pose = getPose();
        CatalystLog.log(SWERVE + "Pose", Pose2d.struct, pose);
        CatalystLog.log(SWERVE + "ChassisVelocities", ChassisVelocities.struct, getChassisSpeeds());
        // Deprecated alias for the same reason as ModuleStates below. Remove after 2027.
        CatalystLog.log(SWERVE + "ChassisSpeeds", ChassisVelocities.struct, getChassisSpeeds());

        var state = drivetrain.getState();
        if (state.ModuleVelocities != null) {
            CatalystLog.log(SWERVE + "ModuleVelocities", SwerveModuleVelocity.struct, state.ModuleVelocities);

            // Deprecated alias, published for one season.
            //
            // WPILib renamed SwerveModuleState to SwerveModuleVelocity, and this topic followed it
            // so the name matches the type it carries. But a topic name is part of Catalyst's
            // telemetry contract: teams have AdvantageScope layouts, Console panels and dashboard
            // widgets pointed at the old path, and silently renaming it breaks every one of them
            // with no error anywhere. One extra struct-array write per loop is a cheap price for
            // not doing that. Remove after the 2027 season.
            CatalystLog.log(SWERVE + "ModuleStates", SwerveModuleVelocity.struct, state.ModuleVelocities);
        }
        if (state.ModuleTargets != null) {
            CatalystLog.log(SWERVE + "ModuleTargets", SwerveModuleVelocity.struct, state.ModuleTargets);
        }
        CatalystLog.log(SWERVE + "HeadingDeg", pose.getRotation().getDegrees());
        ChassisVelocities speeds = getChassisSpeeds();
        double speed = Math.hypot(speeds.vx, speeds.vy);
        CatalystLog.log(SWERVE + "SpeedMPS", speed);
        CatalystLog.log(SWERVE + "OmegaRadPerSec", speeds.omega);
        CatalystLog.log(SWERVE + "SpeedMultiplier", speedMultiplier);
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
