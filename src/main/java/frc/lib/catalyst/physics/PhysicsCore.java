package frc.lib.catalyst.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.physics.diagnostics.CollisionDetector;
import frc.lib.catalyst.physics.diagnostics.CollisionEvent;
import frc.lib.catalyst.physics.estimation.DisturbanceEstimator;
import frc.lib.catalyst.physics.estimation.PhysicalStateEstimator;
import frc.lib.catalyst.physics.estimation.SlipEstimator;
import frc.lib.catalyst.physics.model.DrivetrainModel;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.observation.Observation;
import frc.lib.catalyst.physics.observation.PoseObservation;
import frc.lib.catalyst.physics.observation.VelocityObservation;
import frc.lib.catalyst.physics.prediction.LaunchState;
import frc.lib.catalyst.physics.prediction.LaunchStatePredictor;
import frc.lib.catalyst.physics.prediction.StatePredictor;
import frc.lib.catalyst.util.AlertManager;
import frc.lib.catalyst.util.SignalProcessor;

/**
 * The optional physical-intelligence layer for a Catalyst robot: one object that watches the robot's
 * real behaviour and answers questions a pose alone cannot.
 *
 * <h2>What it is for</h2>
 * Most FRC code treats the drivetrain's {@code Pose2d} as truth and its wheel velocity as fact. Both
 * are good until the physics stops cooperating — a wheel breaks traction leaving the wall, a defender
 * lands a hit mid-auto, vision has not seen a tag in four seconds. Physics Core is the layer that
 * notices, quantifies it, and tells the rest of the robot:
 *
 * <ul>
 *   <li><b>Estimate</b> — a velocity fused from wheels and IMU, and an honest confidence in it.</li>
 *   <li><b>Predict</b> — where the robot will be when a shot actually leaves it.</li>
 *   <li><b>Detect</b> — which module is slipping, and when something hit the robot.</li>
 *   <li><b>Explain</b> — why the measured behaviour differed from the modelled behaviour.</li>
 * </ul>
 *
 * <h2>What it deliberately is not</h2>
 * It is <b>entirely optional</b>, and it takes no control authority. It schedules no commands, blocks
 * no transitions, writes no pose, and changes no setpoint. Every Catalyst API works exactly as it did
 * without it; adding it can only give you more information. The state machine, {@code BehaviorEngine},
 * and {@code Autopilot} keep owning what the robot does — Physics Core only informs them:
 *
 * <pre>{@code
 * transition(STOWED, EXTENDED)
 *     .guard(() -> !physics.analyze().isNearTipping(), "tip-margin");
 * }</pre>
 *
 * <p>It is also not a rigid-body physics engine. There is no collision solver and no contact model;
 * for simulation with game pieces, Catalyst integrates with maple-sim. Physics Core is about the
 * <em>real</em> robot's measured behaviour.
 *
 * <h2>Setting it up</h2>
 * <pre>{@code
 * PhysicsCore physics = PhysicsCore.builder()
 *     .robotModel(RobotModel.builder()
 *         .massKg(54.4)
 *         .footprintMeters(0.74, 0.74)
 *         .centerOfMassHeightMeters(0.20)
 *         .build())
 *     .kinematics(drive.getDrivetrain().getKinematics())
 *     .poseSource(drive::getPose)
 *     .chassisSpeedsSource(drive::getChassisSpeeds)          // robot-relative
 *     .moduleStatesSource(drive::getModuleStates)
 *     .accelerationSource(() -> new Translation2d(imu.getAccelX(), imu.getAccelY()))
 *     .yawRateSource(() -> Units.degreesToRadians(imu.getRate()))
 *     .releaseDelaySeconds(0.12)
 *     .build();
 *
 * // once per loop, after the drivetrain has updated:
 * public void robotPeriodic() {
 *     CommandScheduler.getInstance().run();
 *     physics.update();
 * }
 *
 * // and wherever you already call addVisionMeasurement, tell Physics Core too:
 * physics.observe(PoseObservation.of(estimate.pose(), estimate.timestamp(), "limelight-front"));
 * }</pre>
 *
 * <p>Everything lands under {@code Catalyst/Physics/...} in NetworkTables, so a shadow-mode session
 * is reviewable in AdvantageScope with no dashboard work.
 *
 * <h2>Degrading safely</h2>
 * Sources are optional and independent. No accelerometer means no IMU fusion and no collision
 * detection, but confidence tracking and prediction still work. No module states means no slip
 * scoring. Missing inputs are named in {@link #health()} rather than silently producing plausible
 * nonsense.
 *
 * <p>The clock is injectable and {@link #update(PhysicsSample)} takes every measurement as an
 * argument, so the whole pipeline runs in a unit test with no HAL and no robot.
 *
 * @since 1.5.0
 */
public final class PhysicsCore implements UncertainRobotStateSource {

    private static final String LOG_ROOT = "Catalyst/Physics/";
    private static final String ALERT_KEY = "PhysicsCore";

    private final RobotModel robotModel;
    private final DrivetrainModel drivetrainModel;
    private final PhysicsProfile profile;
    private final PhysicalStateEstimator estimator;
    private final StatePredictor predictor;
    private final LaunchStatePredictor launchPredictor;
    private final SlipEstimator slipEstimator;
    private final DisturbanceEstimator disturbanceEstimator;
    private final CollisionDetector collisionDetector;

    private final Supplier<Pose2d> poseSource;
    private final Supplier<ChassisSpeeds> chassisSpeedsSource;
    private final Supplier<SwerveModuleState[]> moduleStatesSource;
    private final Supplier<Translation2d> accelerationSource;
    private final DoubleSupplier yawRateSource;
    private final DoubleSupplier clock;
    private final double poseOutlierGateMeters;
    private final boolean loggingEnabled;
    private final boolean alertsEnabled;

    private final SignalProcessor.ExponentialMovingAverage wheelAccelX =
            new SignalProcessor.ExponentialMovingAverage(0.25);
    private final SignalProcessor.ExponentialMovingAverage wheelAccelY =
            new SignalProcessor.ExponentialMovingAverage(0.25);

    private PhysicsAnalysis analysis = PhysicsAnalysis.nominal();
    private Translation2d lastKinematicFieldVelocity = Translation2d.kZero;
    private double lastSampleTimestamp = Double.NaN;
    /**
     * Timestamp of the last sample the diagnostics actually ran on. Tracked separately from
     * {@link #lastSampleTimestamp} because diagnostics are skipped on any loop without an
     * accelerometer reading — sharing one timestamp would differentiate a fresh velocity against a
     * stale one after a gap and report an enormous acceleration that never happened.
     */
    private double lastDiagnosticsTimestamp = Double.NaN;
    private long rejectedObservations = 0;
    private boolean alertActive = false;
    private boolean sawAcceleration = false;
    private boolean sawModuleStates = false;

    private PhysicsCore(Builder builder) {
        this.robotModel = builder.robotModel;
        this.drivetrainModel = new DrivetrainModel(builder.robotModel);
        this.profile = builder.profile;
        this.poseSource = builder.poseSource;
        this.chassisSpeedsSource = builder.chassisSpeedsSource;
        this.moduleStatesSource = builder.moduleStatesSource;
        this.accelerationSource = builder.accelerationSource;
        this.yawRateSource = builder.yawRateSource;
        this.clock = builder.clock;
        this.poseOutlierGateMeters = builder.poseOutlierGateMeters;
        this.loggingEnabled = builder.loggingEnabled;
        this.alertsEnabled = builder.alertsEnabled;

        PhysicalStateEstimator.Builder estimatorBuilder = PhysicalStateEstimator.builder();
        if (builder.accelerationSource == null) estimatorBuilder.withoutAccelerometer();
        this.estimator = estimatorBuilder.build();

        this.predictor = StatePredictor.withDefaults();
        this.launchPredictor = new LaunchStatePredictor(builder.releaseDelaySeconds);

        this.slipEstimator = (profile.slipEnabled() && builder.kinematics != null)
                ? SlipEstimator.builder().kinematics(builder.kinematics, builder.moduleCount).build()
                : null;
        this.disturbanceEstimator = profile.diagnosticsEnabled()
                ? new DisturbanceEstimator(drivetrainModel)
                : null;
        this.collisionDetector = disturbanceEstimator != null
                ? CollisionDetector.builder().disturbance(disturbanceEstimator).build()
                : null;
    }

    // ===========================================
    //                  INPUTS
    // ===========================================

    /**
     * Read every configured source and fold the result in. Call once per loop, after the drivetrain
     * has updated its own estimate.
     *
     * @return the updated state, the same object {@link #state()} will return
     * @throws IllegalStateException if the pose and chassis-speeds sources were not configured — those
     *                               two are what makes {@code update()} able to read anything at all
     */
    public PhysicalRobotState update() {
        if (poseSource == null || chassisSpeedsSource == null) {
            throw new IllegalStateException("update() needs poseSource and chassisSpeedsSource; "
                    + "call update(PhysicsSample) instead if you are supplying measurements yourself");
        }
        return update(new PhysicsSample(
                clock.getAsDouble(),
                poseSource.get(),
                chassisSpeedsSource.get(),
                moduleStatesSource == null ? null : moduleStatesSource.get(),
                accelerationSource == null ? null : accelerationSource.get(),
                yawRateSource == null ? 0.0 : yawRateSource.getAsDouble()));
    }

    /**
     * Fold in one loop's measurements. This is the whole pipeline — slip scoring, state fusion,
     * disturbance residuals, collision detection, logging — and it touches no hardware, so it is what
     * unit tests and log replay drive.
     *
     * @param sample the measurements, all taken at {@link PhysicsSample#timestampSeconds()}
     * @return the updated state
     */
    public PhysicalRobotState update(PhysicsSample sample) {
        if (sample.hasAcceleration()) sawAcceleration = true;
        if (sample.hasModuleStates()) sawModuleStates = true;

        double slipFactor = updateSlip(sample);
        PhysicalRobotState updated = estimator.update(
                sample.timestampSeconds(),
                sample.pose(),
                sample.robotRelativeSpeeds(),
                sample.robotRelativeAcceleration(),
                sample.yawRateRadPerSec(),
                slipFactor);

        Optional<CollisionEvent> collision = updateDiagnostics(sample, updated);
        analysis = buildAnalysis(updated, slipFactor, collision);

        lastSampleTimestamp = sample.timestampSeconds();
        if (alertsEnabled) updateAlert();
        if (loggingEnabled) publish(updated);
        return updated;
    }

    /**
     * Supply an outside measurement — a vision pose, a velocity from an optical-flow sensor.
     *
     * <p>A {@link PoseObservation} that lands further than the outlier gate from the pose Physics
     * Core is tracking is rejected and counted rather than accepted: one bad frame should not be able
     * to convince the robot it teleported. Accepted pose observations reset the staleness term in
     * confidence and nothing else — <b>Physics Core never writes your pose.</b> Keep calling
     * {@code drive.addVisionMeasurement(...)} exactly as you do today.
     *
     * @param observation the measurement, timestamped at capture time
     * @return true if it was accepted
     */
    public boolean observe(Observation observation) {
        if (observation == null) return false;
        if (observation instanceof PoseObservation pose) {
            if (estimator.isInitialized()) {
                double error = pose.pose().getTranslation()
                        .getDistance(estimator.state().pose().getTranslation());
                if (error > poseOutlierGateMeters) {
                    rejectedObservations++;
                    if (loggingEnabled) {
                        CatalystLog.log(LOG_ROOT + "RejectedObservations", rejectedObservations);
                        CatalystLog.log(LOG_ROOT + "LastRejectionMeters", error);
                    }
                    return false;
                }
            }
            estimator.recordAbsoluteFix(observation.timestampSeconds());
            return true;
        }
        if (observation instanceof VelocityObservation) {
            // Accepted and logged, but not yet fused: an independent velocity source is only worth
            // weighing once it has been validated against a real robot, which is a later phase. The
            // contract is here now so teams can wire the source and see it in shadow-mode logs.
            if (loggingEnabled) {
                CatalystLog.log(LOG_ROOT + "VelocityObservation/Source", observation.source());
            }
            return true;
        }
        return false;
    }

    // ===========================================
    //                  OUTPUTS
    // ===========================================

    /** The latest fused state. {@link PhysicalRobotState#unknown()} before the first update. */
    public PhysicalRobotState state() {
        return estimator.state();
    }

    /**
     * Where the robot will be {@code horizonSeconds} from now, with confidence degraded to match.
     * Clamped to the predictor's maximum horizon rather than refused.
     */
    public PhysicalRobotState predict(double horizonSeconds) {
        return predictor.predict(state(), horizonSeconds);
    }

    /**
     * The robot's state at the instant a shot commanded now would actually leave, using the configured
     * release delay. Drops straight into {@code AimingSolver} or {@code AimingSolverVector}:
     *
     * <pre>{@code
     * LaunchState launch = physics.predictLaunchState();
     * var shot = aimingSolver.calculate(launch.pose(), launch.fieldVelocity());
     * }</pre>
     */
    public LaunchState predictLaunchState() {
        return launchPredictor.predict(state());
    }

    /**
     * The same, plus a delay that varies shot to shot — waiting on flywheel recovery, an indexer that
     * has to run first.
     *
     * @param additionalDelaySeconds extra seconds beyond the configured release delay
     */
    public LaunchState predictLaunchState(double additionalDelaySeconds) {
        return launchPredictor.predict(state(), additionalDelaySeconds);
    }

    /** What is physically happening to the robot: slip, traction, tipping margin, disturbance, impacts. */
    public PhysicsAnalysis analyze() {
        return analysis;
    }

    /** Whether Physics Core itself is running, fresh, and fully wired. */
    public PhysicsHealth health() {
        double age = Double.isNaN(lastSampleTimestamp)
                ? Double.POSITIVE_INFINITY
                : clock.getAsDouble() - lastSampleTimestamp;
        return new PhysicsHealth(estimator.isInitialized(), age, profile, degradedInputs());
    }

    /** The physical model in use. */
    public RobotModel robotModel() {
        return robotModel;
    }

    /** The traction and tipping limits derived from that model. */
    public DrivetrainModel drivetrainModel() {
        return drivetrainModel;
    }

    /** How many observations have been rejected as outliers since construction. */
    public long rejectedObservationCount() {
        return rejectedObservations;
    }

    /** Clear every estimate and start over. */
    public void reset() {
        estimator.reset();
        if (slipEstimator != null) slipEstimator.reset();
        if (disturbanceEstimator != null) disturbanceEstimator.reset();
        if (collisionDetector != null) collisionDetector.reset();
        wheelAccelX.reset();
        wheelAccelY.reset();
        analysis = PhysicsAnalysis.nominal();
        lastKinematicFieldVelocity = Translation2d.kZero;
        lastSampleTimestamp = Double.NaN;
        lastDiagnosticsTimestamp = Double.NaN;
    }

    // ===========================================
    //          RobotStateSource contract
    // ===========================================

    @Override
    public Pose2d pose() {
        return state().pose();
    }

    @Override
    public ChassisSpeeds fieldVelocity() {
        return state().fieldVelocity();
    }

    @Override
    public double timestampSeconds() {
        return state().timestampSeconds();
    }

    @Override
    public LocalizationQuality quality() {
        return state().quality();
    }

    // ===========================================
    //                 INTERNALS
    // ===========================================

    private double updateSlip(PhysicsSample sample) {
        if (slipEstimator == null || !sample.hasModuleStates()) return 0.0;
        if (sample.moduleStates().length != slipEstimator.moduleCount()) return 0.0;
        return slipEstimator.update(sample.moduleStates(), sample.robotRelativeSpeeds());
    }

    /**
     * Runs the disturbance residual and the collision detector. The residual compares the acceleration
     * the wheels imply against the acceleration the IMU measured, so it needs both — with no
     * accelerometer there is nothing to compare and diagnostics stay quiet rather than reporting a
     * residual equal to the wheel acceleration itself.
     */
    private Optional<CollisionEvent> updateDiagnostics(PhysicsSample sample, PhysicalRobotState updated) {
        if (disturbanceEstimator == null || !sample.hasAcceleration()) return Optional.empty();

        Rotation2d heading = sample.pose().getRotation();
        ChassisSpeeds kinematicField =
                ChassisSpeeds.fromRobotRelativeSpeeds(sample.robotRelativeSpeeds(), heading);
        Translation2d kinematicVelocity =
                new Translation2d(kinematicField.vxMetersPerSecond, kinematicField.vyMetersPerSecond);

        double dt = sample.timestampSeconds() - lastDiagnosticsTimestamp;
        Translation2d wheelAcceleration = Translation2d.kZero;
        if (!Double.isNaN(lastDiagnosticsTimestamp) && dt > 0 && dt <= 0.25) {
            Translation2d raw = kinematicVelocity.minus(lastKinematicFieldVelocity).div(dt);
            wheelAcceleration = new Translation2d(
                    wheelAccelX.calculate(raw.getX()), wheelAccelY.calculate(raw.getY()));
        } else {
            wheelAccelX.reset();
            wheelAccelY.reset();
        }
        lastKinematicFieldVelocity = kinematicVelocity;
        lastDiagnosticsTimestamp = sample.timestampSeconds();

        Translation2d measured = sample.robotRelativeAcceleration().rotateBy(heading);
        disturbanceEstimator.update(wheelAcceleration, measured);
        return collisionDetector == null
                ? Optional.empty()
                : collisionDetector.update(sample.timestampSeconds());
    }

    private PhysicsAnalysis buildAnalysis(PhysicalRobotState updated, double slipFactor,
                                          Optional<CollisionEvent> collision) {
        double peakSlip = slipEstimator == null ? 0.0 : slipEstimator.peakSlip();
        int worstModule = slipEstimator == null ? -1 : slipEstimator.worstModule();
        double disturbance = disturbanceEstimator == null ? 0.0 : disturbanceEstimator.magnitudeMpsSq();
        Rotation2d disturbanceDirection =
                disturbanceEstimator == null ? Rotation2d.kZero : disturbanceEstimator.direction();
        Optional<CollisionEvent> lastCollision = collision.isPresent()
                ? collision
                : (collisionDetector == null ? Optional.empty() : collisionDetector.lastEvent());

        return new PhysicsAnalysis(
                slipFactor,
                peakSlip,
                worstModule,
                drivetrainModel.tractionUsage(updated.fieldAcceleration()),
                drivetrainModel.tippingUsage(updated.fieldAcceleration()),
                disturbance,
                disturbanceDirection,
                lastCollision);
    }

    /** Sources that were configured but never delivered anything, plus ones never configured at all. */
    private List<String> degradedInputs() {
        List<String> degraded = new ArrayList<>(2);
        if (accelerationSource != null && !sawAcceleration) degraded.add("accelerometer");
        if (moduleStatesSource != null && !sawModuleStates) degraded.add("module states");
        return degraded;
    }

    // Warn and clear only on the transition, so the alert list never churns.
    private void updateAlert() {
        boolean degraded = !degradedInputs().isEmpty();
        if (degraded && !alertActive) {
            AlertManager.getInstance().warning(ALERT_KEY, degradedMessage());
            alertActive = true;
        } else if (!degraded && alertActive) {
            AlertManager.getInstance().clearWarning(ALERT_KEY, degradedMessage());
            alertActive = false;
        }
    }

    private String degradedMessage() {
        return "Physics Core is missing configured inputs; estimates are degraded";
    }

    private void publish(PhysicalRobotState state) {
        CatalystLog.log(LOG_ROOT + "Velocity/X", state.fieldVelocity().vxMetersPerSecond);
        CatalystLog.log(LOG_ROOT + "Velocity/Y", state.fieldVelocity().vyMetersPerSecond);
        CatalystLog.log(LOG_ROOT + "Speed", state.speedMetersPerSecond());
        CatalystLog.log(LOG_ROOT + "Acceleration", state.accelerationMetersPerSecSq());
        CatalystLog.log(LOG_ROOT + "Quality/Confidence", state.quality().confidence());
        CatalystLog.log(LOG_ROOT + "Quality/Level", state.quality().level().name());
        CatalystLog.log(LOG_ROOT + "Quality/TranslationStdDev", state.quality().translationStdDevMeters());
        CatalystLog.log(LOG_ROOT + "Quality/Reason", state.quality().reason());
        CatalystLog.log(LOG_ROOT + "Slip/Factor", analysis.slipFactor());
        CatalystLog.log(LOG_ROOT + "Slip/Peak", analysis.peakSlip());
        CatalystLog.log(LOG_ROOT + "Slip/WorstModule", analysis.worstModule());
        if (slipEstimator != null) {
            CatalystLog.log(LOG_ROOT + "Slip/ModuleScores", slipEstimator.moduleScores());
        }
        CatalystLog.log(LOG_ROOT + "TractionUsage", analysis.tractionUsage());
        CatalystLog.log(LOG_ROOT + "TippingUsage", analysis.tippingUsage());
        CatalystLog.log(LOG_ROOT + "Disturbance/MpsSq", analysis.disturbanceMpsSq());
        CatalystLog.log(LOG_ROOT + "Disturbance/Degrees", analysis.disturbanceDirection().getDegrees());
        CatalystLog.log(LOG_ROOT + "SensorDisagreement", estimator.sensorDisagreementMps());
        CatalystLog.log(LOG_ROOT + "Status", analysis.describe());
        analysis.lastCollision().ifPresent(event -> {
            CatalystLog.log(LOG_ROOT + "Collision/Timestamp", event.timestampSeconds());
            CatalystLog.log(LOG_ROOT + "Collision/MpsSq", event.magnitudeMpsSq());
            CatalystLog.log(LOG_ROOT + "Collision/Newtons", event.peakForceNewtons());
        });
    }

    /** Start building a Physics Core. Only {@link Builder#robotModel(RobotModel)} is required. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link PhysicsCore}. */
    public static final class Builder {
        private RobotModel robotModel;
        private PhysicsProfile profile = PhysicsProfile.BALANCED;
        private SwerveDriveKinematics kinematics;
        private int moduleCount = 4;
        private Supplier<Pose2d> poseSource;
        private Supplier<ChassisSpeeds> chassisSpeedsSource;
        private Supplier<SwerveModuleState[]> moduleStatesSource;
        private Supplier<Translation2d> accelerationSource;
        private DoubleSupplier yawRateSource;
        private DoubleSupplier clock = Timer::getFPGATimestamp;
        private double releaseDelaySeconds = 0.0;
        private double poseOutlierGateMeters = 1.0;
        private boolean loggingEnabled = true;
        private boolean alertsEnabled = true;

        /** The robot's physical model — mass, footprint, wheel radius, centre-of-mass height. Required. */
        public Builder robotModel(RobotModel robotModel) {
            this.robotModel = robotModel;
            return this;
        }

        /** How much computation Physics Core may spend per loop. Defaults to {@link PhysicsProfile#BALANCED}. */
        public Builder profile(PhysicsProfile profile) {
            this.profile = profile;
            return this;
        }

        /**
         * The drivetrain's kinematics, which slip scoring measures against. Without it, slip scoring
         * is skipped and everything else still runs.
         */
        public Builder kinematics(SwerveDriveKinematics kinematics, int moduleCount) {
            this.kinematics = kinematics;
            this.moduleCount = moduleCount;
            return this;
        }

        /** Four-module convenience for {@link #kinematics(SwerveDriveKinematics, int)}. */
        public Builder kinematics(SwerveDriveKinematics kinematics) {
            return kinematics(kinematics, 4);
        }

        /** Where the pose comes from, e.g. {@code drive::getPose}. Required for {@link #update()}. */
        public Builder poseSource(Supplier<Pose2d> poseSource) {
            this.poseSource = poseSource;
            return this;
        }

        /**
         * Where the <b>robot-relative</b> chassis velocity comes from, e.g. {@code drive::getChassisSpeeds}.
         * Required for {@link #update()}. Passing field-relative speeds here will produce a quietly
         * wrong estimate, because Physics Core rotates them by the heading itself.
         */
        public Builder chassisSpeedsSource(Supplier<ChassisSpeeds> chassisSpeedsSource) {
            this.chassisSpeedsSource = chassisSpeedsSource;
            return this;
        }

        /** Where measured module states come from, e.g. {@code drive::getModuleStates}. Enables slip scoring. */
        public Builder moduleStatesSource(Supplier<SwerveModuleState[]> moduleStatesSource) {
            this.moduleStatesSource = moduleStatesSource;
            return this;
        }

        /**
         * Where robot-relative translational acceleration comes from, in m/s^2 with gravity removed.
         * Enables IMU fusion, disturbance residuals, and collision detection. Leave it unset on a robot
         * without a usable accelerometer and the estimator falls back to plain odometry.
         */
        public Builder accelerationSource(Supplier<Translation2d> accelerationSource) {
            this.accelerationSource = accelerationSource;
            return this;
        }

        /** Where the gyro yaw rate comes from, in rad/s. */
        public Builder yawRateSource(DoubleSupplier yawRateSource) {
            this.yawRateSource = yawRateSource;
            return this;
        }

        /**
         * Seconds between commanding a shot and the piece leaving the robot, used by
         * {@link PhysicsCore#predictLaunchState()}. Measure it once against a log; defaults to 0.
         */
        public Builder releaseDelaySeconds(double releaseDelaySeconds) {
            this.releaseDelaySeconds = releaseDelaySeconds;
            return this;
        }

        /**
         * How far a pose observation may land from the current estimate before it is rejected as an
         * outlier, in metres. Defaults to 1.0 — generous enough for honest vision error, tight enough
         * to catch a misidentified tag.
         */
        public Builder poseOutlierGate(double poseOutlierGateMeters) {
            this.poseOutlierGateMeters = poseOutlierGateMeters;
            return this;
        }

        /** Source of the current time in seconds. Defaults to the FPGA clock; tests pass their own. */
        public Builder clock(DoubleSupplier clock) {
            this.clock = clock;
            return this;
        }

        /** Turn telemetry publishing on or off. On by default. */
        public Builder withLogging(boolean loggingEnabled) {
            this.loggingEnabled = loggingEnabled;
            return this;
        }

        /**
         * Turn the degraded-input warning on or off. On by default. Turning it off is what a unit
         * test does, since raising an alert goes through the Driver Station.
         */
        public Builder withAlerts(boolean alertsEnabled) {
            this.alertsEnabled = alertsEnabled;
            return this;
        }

        /** Validate and build. */
        public PhysicsCore build() {
            if (robotModel == null) {
                throw new IllegalStateException("robotModel is required - the traction and tipping "
                        + "limits everything else reasons from are derived from it");
            }
            if (clock == null) throw new IllegalStateException("clock must not be null");
            if (releaseDelaySeconds < 0) {
                throw new IllegalStateException("releaseDelaySeconds must be >= 0 (got "
                        + releaseDelaySeconds + ")");
            }
            if (!(poseOutlierGateMeters > 0)) {
                throw new IllegalStateException("poseOutlierGate must be > 0 (got "
                        + poseOutlierGateMeters + ")");
            }
            return new PhysicsCore(this);
        }
    }
}
