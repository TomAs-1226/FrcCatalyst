package frc.lib.catalyst.physics.sim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import frc.lib.catalyst.physics.PhysicalRobotState;
import frc.lib.catalyst.physics.PhysicsCore;
import frc.lib.catalyst.physics.PhysicsSample;
import frc.lib.catalyst.physics.diagnostics.ModelResidualMonitor;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.observation.PoseObservation;

/**
 * Marks Physics Core against a simulation that knows the truth, and reports whether it passed.
 *
 * <p>The RFC lists acceptance criteria — fused velocity beating encoder-only velocity, slip detection
 * catching induced slip without crying wolf, collisions reported once and quickly, confidence tracking
 * reality, release-state prediction improving on naive aiming. On a real robot none of those can be
 * measured, because the true velocity is exactly the thing nobody knows. Against
 * {@link SimulatedRobot} all of them can.
 *
 * <pre>{@code
 * PhysicsValidator validator = PhysicsValidator.builder()
 *     .robotModel(myRobot)              // YOUR robot's mass and geometry
 *     .kinematics(myKinematics)
 *     .build();
 *
 * ValidationReport report = validator.runAll();
 * System.out.println(report.describe());
 * }</pre>
 *
 * <p>Run it against <b>your</b> {@code RobotModel} rather than the defaults. Physics Core's thresholds
 * are tuned for a typical mid-weight swerve, and a robot far from that — very light, very tall, unusual
 * footprint — may need different ones. This tells you before a match rather than during one.
 *
 * <h2>What a pass means, and what it does not</h2>
 * A pass means the estimator recovers a known truth from deliberately imperfect sensors: wheel radius
 * error, accelerometer bias, encoder noise, and odometry that drifts through slip. That is a real
 * result and it is the rung of the validation ladder below shadow mode.
 *
 * <p>It does <b>not</b> mean the model matches carpet. The simulation assumes constant friction, rigid
 * contact, and a drivetrain that slips uniformly until told otherwise; a real field is messier than
 * all three. Passing here earns the right to run shadow mode on a robot — it does not replace it.
 *
 * @since 1.7.0
 */
public final class PhysicsValidator {

    private final RobotModel robotModel;
    private final SwerveDriveKinematics kinematics;
    private final int moduleCount;
    private final long seed;
    private final double releaseDelaySeconds;

    private PhysicsValidator(Builder builder) {
        this.robotModel = builder.robotModel;
        this.kinematics = builder.kinematics;
        this.moduleCount = builder.moduleCount;
        this.seed = builder.seed;
        this.releaseDelaySeconds = builder.releaseDelaySeconds;
    }

    /** Run every scenario and collect the results. */
    public ValidationReport runAll() {
        List<ScenarioResult> results = new ArrayList<>();
        results.add(fusedVelocityBeatsEncodersDuringSlip());
        results.add(differentialSlipIsDetected());
        results.add(cleanDrivingDoesNotReportSlip());
        results.add(uniformSlipShowsUpAsDisturbance());
        results.add(collisionsAreReportedOnceAndQuickly());
        results.add(confidenceTracksVisionAvailability());
        results.add(releaseStatePredictionBeatsAimingAtNow());
        results.add(wheelRadiusErrorSurfacesAsSensorDisagreement());
        results.add(itDegradesSafelyWithoutAnAccelerometer());
        return new ValidationReport(List.copyOf(results));
    }

    // ===========================================
    //                 SCENARIOS
    // ===========================================

    /**
     * The headline claim: during a slip, the fused velocity is closer to the truth than the wheels
     * alone. Measured as RMS error over the slip window.
     */
    private ScenarioResult fusedVelocityBeatsEncodersDuringSlip() {
        SimulatedRobot sim = sim();
        PhysicsCore physics = core(sim);

        settle(sim, physics, new ChassisSpeeds(2.0, 0, 0), 40);

        double fusedSquaredError = 0.0;
        double wheelSquaredError = 0.0;
        int samples = 0;

        sim.setFrictionScale(0.25);                       // hit a slick patch
        sim.command(new ChassisSpeeds(4.5, 0, 0));        // and floor it
        for (int i = 0; i < 30; i++) {
            sim.step();
            PhysicalRobotState state = physics.update(sim.sample());
            Translation2d truth = sim.trueVelocityVector();
            fusedSquaredError += squaredError(
                    new Translation2d(state.fieldVelocity().vxMetersPerSecond,
                            state.fieldVelocity().vyMetersPerSecond), truth);
            wheelSquaredError += squaredError(sim.wheelVelocity(), truth);
            samples++;
        }

        double fusedRms = Math.sqrt(fusedSquaredError / samples);
        double wheelRms = Math.sqrt(wheelSquaredError / samples);
        double improvement = wheelRms > 0 ? 1.0 - fusedRms / wheelRms : 0.0;

        return new ScenarioResult("Fused velocity beats encoders during slip",
                improvement > 0.30,
                Map.of("fused RMS error (m/s)", fusedRms,
                        "encoder-only RMS error (m/s)", wheelRms,
                        "improvement", improvement),
                String.format(Locale.ROOT, "%.0f%% less velocity error than the wheels alone",
                        improvement * 100));
    }

    /** One wheel over-reporting must be caught, and the right corner named. */
    private ScenarioResult differentialSlipIsDetected() {
        SimulatedRobot sim = sim();
        PhysicsCore physics = core(sim);
        settle(sim, physics, new ChassisSpeeds(3.0, 0, 0), 40);

        sim.setModuleSlipBias(2, 2.0);
        double peak = 0.0;
        int worstModule = -1;
        for (int i = 0; i < 15; i++) {
            sim.step();
            physics.update(sim.sample());
            if (physics.analyze().peakSlip() > peak) {
                peak = physics.analyze().peakSlip();
                worstModule = physics.analyze().worstModule();
            }
        }

        boolean passed = peak > 0.5 && worstModule == 2;
        return new ScenarioResult("Differential slip is detected on the right module", passed,
                Map.of("peak slip score", peak, "module named", (double) worstModule),
                passed ? "module 2 identified"
                       : "expected module 2, got " + worstModule + " at peak " + round(peak));
    }

    /** And clean driving must not report any of it. This is the false-positive check. */
    private ScenarioResult cleanDrivingDoesNotReportSlip() {
        SimulatedRobot sim = sim();
        PhysicsCore physics = core(sim);

        double peak = 0.0;
        int loops = 0;
        int falsePositives = 0;
        for (double speed : new double[]{1.0, 3.0, 4.0, 2.0, 0.5}) {
            sim.command(new ChassisSpeeds(speed, 0, 0));
            for (int i = 0; i < 40; i++) {
                sim.step();
                physics.update(sim.sample());
                peak = Math.max(peak, physics.analyze().peakSlip());
                if (physics.analyze().isSlipping()) falsePositives++;
                loops++;
            }
        }

        double rate = (double) falsePositives / loops;
        return new ScenarioResult("Clean driving reports no slip", rate < 0.01,
                Map.of("peak slip score", peak, "false-positive rate", rate),
                String.format(Locale.ROOT, "%d of %d loops flagged", falsePositives, loops));
    }

    /**
     * A whole drivetrain losing grip leaves no per-module residual — the documented blind spot. It
     * must still be visible, through the wheels-versus-IMU disturbance instead.
     */
    private ScenarioResult uniformSlipShowsUpAsDisturbance() {
        SimulatedRobot sim = sim();
        PhysicsCore physics = core(sim);
        settle(sim, physics, new ChassisSpeeds(2.0, 0, 0), 40);

        sim.setFrictionScale(0.2);
        sim.command(new ChassisSpeeds(5.0, 0, 0));
        double peakDisturbance = 0.0;
        double peakModuleSlip = 0.0;
        for (int i = 0; i < 25; i++) {
            sim.step();
            physics.update(sim.sample());
            peakDisturbance = Math.max(peakDisturbance, physics.analyze().disturbanceMpsSq());
            peakModuleSlip = Math.max(peakModuleSlip, physics.analyze().peakSlip());
        }

        return new ScenarioResult("Uniform slip surfaces as disturbance, not module slip",
                peakDisturbance > 2.0,
                Map.of("peak disturbance (m/s^2)", peakDisturbance,
                        "peak module slip score", peakModuleSlip),
                String.format(Locale.ROOT,
                        "disturbance reached %.1f m/s^2 while module scoring stayed at %.2f, as documented",
                        peakDisturbance, peakModuleSlip));
    }

    /** An impact must be reported, once, within a couple of loops. */
    private ScenarioResult collisionsAreReportedOnceAndQuickly() {
        SimulatedRobot sim = sim();
        PhysicsCore physics = core(sim);
        settleGently(sim, physics, new ChassisSpeeds(2.5, 0, 0));

        int events = 0;
        double detectedAt = Double.NaN;
        double impactStarted = sim.timestamp();
        double lastEventTime = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < 40; i++) {
            if (i < 6) sim.applyExternalAcceleration(new Translation2d(0.0, -18.0));
            sim.step();
            physics.update(sim.sample());
            var collision = physics.analyze().lastCollision();
            if (collision.isPresent() && collision.get().timestampSeconds() > lastEventTime) {
                lastEventTime = collision.get().timestampSeconds();
                if (Double.isNaN(detectedAt)) detectedAt = lastEventTime - impactStarted;
                events++;
            }
        }

        boolean passed = events == 1 && detectedAt < 0.15;
        return new ScenarioResult("Impacts are reported once, promptly", passed,
                Map.of("events", (double) events, "detection latency (s)", detectedAt),
                passed ? String.format(Locale.ROOT, "one event, %.0f ms after impact", detectedAt * 1000)
                       : events + " event(s), latency " + round(detectedAt) + " s");
    }

    /** Confidence must fall while vision is unavailable and recover when it returns. */
    private ScenarioResult confidenceTracksVisionAvailability() {
        SimulatedRobot sim = sim();
        PhysicsCore physics = core(sim);
        sim.command(new ChassisSpeeds(1.5, 0, 0));

        for (int i = 0; i < 25; i++) {
            sim.step();
            PhysicsSample sample = sim.sample();
            physics.observe(PoseObservation.of(sim.truePose(), sample.timestampSeconds(), "sim-cam"));
            physics.update(sample);
        }
        double withVision = physics.state().quality().confidence();

        for (int i = 0; i < 250; i++) {          // five seconds of nothing
            sim.step();
            physics.update(sim.sample());
        }
        double blind = physics.state().quality().confidence();

        for (int i = 0; i < 10; i++) {
            sim.step();
            PhysicsSample sample = sim.sample();
            sim.applyPerfectVisionCorrection();
            physics.observe(PoseObservation.of(sim.truePose(), sample.timestampSeconds(), "sim-cam"));
            physics.update(sample);
        }
        double recovered = physics.state().quality().confidence();

        boolean passed = withVision > 0.75 && blind < withVision - 0.2 && recovered > blind + 0.2;
        return new ScenarioResult("Confidence falls without vision and recovers with it", passed,
                Map.of("with vision", withVision, "after 5 s blind", blind, "recovered", recovered),
                String.format(Locale.ROOT, "%.2f -> %.2f -> %.2f", withVision, blind, recovered));
    }

    /**
     * The SOTF claim: predicting the robot's pose at release beats using the pose right now. Measured
     * against where the simulation actually was when the shot would have left.
     */
    private ScenarioResult releaseStatePredictionBeatsAimingAtNow() {
        SimulatedRobot sim = sim();
        PhysicsCore physics = core(sim);
        settle(sim, physics, new ChassisSpeeds(3.5, 0.8, 0), 40);

        int delayLoops = (int) Math.round(releaseDelaySeconds / 0.02);
        double predictedError = 0.0;
        double naiveError = 0.0;
        int shots = 0;

        for (int shot = 0; shot < 12; shot++) {
            // Vary the motion so the prediction is not trivially right.
            sim.command(new ChassisSpeeds(3.5 - shot * 0.15, 0.8, 0.3));
            sim.step();
            PhysicsSample sample = sim.sample();
            physics.update(sample);

            Translation2d predicted = physics.predictLaunchState().pose().getTranslation();
            Translation2d naive = physics.state().pose().getTranslation();
            Pose2d poseAtCommand = sim.odometryPose();

            for (int i = 0; i < delayLoops; i++) {
                sim.step();
                physics.update(sim.sample());
            }
            // Compare in odometry space, which is the frame the aiming solver actually works in.
            Translation2d actual = sim.odometryPose().getTranslation();

            predictedError += predicted.getDistance(actual);
            naiveError += naive.getDistance(actual);
            shots++;
            if (poseAtCommand == null) break;
        }

        double predictedMean = predictedError / shots;
        double naiveMean = naiveError / shots;
        double improvement = naiveMean > 0 ? 1.0 - predictedMean / naiveMean : 0.0;

        return new ScenarioResult("Release-state prediction beats aiming from the present",
                predictedMean < naiveMean,
                Map.of("predicted mean error (m)", predictedMean,
                        "naive mean error (m)", naiveMean,
                        "improvement", improvement),
                String.format(Locale.ROOT, "%.0f cm vs %.0f cm of lead error (%.0f%% better)",
                        predictedMean * 100, naiveMean * 100, improvement * 100));
    }

    /**
     * A systematically wrong wheel radius must show up as a persistent wheels-versus-IMU
     * disagreement, because that is the diagnostic that sends a team to
     * {@code WheelRadiusCalibration} instead of chasing the symptom.
     */
    private ScenarioResult wheelRadiusErrorSurfacesAsSensorDisagreement() {
        SimulatedRobot clean = SimulatedRobot.builder()
                .robotModel(robotModel).kinematics(kinematics, moduleCount)
                .accelerometerNoise(0.25).moduleSpeedNoise(0.015).seed(seed).build();
        SimulatedRobot miscalibrated = SimulatedRobot.builder()
                .robotModel(robotModel).kinematics(kinematics, moduleCount)
                .wheelRadiusError(1.06)
                .accelerometerNoise(0.25).moduleSpeedNoise(0.015).seed(seed).build();

        double cleanBias = runDisagreement(clean);
        double miscalibratedBias = runDisagreement(miscalibrated);

        boolean passed = miscalibratedBias > cleanBias * 2 && miscalibratedBias > 0.05;
        return new ScenarioResult("A wrong wheel radius shows up as sensor disagreement", passed,
                Map.of("correct radius (m/s)", cleanBias, "6% error (m/s)", miscalibratedBias),
                String.format(Locale.ROOT, "%.3f m/s clean vs %.3f m/s miscalibrated",
                        cleanBias, miscalibratedBias));
    }

    /** With no accelerometer at all, the estimate must stay sane rather than degrade unpredictably. */
    private ScenarioResult itDegradesSafelyWithoutAnAccelerometer() {
        SimulatedRobot sim = sim();
        PhysicsCore physics = core(sim);

        sim.command(new ChassisSpeeds(3.0, 0, 0));
        double worstError = 0.0;
        for (int i = 0; i < 100; i++) {
            sim.step();
            PhysicsSample full = sim.sample();
            physics.update(new PhysicsSample(full.timestampSeconds(), full.pose(),
                    full.robotRelativeSpeeds(), full.moduleStates(), null, full.yawRateRadPerSec()));
            worstError = Math.max(worstError,
                    Math.abs(physics.state().speedMetersPerSecond() - sim.trueSpeed()));
        }

        boolean passed = worstError < 0.5 && physics.analyze().disturbanceMpsSq() == 0.0;
        return new ScenarioResult("It degrades safely with no accelerometer", passed,
                Map.of("worst speed error (m/s)", worstError,
                        "disturbance reported", physics.analyze().disturbanceMpsSq()),
                String.format(Locale.ROOT,
                        "estimate stayed within %.2f m/s and diagnostics stayed quiet", worstError));
    }

    // ===========================================
    //                  HELPERS
    // ===========================================

    private SimulatedRobot sim() {
        return SimulatedRobot.builder()
                .robotModel(robotModel)
                .kinematics(kinematics, moduleCount)
                .withRealisticSensors()
                .seed(seed)
                .build();
    }

    private PhysicsCore core(SimulatedRobot sim) {
        return PhysicsCore.builder()
                .robotModel(robotModel)
                .kinematics(kinematics, moduleCount)
                .releaseDelaySeconds(releaseDelaySeconds)
                .clock(sim::timestamp)
                .withLogging(false)
                .withAlerts(false)
                .build();
    }

    /**
     * Ramp up to a speed the drivetrain can actually reach without slipping, then hold it. Commanding
     * a step change slips by construction - the controller asks for more acceleration than grip allows
     * - and a scenario that wants a clean baseline must not start by breaking traction.
     */
    private static void settleGently(SimulatedRobot sim, PhysicsCore physics, ChassisSpeeds target) {
        for (int i = 1; i <= 25; i++) {
            sim.command(new ChassisSpeeds(target.vxMetersPerSecond * i / 25.0,
                    target.vyMetersPerSecond * i / 25.0, target.omegaRadiansPerSecond * i / 25.0));
            sim.step();
            PhysicsSample sample = sim.sample();
            physics.observe(PoseObservation.of(sim.truePose(), sample.timestampSeconds(), "sim-cam"));
            physics.update(sample);
        }
        settle(sim, physics, target, 30);
    }

    /** Drive at a steady speed until the filters have settled. */
    private static void settle(SimulatedRobot sim, PhysicsCore physics, ChassisSpeeds speeds, int loops) {
        sim.command(speeds);
        for (int i = 0; i < loops; i++) {
            sim.step();
            PhysicsSample sample = sim.sample();
            physics.observe(PoseObservation.of(sim.truePose(), sample.timestampSeconds(), "sim-cam"));
            physics.update(sample);
        }
    }

    /** Mean absolute wheels-versus-IMU disagreement over a varied drive. */
    private double runDisagreement(SimulatedRobot sim) {
        PhysicsCore physics = core(sim);
        ModelResidualMonitor monitor = new ModelResidualMonitor("wheels vs IMU", 0.05);
        for (double speed : new double[]{1.0, 3.0, 4.5, 2.0}) {
            sim.command(new ChassisSpeeds(speed, 0, 0));
            for (int i = 0; i < 60; i++) {
                sim.step();
                physics.update(sim.sample());
                monitor.record(physics.state().speedMetersPerSecond() - sim.trueSpeed());
            }
        }
        return Math.abs(monitor.mean());
    }

    private static double squaredError(Translation2d estimate, Translation2d truth) {
        double dx = estimate.getX() - truth.getX();
        double dy = estimate.getY() - truth.getY();
        return dx * dx + dy * dy;
    }

    private static double round(double value) {
        return Math.round(value * 1000.0) / 1000.0;
    }

    // ===========================================
    //                  RESULTS
    // ===========================================

    /**
     * How one scenario went.
     *
     * @param name    what was being checked
     * @param passed  whether it met its criterion
     * @param metrics the numbers behind the verdict, so a near-miss is visible rather than just "fail"
     * @param detail  one line of plain language
     */
    public record ScenarioResult(String name, boolean passed, Map<String, Double> metrics, String detail) {

        /** Compact constructor: copies the metric map so the result is immutable. */
        public ScenarioResult {
            metrics = Map.copyOf(metrics);
        }

        /** One line: pass mark, name, and detail. */
        public String describe() {
            return (passed ? "PASS  " : "FAIL  ") + name + " - " + detail;
        }
    }

    /** Every scenario's result, and whether the whole run passed. */
    public record ValidationReport(List<ScenarioResult> results) {

        /** True only if every scenario passed. */
        public boolean allPassed() {
            return results.stream().allMatch(ScenarioResult::passed);
        }

        /** How many passed. */
        public long passedCount() {
            return results.stream().filter(ScenarioResult::passed).count();
        }

        /** The scenarios that failed, for a test to name in its assertion message. */
        public List<ScenarioResult> failures() {
            return results.stream().filter(r -> !r.passed()).toList();
        }

        /** A full multi-line report, with the metrics behind every verdict. */
        public String describe() {
            StringBuilder text = new StringBuilder();
            text.append(String.format(Locale.ROOT, "Physics Core simulation validation: %d/%d passed%n",
                    passedCount(), results.size()));
            for (ScenarioResult result : results) {
                text.append("  ").append(result.describe()).append('\n');
                Map<String, Double> ordered = new LinkedHashMap<>(result.metrics());
                for (Map.Entry<String, Double> metric : ordered.entrySet()) {
                    text.append(String.format(Locale.ROOT, "        %-32s %.4f%n",
                            metric.getKey(), metric.getValue()));
                }
            }
            text.append("\n  Simulation validates the estimator against a known truth. It does not\n");
            text.append("  validate the model against carpet - that still needs shadow mode.");
            return text.toString();
        }
    }

    /** Start building a validator. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link PhysicsValidator}. */
    public static final class Builder {
        private RobotModel robotModel;
        private SwerveDriveKinematics kinematics;
        private int moduleCount = 4;
        private long seed = 20260805L;
        private double releaseDelaySeconds = 0.12;

        /** Your robot's physical model — run the validation against the real thing, not a default. */
        public Builder robotModel(RobotModel robotModel) {
            this.robotModel = robotModel;
            return this;
        }

        /** Your drivetrain's kinematics. */
        public Builder kinematics(SwerveDriveKinematics kinematics, int moduleCount) {
            this.kinematics = kinematics;
            this.moduleCount = moduleCount;
            return this;
        }

        /** Four-module convenience. */
        public Builder kinematics(SwerveDriveKinematics kinematics) {
            return kinematics(kinematics, 4);
        }

        /** Seed for the simulation's noise. Same seed, same verdict. */
        public Builder seed(long seed) {
            this.seed = seed;
            return this;
        }

        /** Shooter release delay used by the prediction scenario, in seconds. Defaults to 0.12. */
        public Builder releaseDelaySeconds(double releaseDelaySeconds) {
            this.releaseDelaySeconds = releaseDelaySeconds;
            return this;
        }

        /** Validate and build. */
        public PhysicsValidator build() {
            if (robotModel == null) throw new IllegalStateException("a RobotModel is required");
            if (kinematics == null) throw new IllegalStateException("kinematics is required");
            return new PhysicsValidator(this);
        }
    }
}
