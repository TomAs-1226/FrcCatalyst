package frc.lib.catalyst.physics.constraints;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import frc.lib.catalyst.physics.LocalizationQuality;
import frc.lib.catalyst.physics.PhysicsCore;
import frc.lib.catalyst.physics.PhysicsSample;
import frc.lib.catalyst.physics.model.RobotModel;
import frc.lib.catalyst.physics.model.StabilityModel;
import frc.lib.catalyst.physics.observation.PoseObservation;

/**
 * The speed scale a driver feels, driven with the inputs Catalyst X1 recorded on 2026-09-19, when the
 * old scale stepped its top speed 1.0 → 0.75 → 0.45 and back up to 30 times a minute ("it drives like
 * it is shifting gears").
 *
 * <p>Each scenario first shows that the input reproduces the old step — {@link #bandedScale} is the
 * 2.0.0-rc.1 {@code speedScale()} verbatim, applied to the same Physics Core — and then that the
 * scale the drivetrain gets now has no step in it. No HAL, no NetworkTables: an injected clock and
 * {@code update(PhysicsSample)}.
 */
class PhysicsConstraintsEasingTest {

    private static final double DT = 0.02;
    /** Default easing: 1.0/s in, so no loop may lower the scale by more than this. */
    private static final double MAX_FALL_PER_LOOP = 1.0 * DT + 1e-9;
    private static final double MAX_RISE_PER_LOOP = 0.5 * DT + 1e-9;

    // X1: a 28 x 26 in frame with the modules 2.625 in in from each edge.
    private static final double HALF_WHEELBASE = (28.0 - 2 * 2.625) * 0.0254 / 2;
    private static final double HALF_TRACK = (26.0 - 2 * 2.625) * 0.0254 / 2;
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(HALF_WHEELBASE, HALF_TRACK), new Translation2d(HALF_WHEELBASE, -HALF_TRACK),
            new Translation2d(-HALF_WHEELBASE, HALF_TRACK), new Translation2d(-HALF_WHEELBASE, -HALF_TRACK));

    private static RobotModel x1() {
        return RobotModel.builder()
                .massKg(45.0).footprintMeters(2 * HALF_TRACK, 2 * HALF_WHEELBASE)
                .wheelRadiusMeters(0.0508).centerOfMassHeightMeters(0.18)
                .coefficientOfFriction(1.1).build();
    }

    private static PhysicsCore core() {
        return PhysicsCore.builder()
                .robotModel(x1()).kinematics(KINEMATICS)
                .clock(() -> 0.0).withLogging(false).withAlerts(false).build();
    }

    private static PhysicsConstraints limits(PhysicsCore physics) {
        return PhysicsConstraints.builder()
                .physics(physics).stability(StabilityModel.ofChassisOnly(x1())).maxSpeedMps(5.24).build();
    }

    /** Straight ahead at {@code vx}, every wheel agreeing, the IMU reading {@code imuAccel} forward. */
    private static PhysicsSample drive(double t, double vx, double imuAccel) {
        ChassisVelocities speeds = new ChassisVelocities(vx, 0.0, 0.0);
        return new PhysicsSample(t, new Pose2d(vx * t, 0.0, new Rotation2d()), speeds,
                KINEMATICS.toSwerveModuleVelocities(speeds), new Translation2d(imuAccel, 0.0), 0.0);
    }

    /** As {@link #drive}, with every module reading {@code extra} m/s faster than the chassis. */
    private static PhysicsSample slipping(double t, double vx, double extra) {
        ChassisVelocities speeds = new ChassisVelocities(vx, 0.0, 0.0);
        SwerveModuleVelocity[] modules = KINEMATICS.toSwerveModuleVelocities(speeds);
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModuleVelocity(modules[i].velocity + extra, modules[i].angle);
        }
        return new PhysicsSample(t, new Pose2d(vx * t, 0.0, new Rotation2d()), speeds, modules,
                new Translation2d(), 0.0);
    }

    private static void fix(PhysicsCore physics, double t) {
        physics.observe(PoseObservation.of(physics.state().pose(), t, "limelight-ground"));
    }

    /** The 2.0.0-rc.1 speedScale(), banded on confidence, with its 0.25 default floor. */
    private static double bandedScale(PhysicsCore physics) {
        double scale = switch (physics.state().quality().level()) {
            case HIGH -> 1.0;
            case MODERATE -> 0.75;
            case LOW -> 0.45;
            case LOST -> 0.25;
        };
        double slip = physics.analyze().slipFactor();
        if (slip > 0.2) scale = Math.min(scale, 1.0 - 0.5 * Math.min(1.0, slip));
        return Math.max(0.25, Math.min(1.0, scale));
    }

    private static double largestFall(List<Double> series) {
        double worst = 0.0;
        for (int i = 1; i < series.size(); i++) worst = Math.max(worst, series.get(i - 1) - series.get(i));
        return worst;
    }

    private static double largestRise(List<Double> series) {
        double worst = 0.0;
        for (int i = 1; i < series.size(); i++) worst = Math.max(worst, series.get(i) - series.get(i - 1));
        return worst;
    }

    // ===========================================
    //   The two causes X1 recorded
    // ===========================================

    @Test
    void aCameraOffTheTagsNoLongerSlowsTheDriver() {
        // run-134138, 46.4-52.7 s: driving at 0.35 m/s with the tags out of view. The old scale went
        // 1.00 -> 0.75 in one loop 1.8 s after the last accepted frame and back to 1.00 on the next.
        PhysicsCore physics = core();
        PhysicsConstraints limits = limits(physics);
        List<Double> old = new ArrayList<>();
        List<Double> applied = new ArrayList<>();
        List<Double> confidence = new ArrayList<>();
        for (int i = 0; i <= 500; i++) {
            double t = i * DT;
            boolean tagsInView = t < 2.0 || t >= 8.0;
            if (tagsInView && i % 2 == 0) fix(physics, t);   // X1 accepted a frame about every other loop
            physics.update(drive(t, 0.35, 0.0));
            old.add(bandedScale(physics));
            applied.add(limits.speedScale());
            confidence.add(limits.confidenceScale());
        }

        assertTrue(largestFall(old) >= 0.25 - 1e-9 && largestRise(old) >= 0.25 - 1e-9,
                "the recorded input must reproduce the old one-loop 25% step down and back up");
        for (double scale : applied) assertEquals(1.0, scale, 1e-9, "the driver's speed must not move");
        assertTrue(confidence.stream().mapToDouble(Double::doubleValue).min().getAsDouble() < 0.9,
                "the pose really is less trustworthy, and confidenceScale says so");
        assertTrue(largestFall(confidence) <= MAX_FALL_PER_LOOP, "and it says so without a step");
        assertTrue(largestRise(confidence) <= MAX_RISE_PER_LOOP);
    }

    @Test
    void anAccelerationTheImuReadsDifferentlyNoLongerSlowsTheDriver() {
        // Every other X1 dip: vision fresh, the modules agreeing with each other, and a stick push that
        // ramps the wheels at 6-8 m/s^2 (median 6.1 at the 151 onsets) while the Pigeon - which reads
        // ~1.2 m/s^2 at rest - reports less. That disagreement is wheel distrust, and wheel distrust
        // was confidence, and confidence was the driver's top speed.
        PhysicsCore physics = core();
        PhysicsConstraints limits = limits(physics);
        List<Double> old = new ArrayList<>();
        List<Double> applied = new ArrayList<>();
        double v = 0.0;
        for (int i = 0; i <= 150; i++) {
            double t = i * DT;
            boolean pushing = t >= 1.0 && t < 1.26;
            if (pushing) v += 7.0 * DT;
            if (i % 2 == 0) fix(physics, t);
            physics.update(drive(t, v, (pushing ? 3.0 : 0.0) - 1.2));
            old.add(bandedScale(physics));
            applied.add(limits.speedScale());
        }

        assertTrue(old.stream().mapToDouble(Double::doubleValue).min().getAsDouble() <= 0.75,
                "the recorded input must reproduce the old step: min was "
                        + old.stream().mapToDouble(Double::doubleValue).min().getAsDouble());
        assertTrue(largestFall(old) >= 0.25 - 1e-9, "and in one loop");
        for (double scale : applied) assertEquals(1.0, scale, 1e-9, "the driver's speed must not move");
    }

    // ===========================================
    //   What is left in the speed scale is eased
    // ===========================================

    @Test
    void aSlipBlipAtTheStartOfAnAccelerationCostsAFewPercentNotALurch() {
        // Three loops of module disagreement, the length of X1's accel-onset slip readings.
        PhysicsCore physics = core();
        PhysicsConstraints limits = limits(physics);
        List<Double> old = new ArrayList<>();
        List<Double> applied = new ArrayList<>();
        for (int i = 0; i <= 200; i++) {
            double t = i * DT;
            if (i % 2 == 0) fix(physics, t);
            boolean blip = i >= 50 && i < 53;
            physics.update(blip ? slipping(t, 1.5, 1.0) : drive(t, 1.5, 0.0));
            old.add(bandedScale(physics));
            applied.add(limits.speedScale());
        }

        double oldMin = old.stream().mapToDouble(Double::doubleValue).min().getAsDouble();
        double newMin = applied.stream().mapToDouble(Double::doubleValue).min().getAsDouble();
        assertTrue(oldMin < 0.75, "the old scale dropped hard on three loops of slip: " + oldMin);
        assertTrue(newMin > 0.85, "eased, the same blip costs a few percent: " + newMin);
        assertTrue(largestFall(applied) <= MAX_FALL_PER_LOOP);
        assertTrue(largestRise(applied) <= MAX_RISE_PER_LOOP);
        assertEquals(1.0, applied.get(applied.size() - 1), 1e-9, "and it is all given back");
    }

    @Test
    void aSustainedSlipStillSlowsTheRobotThenHoldsBeforeEasingOut() {
        PhysicsCore physics = core();
        PhysicsConstraints limits = limits(physics);
        List<Double> applied = new ArrayList<>();
        double lastSlip = Double.NaN;
        double firstRise = Double.NaN;
        double reached = 1.0;
        for (int i = 0; i <= 250; i++) {
            double t = i * DT;
            if (i % 2 == 0) fix(physics, t);
            boolean slip = i >= 50 && i < 100;   // one second of every wheel spinning 1 m/s fast
            physics.update(slip ? slipping(t, 2.0, 1.0) : drive(t, 2.0, 0.0));
            double scale = limits.speedScale();
            applied.add(scale);
            if (slip) {
                lastSlip = t;
                reached = scale;
            } else if (!Double.isNaN(lastSlip) && Double.isNaN(firstRise)
                    && scale > applied.get(applied.size() - 2) + 1e-9) {
                firstRise = t;
            }
        }

        assertTrue(reached < 0.6, "a second of real slip reaches the limit (~0.5): " + reached);
        assertTrue(firstRise - lastSlip >= 0.5 - 1e-6,
                "the reduction is held for half a second after the slip clears, not given straight back: "
                        + (firstRise - lastSlip));
        assertTrue(largestFall(applied) <= MAX_FALL_PER_LOOP);
        assertTrue(largestRise(applied) <= MAX_RISE_PER_LOOP);
        assertEquals(1.0, applied.get(applied.size() - 1), 1e-9);
    }

    @Test
    void theSlipLimitHasHysteresis() {
        // Every module 0.15 m/s over settles the slip factor at 0.3; 0.075 over at 0.15.
        PhysicsCore physics = core();
        PhysicsConstraints limits = limits(physics);
        for (int i = 0; i <= 50; i++) {
            fix(physics, i * DT);
            physics.update(slipping(i * DT, 2.0, 0.15));
            limits.speedScale();
        }
        assertTrue(limits.targetSpeedScale() < 1.0, "0.3 slip engages the limit");
        for (int i = 51; i <= 100; i++) {
            fix(physics, i * DT);
            physics.update(slipping(i * DT, 2.0, 0.075));
            limits.speedScale();
        }
        assertEquals(0.15, physics.analyze().slipFactor(), 0.01);
        assertTrue(limits.targetSpeedScale() < 1.0, "0.15 slip, once engaged, stays engaged");

        PhysicsCore fresh = core();
        PhysicsConstraints freshLimits = limits(fresh);
        for (int i = 0; i <= 50; i++) {
            fix(fresh, i * DT);
            fresh.update(slipping(i * DT, 2.0, 0.075));
            freshLimits.speedScale();
        }
        assertEquals(1.0, freshLimits.targetSpeedScale(), 1e-9, "0.15 slip does not engage it");
    }

    // ===========================================
    //   Contract
    // ===========================================

    @Test
    void readingTheScaleSeveralTimesInALoopDoesNotAdvanceIt() {
        PhysicsCore physics = core();
        PhysicsConstraints limits = limits(physics);
        for (int i = 0; i <= 50; i++) {
            fix(physics, i * DT);
            physics.update(drive(i * DT, 2.0, 0.0));
            limits.speedScale();
        }
        fix(physics, 51 * DT);
        physics.update(slipping(51 * DT, 2.0, 1.0));
        double first = limits.speedScale();
        for (int k = 0; k < 20; k++) {
            assertEquals(first, limits.speedScale(), 0.0);
            limits.explain();
            limits.isCautionAdvised();
        }
    }

    @Test
    void explainNamesTheLocalisationWithoutApplyingIt() {
        PhysicsCore physics = core();
        physics.update(drive(0.0, 2.0, 0.0));   // never an absolute fix
        PhysicsConstraints limits = limits(physics);

        assertEquals(LocalizationQuality.Level.MODERATE, physics.state().quality().level());
        assertEquals(1.0, limits.speedScale(), 1e-9);
        assertFalse(limits.isCautionAdvised());
        assertTrue(limits.confidenceScale() < 1.0);
        assertTrue(limits.explain().contains("not applied to the speed scale"), limits.explain());
    }

    @Test
    void confidenceScaleIsOneWhenConfidenceScalingIsOff() {
        PhysicsCore physics = core();
        physics.update(drive(0.0, 2.0, 0.0));
        PhysicsConstraints limits = PhysicsConstraints.builder()
                .physics(physics).withoutConfidenceScaling().build();
        assertEquals(1.0, limits.confidenceScale(), 1e-9);
    }

    @Test
    void easingStartsOverAfterAGapOrAReset() {
        PhysicsConstraints.Easing easing = new PhysicsConstraints.Easing(1.0, 0.5, 0.5);
        assertEquals(0.5, easing.step(0.5, 10.0), 1e-9, "the first sample starts at the limit");
        assertEquals(0.5, easing.step(1.0, 10.02), 1e-9, "then holds");
        assertEquals(1.0, easing.step(1.0, 11.0), 1e-9, "a second with nothing sampled is not eased across");
        assertEquals(0.98, easing.step(0.2, 11.02), 1e-9, "falls at 1.0/s");
        assertEquals(0.2, easing.step(0.2, 3.0), 1e-9, "a clock that went backwards starts over");
        assertEquals(0.2, easing.step(0.9, 3.0), 1e-9, "the same instant again changes nothing");
    }

    @Test
    void easingRatesMustBePositive() {
        PhysicsCore physics = core();
        assertThrows(IllegalStateException.class,
                () -> PhysicsConstraints.builder().physics(physics).easing(0.0, 0.5, 0.5).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicsConstraints.builder().physics(physics).easing(1.0, -0.1, 0.5).build());
        assertThrows(IllegalStateException.class,
                () -> PhysicsConstraints.builder().physics(physics).easing(1.0, 0.5, 0.0).build());
    }
}
