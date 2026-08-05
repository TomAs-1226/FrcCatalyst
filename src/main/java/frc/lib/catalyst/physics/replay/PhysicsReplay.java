package frc.lib.catalyst.physics.replay;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Function;

import frc.lib.catalyst.physics.PhysicalRobotState;
import frc.lib.catalyst.physics.PhysicsAnalysis;
import frc.lib.catalyst.physics.PhysicsCore;
import frc.lib.catalyst.physics.PhysicsSample;

/**
 * Runs recorded samples back through a differently-configured Physics Core, so you can ask what a
 * change <em>would</em> have done.
 *
 * <p>This is the counterfactual tool the RFC asks for, and it exists because tuning an estimator on a
 * real robot is a miserable loop: change a threshold, find a field, recreate the situation, hope it
 * happens the same way. Recorded samples remove all of that. The same twelve seconds where the auto
 * went wrong can be replayed a hundred times against a hundred configurations, deterministically,
 * on a laptop.
 *
 * <pre>{@code
 * List<PhysicsSample> samples = // recorded during a match, or captured in a test
 *
 * Result strict  = PhysicsReplay.of(samples).run(() -> buildCore(0.3));   // slip threshold 0.3
 * Result relaxed = PhysicsReplay.of(samples).run(() -> buildCore(0.7));
 *
 * System.out.println(strict.compareTo(relaxed));
 * // "slip peaked at 0.91 vs 0.44; 3 collisions vs 1; mean confidence 0.62 vs 0.78"
 * }</pre>
 *
 * <p>Replay is exact rather than approximate: {@link PhysicsCore#update(PhysicsSample)} takes every
 * measurement as an argument and the clock is injectable, so feeding the same samples in the same
 * order produces the same numbers every time. There is no hidden state and nothing reads the wall
 * clock.
 *
 * <p>Samples can come from anywhere — a {@code .wpilog} decoded back into {@code PhysicsSample}s, a
 * simulation run, or a test that builds them by hand.
 *
 * @since 1.6.0
 */
public final class PhysicsReplay {

    private final List<PhysicsSample> samples;

    private PhysicsReplay(List<PhysicsSample> samples) {
        this.samples = List.copyOf(samples);
    }

    /** Wrap a recorded sequence of samples, in the order they were captured. */
    public static PhysicsReplay of(List<PhysicsSample> samples) {
        if (samples == null || samples.isEmpty()) {
            throw new IllegalArgumentException("at least one sample is required to replay");
        }
        return new PhysicsReplay(samples);
    }

    /** How many samples this replay covers. */
    public int sampleCount() {
        return samples.size();
    }

    /** How much time the recording spans, in seconds. */
    public double durationSeconds() {
        return samples.get(samples.size() - 1).timestampSeconds() - samples.get(0).timestampSeconds();
    }

    /**
     * Feed every sample through a freshly-built core and collect what happened.
     *
     * @param coreFactory builds the core to test. A factory rather than an instance, so each run
     *                    starts from a clean estimator and two runs cannot contaminate each other
     */
    public Result run(java.util.function.Supplier<PhysicsCore> coreFactory) {
        return run(coreFactory, sample -> sample);
    }

    /**
     * The same, with each sample transformed on the way in — for asking what would have happened with
     * a sensor that was not there, or one that was lying.
     *
     * <pre>{@code
     * // what if the accelerometer had been dead the whole time?
     * replay.run(this::buildCore, s -> new PhysicsSample(
     *     s.timestampSeconds(), s.pose(), s.robotRelativeSpeeds(),
     *     s.moduleStates(), null, s.yawRateRadPerSec()));
     * }</pre>
     */
    public Result run(java.util.function.Supplier<PhysicsCore> coreFactory,
                      Function<PhysicsSample, PhysicsSample> transform) {
        PhysicsCore core = coreFactory.get();
        List<PhysicalRobotState> states = new ArrayList<>(samples.size());
        List<PhysicsAnalysis> analyses = new ArrayList<>(samples.size());
        int collisions = 0;
        double previousCollisionAt = Double.NEGATIVE_INFINITY;

        for (PhysicsSample sample : samples) {
            states.add(core.update(transform.apply(sample)));
            PhysicsAnalysis analysis = core.analyze();
            analyses.add(analysis);
            // Count each collision once: lastCollision() keeps reporting the same event afterwards.
            if (analysis.lastCollision().isPresent()) {
                double at = analysis.lastCollision().get().timestampSeconds();
                if (at > previousCollisionAt) {
                    collisions++;
                    previousCollisionAt = at;
                }
            }
        }
        return new Result(List.copyOf(states), List.copyOf(analyses), collisions);
    }

    /**
     * What one replay produced.
     *
     * @param states     the state after each sample, in order
     * @param analyses   the analysis after each sample, in order
     * @param collisions how many distinct collisions were detected
     */
    public record Result(List<PhysicalRobotState> states, List<PhysicsAnalysis> analyses, int collisions) {

        /** Highest slip factor seen across the run. */
        public double peakSlipFactor() {
            return analyses.stream().mapToDouble(PhysicsAnalysis::peakSlip).max().orElse(0.0);
        }

        /** Mean confidence across the run — a single number for "how well did the estimate hold up". */
        public double meanConfidence() {
            return states.stream().mapToDouble(s -> s.quality().confidence()).average().orElse(0.0);
        }

        /** Lowest confidence reached. */
        public double minimumConfidence() {
            return states.stream().mapToDouble(s -> s.quality().confidence()).min().orElse(0.0);
        }

        /** Highest traction usage seen — above 1.0 means the drivetrain could not have been the cause. */
        public double peakTractionUsage() {
            return analyses.stream().mapToDouble(PhysicsAnalysis::tractionUsage).max().orElse(0.0);
        }

        /** Highest speed the fused estimate reported, in m/s. */
        public double peakSpeedMps() {
            return states.stream().mapToDouble(PhysicalRobotState::speedMetersPerSecond).max().orElse(0.0);
        }

        /** The final state of the run. */
        public PhysicalRobotState finalState() {
            return states.get(states.size() - 1);
        }

        /** One line summarising the run. */
        public String describe() {
            return String.format(Locale.ROOT,
                    "%d samples: peak slip %.2f, %d collision(s), confidence mean %.2f / min %.2f, "
                            + "peak traction %.2f",
                    states.size(), peakSlipFactor(), collisions, meanConfidence(),
                    minimumConfidence(), peakTractionUsage());
        }

        /** A side-by-side line against another run — the actual point of replaying twice. */
        public String compareTo(Result other) {
            return String.format(Locale.ROOT,
                    "peak slip %.2f vs %.2f; %d vs %d collision(s); mean confidence %.2f vs %.2f",
                    peakSlipFactor(), other.peakSlipFactor(),
                    collisions, other.collisions,
                    meanConfidence(), other.meanConfidence());
        }
    }
}
