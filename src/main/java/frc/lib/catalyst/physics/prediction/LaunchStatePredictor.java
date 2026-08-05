package frc.lib.catalyst.physics.prediction;

import frc.lib.catalyst.physics.PhysicalRobotState;

/**
 * Turns "when will the piece actually leave?" into a {@link LaunchState} the aiming solver can use.
 *
 * <p>Release is later than the trigger by a delay that is a property of the shooter, not of the
 * robot's motion: indexing the piece, recovering flywheel speed, travelling up the shooter. Measure
 * that delay once — a high-speed phone video against the logs is enough — and set it here. Anything
 * that varies shot to shot, such as waiting on a flywheel that has not recovered yet, is passed in
 * per call.
 *
 * <pre>{@code
 * LaunchStatePredictor launcher = new LaunchStatePredictor(0.12);   // 120 ms, measured
 *
 * LaunchState launch = launcher.predict(physics.state());
 * LaunchState later  = launcher.predict(physics.state(), flywheel.secondsToRecover());
 * }</pre>
 *
 * <p>Prediction uses a {@link StatePredictor} in constant-acceleration mode by default. Over the
 * 50-250 ms a release delay actually spans, the acceleration really is close to constant, and the
 * decaying model would under-predict the motion that has to be compensated for.
 *
 * @since 1.5.0
 */
public final class LaunchStatePredictor {

    private final StatePredictor predictor;
    private final double baseReleaseDelaySeconds;

    /**
     * @param baseReleaseDelaySeconds seconds between commanding a shot and the piece leaving the robot
     */
    public LaunchStatePredictor(double baseReleaseDelaySeconds) {
        this(baseReleaseDelaySeconds, StatePredictor.builder().constantAcceleration().maxHorizon(0.5).build());
    }

    /**
     * @param baseReleaseDelaySeconds seconds between commanding a shot and the piece leaving the robot
     * @param predictor               the motion model to propagate with
     */
    public LaunchStatePredictor(double baseReleaseDelaySeconds, StatePredictor predictor) {
        if (baseReleaseDelaySeconds < 0) {
            throw new IllegalArgumentException("baseReleaseDelaySeconds must be >= 0 (got "
                    + baseReleaseDelaySeconds + ")");
        }
        if (predictor == null) throw new IllegalArgumentException("predictor must not be null");
        this.baseReleaseDelaySeconds = baseReleaseDelaySeconds;
        this.predictor = predictor;
    }

    /** The robot's state at the end of the base release delay. */
    public LaunchState predict(PhysicalRobotState now) {
        return predict(now, 0.0);
    }

    /**
     * The robot's state at the end of the base release delay plus {@code additionalDelaySeconds} —
     * flywheel recovery, an indexer that has to run first, anything that varies shot to shot.
     */
    public LaunchState predict(PhysicalRobotState now, double additionalDelaySeconds) {
        double delay = baseReleaseDelaySeconds + Math.max(0.0, additionalDelaySeconds);
        PhysicalRobotState atRelease = predictor.predict(now, delay);
        return new LaunchState(
                atRelease.pose(),
                atRelease.fieldVelocity(),
                atRelease.timestampSeconds(),
                delay,
                atRelease.quality());
    }

    /** The configured shooter release delay, in seconds. */
    public double baseReleaseDelaySeconds() {
        return baseReleaseDelaySeconds;
    }
}
