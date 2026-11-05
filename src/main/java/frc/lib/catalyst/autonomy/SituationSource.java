package frc.lib.catalyst.autonomy;

/**
 * Where a {@link Situation} comes from, sampled at most once per loop.
 *
 * <p>Split from the snapshot so that every decision in the autonomy layer can be tested by handing
 * it a {@code Situation} literal, with no physics core, no HAL and no robot. The production
 * implementation is {@link PhysicsSituationSource}; a test uses {@code () -> mySituation}.
 *
 * @since 2.1.0
 */
@FunctionalInterface
public interface SituationSource {

    /**
     * The situation as of {@code nowSeconds}.
     *
     * <p>Implementations must be cheap and must never throw: this runs on the robot loop, and a
     * source that cannot answer returns a facet marked invalid rather than a guess or an exception.
     *
     * @param nowSeconds monotonic seconds, from the caller's clock so a test can drive it
     */
    Situation sample(double nowSeconds);

    /** A source that knows nothing. Useful as a default and in tests. */
    static SituationSource blind() {
        return Situation::blind;
    }

    /**
     * Wrap this source so it samples at most once per loop, returning the same snapshot to every
     * caller that asks with the same timestamp.
     *
     * <p>This is the point of the seam. Several consumers want the situation in one loop - the drive
     * governor, a co-pilot, a scored selector - and each used to ask the world its own questions at
     * its own moment, so they could disagree with each other within a single 20 ms window. Caching
     * makes them agree by construction and costs one comparison.
     */
    default SituationSource cachedPerLoop() {
        return new SituationSource() {
            private Situation cached;
            private double cachedAt = Double.NaN;

            @Override
            public Situation sample(double nowSeconds) {
                if (cached == null || nowSeconds != cachedAt) {
                    cached = SituationSource.this.sample(nowSeconds);
                    cachedAt = nowSeconds;
                }
                return cached;
            }
        };
    }
}
