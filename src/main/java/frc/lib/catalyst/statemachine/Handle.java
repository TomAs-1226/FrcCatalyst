package frc.lib.catalyst.statemachine;

/**
 * An opaque, type-carrying token for one bound mechanism.
 *
 * <p>This is the small piece of machinery that makes the whole builder type-safe. Because
 * {@code bind(...)} returns a {@code Handle<LinearGoal>} and {@code StateSpec.set} is declared as
 * {@code <G> set(Handle<G>, G)}, writing
 *
 * <pre>{@code
 * .set(elevator, RotationalGoal.degrees(90))   // elevator is a Handle<LinearGoal>
 * }</pre>
 *
 * is a compile error rather than a robot that tries to drive an elevator to 90 metres. Handles are
 * compared by identity, so two bindings with the same key from different builders never collide.
 *
 * @param <G> the goal type of the bound mechanism
 * @since 1.2.0
 */
public final class Handle<G> {

    private final int index;
    private final String key;
    private final String goalTypeName;

    Handle(int index, String key, String goalTypeName) {
        this.index = index;
        this.key = key;
        this.goalTypeName = goalTypeName;
    }

    /** The binding key, as used in log paths. */
    public String key() {
        return key;
    }

    /** Dense index into the machine's binding array. */
    public int index() {
        return index;
    }

    /** Simple name of the goal type, for error messages and the {@code Graph/Bindings} log. */
    public String goalTypeName() {
        return goalTypeName;
    }

    @Override
    public String toString() {
        return key + ":" + goalTypeName;
    }

    @Override
    public boolean equals(Object o) {
        return this == o;
    }

    @Override
    public int hashCode() {
        return System.identityHashCode(this);
    }
}
