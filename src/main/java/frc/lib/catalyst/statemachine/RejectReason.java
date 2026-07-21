package frc.lib.catalyst.statemachine;

/**
 * Why a transition request was refused.
 *
 * <p>Every rejection is logged with one of these plus a free-text detail naming the specific
 * guard, interlock or mechanism involved. This exists because the single worst failure mode of a
 * superstructure state machine is a button that silently does nothing: without a reason code, a
 * refused request and a broken binding look identical from the driver's seat.
 *
 * @since 1.2.0
 */
public enum RejectReason {

    /** No rejection — the request was accepted. */
    NONE,

    /** The named state is not a constant of this machine's enum (only reachable via the String API). */
    UNKNOWN_STATE,

    /** No edge exists from the current state to the target, and routing is {@link Routing#DIRECT_ONLY}. */
    NO_EDGE,

    /** Routing is {@link Routing#SHORTEST_PATH} but no legal path exists right now. */
    NO_ROUTE,

    /** An edge guard on the chosen path evaluated false. The detail carries the guard's reason string. */
    GUARD_BLOCKED,

    /** A global interlock is unsatisfied and blocks the target. The detail carries the interlock name. */
    INTERLOCK_BLOCKED,

    /** The target state's own entry guard evaluated false. */
    ENTRY_GUARD_BLOCKED,

    /** A gating binding of the target reports it has not been homed. The detail names the binding. */
    NOT_ZEROED,

    /** Already transitioning to exactly this target. Informational, not an error. */
    ALREADY_THERE,

    /** Reserved for machines configured to refuse concurrent requests rather than supersede. */
    BUSY,

    /** The machine is faulted and needs {@code clearFault()} before it will accept anything. */
    FAULTED,

    /** The machine is disabled. */
    DISABLED,

    /**
     * The machine has never confirmed a state, so it does not know where it is starting from.
     * Call {@code seed(state)} once at robot init. Requests targeting a declared recovery state
     * are exempt.
     */
    NOT_SEEDED
}
