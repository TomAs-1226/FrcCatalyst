package frc.lib.catalyst.statemachine;

/**
 * What the state machine is doing right now.
 *
 * <p>Published as both {@code Phase} (the name) and {@code PhaseOrdinal} (for numeric plots), and
 * intended to be read as a second timeline row underneath {@code State} in AdvantageScope: the
 * state row says where the robot believes it is, the phase row says whether that belief is
 * currently being acted on, doubted, or was abandoned.
 *
 * @since 1.2.0
 */
public enum Phase {

    /** No transition in flight and nothing is being held — the machine is freshly built or seeded. */
    IDLE,

    /** A transition is in flight and at least one gating binding has not yet reached its goal. */
    MOVING,

    /** Every gating binding reports at-goal, but the state's settle window has not yet elapsed. */
    SETTLING,

    /** Arrived and confirmed. Goals stay applied so the mechanisms hold position. */
    HOLDING,

    /** The most recent request was refused. Lasts exactly one loop, then reverts. */
    REJECTED,

    /** A hop blew its deadline. Under {@code strict(false)} this is downgraded to {@link #HOLDING}. */
    TIMED_OUT,

    /** A deadline expired in strict mode, or a fault policy escalated. Requires {@code clearFault()}. */
    FAULTED,

    /** A bound mechanism was taken over by another command — usually a driver override. */
    YIELDED,

    /** The robot is disabled. Deadline accounting is frozen and no goals are issued. */
    DISABLED
}
