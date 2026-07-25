package frc.lib.catalyst.statemachine;

import java.util.List;

/**
 * Thrown by {@code build()} when a state machine's configuration is invalid.
 *
 * <p>Carries <b>every</b> problem found, not just the first. That is a deliberate ergonomic
 * choice: a deploy cycle in a pit is the better part of a minute, so surfacing six typos in one
 * message instead of six consecutive crashes is worth the small extra work in the builder.
 *
 * @since 1.2.0
 */
public final class StateMachineConfigException extends RuntimeException {

    private static final long serialVersionUID = 1L;

    private final transient List<String> errors;

    /**
     * @param machineName the machine's name, used in the message header
     * @param errors      every problem found, in discovery order
     */
    public StateMachineConfigException(String machineName, List<String> errors) {
        super(render(machineName, errors));
        this.errors = List.copyOf(errors);
    }

    /** Every problem found, in discovery order. */
    public List<String> errors() {
        return errors;
    }

    private static String render(String machineName, List<String> errors) {
        StringBuilder sb = new StringBuilder();
        sb.append("State machine '").append(machineName).append("' has ")
          .append(errors.size()).append(errors.size() == 1 ? " problem:" : " problems:");
        for (int i = 0; i < errors.size(); i++) {
            sb.append("\n  ").append(i + 1).append(". ").append(errors.get(i));
        }
        return sb.toString();
    }
}
