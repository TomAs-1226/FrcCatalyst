package frc.lib.catalyst.statemachine;

import java.util.List;

/**
 * The result of checking a state machine's configuration without building it.
 *
 * <p>Produced by {@code Builder.validate()}, which touches no hardware, no NetworkTables and no
 * command scheduler. That makes it the one piece of superstructure logic that can be exercised
 * from a JUnit test on a laptop — a configuration mistake that would otherwise crash at
 * {@code robotInit} on the field becomes a failing test at commit time.
 *
 * @param errors   problems that make the configuration unbuildable
 * @param warnings problems worth knowing about that do not block a build
 * @since 1.2.0
 */
public record ValidationReport(List<String> errors, List<String> warnings) {

    /** Defensively copies both lists. */
    public ValidationReport {
        errors = errors == null ? List.of() : List.copyOf(errors);
        warnings = warnings == null ? List.of() : List.copyOf(warnings);
    }

    /** True when there are no errors. Warnings do not make a report invalid. */
    public boolean ok() {
        return errors.isEmpty();
    }

    /**
     * Throw a {@link StateMachineConfigException} listing every error, or return quietly.
     *
     * @param machineName used in the exception message header
     */
    public void throwIfInvalid(String machineName) {
        if (!ok()) throw new StateMachineConfigException(machineName, errors);
    }

    @Override
    public String toString() {
        if (ok() && warnings.isEmpty()) return "OK";
        StringBuilder sb = new StringBuilder();
        sb.append(errors.size()).append(" error(s), ").append(warnings.size()).append(" warning(s)");
        for (String e : errors) sb.append("\n  ERROR   ").append(e);
        for (String w : warnings) sb.append("\n  WARNING ").append(w);
        return sb.toString();
    }
}
