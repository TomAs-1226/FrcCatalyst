/**
 * IO contracts and input snapshot types for Catalyst mechanisms.
 *
 * <p>Each mechanism in {@code frc.lib.catalyst.mechanisms} produces an
 * {@link frc.lib.catalyst.logging.CatalystInputs} snapshot every loop. The
 * snapshot is the single source of truth that:
 * <ul>
 *   <li>The mechanism reads from when deciding what to do</li>
 *   <li>The active {@link frc.lib.catalyst.logging.LogSink} records for
 *       dashboards, replay, or AdvantageKit-bridge consumption</li>
 *   <li>Replay tooling injects back via {@code fromLog} to deterministically
 *       reproduce a past match</li>
 * </ul>
 *
 * <p>The {@code *IO} interfaces declared in this package describe the
 * hardware contract for each mechanism — what control inputs the
 * mechanism can issue and how it pulls back sensor readings. Default
 * Phoenix 6 / WPILib simulation implementations are scheduled for v0.4.
 * In v0.3 the interfaces and input POJOs are stable; the implementations
 * users build against them are stable too.
 *
 * <p>Catalyst does not depend on AdvantageKit. See
 * {@code docs/advanced/logging.md} for the 10-line bridge that lets teams
 * forward every Catalyst input/output into an AdvantageKit pipeline
 * without Catalyst ever importing it.
 */
package frc.lib.catalyst.io;
