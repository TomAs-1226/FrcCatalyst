package frc.lib.catalyst.fieldlab;

import org.wpilib.command3.Command;

import java.util.function.Supplier;

/**
 * One step of a {@link TestProcedure}: something the robot does, or something a person does.
 *
 * <h2>Why a person is part of the type</h2>
 *
 * <p>The numbers a robot most needs are the ones it cannot measure. Wheel radius it can infer from a
 * gyro; odometry drift it cannot, because the drift is the difference between where it believes it is
 * and where it actually is, and only a tape measure knows the second one. The same goes for where a
 * shot landed, whether a mechanism reached its real hard stop, and where the robot was parked. A test
 * framework that can only run robot code collects the cheap half of the data and leaves the expensive
 * half to a clipboard, which is where it is lost.
 *
 * <p>So a human step is a first-class step, and a campaign that waits two minutes for a tape measure
 * is not stalled — it is running. {@link Measure} takes a number, {@link Confirm} takes an
 * acknowledgement, and both arrive over the tunables path the dashboards can already write, because a
 * measurement is data and not a command. Nothing here gives a dashboard a way to make the robot move;
 * see {@link CampaignRunner} for why that matters.
 *
 * @since 2.0.0
 */
public sealed interface TestStep {

    /** What to show the operator while this step is the current one. */
    String label();

    /** Roughly how long this step takes, for the campaign's time estimate. Never exact. */
    double estimatedSeconds();

    /**
     * The robot does something, and the step ends when the command does.
     *
     * <p>The command is a {@link Supplier} rather than a command, because a campaign is built once and
     * may be run more than once — after a battery swap, or on the second half of a field slot — and a
     * command instance that has already run is spent. Building it at the moment it is needed also lets
     * a step read a measurement an earlier step took.
     *
     * @param label what the operator sees
     * @param command built fresh each time the step runs
     * @param timeoutSeconds a hard cap, so one wedged step cannot eat the whole field slot; the step
     *     is recorded as timed out rather than failing the campaign
     * @param estimatedSeconds for the time estimate
     */
    record Act(String label, Supplier<Command> command, double timeoutSeconds, double estimatedSeconds)
            implements TestStep {

        /** An action whose estimate is its timeout. */
        public static Act of(String label, Supplier<Command> command, double timeoutSeconds) {
            return new Act(label, command, timeoutSeconds, timeoutSeconds);
        }
    }

    /**
     * A person measures something and types the number in.
     *
     * <p>The value lands under {@code /Catalyst/Tuning/FieldLab/Value} and is committed by bumping
     * {@code /Catalyst/Tuning/FieldLab/Advance} — both ordinary tunables, so Catalyst Console and
     * Catalyst Tab can already write them with no new API and no new permission. {@code min} and
     * {@code max} are a sanity range, not a constraint: a value outside it is still recorded, with a
     * note, because a surprising measurement is usually the interesting one and silently rejecting it
     * is how a real finding gets thrown away.
     *
     * @param label the instruction, e.g. "measure from the tape to the front bumper, in metres"
     * @param key the name the value is recorded under, within this procedure
     * @param unit for the report; free text
     * @param min low end of the plausible range, or {@link Double#NaN} for no opinion
     * @param max high end, or {@link Double#NaN}
     */
    record Measure(String label, String key, String unit, double min, double max, double estimatedSeconds)
            implements TestStep {

        /** A measurement with a plausible range and a 60-second estimate. */
        public static Measure of(String label, String key, String unit, double min, double max) {
            return new Measure(label, key, unit, min, max, 60.0);
        }

        /** A measurement with no plausible range declared. */
        public static Measure of(String label, String key, String unit) {
            return new Measure(label, key, unit, Double.NaN, Double.NaN, 60.0);
        }

        /** Whether {@code value} falls inside the declared plausible range. */
        public boolean plausible(double value) {
            if (!Double.isNaN(min) && value < min) {
                return false;
            }
            return Double.isNaN(max) || !(value > max);
        }
    }

    /**
     * A person does something physical and acknowledges when it is done — place the robot on a line,
     * put a bumper against the wall, clear the field, swap a battery.
     *
     * <p>Acknowledged by bumping {@code /Catalyst/Tuning/FieldLab/Advance}, the same control
     * {@link Measure} uses, so an operator learns one gesture rather than two.
     */
    record Confirm(String label, double estimatedSeconds) implements TestStep {

        /** A confirmation with a 30-second estimate. */
        public static Confirm of(String label) {
            return new Confirm(label, 30.0);
        }
    }

    /**
     * Wait, doing nothing, so a measurement is not taken while something is still moving or still hot.
     *
     * <p>Distinct from an {@link Act} that waits, because a settle is not a robot action and should
     * read as dead time in the report rather than as a step that did something.
     */
    record Settle(String label, double seconds) implements TestStep {

        @Override
        public double estimatedSeconds() {
            return seconds;
        }

        /** A settle of {@code seconds}. */
        public static Settle of(String label, double seconds) {
            return new Settle(label, seconds);
        }
    }

    /**
     * Refuse to go on below a battery voltage, and resume once it is back.
     *
     * <p>This is the step that makes an hour of field time work. Characterisation data taken on a
     * flat battery is not slightly worse, it is wrong in a direction that looks like a real result:
     * kV comes out high, a flywheel never reaches its setpoint, a slip-current sweep slips early.
     * Worse, the robot browns out mid-procedure and the run is a write-off discovered later, at the
     * shop, when the field is gone.
     *
     * <p>So a gate goes in front of every procedure whose numbers depend on supply voltage. The
     * campaign pauses, says which battery it wants, and carries on from exactly there when one
     * arrives — it does not restart, and it does not re-run the procedures already banked.
     *
     * @param minVolts the floor to start or continue at
     * @param resumeVolts the voltage that clears the pause; above {@code minVolts} on purpose, so a
     *     pack that sags to the floor under load does not flap the gate open and shut
     */
    record BatteryGate(double minVolts, double resumeVolts) implements TestStep {

        /** A gate at {@code minVolts}, clearing 0.3 V above it. */
        public static BatteryGate at(double minVolts) {
            return new BatteryGate(minVolts, minVolts + 0.3);
        }

        @Override
        public String label() {
            return String.format("waiting for a battery above %.1f V", resumeVolts);
        }

        @Override
        public double estimatedSeconds() {
            // A gate that passes costs nothing; one that does not costs however long a battery swap
            // takes, which is not the campaign's business to predict. Zero, so the estimate stays an
            // estimate of the work rather than of the pit crew.
            return 0.0;
        }
    }
}
