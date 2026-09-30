package frc.lib.catalyst.fieldlab;

import java.util.ArrayList;
import java.util.List;

/**
 * One measurement, start to finish: its steps, what it is for, and roughly how long it takes.
 *
 * <p>A procedure is the unit a campaign banks. {@link CampaignRunner} checkpoints after each one
 * completes, so a procedure is also the unit of work you can afford to lose — which is why they are
 * sized in minutes rather than in tens of minutes. A sweep that takes half an hour should be several
 * procedures, one per block of setpoints, so a dead battery costs one block instead of the sweep.
 *
 * <p>A procedure produces no result object. Everything it learns is logged, under
 * {@code FieldLab/<session>/<procedure id>/...}, and the fitting happens offline in
 * {@code tools/fieldlab.py}. That split is deliberate: a robot has twenty milliseconds and no
 * business running a least-squares fit, and a fit you can re-run against the same log with a
 * different model is worth more than a number the robot computed once and threw the data away for.
 *
 * @since 2.0.0
 */
public record TestProcedure(String id, String title, String why, List<TestStep> steps) {

    public TestProcedure {
        if (id == null || id.isBlank()) {
            throw new IllegalArgumentException("a procedure needs an id; it is the log namespace and the checkpoint key");
        }
        if (id.contains("/")) {
            throw new IllegalArgumentException("a procedure id becomes part of a NetworkTables key, so it cannot contain '/': " + id);
        }
        steps = List.copyOf(steps);
    }

    /** The sum of the steps' estimates. Optimistic by construction — nothing here counts a retry. */
    public double estimatedSeconds() {
        double total = 0;
        for (TestStep s : steps) {
            total += s.estimatedSeconds();
        }
        return total;
    }

    /** Whether any step needs a person, which is what decides if a campaign can be left alone. */
    public boolean needsAnOperator() {
        for (TestStep s : steps) {
            if (s instanceof TestStep.Measure || s instanceof TestStep.Confirm) {
                return true;
            }
        }
        return false;
    }

    /** Start building a procedure. */
    public static Builder builder(String id, String title) {
        return new Builder(id, title);
    }

    /** Builds a {@link TestProcedure}. */
    public static final class Builder {

        private final String id;
        private final String title;
        private String why = "";
        private final List<TestStep> steps = new ArrayList<>();

        private Builder(String id, String title) {
            this.id = id;
            this.title = title;
        }

        /**
         * What this procedure is for, in one or two sentences, written for whoever reads the report in
         * three months. Not optional in spirit: a measurement nobody can explain the purpose of is a
         * measurement nobody will act on.
         */
        public Builder why(String why) {
            this.why = why;
            return this;
        }

        /** Add a step. */
        public Builder step(TestStep step) {
            steps.add(step);
            return this;
        }

        /** Add several steps. */
        public Builder steps(TestStep... more) {
            for (TestStep s : more) {
                steps.add(s);
            }
            return this;
        }

        /** Require a battery above {@code volts} before going on. */
        public Builder needsBattery(double volts) {
            steps.add(TestStep.BatteryGate.at(volts));
            return this;
        }

        public TestProcedure build() {
            return new TestProcedure(id, title, why, steps);
        }
    }
}
