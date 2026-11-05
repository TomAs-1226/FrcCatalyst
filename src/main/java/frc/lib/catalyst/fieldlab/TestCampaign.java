package frc.lib.catalyst.fieldlab;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * An ordered set of {@link TestProcedure}s: a session's worth of field time, written down.
 *
 * <h2>What this is for</h2>
 *
 * <p>A team gets a field for an afternoon perhaps twice a season, and what it does with that time
 * decides how good the robot is for the rest of it. What usually happens is that the time goes on
 * driving practice and on whatever broke, and the measurements that only a real field can give —
 * odometry over twelve metres, a shot from the far side, where the vision pose actually is — are
 * taken ad hoc, by hand, into a notebook, or not at all.
 *
 * <p>A campaign is that afternoon planned in advance and run by the robot. It knows how long it
 * expects to take, it banks each procedure as it finishes, it waits for a person when it needs one,
 * and it stops rather than collect junk on a flat battery. The output is one log file and a report.
 *
 * <h2>Order matters</h2>
 *
 * <p>Put the procedures whose results everything else depends on first — wheel radius before odometry
 * drift, feedforward before path fidelity — because the campaign will be cut short. It always is. A
 * campaign is a priority list as much as a plan, and the right way to read "we only got through six
 * of the nine" is that the three that were dropped were the three you chose to risk.
 *
 * @since 2.0.0
 */
public record TestCampaign(String name, List<TestProcedure> procedures) {

    public TestCampaign {
        procedures = List.copyOf(procedures);
        Set<String> seen = new LinkedHashSet<>();
        for (TestProcedure p : procedures) {
            if (!seen.add(p.id())) {
                throw new IllegalArgumentException(
                        "two procedures share the id '" + p.id() + "'; ids are the checkpoint keys, so a"
                                + " duplicate would make the campaign resume into the wrong procedure");
            }
        }
    }

    /** The sum of every procedure's estimate. */
    public double estimatedSeconds() {
        double total = 0;
        for (TestProcedure p : procedures) {
            total += p.estimatedSeconds();
        }
        return total;
    }

    /** The estimate, as a human string: {@code "1 h 12 min"}. */
    public String estimate() {
        int seconds = (int) Math.round(estimatedSeconds());
        int hours = seconds / 3600;
        int minutes = (seconds % 3600) / 60;
        if (hours > 0) {
            return hours + " h " + minutes + " min";
        }
        return minutes + " min";
    }

    /** The procedure with this id, if the campaign has one. */
    public Optional<TestProcedure> byId(String id) {
        for (TestProcedure p : procedures) {
            if (p.id().equals(id)) {
                return Optional.of(p);
            }
        }
        return Optional.empty();
    }

    /** How many procedures need a person present. */
    public long operatorProcedures() {
        return procedures.stream().filter(TestProcedure::needsAnOperator).count();
    }

    /** Start building a campaign. */
    public static Builder builder(String name) {
        return new Builder(name);
    }

    /** Builds a {@link TestCampaign}. */
    public static final class Builder {

        private final String name;
        private final List<TestProcedure> procedures = new ArrayList<>();

        private Builder(String name) {
            this.name = name;
        }

        /** Append a procedure. Order is priority order; see the class javadoc. */
        public Builder add(TestProcedure procedure) {
            procedures.add(procedure);
            return this;
        }

        /** Append several procedures. */
        public Builder add(TestProcedure... more) {
            for (TestProcedure p : more) {
                procedures.add(p);
            }
            return this;
        }

        public TestCampaign build() {
            return new TestCampaign(name, procedures);
        }
    }
}
