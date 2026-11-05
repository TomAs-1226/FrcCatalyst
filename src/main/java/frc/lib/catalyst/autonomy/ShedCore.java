package frc.lib.catalyst.autonomy;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Who gives up current when there is not enough, and how much they may safely give up.
 *
 * <h2>Shed only, never boost</h2>
 *
 * <p>Nothing here raises a limit above what the mechanism was built with. That is not timidity, it
 * is the one property that makes an automatic power system safe to leave running: a bug in the
 * arithmetic can make the robot weaker and cannot make it stronger than its author allowed. This
 * library has twice shipped a limit that could rise - an unbounded heading loop and an ignored speed
 * governor - and both spun a robot on a bench. A current allocator that can raise a limit is the
 * same shape with more energy behind it.
 *
 * <h2>Floors, because the obvious implementation drops things</h2>
 *
 * <p>An elevator holds its height with current. A wrist holds its angle with current. Shedding a
 * mechanism to a fraction of its limit does not make it politely slower - below the current its
 * gravity feedforward needs, it falls, and it falls at the moment the robot is already struggling.
 * So every claim carries a floor, this never sheds below it, and a claim whose floor is its whole
 * limit is simply not sheddable. Getting that wrong is worse than never shedding at all.
 *
 * <h2>Order</h2>
 *
 * <p>Lowest priority sheds first, and within a priority the largest consumer sheds first, because
 * taking 30 A from one mechanism disturbs the robot less than taking 5 A from six of them. Stops as
 * soon as the deficit is covered - a brownout is not a reason to slow down everything on the robot.
 *
 * <p>Pure and deterministic: no clock, no hardware, no state.
 *
 * @since 2.1.0
 */
public final class ShedCore {
    private ShedCore() {}

    /**
     * One mechanism's stake in the budget.
     *
     * @param name     for telemetry
     * @param priority higher is protected for longer; shed order is lowest first
     * @param nowAmps  what it is allowed to draw at the moment
     * @param floorAmps the least it can hold safely - gravity, grip, whatever it is doing. Never
     *                 shed below this. Equal to {@code nowAmps} means "not sheddable".
     */
    public record Claim(String name, int priority, double nowAmps, double floorAmps) {

        /** How much this claim could give up without falling over. */
        public double sheddable() {
            return Math.max(0.0, nowAmps - floorAmps);
        }
    }

    /**
     * One instruction. Always a reduction.
     *
     * @param name   which claim
     * @param toAmps the new limit; never below the claim's floor, never above its present value
     */
    public record Cut(String name, double toAmps) {}

    /**
     * @param cuts      what to shed, in the order decided
     * @param shedAmps  how much was found
     * @param shortfall what could not be found; above zero means the floors bind and something
     *                  else has to give - stop a mechanism outright, or accept the sag
     * @param explain   a sentence for the dashboard
     */
    public record Plan(List<Cut> cuts, double shedAmps, double shortfall, String explain) {

        /** True when the deficit was fully covered. */
        public boolean sufficient() {
            return shortfall <= 1e-9;
        }
    }

    /**
     * Find {@code deficitAmps} of current among the claims.
     *
     * @param claims      every mechanism's stake; not modified
     * @param deficitAmps how much is needed; zero or less returns an empty plan
     * @param minCutAmps  a cut smaller than this is not worth a CAN write and is skipped
     */
    public static Plan plan(List<Claim> claims, double deficitAmps, double minCutAmps) {
        if (deficitAmps <= 0 || claims == null || claims.isEmpty()) {
            return new Plan(List.of(), 0.0, Math.max(0.0, deficitAmps),
                    deficitAmps <= 0 ? "nothing to shed" : "no claims to shed from");
        }

        List<Claim> order = new ArrayList<>(claims);
        // Lowest priority first; within a priority, the biggest giver first, so the deficit is
        // covered in as few disturbances as possible. Name last, so the same robot in the same state
        // sheds the same things twice.
        order.sort(Comparator.comparingInt(Claim::priority)
                .thenComparing(Comparator.comparingDouble(Claim::sheddable).reversed())
                .thenComparing(Claim::name));

        List<Cut> cuts = new ArrayList<>();
        double found = 0.0;
        for (Claim c : order) {
            if (found >= deficitAmps) {
                break;
            }
            double available = c.sheddable();
            if (available < minCutAmps) {
                continue;
            }
            double take = Math.min(available, deficitAmps - found);
            if (take < minCutAmps) {
                continue;
            }
            cuts.add(new Cut(c.name(), c.nowAmps() - take));
            found += take;
        }

        double shortfall = Math.max(0.0, deficitAmps - found);
        String explain;
        if (cuts.isEmpty()) {
            explain = String.format("need %.0f A and nothing can give it up safely", deficitAmps);
        } else if (shortfall > 1e-9) {
            explain = String.format("shed %.0f A of %.0f from %d, still %.0f A short",
                    found, deficitAmps, cuts.size(), shortfall);
        } else {
            explain = String.format("shed %.0f A from %d mechanism%s",
                    found, cuts.size(), cuts.size() == 1 ? "" : "s");
        }
        return new Plan(List.copyOf(cuts), found, shortfall, explain);
    }
}
