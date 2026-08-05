package frc.lib.catalyst.physics.prediction;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.DoubleSupplier;

/**
 * Answers "what happens to the bus if I also do this?" before you do it.
 *
 * <p>Catalyst already has {@code BrownoutMonitor}, and this does not replace it. They answer different
 * questions and both are worth having:
 *
 * <ul>
 *   <li><b>{@code BrownoutMonitor}</b> watches the present. It reads the current the robot is drawing
 *       right now and backs off when the predicted sag gets dangerous. It is a reflex.</li>
 *   <li><b>{@code PowerPredictor}</b> is asked about the future. Given what the robot is drawing now
 *       and what an action <em>would</em> draw, it says whether the two fit — before the action is
 *       scheduled. It is a plan.</li>
 * </ul>
 *
 * <p>The physics is the same in both cases, {@code V = V_open − I·R}, which is why they agree. What
 * differs is when you ask. A robot that raises its elevator, spins its shooter, and drives away
 * simultaneously can brown out even though no single action would have; the reflex catches it a
 * moment too late, and the plan avoids it entirely.
 *
 * <pre>{@code
 * PowerPredictor power = PowerPredictor.builder()
 *     .presentCurrent(pdh::getTotalCurrent)
 *     .presentVoltage(RobotController::getBatteryVoltage)
 *     .internalResistance(0.020)          // or the measured value from BatteryResistanceIdentifier
 *     .minimumVoltage(7.5)
 *     .build();
 *
 * if (power.canSustain(60.0)) { elevator.raise().schedule(); }
 *
 * PowerPlan plan = power.plan(
 *     new PowerDemand("Elevator", 60),
 *     new PowerDemand("Shooter", 45),
 *     new PowerDemand("Intake", 25));
 * // plan.waves() → the groups that fit together, in order
 * }</pre>
 *
 * <p>The open-circuit voltage is inferred from the present reading rather than assumed:
 * {@code V_open = V_now + I_now·R}. That way the prediction follows the battery down over a match
 * instead of pretending it is always fresh.
 *
 * <p>The suppliers are read on demand, so a test drives it from plain values with no HAL.
 *
 * @since 1.6.0
 */
public final class PowerPredictor {

    private final DoubleSupplier presentCurrent;
    private final DoubleSupplier presentVoltage;
    private final double internalResistance;
    private final double minimumVoltage;

    private PowerPredictor(Builder builder) {
        this.presentCurrent = builder.presentCurrent;
        this.presentVoltage = builder.presentVoltage;
        this.internalResistance = builder.internalResistance;
        this.minimumVoltage = builder.minimumVoltage;
    }

    /** Current total draw, in amps. */
    public double presentCurrentAmps() {
        return presentCurrent.getAsDouble();
    }

    /** Current bus voltage, in volts. */
    public double presentVoltage() {
        return presentVoltage.getAsDouble();
    }

    /**
     * Open-circuit voltage inferred from the present reading, in volts — {@code V_now + I_now·R}.
     * This is the battery's true state of charge as best the robot can see it, and it sags over a
     * match the way the real one does.
     */
    public double openCircuitVolts() {
        return presentVoltage() + presentCurrentAmps() * internalResistance;
    }

    /** Bus voltage if {@code additionalAmps} were drawn on top of what is already flowing. */
    public double predictedVoltage(double additionalAmps) {
        return openCircuitVolts() - (presentCurrentAmps() + additionalAmps) * internalResistance;
    }

    /** True when drawing {@code additionalAmps} more would keep the bus above the configured floor. */
    public boolean canSustain(double additionalAmps) {
        return predictedVoltage(additionalAmps) >= minimumVoltage;
    }

    /**
     * How many more amps the robot can draw before the bus reaches the floor. Negative when it is
     * already past it — which is a real answer, not an error, and tells you how much to shed.
     */
    public double headroomAmps() {
        if (internalResistance <= 0) return Double.POSITIVE_INFINITY;
        return (openCircuitVolts() - minimumVoltage) / internalResistance - presentCurrentAmps();
    }

    /** The voltage floor this predictor plans against. */
    public double minimumVoltage() {
        return minimumVoltage;
    }

    /** The internal resistance in use, in ohms. */
    public double internalResistanceOhms() {
        return internalResistance;
    }

    /**
     * Group demands into the fewest waves that each fit within the present headroom.
     *
     * <p>Greedy and deterministic: demands are taken in the order given and packed into the current
     * wave until the next one would not fit, then a new wave starts. Taking them in the caller's order
     * matters — it means the sequencing respects the priority you already decided, rather than
     * silently reordering the robot's behaviour to optimise a number.
     *
     * <p>A single demand larger than the whole headroom gets a wave to itself and is flagged, because
     * no amount of sequencing makes it fit.
     */
    public PowerPlan plan(PowerDemand... demands) {
        double headroom = headroomAmps();
        List<List<PowerDemand>> waves = new ArrayList<>();
        List<String> oversized = new ArrayList<>();
        List<PowerDemand> current = new ArrayList<>();
        double currentTotal = 0.0;

        for (PowerDemand demand : demands) {
            if (demand.amps() > headroom) {
                if (!current.isEmpty()) {
                    waves.add(List.copyOf(current));
                    current.clear();
                    currentTotal = 0.0;
                }
                waves.add(List.of(demand));
                oversized.add(demand.name());
                continue;
            }
            if (currentTotal + demand.amps() > headroom && !current.isEmpty()) {
                waves.add(List.copyOf(current));
                current.clear();
                currentTotal = 0.0;
            }
            current.add(demand);
            currentTotal += demand.amps();
        }
        if (!current.isEmpty()) waves.add(List.copyOf(current));

        return new PowerPlan(List.copyOf(waves), List.copyOf(oversized), headroom, minimumVoltage);
    }

    /** One line naming the present state and the headroom. */
    public String describe() {
        return String.format(Locale.ROOT,
                "PowerPredictor[%.1f V at %.0f A, open-circuit %.2f V, %.0f A headroom to %.1f V]",
                presentVoltage(), presentCurrentAmps(), openCircuitVolts(), headroomAmps(), minimumVoltage);
    }

    /**
     * One thing that wants power.
     *
     * @param name what it is, for the plan's description
     * @param amps how much it draws while it runs
     */
    public record PowerDemand(String name, double amps) {

        /** Compact constructor: rejects a negative draw. */
        public PowerDemand {
            if (amps < 0) {
                throw new IllegalArgumentException("amps must be >= 0 for '" + name + "' (got " + amps + ")");
            }
        }
    }

    /**
     * How a set of demands can be sequenced so the bus survives.
     *
     * @param waves          groups that can run together, in the order they should run
     * @param oversized      demands that exceed the whole headroom on their own — sequencing cannot
     *                       help these, they need a lower current limit or a better battery
     * @param headroomAmps   the headroom the plan was built against
     * @param minimumVoltage the floor being protected
     */
    public record PowerPlan(List<List<PowerDemand>> waves, List<String> oversized,
                            double headroomAmps, double minimumVoltage) {

        /** True when everything fits at once — no sequencing needed. */
        public boolean fitsInOneWave() {
            return waves.size() <= 1 && oversized.isEmpty();
        }

        /** How many sequential steps the plan needs. */
        public int waveCount() {
            return waves.size();
        }

        /** A short summary of the sequencing, suitable for a dashboard string. */
        public String describe() {
            if (waves.isEmpty()) return "nothing to sequence";
            StringBuilder text = new StringBuilder();
            for (int i = 0; i < waves.size(); i++) {
                if (i > 0) text.append(" then ");
                List<String> names = waves.get(i).stream().map(PowerDemand::name).toList();
                text.append(String.join(" + ", names));
            }
            if (!oversized.isEmpty()) {
                text.append(String.format(Locale.ROOT, "  (%s exceed%s the %.0f A headroom alone)",
                        String.join(", ", oversized), oversized.size() == 1 ? "s" : "", headroomAmps));
            }
            return text.toString();
        }
    }

    /** Start building a predictor. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link PowerPredictor}. */
    public static final class Builder {
        private DoubleSupplier presentCurrent;
        private DoubleSupplier presentVoltage;
        private double internalResistance = 0.020;
        private double minimumVoltage = 7.5;

        /** Where the robot's total current draw comes from, e.g. {@code pdh::getTotalCurrent}. Required. */
        public Builder presentCurrent(DoubleSupplier presentCurrent) {
            this.presentCurrent = presentCurrent;
            return this;
        }

        /** Where the bus voltage comes from, e.g. {@code RobotController::getBatteryVoltage}. Required. */
        public Builder presentVoltage(DoubleSupplier presentVoltage) {
            this.presentVoltage = presentVoltage;
            return this;
        }

        /**
         * Battery internal resistance in ohms. Defaults to 0.020, the usual FRC assumption. Real
         * batteries vary by a factor of two, so prefer the measured value from
         * {@link frc.lib.catalyst.physics.estimation.BatteryResistanceIdentifier}.
         */
        public Builder internalResistance(double internalResistance) {
            this.internalResistance = internalResistance;
            return this;
        }

        /**
         * The bus voltage to stay above, in volts. Defaults to 7.5 — above the roboRIO's 6.8&nbsp;V
         * brownout threshold, leaving room for the sag a plan cannot foresee.
         */
        public Builder minimumVoltage(double minimumVoltage) {
            this.minimumVoltage = minimumVoltage;
            return this;
        }

        /** Validate and build. */
        public PowerPredictor build() {
            if (presentCurrent == null || presentVoltage == null) {
                throw new IllegalStateException("presentCurrent and presentVoltage are both required - "
                        + "the open-circuit voltage is inferred from them");
            }
            if (!(internalResistance > 0)) {
                throw new IllegalStateException("internalResistance must be > 0 (got "
                        + internalResistance + ")");
            }
            if (!(minimumVoltage > 0)) {
                throw new IllegalStateException("minimumVoltage must be > 0 (got " + minimumVoltage + ")");
            }
            return new PowerPredictor(this);
        }
    }
}
