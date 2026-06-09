package frc.lib.catalyst.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.DoubleSupplier;

/**
 * Predicts an oncoming brownout and acts <em>before</em> the radio drops,
 * instead of after.
 *
 * <p>A battery isn't an ideal source — under load its terminal voltage sags
 * by {@code I × R_internal}. The roboRIO browns out (and the radio can reset)
 * when terminal voltage falls below ~6.8 V. This monitor estimates the
 * voltage the battery <em>would</em> sag to if you commanded full current
 * right now:
 *
 * <pre>
 *   predicted = measuredVoltage − (additionalCurrentHeadroom × R_internal)
 * </pre>
 *
 * <p>By default the monitor is <b>passive</b>: it estimates and publishes,
 * but takes no action. {@link #outputScale()} returns {@code 1.0} and
 * nothing trips. The two aggressive behaviours are each explicit opt-ins.
 *
 * <p>⚠️ <b>The aggressive behaviours are aggressive.</b> Output throttling
 * cuts your robot's power mid-match the instant predicted voltage dips, and
 * a preemptive {@code RobotSafety} trip can stop the robot outright. Battery
 * internal resistance varies a lot with age and temperature, so the
 * prediction is approximate — a too-high {@code warnVoltage} or too-low
 * {@code R} can throttle a perfectly healthy robot during a hard
 * acceleration. <b>Test thoroughly with realistic loads before enabling
 * either, and tune {@code R} from your own match logs (ΔV / ΔI).</b>
 *
 * <p>Call {@link #update()} once per loop. The estimate publishes to
 * {@code /Catalyst/Brownout/...} whether or not you act on it — so you can
 * watch the prediction for a few events before deciding to enable
 * throttling.
 *
 * <pre>{@code
 * // Passive (recommended starting point) — just watch /Catalyst/Brownout:
 * BrownoutMonitor brownout = BrownoutMonitor.builder()
 *     .totalCurrent(() -> pdh.getTotalCurrent())
 *     .batteryInternalResistance(0.020)
 *     .build();
 * brownout.update();   // outputScale() stays 1.0, nothing trips
 *
 * // Opt in to the aggressive behaviours only after testing:
 * BrownoutMonitor active = BrownoutMonitor.builder()
 *     .totalCurrent(() -> pdh.getTotalCurrent())
 *     .batteryInternalResistance(0.020)
 *     .warnVoltage(7.2).tripVoltage(6.9)
 *     .enableThrottling()      // outputScale() now ramps below warn
 *     .tripsRobotSafety(true)  // trips RobotSafety at tripVoltage
 *     .build();
 * }</pre>
 */
public final class BrownoutMonitor {

    private final DoubleSupplier totalCurrent;
    private final double rInternal;
    private final double warnVoltage;
    private final double tripVoltage;
    private final boolean throttlingEnabled;
    private final boolean tripsRobotSafety;
    private final double floorVoltage;

    private final NetworkTable nt;

    private double predictedVoltage = 12.0;
    private double outputScale = 1.0;
    private boolean tripped = false;

    private BrownoutMonitor(Builder b) {
        this.totalCurrent = b.totalCurrent;
        this.rInternal = b.rInternal;
        this.warnVoltage = b.warnVoltage;
        this.tripVoltage = b.tripVoltage;
        this.throttlingEnabled = b.throttlingEnabled;
        this.tripsRobotSafety = b.tripsRobotSafety;
        this.floorVoltage = b.floorVoltage;
        this.nt = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("Brownout");
    }

    /** Call once per loop. */
    public void update() {
        double v = RobotState.batteryVoltage();
        double i = safeCurrent();

        // The voltage we'd sag to if a bit more current were demanded — a
        // forward-looking margin rather than reacting only to the live dip.
        predictedVoltage = v - i * rInternal;

        // Output scale stays 1.0 (no action) unless throttling is explicitly
        // enabled. When enabled: linear throttle between warn and floor.
        if (!throttlingEnabled || predictedVoltage >= warnVoltage) {
            outputScale = 1.0;
        } else if (predictedVoltage <= floorVoltage) {
            outputScale = 0.0;
        } else {
            outputScale = (predictedVoltage - floorVoltage) / (warnVoltage - floorVoltage);
        }

        if (!tripped && predictedVoltage <= tripVoltage) {
            tripped = true;
            if (tripsRobotSafety) {
                RobotSafety.trip(String.format("brownout risk: predicted %.1fV at %.0fA", predictedVoltage, i));
            }
        } else if (tripped && predictedVoltage > warnVoltage) {
            tripped = false;
        }

        nt.getEntry("MeasuredVoltage").setDouble(v);
        nt.getEntry("TotalCurrent").setDouble(i);
        nt.getEntry("PredictedVoltage").setDouble(predictedVoltage);
        nt.getEntry("OutputScale").setDouble(outputScale);
        nt.getEntry("AtRisk").setBoolean(predictedVoltage <= warnVoltage);
    }

    /** Predicted sag voltage from the last {@link #update()}. */
    public double predictedVoltage() {
        return predictedVoltage;
    }

    /**
     * Suggested output multiplier [0, 1]. <b>Always 1.0 unless
     * {@code enableThrottling()} was set</b> — then it ramps toward 0 as the
     * predicted voltage approaches the floor. Multiply drive / mechanism
     * output by this to ease off proactively. ⚠️ aggressive; see class docs.
     */
    public double outputScale() {
        return outputScale;
    }

    /** True while predicted voltage is at or below the warn threshold. */
    public boolean atRisk() {
        return predictedVoltage <= warnVoltage;
    }

    private double safeCurrent() {
        try {
            double i = totalCurrent.getAsDouble();
            return Double.isFinite(i) && i > 0 ? i : 0;
        } catch (Throwable t) {
            return 0;
        }
    }

    // ============================================================
    //                        BUILDER
    // ============================================================

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private DoubleSupplier totalCurrent = () -> 0;
        private double rInternal = 0.020;   // ohms, healthy battery
        private double warnVoltage = 7.5;
        private double tripVoltage = 7.0;
        private boolean throttlingEnabled = false;
        private boolean tripsRobotSafety = false;
        private double floorVoltage = 6.8;  // roboRIO brownout floor

        /** Total robot current draw (A) — e.g. {@code () -> pdh.getTotalCurrent()}. */
        public Builder totalCurrent(DoubleSupplier totalCurrent) {
            this.totalCurrent = totalCurrent;
            return this;
        }

        /**
         * Battery internal resistance in ohms. ~0.015–0.025 for a healthy
         * FRC battery; rises as the battery ages. Estimate it from match logs
         * (ΔV / ΔI) for best accuracy. Default 0.020.
         */
        public Builder batteryInternalResistance(double ohms) {
            this.rInternal = Math.max(0, ohms);
            return this;
        }

        /** Predicted voltage at which to start easing off output. Default 7.5 V. */
        public Builder warnVoltage(double volts) {
            this.warnVoltage = volts;
            return this;
        }

        /** Predicted voltage at which to trip (if enabled). Default 7.0 V. */
        public Builder tripVoltage(double volts) {
            this.tripVoltage = volts;
            return this;
        }

        /** roboRIO brownout floor — output scale hits 0 here. Default 6.8 V. */
        public Builder floorVoltage(double volts) {
            this.floorVoltage = volts;
            return this;
        }

        /**
         * Opt in to output throttling — {@link #outputScale()} will ramp below
         * 1.0 as predicted voltage approaches the floor. ⚠️ aggressive: cuts
         * robot power mid-match. Off by default. Test before enabling.
         */
        public Builder enableThrottling() {
            this.throttlingEnabled = true;
            return this;
        }

        /**
         * Opt in to a preemptive {@link RobotSafety} trip at {@code tripVoltage}.
         * ⚠️ aggressive: can stop the robot. Off by default. Test before enabling.
         */
        public Builder tripsRobotSafety(boolean trips) {
            this.tripsRobotSafety = trips;
            return this;
        }

        public BrownoutMonitor build() {
            return new BrownoutMonitor(this);
        }
    }
}
