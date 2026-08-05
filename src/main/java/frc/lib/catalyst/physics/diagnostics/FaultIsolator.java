package frc.lib.catalyst.physics.diagnostics;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.Consumer;

/**
 * Ranks likely causes when several models disagree with reality at once.
 *
 * <p>One biased residual tells you something is wrong. Which residuals are biased <em>together</em>
 * tells you what. A slipping front-left wheel and a mis-calibrated front-left wheel radius both make
 * that module's speed residual go one way — but slip also shows up as the wheels disagreeing with the
 * IMU, and a radius error does not, because a wheel that is rolling correctly at the wrong assumed
 * size still agrees about acceleration. The pattern separates them.
 *
 * <p>You describe each candidate cause by the residuals it should disturb and the ones it should
 * leave alone. Scoring rewards the first and penalises the second:
 *
 * <pre>
 * score = ( Σ weightᵢ · normalizedBiasᵢ  −  Σ penaltyⱼ · normalizedBiasⱼ ) / Σ weightᵢ
 *           over residuals this cause    over residuals it should not
 *           should disturb               have disturbed
 * </pre>
 *
 * <h2>These are diagnostic scores, not probabilities</h2>
 * A score of 0.9 does not mean "90% likely". It means this cause explains the observed pattern
 * roughly nine tenths as well as a textbook instance of it would. Nothing here has been validated
 * against a population of real faults, and calling these probabilities would imply a rigour that does
 * not exist. They are for <em>ranking</em> — for pointing a student at the right corner of the robot
 * first — and the RFC is explicit that they stay honest scores until someone does the statistics.
 *
 * <pre>{@code
 * FaultIsolator isolator = FaultIsolator.builder()
 *     .monitor(frontLeftSpeedResidual)
 *     .monitor(wheelsVsImuResidual)
 *     .candidate("Front-left wheel slip", c -> c
 *         .expects("FL wheel speed", 1.0)
 *         .expects("wheels vs IMU", 0.8))
 *     .candidate("Front-left wheel radius calibration", c -> c
 *         .expects("FL wheel speed", 1.0)
 *         .expectsQuiet("wheels vs IMU", 1.0))
 *     .build();
 *
 * for (Finding f : isolator.rank()) System.out.println(f.describe());
 * }</pre>
 *
 * <p>Pure arithmetic over the monitors you hand it — no hardware, no HAL.
 *
 * @since 1.6.0
 */
public final class FaultIsolator {

    private final Map<String, ModelResidualMonitor> monitors;
    private final List<Candidate> candidates;

    private FaultIsolator(Builder builder) {
        this.monitors = new LinkedHashMap<>(builder.monitors);
        this.candidates = List.copyOf(builder.candidates);
    }

    /**
     * Every candidate cause, scored and ordered best-explanation-first. Causes that explain nothing
     * are dropped rather than listed at zero, so an empty list means "no model is currently
     * disagreeing with reality" — which is the normal state and worth being able to see.
     */
    public List<Finding> rank() {
        List<Finding> findings = new ArrayList<>(candidates.size());
        for (Candidate candidate : candidates) {
            Finding finding = score(candidate);
            if (finding.score() > 0) findings.add(finding);
        }
        findings.sort(Comparator.comparingDouble(Finding::score).reversed());
        return findings;
    }

    /** The best explanation, or empty when nothing is currently anomalous. */
    public java.util.Optional<Finding> mostLikely() {
        List<Finding> ranked = rank();
        return ranked.isEmpty() ? java.util.Optional.empty() : java.util.Optional.of(ranked.get(0));
    }

    /** True when at least one residual this isolator watches is biased. */
    public boolean hasAnomaly() {
        return monitors.values().stream().anyMatch(ModelResidualMonitor::isBiased);
    }

    /** The residual monitors this isolator reads, by name. */
    public Map<String, ModelResidualMonitor> monitors() {
        return Map.copyOf(monitors);
    }

    /** A multi-line summary: the ranked causes, or a line saying everything looks nominal. */
    public String describe() {
        List<Finding> ranked = rank();
        if (ranked.isEmpty()) return "no model residuals are biased - nothing to isolate";
        StringBuilder text = new StringBuilder();
        for (Finding finding : ranked) {
            if (text.length() > 0) text.append('\n');
            text.append(finding.describe());
        }
        return text.toString();
    }

    private Finding score(Candidate candidate) {
        double positive = 0.0;
        double totalWeight = 0.0;
        double penalty = 0.0;
        List<String> supporting = new ArrayList<>();
        List<String> contradicting = new ArrayList<>();

        for (Map.Entry<String, Double> expectation : candidate.expected.entrySet()) {
            ModelResidualMonitor monitor = monitors.get(expectation.getKey());
            double weight = expectation.getValue();
            totalWeight += weight;
            if (monitor != null && monitor.isBiased()) {
                // Cap each contribution at its weight so one wildly off residual cannot carry a cause
                // that the rest of the pattern contradicts.
                positive += weight * Math.min(1.0, monitor.normalizedBias());
                supporting.add(monitor.name());
            }
        }

        for (Map.Entry<String, Double> quiet : candidate.expectedQuiet.entrySet()) {
            ModelResidualMonitor monitor = monitors.get(quiet.getKey());
            if (monitor != null && monitor.isBiased()) {
                penalty += quiet.getValue() * Math.min(1.0, monitor.normalizedBias());
                contradicting.add(monitor.name());
            }
        }

        double score = totalWeight > 0 ? Math.max(0.0, (positive - penalty) / totalWeight) : 0.0;
        return new Finding(candidate.name, score, List.copyOf(supporting), List.copyOf(contradicting));
    }

    /**
     * One candidate cause and how well it explains what is currently being observed.
     *
     * @param cause         the candidate, as named when it was registered
     * @param score         diagnostic score from 0 upward — a ranking aid, <b>not</b> a probability
     * @param supporting    residuals that are biased as this cause predicts
     * @param contradicting residuals this cause said should stay quiet, but did not
     */
    public record Finding(String cause, double score, List<String> supporting, List<String> contradicting) {

        /** True for a cause that explains the pattern well and has nothing arguing against it. */
        public boolean isStrong() {
            return score >= 0.7 && contradicting.isEmpty();
        }

        /** One line naming the cause, its score, and what is arguing for and against it. */
        public String describe() {
            StringBuilder text = new StringBuilder(String.format(Locale.ROOT,
                    "%s (score %.2f)", cause, score));
            if (!supporting.isEmpty()) text.append(" - supported by ").append(String.join(", ", supporting));
            if (!contradicting.isEmpty()) {
                text.append("; but ").append(String.join(", ", contradicting))
                        .append(" should have stayed quiet");
            }
            return text.toString();
        }
    }

    /** A registered candidate cause and the residual pattern it predicts. */
    private static final class Candidate {
        private final String name;
        private final Map<String, Double> expected = new LinkedHashMap<>();
        private final Map<String, Double> expectedQuiet = new LinkedHashMap<>();

        Candidate(String name) {
            this.name = name;
        }
    }

    /** Describes which residuals a candidate cause should and should not disturb. */
    public static final class CandidateSpec {
        private final Candidate candidate;

        private CandidateSpec(Candidate candidate) {
            this.candidate = candidate;
        }

        /**
         * This cause should bias {@code monitorName}. Weight is that residual's share of the evidence —
         * use a larger number for the residual that most distinguishes this cause from its neighbours.
         */
        public CandidateSpec expects(String monitorName, double weight) {
            if (!(weight > 0)) {
                throw new IllegalArgumentException("weight must be > 0 for '" + monitorName + "'");
            }
            candidate.expected.put(monitorName, weight);
            return this;
        }

        /** This cause should bias {@code monitorName}, with the default weight of 1.0. */
        public CandidateSpec expects(String monitorName) {
            return expects(monitorName, 1.0);
        }

        /**
         * This cause should leave {@code monitorName} alone. If that residual is biased anyway, it is
         * evidence against this cause, scaled by {@code penalty}. This is what separates causes that
         * would otherwise look identical.
         */
        public CandidateSpec expectsQuiet(String monitorName, double penalty) {
            if (!(penalty > 0)) {
                throw new IllegalArgumentException("penalty must be > 0 for '" + monitorName + "'");
            }
            candidate.expectedQuiet.put(monitorName, penalty);
            return this;
        }

        /** This cause should leave {@code monitorName} alone, with the default penalty of 1.0. */
        public CandidateSpec expectsQuiet(String monitorName) {
            return expectsQuiet(monitorName, 1.0);
        }
    }

    /** Start building an isolator. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link FaultIsolator}. */
    public static final class Builder {
        private final Map<String, ModelResidualMonitor> monitors = new LinkedHashMap<>();
        private final List<Candidate> candidates = new ArrayList<>();

        /** Register a residual monitor. Candidates refer to it by its {@link ModelResidualMonitor#name()}. */
        public Builder monitor(ModelResidualMonitor monitor) {
            if (monitor == null) throw new IllegalArgumentException("monitor must not be null");
            monitors.put(monitor.name(), monitor);
            return this;
        }

        /**
         * Register a candidate cause and describe the residual pattern it produces.
         *
         * @param name what to call this cause when it is reported
         * @param spec fills in the {@code expects} / {@code expectsQuiet} pattern
         */
        public Builder candidate(String name, Consumer<CandidateSpec> spec) {
            if (name == null || name.isBlank()) {
                throw new IllegalArgumentException("a candidate name is required");
            }
            Candidate candidate = new Candidate(name);
            spec.accept(new CandidateSpec(candidate));
            if (candidate.expected.isEmpty()) {
                throw new IllegalStateException("candidate '" + name + "' expects no residuals - it "
                        + "could never be selected. Give it at least one expects(...)");
            }
            candidates.add(candidate);
            return this;
        }

        /** Validate and build. Rejects a candidate naming a monitor that was never registered. */
        public FaultIsolator build() {
            for (Candidate candidate : candidates) {
                for (String name : candidate.expected.keySet()) requireKnown(candidate.name, name);
                for (String name : candidate.expectedQuiet.keySet()) requireKnown(candidate.name, name);
            }
            return new FaultIsolator(this);
        }

        private void requireKnown(String candidateName, String monitorName) {
            if (!monitors.containsKey(monitorName)) {
                throw new IllegalStateException("candidate '" + candidateName + "' refers to residual '"
                        + monitorName + "', which was never registered with monitor(...)");
            }
        }
    }
}
