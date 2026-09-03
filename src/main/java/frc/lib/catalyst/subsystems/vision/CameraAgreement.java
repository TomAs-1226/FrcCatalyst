package frc.lib.catalyst.subsystems.vision;

import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Catches a camera whose mounting transform is wrong, by asking the other cameras.
 *
 * <h2>The gap this fills</h2>
 *
 * <p>A {@code robotToCamera} transform cannot be checked against anything on a one-camera robot.
 * {@link LimelightSource} says so in its own constructor: it publishes the numbers to NetworkTables
 * so a human can measure them against the machine, because the failure is not a crash. A camera
 * mounted 12 cm from where the code thinks, or with a sign flipped on its side offset, produces
 * poses that are confidently, consistently wrong by that much — which reads as "vision is a bit
 * noisy" and gets tuned around rather than fixed.
 *
 * <p>With several cameras it stops being unknowable. Two cameras looking at tags in the same loop
 * are describing the same robot, so their estimates must agree to within their noise. A persistent
 * disagreement is not noise; it is one of the transforms being wrong.
 *
 * <h2>Why the offset is measured in the robot's frame</h2>
 *
 * <p>Because that is what makes it attributable. A mounting error is fixed <em>on the robot</em>, so
 * in the robot's own frame it is a constant — the same 12 cm forward whichever way the robot is
 * pointing. In the field frame the same error spins with the robot and averages towards nothing over
 * a match, which is exactly how a real bias hides. So each disagreement is rotated into the robot
 * frame before it is accumulated, and the median of those is the number reported.
 *
 * <h2>Two cameras can disagree; three can say who is wrong</h2>
 *
 * <p>This is the part that four cameras actually buys, and it is why the report distinguishes them.
 * With two cameras a disagreement is real but unattributable — one of them is wrong and there is no
 * third opinion to say which. With three or more, the consensus is the median and a single outlier
 * is named. The wording of the warning changes accordingly rather than pretending to know.
 *
 * <p>Advisory only. Nothing here rejects a pose or changes what reaches the estimator; a
 * mis-mounted camera still needs a human with a tape measure, and this exists to tell them which
 * camera to measure.
 *
 * @since 2.0.0
 */
final class CameraAgreement {

    /**
     * How far a camera's median robot-frame offset may sit from the consensus before it is reported.
     *
     * <p>20 cm. Comfortably above the disagreement two correctly-mounted cameras show at range —
     * that is a few centimetres and it is zero-mean — and comfortably below a mistake worth finding,
     * since the smallest interesting one is a sign flip on a mount offset, which on 581's side
     * cameras is 2 x 13.9 inches.
     */
    private static final double REPORT_THRESHOLD_METERS = 0.20;

    /**
     * Samples a camera needs before it may be accused of anything.
     *
     * <p>Enough that a handful of bad frames while the robot is moving cannot trip it, and few
     * enough to be reached during a practice match rather than only in eliminations.
     */
    private static final int MIN_SAMPLES = 40;

    /** Per-camera history depth. A median over this is what gets compared. */
    private static final int HISTORY = 100;

    private final Map<String, double[]> offsetX = new HashMap<>();
    private final Map<String, double[]> offsetY = new HashMap<>();
    private final Map<String, Integer> counts = new HashMap<>();
    private final Map<String, Boolean> reported = new HashMap<>();

    /** One camera's estimate this cycle: its name and where it thinks the robot is. */
    record Sighting(String cameraName, Pose2d pose) {}

    /**
     * Take one cycle's worth of simultaneous estimates.
     *
     * <p>Does nothing with fewer than two: a single camera has nothing to be checked against, which
     * is the whole problem.
     *
     * @param sightings   every camera that produced an accepted estimate this cycle
     * @param robotHeading the robot's current heading, used to rotate offsets into the robot frame
     */
    void observe(List<Sighting> sightings, org.wpilib.math.geometry.Rotation2d robotHeading) {
        if (sightings.size() < 2) {
            return;
        }

        double consensusX = median(sightings.stream().mapToDouble(s -> s.pose().getX()).toArray());
        double consensusY = median(sightings.stream().mapToDouble(s -> s.pose().getY()).toArray());

        for (Sighting s : sightings) {
            // Field-frame disagreement, then rotated into the robot frame so a mounting error is a
            // constant rather than something that spins with the robot.
            Translation2d fieldOffset = new Translation2d(
                    s.pose().getX() - consensusX,
                    s.pose().getY() - consensusY);
            Translation2d robotOffset = fieldOffset.rotateBy(robotHeading.unaryMinus());

            String name = s.cameraName();
            int n = counts.getOrDefault(name, 0);
            offsetX.computeIfAbsent(name, k -> new double[HISTORY])[n % HISTORY] = robotOffset.getX();
            offsetY.computeIfAbsent(name, k -> new double[HISTORY])[n % HISTORY] = robotOffset.getY();
            counts.put(name, n + 1);
        }

        publish(sightings.size());
        report(sightings.size());
    }

    /** The median robot-frame offset for a camera, once it has enough samples. */
    Optional<Translation2d> medianOffset(String cameraName) {
        int n = counts.getOrDefault(cameraName, 0);
        if (n < MIN_SAMPLES) {
            return Optional.empty();
        }
        int len = Math.min(n, HISTORY);
        return Optional.of(new Translation2d(
                median(Arrays.copyOf(offsetX.get(cameraName), len)),
                median(Arrays.copyOf(offsetY.get(cameraName), len))));
    }

    private void publish(int camerasThisCycle) {
        CatalystLog.log("Vision/Agreement/CamerasAgreeing", (double) camerasThisCycle);
        for (String name : counts.keySet()) {
            medianOffset(name).ifPresent(o -> {
                CatalystLog.log("Vision/" + name + "/OffsetFromConsensusM", o.getNorm());
                CatalystLog.log("Vision/" + name + "/OffsetForwardM", o.getX());
                CatalystLog.log("Vision/" + name + "/OffsetLeftM", o.getY());
            });
        }
    }

    private void report(int camerasThisCycle) {
        for (String name : new ArrayList<>(counts.keySet())) {
            if (Boolean.TRUE.equals(reported.get(name))) {
                continue;
            }
            Optional<Translation2d> off = medianOffset(name);
            if (off.isEmpty() || off.get().getNorm() < REPORT_THRESHOLD_METERS) {
                continue;
            }
            reported.put(name, true);
            Translation2d o = off.get();

            String attribution = camerasThisCycle >= 3
                    ? "Two other cameras agree with each other and not with this one, so this is "
                            + "the one to measure."
                    : "With only two cameras there is no third opinion, so this says they disagree "
                            + "and not which of them is wrong - check both, or add a third camera.";

            DriverStationErrors.reportWarning(String.format(
                    "[Catalyst] Camera '%s' sits %.2f m from where the other cameras put the robot, "
                            + "consistently: %.2f m forward and %.2f m left of consensus in the "
                            + "ROBOT frame, held over %d samples. A disagreement that stays fixed in "
                            + "the robot's own frame is a mounting transform that is wrong, not "
                            + "noise - noise averages out and a wrong robotToCamera does not. %s",
                    name, o.getNorm(), o.getX(), o.getY(),
                    Math.min(counts.get(name), HISTORY), attribution), false);
        }
    }

    /** Median without mutating the caller's array. Even lengths take the lower middle. */
    private static double median(double[] values) {
        if (values.length == 0) {
            return 0.0;
        }
        double[] copy = values.clone();
        Arrays.sort(copy);
        return copy[(copy.length - 1) / 2];
    }
}
