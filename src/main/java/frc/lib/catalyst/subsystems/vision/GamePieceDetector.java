package frc.lib.catalyst.subsystems.vision;

import com.limelightvision.Limelight;

import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

/**
 * Game pieces the camera can see, from a Limelight object-detection pipeline.
 *
 * <p>Systemcore has a Hailo accelerator on USB ports 0 and 1, which makes neural object detection
 * cheap enough to run alongside AprilTag tracking instead of instead of it. Catalyst had no way to
 * consume the results.
 *
 * <p><b>This is not {@code GamePieceTracker}, and the distinction matters.</b>
 * {@code GamePieceTracker} follows a piece the robot already <em>has</em>, through the stages of a
 * mechanism, using the robot's own sensors. This reports pieces the robot can <em>see</em> and does
 * not have. They answer different questions — "am I loaded?" versus "where should I drive?" — and
 * conflating them would make both worse.
 *
 * <p>What this is for is the input side of {@code Autopilot} and {@code BehaviorEngine}: a behaviour
 * that wants to go and collect something needs a bearing to it.
 *
 * <h2>On distance, and why it is opt-in</h2>
 *
 * <p>A detection is a box in an image. Turning that into a distance needs an assumption, and the
 * usual one — that the piece sits on the floor — is only true for pieces on the floor. So there is
 * no {@code getDistance()} here. {@link #groundRangeMeters(Detection, double, double)} will do the
 * trigonometry, but it makes you pass the camera height and pitch, because a range computed from
 * numbers nobody checked is worse than no range at all.
 *
 * @since 2.0.0
 */
public final class GamePieceDetector {

    /**
     * One detected object.
     *
     * @param className   the class the neural network assigned, as named in the pipeline
     * @param confidence  detection confidence, 0–1
     * @param txDegrees   horizontal angle from the crosshair, positive right
     * @param tyDegrees   vertical angle from the crosshair, positive up
     * @param areaPercent share of the image the detection covers, 0–100
     * @param trackId     stable id while the camera keeps hold of this object, or -1
     */
    public record Detection(
            String className,
            double confidence,
            double txDegrees,
            double tyDegrees,
            double areaPercent,
            int trackId) {

        /** Bearing to the object as a {@link Rotation2d}, CCW positive to match the rest of WPILib. */
        public Rotation2d bearing() {
            return Rotation2d.fromDegrees(-txDegrees);
        }
    }

    private final LimelightSource camera;
    private final double minConfidence;

    /**
     * Read detections from a camera.
     *
     * @param camera        the Limelight running an object-detection pipeline
     * @param minConfidence detections below this are discarded, 0–1
     */
    public GamePieceDetector(LimelightSource camera, double minConfidence) {
        this.camera = camera;
        this.minConfidence = minConfidence;
    }

    /** Read detections with a 0.5 confidence floor. */
    public GamePieceDetector(LimelightSource camera) {
        this(camera, 0.5);
    }

    /**
     * Everything the camera currently sees, above the confidence floor.
     *
     * <p>Ordered by area, largest first — which for objects of one kind is the closest, and is the
     * order a "go get one" behaviour wants. Empty when the camera is not connected or is running a
     * pipeline that does not detect objects, rather than throwing.
     */
    public List<Detection> detections() {
        Limelight.LimelightResults results;
        try {
            results = camera.getLimelight().getLatestResults();
        } catch (RuntimeException ignored) {
            return List.of();
        }
        if (results == null || results.detectorTargets == null) {
            return List.of();
        }

        List<Detection> out = new ArrayList<>(results.detectorTargets.length);
        for (Limelight.DetectorTarget t : results.detectorTargets) {
            if (t == null || t.confidence < minConfidence) {
                continue;
            }
            out.add(new Detection(
                    t.className == null ? "" : t.className,
                    t.confidence,
                    t.txDegrees,
                    t.tyDegrees,
                    t.targetAreaPercent,
                    t.trackId));
        }
        out.sort(Comparator.comparingDouble(Detection::areaPercent).reversed());
        return List.copyOf(out);
    }

    /** Only the detections of a given class, largest first. */
    public List<Detection> detectionsOf(String className) {
        return detections().stream()
                .filter(d -> d.className().equalsIgnoreCase(className))
                .toList();
    }

    /** The largest detection, which for one class of object is the nearest. */
    public Optional<Detection> best() {
        List<Detection> all = detections();
        return all.isEmpty() ? Optional.empty() : Optional.of(all.get(0));
    }

    /** The largest detection of a given class. */
    public Optional<Detection> best(String className) {
        List<Detection> matching = detectionsOf(className);
        return matching.isEmpty() ? Optional.empty() : Optional.of(matching.get(0));
    }

    /** Whether anything above the confidence floor is visible. */
    public boolean hasDetection() {
        return !detections().isEmpty();
    }

    /**
     * Range to a detection, assuming it is sitting on the floor.
     *
     * <p>Plain trigonometry: the camera is at a known height and pitch, the detection is at a known
     * vertical angle, and the floor is flat. That chain is only as good as its assumptions —
     * a piece on a ramp, in a chute, held by another robot, or simply mis-detected will produce a
     * confident number that is wrong. Treat it as a hint for a behaviour, not as a measurement to
     * drive a controller from.
     *
     * @param detection        the detection to range
     * @param cameraHeightM    camera lens height above the floor, metres
     * @param cameraPitchDeg   camera pitch, positive upward
     * @return range in metres, or empty when the geometry does not close — the detection is at or
     *         above the horizon, which means it is not on the floor
     */
    public static Optional<Double> groundRangeMeters(Detection detection,
                                                     double cameraHeightM,
                                                     double cameraPitchDeg) {
        double angleToTarget = Math.toRadians(cameraPitchDeg + detection.tyDegrees());

        // At or above the horizon there is no intersection with the floor, and tan() would happily
        // return a large positive number for a target slightly above it.
        if (angleToTarget >= -1e-6) {
            return Optional.empty();
        }
        return Optional.of(cameraHeightM / -Math.tan(angleToTarget));
    }

    /** Publish the current detections under {@code /Catalyst/Vision/<camera>/Detections}. */
    public void publish() {
        List<Detection> all = detections();
        String[] rows = new String[all.size()];
        for (int i = 0; i < all.size(); i++) {
            Detection d = all.get(i);
            rows[i] = String.format("%s|%.2f|%.1f|%.1f|%.2f",
                    d.className(), d.confidence(), d.txDegrees(), d.tyDegrees(), d.areaPercent());
        }
        CatalystLog.log("Vision/" + camera.getName() + "/Detections", rows);
        CatalystLog.log("Vision/" + camera.getName() + "/DetectionCount", all.size());
    }
}
