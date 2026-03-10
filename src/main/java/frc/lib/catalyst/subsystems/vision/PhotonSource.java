package frc.lib.catalyst.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

/**
 * Camera source implementation for PhotonVision cameras.
 *
 * <p>Uses multi-tag PnP as primary strategy with single-tag fallback.
 *
 * <p>Example:
 * <pre>{@code
 * PhotonSource cam = new PhotonSource("cam-back",
 *     new Transform3d(-0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-20), Math.PI)),
 *     fieldLayout);
 * }</pre>
 */
public class PhotonSource implements CameraSource {

    private final String name;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonSource(String name, Transform3d robotToCamera, AprilTagFieldLayout fieldLayout) {
        this.name = name;
        this.camera = new PhotonCamera(name);
        this.poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Optional<PoseEstimate> getEstimatedPose() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }

        // Use the latest result
        PhotonPipelineResult latestResult = results.get(results.size() - 1);

        if (!latestResult.hasTargets()) {
            return Optional.empty();
        }

        var estimatedPose = poseEstimator.update(latestResult);
        if (estimatedPose.isEmpty()) {
            return Optional.empty();
        }

        var estimate = estimatedPose.get();
        Pose2d pose2d = estimate.estimatedPose.toPose2d();

        // Calculate tag count and average distance
        List<PhotonTrackedTarget> targets = latestResult.getTargets();
        int tagCount = targets.size();
        double totalDist = 0;
        double maxAmbiguity = 0;
        for (PhotonTrackedTarget target : targets) {
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            totalDist += Math.hypot(
                    bestCameraToTarget.getX(),
                    Math.hypot(bestCameraToTarget.getY(), bestCameraToTarget.getZ()));
            maxAmbiguity = Math.max(maxAmbiguity, target.getPoseAmbiguity());
        }
        double avgDist = tagCount > 0 ? totalDist / tagCount : 999;

        return Optional.of(new PoseEstimate(
                pose2d,
                estimate.timestampSeconds,
                tagCount,
                avgDist,
                maxAmbiguity
        ));
    }
}
