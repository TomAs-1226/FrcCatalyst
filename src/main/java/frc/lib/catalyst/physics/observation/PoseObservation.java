package frc.lib.catalyst.physics.observation;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * An absolute field-relative pose measured by something outside the drivetrain — an AprilTag camera,
 * a QuestNav headset, a known contact against a field wall.
 *
 * <p>This is the observation type that resets drift. Physics Core uses it to decide how much to
 * believe the pose it is being handed, and how stale that belief has become: every accepted pose
 * observation resets {@link frc.lib.catalyst.physics.LocalizationQuality#secondsSinceAbsoluteFix()},
 * and confidence decays from there.
 *
 * <p><b>Physics Core does not write your pose.</b> In shadow mode it consumes these purely as a
 * confidence signal; your {@code SwerveSubsystem} pose estimator remains the single writer of
 * {@code Pose2d}. Keep calling {@code drive.addVisionMeasurement(...)} exactly as you do today and
 * pass the same measurement here as well.
 *
 * @param pose               the measured field-relative pose
 * @param timestampSeconds   capture time on the FPGA clock, not arrival time
 * @param standardDeviation  1-sigma translational uncertainty of the measurement, in metres
 * @param source             short label for logging, e.g. {@code "limelight-front"}
 * @since 1.5.0
 */
public record PoseObservation(
        Pose2d pose,
        double timestampSeconds,
        double standardDeviation,
        String source) implements Observation {

    /** Compact constructor: rejects a null pose and a non-positive standard deviation. */
    public PoseObservation {
        if (pose == null) throw new IllegalArgumentException("pose must not be null");
        if (!(standardDeviation > 0)) {
            throw new IllegalArgumentException("standardDeviation must be > 0 (got " + standardDeviation
                    + ") - a measurement claiming zero error cannot be weighed against anything");
        }
        source = source == null ? "unknown" : source;
    }

    /** A pose observation with the 5 cm standard deviation typical of a close, well-lit AprilTag. */
    public static PoseObservation of(Pose2d pose, double timestampSeconds, String source) {
        return new PoseObservation(pose, timestampSeconds, 0.05, source);
    }
}
