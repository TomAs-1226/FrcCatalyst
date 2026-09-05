package frc.lib.catalyst.subsystems.vision;

import frc.lib.catalyst.logging.CatalystLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.system.Timer;

import java.util.OptionalDouble;

/**
 * A pose that comes from vision alone.
 *
 * <p>For a robot without a Catalyst drivetrain, or a bench with cameras and nothing else: hand this to
 * {@code VisionConfig.builder().poseSink(...)} and {@link VisionSubsystem} runs its full pipeline -
 * every filter, per-camera telemetry, cross-camera agreement, health reporting - with this as the
 * thing being estimated.
 *
 * <h2>What it is, exactly</h2>
 *
 * <p>The most recent accepted estimate. Not a Kalman filter, not a blend: with no odometry there is
 * no motion model to blend against, and averaging two cameras that disagree would hide exactly the
 * disagreement a bench test exists to find. When a camera is right, this is right; when cameras
 * disagree, this jumps between them and the per-camera telemetry says which one moved it. That is
 * the honest behaviour for something with one source of information.
 *
 * <p>Velocity is reported as zero, so the spin and speed gates never reject anything - there is no
 * sensor here that could say the robot was moving.
 *
 * <p>Before the first accepted estimate, {@link #hasPose()} is false and {@link #getPose()} is the
 * origin. {@code VisionSubsystem} reads the flag and stands down the two gates that would otherwise
 * compare every estimate against an origin nobody is at.
 *
 * <p>Publishes {@code Vision/Standalone/Pose} as {@code [x, y, theta]} (metres, radians), the same
 * shape as {@code Physics/PoseArray}, so a dashboard field view can be pointed at it.
 *
 * @since 2.0.0
 */
public final class StandaloneVisionPose implements VisionPoseSink {

    private static final ChassisVelocities AT_REST = new ChassisVelocities();

    private Pose2d pose = Pose2d.kZero;
    private double lastMeasurementTs = Double.NaN;
    private int measurements = 0;

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public ChassisVelocities getChassisSpeeds() {
        return AT_REST;
    }

    @Override
    public boolean hasPose() {
        return measurements > 0;
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds,
                                     Matrix<N3, N1> stdDevs) {
        // Ignore anything captured before the current pose. VisionSubsystem sorts a loop's
        // estimates by capture time, but across loops a slow camera can deliver a frame older than
        // one a faster camera already delivered, and letting the older one win would move the pose
        // backwards in time.
        if (!Double.isNaN(lastMeasurementTs) && timestampSeconds < lastMeasurementTs) {
            return;
        }
        pose = visionPose;
        lastMeasurementTs = timestampSeconds;
        measurements++;
        CatalystLog.log("Vision/Standalone/Pose", new double[] {
                visionPose.getX(), visionPose.getY(), visionPose.getRotation().getRadians()});
        CatalystLog.log("Vision/Standalone/Measurements", (double) measurements);
    }

    /** Capture time of the estimate currently held, or empty before the first. */
    public OptionalDouble lastMeasurementTimestamp() {
        return Double.isNaN(lastMeasurementTs) ? OptionalDouble.empty()
                : OptionalDouble.of(lastMeasurementTs);
    }

    /** How old the held pose is, in seconds, or empty before the first measurement. */
    public OptionalDouble secondsSinceMeasurement() {
        return Double.isNaN(lastMeasurementTs) ? OptionalDouble.empty()
                : OptionalDouble.of(Timer.getTimestamp() - lastMeasurementTs);
    }

    /** How many estimates have been accepted into this pose since construction or reset. */
    public int measurementCount() {
        return measurements;
    }

    /** Forget everything. {@link #hasPose()} goes false again. */
    public void reset() {
        pose = Pose2d.kZero;
        lastMeasurementTs = Double.NaN;
        measurements = 0;
    }
}
