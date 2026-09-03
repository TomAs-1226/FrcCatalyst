package frc.lib.catalyst.subsystems.vision;

import com.limelightvision.Limelight;

import frc.lib.catalyst.logging.CatalystLog;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Timer;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;

import java.util.Optional;

/**
 * Camera source for Limelight cameras, on LimelightLib 2.
 *
 * <p><b>This was rewritten for 2027, and it had to be.</b> Catalyst used to read Limelight
 * NetworkTables keys directly — {@code botpose_wpiblue}, {@code tx}, {@code ta} — specifically so it
 * would not need a vendordep. Limelight OS 2027.0 ends that: results are published as a single
 * MessagePack topic, and the classic per-key API is <em>disabled by default</em>. The old code
 * compiles perfectly against 2027 and silently sees nothing at all, which is the worst way for
 * vision to fail. Decoding MessagePack by hand to keep avoiding a dependency would have been a
 * worse trade than taking it.
 *
 * <p>What comes with the change, beyond working:
 *
 * <ul>
 *   <li><b>Filtering the camera already does.</b> LimelightLib 2 applies its own rejection rules and
 *       distance/tag-count standard-deviation scaling, and reports why an estimate was rejected.
 *       {@link #getEstimatedPose()} drains the <em>accepted</em> queue, so obviously-bad poses never
 *       reach the estimator.</li>
 *   <li><b>Real standard deviations.</b> The camera computes them from tag count, distance and area.
 *       Catalyst previously had to approximate this from tag count alone.</li>
 *   <li><b>Every frame, not just the newest.</b> The camera queues estimates between robot loops.
 *       Reading the queue rather than a single latest value keeps the frames a 20 ms loop would
 *       otherwise throw away.</li>
 * </ul>
 *
 * <p><b>Coordinate systems changed too.</b> Limelight OS 2027.0 unified everything on NWU
 * right-handed (X forward, Y left, Z up). A camera transform carried over from 2026 needs its mount
 * side and pitch signs re-checked — this class passes the transform straight through, so a wrong
 * sign here is a wrong pose with no warning.
 *
 * <p>USB-attached Limelights on Systemcore are named by {@link Limelight#SYSTEMCORE_USB0} and
 * friends; see {@link #usb(int, Transform3d)}.
 *
 * @since 2.0.0
 */
public class LimelightSource implements CameraSource {

    private final String name;
    private final Limelight limelight;
    private final boolean useMegaTag2;

    /**
     * How stale the robot yaw may be before MegaTag2 estimates stop being trusted.
     *
     * <p>Half a second is about twenty-five robot loops. Anything that is still publishing yaw at
     * all will be well inside it, and anything outside it has stopped, which is the case worth
     * catching.
     */
    private static final double ORIENTATION_STALE_SECONDS = 0.5;

    /** When this camera was last told the robot's yaw. */
    private double lastOrientationTs = Double.NEGATIVE_INFINITY;

    /** When ANY camera was last told the robot's yaw through the shared table. */
    private static volatile double lastSharedOrientationTs = Double.NEGATIVE_INFINITY;

    /** So the warning below is said once per camera, not fifty times a second. */
    private boolean warnedAboutMissingYaw = false;

    /**
     * A Limelight by name, using MegaTag2.
     *
     * @param name          camera name, e.g. {@code "limelight-front"}
     * @param robotToCamera camera pose in robot space, NWU
     */
    public LimelightSource(String name, Transform3d robotToCamera) {
        this(name, robotToCamera, true);
    }

    /**
     * A Limelight by name, choosing the localisation mode.
     *
     * @param name          camera name
     * @param robotToCamera camera pose in robot space, NWU
     * @param useMegaTag2   true for MegaTag2 (needs robot yaw published each loop), false for
     *                      MegaTag1, which is self-contained but less stable at range
     */
    public LimelightSource(String name, Transform3d robotToCamera, boolean useMegaTag2) {
        this.name = name;
        this.useMegaTag2 = useMegaTag2;
        this.limelight = new Limelight(name, new Pose3d(
                robotToCamera.getTranslation(), robotToCamera.getRotation()));

        // Publish the transform once, at construction, because a wrong one cannot be detected later.
        //
        // Limelight OS 2027.0 unified every 3D space on NWU right-handed. A transform carried over
        // from 2026 may need its mount side and pitch signs flipped, and the failure mode is not a
        // crash - it is a pose that is confidently, consistently wrong by a few degrees of camera
        // pitch, which reads as "vision is a bit noisy" rather than as a configuration error.
        // Catalyst cannot tell a correct transform from an incorrect one, so it does the next best
        // thing and puts the numbers somewhere a human can check them against the robot.
        CatalystLog.log("Vision/" + name + "/RobotToCamera", new double[] {
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                Math.toDegrees(robotToCamera.getRotation().getX()),
                Math.toDegrees(robotToCamera.getRotation().getY()),
                Math.toDegrees(robotToCamera.getRotation().getZ())
        });
    }

    /**
     * A Limelight attached to one of Systemcore's USB ports.
     *
     * <p>Systemcore runs four independent vision instances, so a robot can carry four cameras with
     * no network switch and no extra wiring. Ports are numbered 0–3.
     *
     * @param usbPort       0–3
     * @param robotToCamera camera pose in robot space, NWU
     */
    public static LimelightSource usb(int usbPort, Transform3d robotToCamera) {
        String name = switch (usbPort) {
            case 0 -> Limelight.SYSTEMCORE_USB0;
            case 1 -> Limelight.SYSTEMCORE_USB1;
            case 2 -> Limelight.SYSTEMCORE_USB2;
            case 3 -> Limelight.SYSTEMCORE_USB3;
            default -> throw new IllegalArgumentException(
                    "Systemcore has USB vision ports 0-3, got " + usbPort);
        };
        return new LimelightSource(name, robotToCamera);
    }

    @Override
    public String getName() {
        return name;
    }

    /**
     * Whether this camera has been given the robot's yaw recently enough to trust MegaTag2.
     *
     * <p>Either route counts. A robot with four cameras is expected to use the shared publish, and
     * one with a single camera usually calls the instance method; neither should look unwired
     * because it chose the other.
     */
    private boolean orientationIsFresh() {
        double newest = Math.max(lastOrientationTs, lastSharedOrientationTs);
        return (Timer.getTimestamp() - newest) <= ORIENTATION_STALE_SECONDS;
    }

    /**
     * Whether MegaTag2 is currently being fed, for a dashboard or a pre-match check.
     *
     * <p>Exposed because "vision is not returning anything" and "vision is not being told where the
     * robot is pointing" look identical from the outside and have completely different fixes.
     */
    public boolean isReceivingRobotOrientation() {
        return !useMegaTag2 || orientationIsFresh();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Returns the newest estimate the camera has <em>accepted</em>. Rejected ones are dropped by
     * the camera before they get here, so this no longer needs Catalyst-side sanity checks.
     */
    @Override
    public Optional<PoseEstimate> getEstimatedPose() {
        Limelight.PoseEstimateType type = useMegaTag2
                ? Limelight.PoseEstimateType.MT2_WPIBLUE
                : Limelight.PoseEstimateType.MT1_WPIBLUE;

        // MegaTag2 without a robot yaw is not a degraded estimate, it is a wrong one.
        //
        // MT2 resolves tag ambiguity using an externally supplied heading. If nobody supplies it,
        // the camera does not fail and does not complain - it uses the last value it was given,
        // which at boot is zero. Every pose it then produces is confidently wrong by exactly the
        // robot's true heading, and it looks like a plausible pose, so it goes into the estimator
        // and drags the fused pose with it.
        //
        // Confirmed on a Limelight 4 on the bench rather than reasoned about: an untouched camera
        // reports botorient = {alpha: 0.001, imumode: 0, interpbotyaw: 0.0}. imumode 0 is
        // "use the externally supplied yaw", and interpbotyaw stays 0.0 until something publishes.
        // So a robot that constructs LimelightSource with useMegaTag2 and never wires the yaw feed
        // is asking the camera to localise a robot it believes is facing field-X, forever.
        //
        // Refusing is the right answer rather than warning and returning anyway: there is no sense
        // in which the estimate is usable, and handing it to a pose estimator is strictly worse
        // than handing it nothing. A camera that has simply not been fed yet - the first loops
        // after boot - is the same case and is also correctly refused, because MT2 genuinely has
        // no answer yet.
        if (useMegaTag2 && !orientationIsFresh()) {
            if (!warnedAboutMissingYaw) {
                warnedAboutMissingYaw = true;
                DriverStationErrors.reportWarning(
                        "[Catalyst] " + name + " is configured for MegaTag2 but has not been given "
                                + "the robot's yaw for " + ORIENTATION_STALE_SECONDS + "s. MegaTag2 "
                                + "resolves tags against that heading and silently assumes 0 when "
                                + "it is missing, so its poses would be wrong by the robot's true "
                                + "heading. Call setRobotOrientation(...) or "
                                + "LimelightSource.setSharedRobotOrientation(...) every loop, or "
                                + "construct this source with useMegaTag2 = false.",
                        false);
            }
            return Optional.empty();
        }

        Limelight.PoseEstimate[] accepted;
        try {
            accepted = limelight.readAcceptedPoseEstimates(type);
        } catch (RuntimeException ignored) {
            return Optional.empty();
        }
        if (accepted == null || accepted.length == 0) {
            return Optional.empty();
        }

        // Newest wins. The queue exists so nothing is lost between loops; a pose estimator wants the
        // most recent measurement, and older ones in the same batch are superseded by it.
        Limelight.PoseEstimate latest = accepted[accepted.length - 1];
        if (!latest.isValid()) {
            return Optional.empty();
        }

        return Optional.of(new PoseEstimate(
                latest.pose,
                latest.timestampSeconds,
                latest.fieldedTagCount,
                latest.avgTagDistanceMeters,
                // LimelightLib 2 filters on ambiguity itself and does not surface a single figure
                // here. Reporting 0 says "this passed the camera's ambiguity gate", which is true
                // and is what a downstream consumer of this record actually wants to know.
                0.0));
    }

    /**
     * {@inheritDoc}
     *
     * <p>MegaTag2 needs the robot's yaw to resolve tag ambiguity. Publish the estimator's fused yaw
     * here, not a raw IMU reading — feeding back a heading the vision system helped produce is the
     * classic way to build a loop that slowly convinces itself of a wrong answer.
     *
     * <p>With several cameras, prefer {@link #setSharedRobotOrientation(double)}: one publish feeds
     * every Limelight, instead of one per camera per loop.
     */
    @Override
    public void setRobotOrientation(double yawDegrees, double yawRate,
                                    double pitchDegrees, double rollDegrees) {
        if (!useMegaTag2) {
            return;
        }
        // Stamped before the call, not after: the point is that a caller is feeding yaw this
        // loop. Whether the camera is reachable right now is a different question, and one that
        // must not make a correctly-wired robot look unwired.
        lastOrientationTs = Timer.getTimestamp();
        warnedAboutMissingYaw = false;
        try {
            limelight.setRobotOrientation(yawDegrees, yawRate, pitchDegrees, 0, rollDegrees, 0);
        } catch (RuntimeException ignored) {
            // A camera that is not connected yet is normal at startup, not an error.
        }
    }

    /**
     * Publish robot yaw once, for every Limelight on the robot.
     *
     * <p>New in Limelight OS 2027.0: cameras read a shared orientation table by default. With four
     * cameras this replaces four writes per loop with one.
     *
     * @param yawDegrees fused robot yaw, CCW positive
     */
    public static void setSharedRobotOrientation(double yawDegrees) {
        lastSharedOrientationTs = Timer.getTimestamp();
        try {
            Limelight.setSharedRobotOrientation(yawDegrees);
        } catch (RuntimeException ignored) {
            // Nothing listening yet.
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>True: {@link #getEstimatedPose()} drains the camera's <em>accepted</em> queue, so tag
     * count, ambiguity, distance, area, field bounds and frame age have already been applied by
     * LimelightLib using the raw detections.
     */
    @Override
    public boolean isPrefiltered() {
        return true;
    }

    /** Whether the camera currently sees a target. */
    public boolean hasTarget() {
        try {
            return limelight.hasTarget();
        } catch (RuntimeException ignored) {
            return false;
        }
    }

    /** Horizontal offset to the target, in degrees. Positive is to the right of the crosshair. */
    public double getTX() {
        try {
            return limelight.getTXDegrees();
        } catch (RuntimeException ignored) {
            return 0.0;
        }
    }

    /** Vertical offset to the target, in degrees. */
    public double getTY() {
        try {
            return limelight.getTYDegrees();
        } catch (RuntimeException ignored) {
            return 0.0;
        }
    }

    /** Target area, 0–100 as a percentage of the image. Consistent across pipelines since 2027.0. */
    public double getTargetArea() {
        try {
            return limelight.getTargetAreaPercent();
        } catch (RuntimeException ignored) {
            return 0.0;
        }
    }

    /** Whether the camera is connected and publishing. */
    public boolean isConnected() {
        try {
            return limelight.isConnected();
        } catch (RuntimeException ignored) {
            return false;
        }
    }

    /** The underlying LimelightLib object, for pipeline control and everything not wrapped here. */
    public Limelight getLimelight() {
        return limelight;
    }
}
