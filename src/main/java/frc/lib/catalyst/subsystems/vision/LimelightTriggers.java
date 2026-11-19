package frc.lib.catalyst.subsystems.vision;

import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.command3.Trigger;

/**
 * Wrap a Limelight's NetworkTables keys as WPILib {@link Trigger}s for
 * binding in {@code RobotContainer.configureBindings()}.
 *
 * <p>The Limelight publishes a fixed set of NT keys under its own table
 * ({@code limelight}, {@code limelight-front}, etc.) — {@code tv}, {@code tid},
 * {@code ta}, {@code tx}, {@code tclass}, {@code tl} — and these helpers
 * turn polling them into one-liners:
 *
 * <pre>{@code
 * LimelightTriggers front = new LimelightTriggers("limelight-front");
 *
 * front.hasTarget().onTrue(leds.greenCommand());
 * front.tagInView(7).whileTrue(swerve.pathfindToScore7());
 * front.detectorClass("note").onTrue(intake.intakeCommand());
 * front.targetWithinArea(2.0).onTrue(rumbleDriver());
 * }</pre>
 *
 * <p>These read the classic per-key NetworkTables API: Limelight OS 2026, or OS 2027 with that API
 * switched back on (2027 disables it by default, and then no trigger ever fires).
 *
 * <p>Every trigger is false while the camera's heartbeat ({@code hb}) has not advanced for
 * {@value #ALIVE_SECONDS} s. A camera that drops off leaves its last values on the server, so through
 * 2.0.0 {@code tagInView(7).whileTrue(drive...)} stayed true and kept driving after the camera was
 * unplugged.
 */
public final class LimelightTriggers {

    /** A heartbeat older than this is a stopped camera, whatever its last values say. */
    public static final double ALIVE_SECONDS = 0.5;

    private final NetworkTable table;
    private final DoubleSubscriber heartbeat;

    /** @param limelightName the table name (e.g. {@code "limelight"} or {@code "limelight-front"}). */
    public LimelightTriggers(String limelightName) {
        this.table = NetworkTableInstance.getDefault().getTable(limelightName);
        this.heartbeat = table.getDoubleTopic("hb").subscribe(0.0);
    }

    /** Whether the camera's heartbeat advanced within {@link #ALIVE_SECONDS}. */
    public boolean isAlive() {
        long stamp = heartbeat.getAtomic().timestamp;
        // WPILib 2027 NetworkTables time is in nanoseconds, as LegacyLimelightReader measured.
        return stamp != 0 && (NetworkTablesJNI.now() - stamp) / LegacyLimelightReader.NT_TIME_PER_SECOND
                < ALIVE_SECONDS;
    }

    /** True while the Limelight is alive and reports any valid target. */
    public Trigger hasTarget() {
        return new Trigger(() -> isAlive() && table.getEntry("tv").getDouble(0) > 0.5);
    }

    /** True while the Limelight currently sees the given AprilTag id. */
    public Trigger tagInView(int fiducialId) {
        return new Trigger(() -> isAlive()
                && table.getEntry("tv").getDouble(0) > 0.5
                && (int) table.getEntry("tid").getDouble(-1) == fiducialId);
    }

    /**
     * True while the Limelight reports a neural-detector classification
     * matching {@code className}. Useful for piece-detection pipelines
     * (e.g. {@code "note"}, {@code "cone"}, {@code "algae"}).
     */
    public Trigger detectorClass(String className) {
        return new Trigger(() -> isAlive()
                && table.getEntry("tv").getDouble(0) > 0.5
                && className.equalsIgnoreCase(table.getEntry("tclass").getString("")));
    }

    /**
     * True while the target's bounding-box area is at least {@code minArea}
     * percent of the frame. A rough proxy for "close enough" without
     * needing real range.
     */
    public Trigger targetWithinArea(double minAreaPercent) {
        return new Trigger(() -> isAlive() && table.getEntry("ta").getDouble(0) >= minAreaPercent);
    }

    /**
     * True while the horizontal offset from crosshair to target is
     * within {@code degrees} (positive or negative). Useful for "aligned
     * enough to shoot".
     */
    public Trigger horizontalErrorBelow(double degrees) {
        return new Trigger(() -> isAlive()
                && table.getEntry("tv").getDouble(0) > 0.5
                && Math.abs(table.getEntry("tx").getDouble(99)) <= degrees);
    }

    // ----- Convenience readers (mostly for diagnostics) -----

    /** Most recent target horizontal offset in degrees, or {@code 0} when no target. */
    public double tx() { return table.getEntry("tx").getDouble(0); }
    /** Most recent target vertical offset in degrees. */
    public double ty() { return table.getEntry("ty").getDouble(0); }
    /** Most recent target area as a percentage of the frame. */
    public double ta() { return table.getEntry("ta").getDouble(0); }
    /** Most recent fiducial id (or {@code -1} when no AprilTag in view). */
    public int tid()   { return (int) table.getEntry("tid").getDouble(-1); }
    /** Most recent pipeline latency in ms. */
    public double latencyMs() { return table.getEntry("tl").getDouble(0); }
}
