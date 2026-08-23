package frc.lib.catalyst.subsystems.vision;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
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
 * <p>The Limelight must be on the same robot network and have its NT
 * publishing enabled (the default). Nothing else to configure on the
 * library side.
 */
public final class LimelightTriggers {

    private final NetworkTable table;

    /** @param limelightName the table name (e.g. {@code "limelight"} or {@code "limelight-front"}). */
    public LimelightTriggers(String limelightName) {
        this.table = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    /** True while the Limelight reports any valid target. */
    public Trigger hasTarget() {
        return new Trigger(() -> table.getEntry("tv").getDouble(0) > 0.5);
    }

    /** True while the Limelight currently sees the given AprilTag id. */
    public Trigger tagInView(int fiducialId) {
        return new Trigger(() ->
                table.getEntry("tv").getDouble(0) > 0.5
                        && (int) table.getEntry("tid").getDouble(-1) == fiducialId);
    }

    /**
     * True while the Limelight reports a neural-detector classification
     * matching {@code className}. Useful for piece-detection pipelines
     * (e.g. {@code "note"}, {@code "cone"}, {@code "algae"}).
     */
    public Trigger detectorClass(String className) {
        return new Trigger(() ->
                table.getEntry("tv").getDouble(0) > 0.5
                        && className.equalsIgnoreCase(table.getEntry("tclass").getString("")));
    }

    /**
     * True while the target's bounding-box area is at least {@code minArea}
     * percent of the frame. A rough proxy for "close enough" without
     * needing real range.
     */
    public Trigger targetWithinArea(double minAreaPercent) {
        return new Trigger(() -> table.getEntry("ta").getDouble(0) >= minAreaPercent);
    }

    /**
     * True while the horizontal offset from crosshair to target is
     * within {@code degrees} (positive or negative). Useful for "aligned
     * enough to shoot".
     */
    public Trigger horizontalErrorBelow(double degrees) {
        return new Trigger(() ->
                table.getEntry("tv").getDouble(0) > 0.5
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
