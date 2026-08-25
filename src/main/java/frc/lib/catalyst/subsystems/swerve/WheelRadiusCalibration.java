package frc.lib.catalyst.subsystems.swerve;

import frc.lib.catalyst.command.CatalystCommand;
import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Coroutine;

/**
 * Measures the <b>actual</b> swerve wheel radius by spinning the robot in
 * place — correcting the CAD value, which is a documented source of
 * autonomous inaccuracy (wheels wear; tread compresses under load).
 *
 * <p>The physics: when the robot rotates by an angle θ, each module —
 * sitting a distance {@code driveBaseRadius} from the center — rolls a true
 * arc of {@code θ · driveBaseRadius}. The motor's angular rotation is fixed
 * by what physically happened, so the only unknown is the wheel radius that
 * converts rotations to distance. Comparing the gyro arc to the distance
 * odometry <em>thinks</em> each wheel rolled (which uses the current radius)
 * gives the correction:
 *
 * <pre>
 *   trueRadius = currentRadius × (gyroArc / measuredArc)
 * </pre>
 *
 * <p>Run it on blocks or in open space — the robot will spin several times.
 * The result and a copy-paste constant publish to
 * {@code /Catalyst/Calibration/WheelRadius/...}.
 *
 * <pre>{@code
 * Command cal = WheelRadiusCalibration.builder(drive)
 *     .currentWheelRadius(0.0508)   // your configured radius (m)
 *     .driveBaseRadius(0.42)        // center → module distance (m)
 *     .rotations(4)                 // spin 4 full turns
 *     .omega(0.6)                   // rad/s
 *     .build();
 *
 * test.a().onTrue(cal);
 * }</pre>
 */
public final class WheelRadiusCalibration {

    private WheelRadiusCalibration() {}

    public static Builder builder(SwerveSubsystem drive) {
        return new Builder(drive);
    }

    public static class Builder {
        private final SwerveSubsystem drive;
        private double currentWheelRadius = 0.0508; // 2" default
        private double driveBaseRadius = 0.4;
        private double rotations = 4.0;
        private double omega = 0.6;     // rad/s

        private Builder(SwerveSubsystem drive) {
            this.drive = drive;
        }

        /** The wheel radius currently configured on the drivetrain (m). */
        public Builder currentWheelRadius(double meters) { this.currentWheelRadius = meters; return this; }

        /** Distance from robot center to a module (m). */
        public Builder driveBaseRadius(double meters) { this.driveBaseRadius = meters; return this; }

        /** Number of full robot rotations to spin. More = more accurate. Default 4. */
        public Builder rotations(double n) { this.rotations = n; return this; }

        /** Spin rate in rad/s. Keep it slow for accuracy. Default 0.6. */
        public Builder omega(double radPerSec) { this.omega = radPerSec; return this; }

        public CatalystCommand build() {
            return CatalystCommand.of(
                    new CalCommand(drive, currentWheelRadius, driveBaseRadius, rotations, omega));
        }
    }

    private static final class CalCommand implements Command {

        private final java.util.Set<Mechanism> required;

        @Override
        public String name() {
            return "WheelRadiusCalibration";
        }

        @Override
        public java.util.Set<Mechanism> requirements() {
            return required;
        }

        private final SwerveSubsystem drive;
        private final double currentRadius;
        private final double driveBaseRadius;
        private final double targetRad;
        private final double omega;
        private final NetworkTable nt;

        private double[] startDistances;
        private double accumHeadingRad;
        private double lastHeadingRad;

        CalCommand(SwerveSubsystem drive, double currentRadius, double driveBaseRadius,
                   double rotations, double omega) {
            this.drive = drive;
            this.currentRadius = currentRadius;
            this.driveBaseRadius = driveBaseRadius;
            this.targetRad = Math.abs(rotations) * 2.0 * Math.PI;
            this.omega = omega;
            this.nt = NetworkTableInstance.getDefault()
                    .getTable("Catalyst").getSubTable("Calibration").getSubTable("WheelRadius");
            this.required = java.util.Set.of(drive);
        }

        /** v3 collapses initialize/execute/isFinished/end into a single coroutine body. */
        @Override
        public void run(Coroutine coroutine) {
            startDistances = drive.getModuleDistances();
            accumHeadingRad = 0;
            lastHeadingRad = drive.getHeading().getRadians();
            nt.getEntry("Status").setString("running");

            while (Math.abs(accumHeadingRad) < targetRad) {
                step();
                coroutine.yield();
            }
            finish(false);
        }

        @Override
        public void onCancel() {
            // true, not false. Passing "not interrupted" sent a cancelled run down the success
            // path: the dashboard showed done, a corrected radius, and a ready-to-paste snippet -
            // all computed from a partial arc measured against a full-rotation gyro reading. A team
            // pastes that into their constants and every odometry distance is wrong for the event.
            finish(true);
        }

        private void step() {
            drive.driveFieldCentric(0, 0, omega);
            double h = drive.getHeading().getRadians();
            accumHeadingRad += MathUtil.angleModulus(h - lastHeadingRad);
            lastHeadingRad = h;
            nt.getEntry("AccumRotations").setDouble(Math.abs(accumHeadingRad) / (2 * Math.PI));
        }

        /** Runs on natural completion; {@link #onCancel()} routes an interrupted run here too. */
        private void finish(boolean interrupted) {
            drive.driveFieldCentric(0, 0, 0);

            double[] now = drive.getModuleDistances();
            double measuredArc = 0;
            int n = Math.min(now.length, startDistances.length);
            for (int i = 0; i < n; i++) {
                measuredArc += Math.abs(now[i] - startDistances[i]);
            }
            measuredArc = n > 0 ? measuredArc / n : 0;

            double gyroArc = Math.abs(accumHeadingRad) * driveBaseRadius;

            if (measuredArc < 1e-6 || interrupted) {
                nt.getEntry("Status").setString(interrupted ? "interrupted" : "no motion measured");
                return;
            }

            double corrected = currentRadius * (gyroArc / measuredArc);
            nt.getEntry("CorrectedRadiusMeters").setDouble(corrected);
            nt.getEntry("CorrectedRadiusInches").setDouble(corrected / 0.0254);
            nt.getEntry("PercentChange").setDouble((corrected - currentRadius) / currentRadius * 100.0);
            nt.getEntry("Snippet").setString(
                    String.format("kWheelRadius = %.5f; // m  (was %.5f, %.1f%%)",
                            corrected, currentRadius, (corrected - currentRadius) / currentRadius * 100.0));
            nt.getEntry("Status").setString("done");
        }
    }
}
