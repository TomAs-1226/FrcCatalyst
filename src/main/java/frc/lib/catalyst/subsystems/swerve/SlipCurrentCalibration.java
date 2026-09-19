package frc.lib.catalyst.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.lib.catalyst.command.CatalystCommand;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;

/**
 * Measures the drive current at which the wheels break traction - Phoenix's <b>slip current</b>, the drive
 * motors' stator current limit ({@code kSlipCurrent} in a Tuner X constants file, {@code withSlipCurrent} on the
 * module constants). A value copied from another robot is wrong for this one: a light robot's wheels break loose
 * far below a heavy one's, and a wheel spinning on the carpet is motion the odometry believes.
 *
 * <p>The method is CTRE's: the robot's front bumper squarely against a wall, the modules pointed straight
 * ({@link SwerveRequest.SysIdSwerveTranslation}), the drive voltage ramped slowly. Held by the wall, the wheels
 * cannot turn while they grip, so the stator current climbs with the voltage; when one slips it spins up and its
 * current falls. Its peak is the slip current. It stops by itself at the slip, at its voltage cap, when a wheel
 * turns on a few amps (the robot is rolling, not held - nothing is measured), or when a motor reports nothing,
 * and it leaves the drive at 0 V however it ends, cancellation included.
 *
 * <p>Progress and the result publish to {@code /Catalyst/Calibration/SlipCurrent/...}: {@code Status},
 * {@code Volts}, {@code PeakAmps}, then {@code SlipAmps}, {@code RecommendedAmps} (rounded down to 5 A),
 * {@code Result} (a sentence) and {@code Snippet} (a line to paste). Catalyst Console shows them as it shows
 * {@link WheelRadiusCalibration}'s.
 *
 * <pre>{@code
 * Command cal = SlipCurrentCalibration.builder(drive)
 *     .currentSlipAmps(120)     // what the constants say today, for the snippet
 *     .build();
 * }</pre>
 */
public final class SlipCurrentCalibration {

    private SlipCurrentCalibration() {}

    public static Builder builder(SwerveSubsystem drive) {
        return new Builder(drive);
    }

    public static class Builder {
        private final SwerveSubsystem drive;
        private double currentSlipAmps = Double.NaN;
        private double rampVoltsPerSecond = SlipCurrentDetector.RAMP_V_PER_S;
        private double maxVolts = SlipCurrentDetector.MAX_VOLTS;

        private Builder(SwerveSubsystem drive) {
            this.drive = drive;
        }

        /** The slip current the constants hold today, A - only for the snippet's "was". */
        public Builder currentSlipAmps(double amps) { this.currentSlipAmps = amps; return this; }

        /** How fast the drive voltage ramps, V/s (0.1-2). Default 0.5: slow enough for the current to follow. */
        public Builder rampVoltsPerSecond(double v) { this.rampVoltsPerSecond = v; return this; }

        /** The most voltage it applies (1-8). Default 6. */
        public Builder maxVolts(double v) { this.maxVolts = v; return this; }

        public CatalystCommand build() {
            return CatalystCommand.of(new CalCommand(drive, currentSlipAmps, rampVoltsPerSecond, maxVolts));
        }
    }

    private static final class CalCommand implements Command {

        private final SwerveSubsystem drive;
        private final double currentSlipAmps;
        private final double rampVoltsPerSecond;
        private final double maxVolts;
        private final NetworkTable nt;
        private final java.util.Set<Mechanism> required;
        private final SwerveRequest.SysIdSwerveTranslation request = new SwerveRequest.SysIdSwerveTranslation();
        private SlipCurrentDetector detector;

        CalCommand(SwerveSubsystem drive, double currentSlipAmps, double rampVoltsPerSecond, double maxVolts) {
            this.drive = drive;
            this.currentSlipAmps = currentSlipAmps;
            this.rampVoltsPerSecond = rampVoltsPerSecond;
            this.maxVolts = maxVolts;
            this.nt = NetworkTableInstance.getDefault()
                    .getTable("Catalyst").getSubTable("Calibration").getSubTable("SlipCurrent");
            this.required = java.util.Set.of(drive);
        }

        @Override
        public String name() {
            return "SlipCurrentCalibration";
        }

        @Override
        public java.util.Set<Mechanism> requirements() {
            return required;
        }

        @Override
        public void run(Coroutine coroutine) {
            SwerveDrivetrain<?, ?, ?> drivetrain = drive.getDrivetrain();
            var modules = drivetrain.getModules();
            int n = modules.length;
            StatusSignal<?>[] current = new StatusSignal<?>[n];
            for (int i = 0; i < n; i++) {
                current[i] = modules[i].getDriveMotor().getStatorCurrent();
            }
            // The stator current arrives a few times a second by default; the ramp needs it every loop.
            BaseStatusSignal.setUpdateFrequencyForAll(100, current);
            detector = new SlipCurrentDetector(rampVoltsPerSecond, maxVolts);
            nt.getEntry("Status").setString("running");
            nt.getEntry("SlipAmps").setDouble(Double.NaN);
            nt.getEntry("RecommendedAmps").setDouble(Double.NaN);
            nt.getEntry("Result").setString("");
            nt.getEntry("Snippet").setString("");
            nt.getEntry("BreakAmps").setDoubleArray(new double[0]);

            double volts = 0.0;
            while (!detector.done()) {
                drivetrain.setControl(request.withVolts(volts));
                coroutine.yield();
                BaseStatusSignal.refreshAll(current);
                var state = drivetrain.getState();
                double[] speed = new double[n];
                double[] amps = new double[n];
                for (int i = 0; i < n; i++) {
                    speed[i] = state.ModuleVelocities[i].velocity;
                    amps[i] = current[i].getValueAsDouble();
                }
                volts = detector.step(Timer.getTimestamp(), speed, amps);
                nt.getEntry("Volts").setDouble(detector.volts());
                nt.getEntry("PeakAmps").setDouble(detector.peakAmps());
                // Per wheel, every loop: what a recording needs to tell a light wheel from a jammed one.
                nt.getEntry("WheelAmps").setDoubleArray(detector.wheelAmps());
                nt.getEntry("WheelMps").setDoubleArray(speed);
            }
            drivetrain.setControl(request.withVolts(0.0));
            nt.getEntry("BreakAmps").setDoubleArray(detector.breakAmps());
            StringBuilder light = new StringBuilder();
            for (int i : detector.lightWheels()) {
                light.append(light.length() == 0 ? "; spun early, carrying little weight: module " : ", module ")
                        .append(i).append(String.format(" at %.0f A", detector.breakAmps()[i]));
            }

            if (detector.phase() == SlipCurrentDetector.Phase.SLIPPED) {
                double slip = detector.slipAmps();
                double set = detector.recommendedAmps();
                nt.getEntry("SlipAmps").setDouble(slip);
                nt.getEntry("RecommendedAmps").setDouble(set);
                nt.getEntry("Result").setString(String.format(
                        "Wheels slip at %.0f A (module %d first under load): set %.0f A%s",
                        slip, detector.slipWheel(), set, light));
                nt.getEntry("Snippet").setString(String.format("kSlipCurrent = Amps.of(%.0f); // measured %.1f A%s",
                        set, slip, Double.isFinite(currentSlipAmps) ? String.format(" (was %.0f A)", currentSlipAmps) : ""));
                nt.getEntry("Status").setString("done");
            } else {
                nt.getEntry("Status").setString("stopped: " + detector.reason());
            }
        }

        @Override
        public void onCancel() {
            // A cancelled coroutine is abandoned, not unwound: nothing after its last yield runs. Phoenix would keep
            // applying the last ramp voltage, and would again on the next enable until another request replaced it.
            drive.getDrivetrain().setControl(request.withVolts(0.0));
            nt.getEntry("Status").setString("stopped: cancelled before a wheel slipped");
        }
    }
}
