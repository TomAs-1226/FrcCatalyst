package frc.lib.catalyst.fieldlab;

import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.subsystems.swerve.SlipCurrentCalibration;
import frc.lib.catalyst.subsystems.swerve.SwerveSubsystem;
import frc.lib.catalyst.subsystems.swerve.WheelRadiusCalibration;
import frc.lib.catalyst.sysid.SysIdRoutine;

import org.wpilib.command3.Command;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

/**
 * The procedures worth running on a real field, ready to drop into a {@link TestCampaign}.
 *
 * <p>Most of these wrap a routine Catalyst already had. That is the point: the library has had
 * {@code WheelRadiusCalibration}, {@code SlipCurrentCalibration}, {@code SysIdRoutine} and
 * {@code SystemCheck} for a while, and what was missing was never the measurement — it was the thing
 * that runs nine of them in order, waits for a person, and survives a battery swap. The ones that are
 * genuinely new here are the ones a robot cannot do alone: odometry drift, which needs a tape measure,
 * and {@link #sweep} , which needs someone to say where the game piece landed.
 *
 * <h2>Order</h2>
 *
 * <p>Put them in a campaign roughly in this order, because each depends on the ones above it:
 *
 * <ol>
 *   <li>{@link #wheelRadius} — every distance the robot believes is scaled by this
 *   <li>{@link #slipCurrent} — the current limit above which odometry is measuring a spinning wheel
 *   <li>{@link #driveFeedforward} — kS/kV/kA, which every path and every velocity setpoint rests on
 *   <li>{@link #odometryDrift} — meaningless until the three above are right
 *   <li>{@link #sweep} — the shot map, or any other setpoint table
 *   <li>{@link #thermalSoak} — last, because it leaves everything hot
 * </ol>
 *
 * @since 2.0.0
 */
public final class Procedures {

    private Procedures() {}

    /**
     * Wheel radius, from a gyro arc against the encoders.
     *
     * <p>Wraps {@link WheelRadiusCalibration}, which publishes its own corrected radius; the campaign
     * adds the two things that routine cannot do for itself — it makes sure someone has cleared the
     * space first, and it asks for the tape-measure check afterwards, because the routine's answer is
     * only as good as the drive-base radius it was told.
     *
     * @param drive the drivetrain
     * @param currentWheelRadiusMeters what the robot believes today
     * @param driveBaseRadiusMeters centre to a module, from CAD
     */
    public static TestProcedure wheelRadius(SwerveSubsystem drive,
                                            double currentWheelRadiusMeters,
                                            double driveBaseRadiusMeters) {
        return TestProcedure.builder("wheel-radius", "Wheel radius")
                .why("Every distance the robot believes is this number times an encoder count. A wheel"
                        + " worn 1 mm down over a season is a 1.3% error on every path and every"
                        + " odometry-fused pose, and it is the cheapest thing on this list to measure.")
                .needsBattery(12.0)
                .step(TestStep.Confirm.of("clear a 2 m circle around the robot, then advance"))
                .step(TestStep.Act.of("spin in place and compare the gyro arc to the encoders",
                        () -> WheelRadiusCalibration.builder(drive)
                                .currentWheelRadius(currentWheelRadiusMeters)
                                .driveBaseRadius(driveBaseRadiusMeters)
                                .build(),
                        45.0))
                .step(TestStep.Measure.of(
                        "measure one wheel's diameter with calipers, in inches, as a cross-check",
                        "caliper-diameter-in", "in", 3.0, 4.5))
                .build();
    }

    /**
     * Slip current: the drive stator current at which the wheels break traction.
     *
     * <p>Wraps {@link SlipCurrentCalibration}. A value copied from another robot is wrong for this
     * one — a light robot's wheels let go far below a heavy one's — and above the real slip current
     * the odometry is measuring a wheel spinning on the carpet, which it believes is motion.
     *
     * @param drive the drivetrain
     * @param currentSlipAmps what the robot believes today
     */
    public static TestProcedure slipCurrent(SwerveSubsystem drive, double currentSlipAmps) {
        return TestProcedure.builder("slip-current", "Slip current")
                .why("Above this current the wheels spin and the odometry believes the robot moved."
                        + " It is also the drive stator limit, so guessing it high costs traction"
                        + " control and guessing it low costs acceleration.")
                .needsBattery(12.2)
                .step(TestStep.Confirm.of(
                        "put the front bumper squarely against a wall, wheels pointed straight, then advance"))
                .step(TestStep.Act.of("ramp the drive voltage until a wheel slips",
                        () -> SlipCurrentCalibration.builder(drive).currentSlipAmps(currentSlipAmps).build(),
                        30.0))
                .step(TestStep.Settle.of("let the drive motors cool", 20.0))
                .build();
    }

    /**
     * Drive feedforward: the four SysId sweeps, in one procedure.
     *
     * <p>Quasistatic and dynamic, forward and reverse. Each needs clear runway and the campaign asks
     * for the robot to be repositioned between the pairs, because a sweep that runs out of floor
     * half-way stops early and its data fits a line through the wrong half of the range.
     *
     * <p>The fitting is WPILib's SysId tool's job, or {@code tools/fieldlab.py}'s; the routine already
     * writes {@code SysIdRoutineLog} so both can read it.
     *
     * @param routine the drivetrain's SysId routine
     * @param runwayMeters how much clear floor the robot has; sizes the timeout
     */
    public static TestProcedure driveFeedforward(SysIdRoutine routine, double runwayMeters) {
        // A quasistatic sweep crawls; a dynamic one is over in a couple of seconds. Sizing the cap off
        // the runway rather than a constant is what stops a long field from wasting the slow sweeps
        // and a short one from slamming into a wall.
        double quasistatic = Math.max(6.0, Math.min(20.0, runwayMeters * 1.5));
        double dynamic = Math.max(2.0, Math.min(6.0, runwayMeters * 0.5));

        return TestProcedure.builder("drive-ff", "Drive feedforward (kS, kV, kA)")
                .why("kS, kV and kA are what every velocity setpoint and every path rests on. Numbers"
                        + " from Tuner X or from another team's robot are a starting point, not a"
                        + " measurement, and the difference shows up as a path that undershoots"
                        + " consistently.")
                .needsBattery(12.3)
                .step(TestStep.Confirm.of("put the robot at one end of a clear straight run, then advance"))
                .step(TestStep.Act.of("quasistatic, forward",
                        () -> routine.quasistatic(SysIdRoutine.Direction.FORWARD), quasistatic))
                .step(TestStep.Settle.of("settle", 3.0))
                .step(TestStep.Act.of("quasistatic, reverse",
                        () -> routine.quasistatic(SysIdRoutine.Direction.REVERSE), quasistatic))
                .step(TestStep.Settle.of("settle", 3.0))
                .step(TestStep.Confirm.of("reposition at the end of the run again, then advance"))
                .step(TestStep.Act.of("dynamic, forward",
                        () -> routine.dynamic(SysIdRoutine.Direction.FORWARD), dynamic))
                .step(TestStep.Settle.of("settle", 3.0))
                .step(TestStep.Act.of("dynamic, reverse",
                        () -> routine.dynamic(SysIdRoutine.Direction.REVERSE), dynamic))
                .build();
    }

    /**
     * Odometry drift over a closed loop, measured with a tape.
     *
     * <p>The one number on this list that a robot cannot get for itself. Drift is the difference
     * between where the robot believes it is and where it actually is, and only a person with a tape
     * measure knows the second half. Driving a closed loop and returning to the start makes the error
     * measurable: whatever the tape says is the accumulated error over the path the robot drove.
     *
     * <p>Run it three times. One loop tells you the magnitude; three tell you whether it is a
     * systematic scale error — which {@link #wheelRadius} fixes — or slip, which it does not.
     *
     * @param drive the drivetrain
     * @param loop a command that drives a closed loop and returns to roughly the start
     * @param loopSeconds how long that takes
     */
    public static TestProcedure odometryDrift(SwerveSubsystem drive,
                                              java.util.function.Supplier<Command> loop,
                                              double loopSeconds) {
        TestProcedure.Builder b = TestProcedure.builder("odometry-drift", "Odometry drift over a closed loop")
                .why("A robot cannot measure its own drift: it is the gap between belief and fact, and"
                        + " only a tape knows the fact. Three loops separate a systematic scale error"
                        + " (fix the wheel radius) from slip (fix the current limit or the driving).")
                .needsBattery(12.2)
                .step(TestStep.Confirm.of(
                        "mark the robot's starting position on the floor - tape at two bumper corners - then advance"));

        for (int i = 1; i <= 3; i++) {
            final int lap = i;
            b.step(TestStep.Act.of("lap " + lap + " of 3", loop, loopSeconds + 5.0));
            b.step(TestStep.Settle.of("let it come to rest", 2.0));
            b.step(TestStep.Measure.of(
                    "lap " + lap + ": measure from the tape to the same bumper corner, in metres",
                    "lap" + lap + "-error-m", "m", 0.0, 2.0));
        }
        return b.build();
    }

    /**
     * A setpoint sweep with a person scoring the result — the shot map, and anything shaped like it.
     *
     * <p>This is the procedure that fills a field slot, and the one that replaces a table inherited
     * from another robot with a table measured on this one. At each setpoint the robot is commanded,
     * given time to settle, told to fire, and then a person says what happened.
     *
     * <p>It is generic on purpose. The library cannot know what a shooter is, but every table of this
     * shape — distance to RPM, distance to hood angle, angle to time-of-flight — is the same
     * procedure: set, settle, act, observe. A team's shot map becomes about thirty lines of
     * configuration rather than a new routine.
     *
     * <pre>{@code
     * Procedures.sweep("shot-map", "Shot map",
     *         hood::setAngle, shooter::setRpm, shooter::atSpeed, shooter::fireOnce,
     *         List.of(Setpoint.of(2.0, 42, 1800), Setpoint.of(3.0, 38, 2100)),
     *         "did it score? 1 = in, 0.5 = rim, 0 = miss");
     * }</pre>
     *
     * <p>Note the order of a {@link Setpoint}: the distance the table is keyed on, then the value for
     * {@code first}, then the value for {@code second}. Here {@code first} is the hood and
     * {@code second} the flywheel, because that is the order they were passed in.
     *
     * @param id procedure id
     * @param title what the operator sees
     * @param first applied at each setpoint — the hood angle, say
     * @param second applied at each setpoint — the flywheel speed, say
     * @param ready true when both have settled; the sweep waits for it before acting
     * @param act fired once the setpoints are ready
     * @param setpoints the table to sweep
     * @param question what the operator is asked after each shot
     */
    public static TestProcedure sweep(String id,
                                      String title,
                                      DoubleConsumer first,
                                      DoubleConsumer second,
                                      BooleanSupplier ready,
                                      java.util.function.Supplier<Command> act,
                                      List<Setpoint> setpoints,
                                      String question) {
        TestProcedure.Builder b = TestProcedure.builder(id, title)
                .why("A table measured on this robot, at these setpoints, instead of one inherited from"
                        + " another. The robot can hold a setpoint and fire; only a person can say"
                        + " where it went.")
                .needsBattery(12.3);

        for (Setpoint sp : setpoints) {
            b.step(TestStep.Act.of(
                    String.format("%s: %.2f -> %.0f / %.0f", sp.label(), sp.at(), sp.first(), sp.second()),
                    () -> Command.noRequirements(co -> {
                                first.accept(sp.first());
                                second.accept(sp.second());
                                CatalystLog.log("FieldLab/" + id + "/At", sp.at());
                                CatalystLog.log("FieldLab/" + id + "/First", sp.first());
                                CatalystLog.log("FieldLab/" + id + "/Second", sp.second());
                                // No timeout argument here on purpose: alpha-6's commands v3 has only
                                // the one-argument waitUntil, and keeping this file identical on both
                                // WPILib lines is worth more than a second guard. The step's own 8 s
                                // cap below already bounds a setpoint that never settles, so nothing
                                // hangs — the sweep simply records that row as having hit its cap.
                                co.waitUntil(ready);
                            })
                            .named("setpoint " + sp.label()),
                    8.0));
            b.step(TestStep.Act.of(sp.label() + ": fire", act, 6.0));
            // Recorded against the setpoint's own key, so the analyser can pair the observation with
            // the commanded values without relying on step ordering.
            b.step(TestStep.Measure.of(sp.label() + ": " + question, sp.key(), "score", 0.0, 1.0));
        }
        return b.build();
    }

    /**
     * One row of a {@link #sweep}.
     *
     * @param label what the operator sees, e.g. {@code "3.0 m"}
     * @param at the independent variable the table is keyed on — distance, usually
     * @param first the first commanded value
     * @param second the second commanded value
     */
    public record Setpoint(String label, double at, double first, double second) {

        /** A setpoint labelled from its own key value. */
        public static Setpoint of(double at, double first, double second) {
            return new Setpoint(String.format("%.2f", at), at, first, second);
        }

        /** The key this row's observation is recorded under. */
        public String key() {
            return String.format("at-%s", label.replaceAll("[^A-Za-z0-9._-]+", "_"));
        }
    }

    /**
     * Drive a duty cycle for a while and watch the temperatures and currents.
     *
     * <p>Last in a campaign, because it leaves everything hot. What it is for: every current limit on
     * the robot is set against a thermal envelope nobody has measured, and the first time that
     * envelope is tested for real is usually the third match of an elimination, where the symptom is a
     * drivetrain that feels fine for two minutes and then does not.
     *
     * <p>It measures nothing by itself — the mechanisms already publish their own temperatures and
     * currents, and {@code MotorHistory} is already recording the peaks. The procedure's job is to
     * apply a realistic load for long enough that those numbers mean something, and to have a person
     * put a hand on the gearboxes afterwards, which finds a dragging bearing that no telemetry will.
     *
     * @param cycle one repetition of a representative driving pattern
     * @param cycleSeconds how long one repetition takes
     * @param minutes how long to soak for
     */
    public static TestProcedure thermalSoak(java.util.function.Supplier<Command> cycle,
                                            double cycleSeconds,
                                            double minutes) {
        int repetitions = (int) Math.max(1, Math.round(minutes * 60.0 / Math.max(1.0, cycleSeconds)));

        TestProcedure.Builder b = TestProcedure.builder("thermal-soak", "Thermal soak")
                .why("Every current limit on the robot is set against a thermal envelope nobody has"
                        + " measured. This applies a realistic load long enough for the temperatures"
                        + " and currents the mechanisms already publish to mean something.")
                .needsBattery(12.4)
                .step(TestStep.Confirm.of("clear the field; this runs for about "
                        + Math.round(minutes) + " minutes. Advance to start."));

        for (int i = 1; i <= repetitions; i++) {
            b.step(TestStep.Act.of("cycle " + i + " of " + repetitions, cycle, cycleSeconds + 5.0));
        }

        b.step(TestStep.Measure.of(
                "put a hand on each gearbox: how many are noticeably hotter than the others?",
                "hot-gearboxes", "count", 0.0, 8.0));
        b.step(TestStep.Confirm.of("note anything that smells, squeals or drags, then advance"));
        return b.build();
    }

    /**
     * Every procedure that needs no robot-specific knowledge, for a campaign you can assemble from a
     * drivetrain alone.
     *
     * <p>A reasonable first field day: the three calibrations plus drift, which is most of what makes
     * a drivetrain trustworthy, and about twenty-five minutes.
     */
    public static List<TestProcedure> drivetrainBasics(SwerveSubsystem drive,
                                                       SysIdRoutine driveSysId,
                                                       double currentWheelRadiusMeters,
                                                       double driveBaseRadiusMeters,
                                                       double currentSlipAmps,
                                                       double runwayMeters,
                                                       java.util.function.Supplier<Command> closedLoop,
                                                       double loopSeconds) {
        List<TestProcedure> out = new ArrayList<>(4);
        out.add(wheelRadius(drive, currentWheelRadiusMeters, driveBaseRadiusMeters));
        out.add(slipCurrent(drive, currentSlipAmps));
        out.add(driveFeedforward(driveSysId, runwayMeters));
        out.add(odometryDrift(drive, closedLoop, loopSeconds));
        return out;
    }
}
