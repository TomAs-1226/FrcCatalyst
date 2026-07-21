package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.mechanisms.ClawMechanism;
import frc.lib.catalyst.mechanisms.DifferentialWristMechanism;
import frc.lib.catalyst.mechanisms.FlywheelMechanism;
import frc.lib.catalyst.mechanisms.LinearMechanism;
import frc.lib.catalyst.mechanisms.PneumaticMechanism;
import frc.lib.catalyst.mechanisms.RollerMechanism;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.lib.catalyst.mechanisms.TurretMechanism;
import frc.lib.catalyst.mechanisms.WinchMechanism;
import frc.lib.catalyst.sim.SimDashboard;

/**
 * A self-contained demonstration of the <b>generic</b> {@link SimDashboard}.
 *
 * <p>It builds one of <em>every</em> Catalyst mechanism kind — linear,
 * rotational, roller, flywheel, turret, claw, differential wrist, winch and
 * pneumatic — on its own CAN IDs (30–39) and registers each with the dashboard.
 * Open <a href="http://localhost:5806">localhost:5806</a> in sim and every
 * mechanism shows a live, fitting widget you can drive. Nothing here is specific
 * to this year's game: the exact same calls work against your real robot's
 * mechanisms.
 *
 * <p>This is the proof that the simulation is no longer "dead" / single-robot:
 * the cockpit adapts to whatever mechanisms you register, including a team's own
 * {@link frc.lib.catalyst.mechanisms.CatalystMechanism} subclass.
 *
 * <p>The mechanisms are {@code SubsystemBase}s, so simply constructing them
 * registers their {@code simulationPeriodic()} with the {@code CommandScheduler}
 * — the physics run automatically whenever the robot is enabled.
 *
 * <p>This lab drives each mechanism <em>individually</em>. For the other half —
 * all nine bound into one guarded, logged state machine so you ask for a whole-robot
 * state instead of moving mechanisms one at a time — see
 * {@link ShowcaseSuperstructure}, which builds a {@link
 * frc.lib.catalyst.statemachine.robot.Superstructure} over exactly these mechanism
 * kinds. It is kept separate from this lab on purpose: the state machine installs a
 * holding default command on each mechanism, which would fight the manual sliders here.
 */
public final class MechanismShowcase {

    private final SimDashboard dash = new SimDashboard(5806).title("Catalyst Mechanism Lab");

    private final LinearMechanism elevator = new LinearMechanism(
            LinearMechanism.Config.builder()
                    .name("Elevator")
                    .motor(30)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(10.0)
                    .drumRadius(0.022)
                    .range(0.0, 0.60)
                    .mass(5.0)
                    .currentLimit(60)
                    .pid(60, 0, 0)
                    .motionMagic(1.6, 4.0, 40.0)
                    .position("DOWN", 0.0)
                    .position("UP", 0.55)
                    .build());

    private final RotationalMechanism arm = new RotationalMechanism(
            RotationalMechanism.Config.builder()
                    .name("Arm")
                    .motor(31)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(50.0)
                    .length(0.4)
                    .mass(2.5)
                    .range(-10, 110)
                    .currentLimit(40)
                    .pid(60, 0, 1.0)
                    .gravityGain(0.30)
                    .motionMagic(120, 240, 2400)
                    .build());

    private final RollerMechanism roller = new RollerMechanism(
            RollerMechanism.Config.builder()
                    .name("Roller")
                    .motor(32)
                    .intakeSpeed(0.9)
                    .ejectSpeed(-0.7)
                    .currentLimit(30)
                    .stallDetection(25, 0.2)
                    .build());

    private final FlywheelMechanism flywheel = new FlywheelMechanism(
            FlywheelMechanism.Config.builder()
                    .name("Flywheel")
                    .motor(33)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(1.0)
                    .moi(0.0016)
                    .currentLimit(80)
                    .pid(0.45, 0, 0)
                    .feedforward(0.18, 0.119)
                    .velocityTolerance(2.0)
                    .build());

    private final TurretMechanism turret = new TurretMechanism(
            TurretMechanism.Config.builder()
                    .name("Turret")
                    .motor(34)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(30.0)
                    .range(-180, 180)
                    .currentLimit(40)
                    .pid(60, 0, 0.8)
                    .feedforward(0.12, 3.0)
                    .motionMagic(16, 60, 600)
                    .tolerance(2.0)
                    .simMOI(0.03)
                    .build());

    private final ClawMechanism claw = new ClawMechanism(
            ClawMechanism.Config.builder()
                    .name("Claw")
                    .motor(35)
                    .currentLimit(30)
                    .closeVoltage(3.0)
                    .openVoltage(-3.0)
                    .holdVoltage(1.0)
                    .build());

    private final DifferentialWristMechanism wrist = new DifferentialWristMechanism(
            DifferentialWristMechanism.Config.builder()
                    .name("Wrist")
                    .leftMotor(36)
                    .rightMotor(37)
                    .gearRatio(40.0)
                    .pitchRange(-40, 40)
                    .rollRange(-40, 40)
                    .pid(40, 0, 0.5)
                    .motionMagic(2.0, 6.0, 60.0)
                    .tolerance(2.0)
                    .build());

    private final WinchMechanism winch = new WinchMechanism(
            WinchMechanism.Config.builder()
                    .name("Winch")
                    .motor(38)
                    .gearRatio(25.0)
                    .spoolRadius(0.02)
                    .range(0.0, 0.5)
                    .loadMass(8.0)
                    .build());

    private final PneumaticMechanism solenoid = new PneumaticMechanism(
            PneumaticMechanism.Config.builder()
                    .name("Grabber")
                    .doubleSolenoid(PneumaticsModuleType.REVPH, 6, 7)
                    .build());

    public MechanismShowcase() {
        dash.add(elevator)
                .slider("Height (m)", 0.0, 0.60, v -> sched(elevator.goTo(v)))
                .command("Stow", () -> elevator.goTo("DOWN"))
                .command("Top", () -> elevator.goTo("UP"));

        dash.add(arm)
                .slider("Angle (deg)", -10, 110, v -> sched(arm.goTo(v)))
                .command("Zero", () -> arm.goTo(0))
                .command("Up", () -> arm.goTo(90));

        dash.add(roller)
                .command("Intake", roller::intake)
                .command("Eject", roller::eject)
                .command("Stop", () -> roller.runAtSpeed(0))
                .slider("Speed", -1.0, 1.0, v -> sched(roller.runAtSpeed(v)))
                .toggle("Game piece", roller::setSimHasPiece);

        dash.add(flywheel)
                .slider("Target (rps)", 0.0, 90.0, v -> sched(flywheel.spinUp(v)))
                .command("Spin 60", () -> flywheel.spinUp(60))
                .command("Stop", () -> flywheel.runVoltage(0));

        dash.add(turret)
                .slider("Angle (deg)", -180, 180, v -> sched(turret.goToAngle(v)))
                .command("Forward", turret::lockForward)
                .command("Hold", turret::holdAngle);

        dash.add(claw)
                .command("Open", claw::open)
                .command("Close", claw::close)
                .command("Hold", claw::hold)
                .toggle("Game piece", claw::setSimHasPiece);

        dash.add(wrist)
                .slider("Pitch (deg)", -40, 40, v -> sched(wrist.goTo(v, wrist.getRollSetpoint())))
                .slider("Roll (deg)", -40, 40, v -> sched(wrist.goTo(wrist.getPitchSetpoint(), v)))
                .command("Flat", () -> wrist.goTo(0, 0));

        dash.add(winch)
                .slider("Speed", -1.0, 1.0, v -> sched(winch.runAtSpeed(v)))
                .command("Extend", winch::extend)
                .command("Retract", winch::retract)
                .command("Stop", () -> winch.runAtSpeed(0));

        dash.add(solenoid)
                .command("Extend", solenoid::extend)
                .command("Retract", solenoid::retract)
                .command("Off", solenoid::off)
                .command("Toggle", solenoid::toggle);

        dash.start();
    }

    /** Run queued browser commands + snapshot state. Call once per loop in sim. */
    public void update() {
        dash.update();
    }

    /** Schedule a command on the main thread (slider callbacks already run there). */
    private static void sched(Command c) {
        CommandScheduler.getInstance().schedule(c);
    }
}
