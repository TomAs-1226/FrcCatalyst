package frc.robot;

import frc.lib.catalyst.mechanisms.ClawMechanism;
import frc.lib.catalyst.mechanisms.DifferentialWristMechanism;
import frc.lib.catalyst.mechanisms.FlywheelMechanism;
import frc.lib.catalyst.mechanisms.LinearMechanism;
import frc.lib.catalyst.mechanisms.PneumaticMechanism;
import frc.lib.catalyst.mechanisms.RollerMechanism;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.lib.catalyst.mechanisms.TurretMechanism;
import frc.lib.catalyst.mechanisms.WinchMechanism;

import frc.lib.catalyst.statemachine.Handle;
import frc.lib.catalyst.statemachine.Routing;
import frc.lib.catalyst.statemachine.goals.ClawGoal;
import frc.lib.catalyst.statemachine.goals.FlywheelGoal;
import frc.lib.catalyst.statemachine.goals.LinearGoal;
import frc.lib.catalyst.statemachine.goals.PneumaticGoal;
import frc.lib.catalyst.statemachine.goals.RollerGoal;
import frc.lib.catalyst.statemachine.goals.RotationalGoal;
import frc.lib.catalyst.statemachine.goals.TurretGoal;
import frc.lib.catalyst.statemachine.goals.WinchGoal;
import frc.lib.catalyst.statemachine.goals.WristGoal;
import frc.lib.catalyst.statemachine.mech.Mechanisms;
import frc.lib.catalyst.statemachine.robot.Superstructure;

import java.util.function.BooleanSupplier;

/**
 * A whole-robot state machine built over every mechanism in {@link MechanismShowcase}.
 *
 * <p>The showcase already builds one of each of the nine Catalyst mechanism kinds so you can
 * poke them individually at <a href="http://localhost:5806">localhost:5806</a>. That proves the
 * mechanisms work; it does not prove they work <em>together</em>. This class is the second half:
 * all nine are bound into one {@link Superstructure}, so instead of driving an elevator and then
 * remembering to move the arm, you ask for a state and the machine decides whether that is legal,
 * in what order things move, and whether you actually got there.
 *
 * <h2>What each required feature looks like here</h2>
 *
 * <ul>
 *   <li><b>All nine mechanisms bound.</b> Elevator, arm, wrist, roller, flywheel, turret, claw,
 *       winch and solenoid are each wrapped by the matching {@link Mechanisms} factory, so the
 *       handle carries its goal type and handing a {@code RotationalGoal} to the elevator does not
 *       compile.</li>
 *   <li><b>A staged edge</b> on {@code STOW -> AIM}. The elevator goes up first; only once it has
 *       arrived do the arm and wrist swing out. Without the staging the arm sweeps through the
 *       space the elevator has not vacated yet, which on a real robot is a bent arm.</li>
 *   <li><b>An entry guard</b> on {@code SCORE}: the claw must actually report a game piece. Without
 *       it, a mistimed button opens an empty claw at the far end of the field and the driver has no
 *       idea why nothing scored. Flip the claw panel's "Game piece" toggle in the mechanism lab to
 *       satisfy it, and watch the request get refused with a reason until you do.</li>
 *   <li><b>An interlock</b> named {@code winchStowed}. Once the winch is fully extended, every state
 *       except {@code CLIMB} and {@code STOW} is refused — an interlock is global, so it catches the
 *       transition you forgot to think about as well as the ones you did.</li>
 *   <li><b>A released mechanism:</b> {@code CLIMB} releases the turret, handing it back so a driver
 *       (or the mechanism lab's turret slider) can move it freely while the machine holds
 *       everything else.</li>
 * </ul>
 *
 * <h2>Why the states are ordered the way they are</h2>
 *
 * <p>{@code STOW} is the hub: it connects to and from everything except {@code SCORE}.
 * {@code SCORE} is reachable only from {@code AIM}, which is the point of a transition graph — you
 * cannot score from the stowed position, so that edge simply does not exist and a request for it is
 * refused rather than half-attempted. Because routing is {@link Routing#SHORTEST_PATH}, asking for
 * {@code SCORE} from {@code STOW} is not an error either: the machine plans {@code STOW -> AIM ->
 * SCORE} and runs both hops, proving each one before starting the next.
 *
 * @since 1.2.0
 */
public final class ShowcaseSuperstructure {

    /** Not instantiable — this is a factory holder for one configured machine. */
    private ShowcaseSuperstructure() {
    }

    /** The states this demo robot can be in. */
    public enum State {
        /** Everything folded inside the frame. The safe posture and the hub of the graph. */
        STOW,
        /** Arm down, claw open, roller pulling, grabber deployed. */
        INTAKE,
        /** Piece held, arm tucked up, safe to drive across the field. */
        CARRY,
        /** Elevator and arm out, flywheel spun up, turret offset. */
        AIM,
        /** Release: claw opens and the roller ejects. Only reachable from AIM. */
        SCORE,
        /** Winch extended, turret released to the driver. */
        CLIMB
    }

    /**
     * Builds the machine over the showcase's mechanisms.
     *
     * <p>Every mechanism passed in must have no default command of its own. {@code build()}
     * installs a {@code GoalRunner} as each one's default command, and it deliberately fails the
     * build rather than silently replacing a default somebody else set — a stomped
     * {@code setDefaultCommand(holdPosition())} is a very confusing afternoon.
     *
     * @param elevator   the linear stage
     * @param arm        the rotational shoulder
     * @param wrist      the differential wrist
     * @param roller     the intake roller
     * @param flywheel   the shooter wheel
     * @param turret     the turret
     * @param claw       the gripper
     * @param winch      the climber winch
     * @param solenoid   the pneumatic grabber
     * @param climbArmed the CLIMB entry guard — normally an endgame test, wired here to a toggle
     *                   in the mechanism lab so it can be exercised in simulation
     * @return the built, validated superstructure, already holding its mechanisms' defaults
     */
    public static Superstructure<State> build(
            LinearMechanism elevator,
            RotationalMechanism arm,
            DifferentialWristMechanism wrist,
            RollerMechanism roller,
            FlywheelMechanism flywheel,
            TurretMechanism turret,
            ClawMechanism claw,
            WinchMechanism winch,
            PneumaticMechanism solenoid,
            BooleanSupplier climbArmed) {

        Superstructure.Builder<State> b =
                Superstructure.builder(State.class, "ShowcaseSuperstructure");

        // Each bind returns a type-carrying handle. Keep them local: nothing outside this method
        // has any business naming a mechanism, which is the whole point of asking for a state.
        Handle<LinearGoal> hElevator = b.bind("elevator", Mechanisms.linear("elevator", elevator));
        Handle<RotationalGoal> hArm = b.bind("arm", Mechanisms.rotational("arm", arm));
        Handle<WristGoal> hWrist = b.bind("wrist", Mechanisms.wrist("wrist", wrist));
        Handle<RollerGoal> hRoller = b.bind("roller", Mechanisms.roller("roller", roller));
        Handle<FlywheelGoal> hFlywheel = b.bind("flywheel", Mechanisms.flywheel("flywheel", flywheel));
        Handle<TurretGoal> hTurret = b.bind("turret", Mechanisms.turret("turret", turret));
        Handle<ClawGoal> hClaw = b.bind("claw", Mechanisms.claw("claw", claw));
        Handle<WinchGoal> hWinch = b.bind("winch", Mechanisms.winch("winch", winch));
        Handle<PneumaticGoal> hGrabber = b.bind("grabber", Mechanisms.pneumatic("grabber", solenoid));

        return b
                .logPrefix("Showcase")
                .alertSubsystem("Showcase")
                .initialState(State.STOW)
                .routing(Routing.SHORTEST_PATH)
                .defaultTimeout(4.0)
                .historyCapacity(50)

                // The safe posture, stated once. Every state inherits it unless it says otherwise,
                // so the mechanism you forget in one state does not stay where the last state
                // parked it.
                .defaults(s -> s
                        .set(hWrist, WristGoal.level())
                        .set(hRoller, RollerGoal.idle())
                        .set(hFlywheel, FlywheelGoal.idle())
                        .set(hTurret, TurretGoal.forward())
                        .set(hClaw, ClawGoal.hold())
                        .set(hWinch, WinchGoal.stop())
                        .set(hGrabber, PneumaticGoal.retracted()))

                .state(State.STOW, s -> s
                        .set(hElevator, LinearGoal.preset("DOWN"))
                        .set(hArm, RotationalGoal.degrees(0)))

                // The roller goal carries its own 3 s cap, so INTAKE completes on a piece or on
                // that cap. The state deadline is longer than the cap on purpose: a deadline that
                // fires before the goal can possibly finish reports a fault that is really a
                // configuration mistake.
                .state(State.INTAKE, s -> s
                        .set(hElevator, LinearGoal.meters(0.05))
                        .set(hArm, RotationalGoal.degrees(-5))
                        .set(hWrist, WristGoal.of(-20, 0))
                        .set(hRoller, RollerGoal.intakeUntilPiece(3.0))
                        .set(hClaw, ClawGoal.open())
                        .set(hGrabber, PneumaticGoal.extended())
                        .timeout(6.0))

                .state(State.CARRY, s -> s
                        .set(hElevator, LinearGoal.meters(0.15))
                        .set(hArm, RotationalGoal.degrees(15))
                        .set(hClaw, ClawGoal.close()))

                .state(State.AIM, s -> s
                        .set(hElevator, LinearGoal.meters(0.35))
                        .set(hArm, RotationalGoal.degrees(45))
                        .set(hWrist, WristGoal.of(10, 0))
                        .set(hFlywheel, FlywheelGoal.rps(45))
                        .set(hTurret, TurretGoal.robotRelative(30))
                        .set(hClaw, ClawGoal.close()))

                // settleFor guards against a mechanism clipping through its tolerance band on the
                // way past: everything has to read at-goal continuously for 0.2 s before the claw
                // is allowed to have counted as open.
                .state(State.SCORE, s -> s
                        .set(hElevator, LinearGoal.meters(0.35))
                        .set(hArm, RotationalGoal.degrees(45))
                        .set(hWrist, WristGoal.of(10, 0))
                        .set(hFlywheel, FlywheelGoal.rps(45))
                        .set(hTurret, TurretGoal.robotRelative(30))
                        .set(hClaw, ClawGoal.open())
                        .set(hRoller, RollerGoal.eject(0.4))
                        .settleFor(0.2)
                        .entryGuard(claw::hasPiece, "no piece"))

                // The winch is slow and open-loop, so it gets a long deadline; if it blows anyway
                // the machine drives back to STOW rather than sitting half-climbed.
                .state(State.CLIMB, s -> s
                        .set(hElevator, LinearGoal.preset("DOWN"))
                        .set(hArm, RotationalGoal.degrees(0))
                        .set(hWinch, WinchGoal.extend())
                        .release(hTurret)
                        .entryGuard(climbArmed, "not armed")
                        .timeout(8.0)
                        .recoverTo(State.STOW))

                // The graph. STOW is the hub; SCORE hangs off AIM alone, so "score from stowed"
                // is not a thing this robot can be asked to do in one hop.
                .allowBoth(State.STOW, State.INTAKE)
                .allowBoth(State.STOW, State.CARRY)
                .allowBoth(State.STOW, State.AIM)
                .allowBoth(State.STOW, State.CLIMB)
                .allow(State.INTAKE, State.CARRY)
                .allowBoth(State.CARRY, State.AIM)
                .allow(State.AIM, State.SCORE)
                .allow(State.SCORE, State.CARRY, State.STOW)

                // Raise the elevator BEFORE the arm and wrist swing out, or the arm hits the
                // chassis. Anything not named in a stage moves immediately, in parallel with
                // stage 0 — here that is the flywheel and the turret, which have nothing to hit.
                .edge(State.STOW, State.AIM, e -> e
                        .stage(hElevator)
                        .stage(hArm, hWrist)
                        .timeout(5.0))

                // Global and positively named: "the winch is stowed". While that is false, every
                // state other than CLIMB and STOW is refused, whichever edge was asked for.
                .interlock("winchStowed",
                        () -> !winch.isFullyExtended(),
                        s -> s != State.CLIMB && s != State.STOW)

                .build();
    }
}
