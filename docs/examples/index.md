---
layout: default
title: Examples
nav_order: 6
has_children: false
---

# Examples
{: .no_toc }

Complete, copy-pasteable robot examples built with FrcCatalyst.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Basic Robot: Elevator + Intake

A simple competition robot with an elevator for scoring and a roller intake for game piece handling. This is the minimal starting point most FRC teams need.

### Full RobotContainer

```java
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.catalyst.mechanisms.LinearMechanism;
import frc.lib.catalyst.mechanisms.RollerMechanism;
import frc.lib.catalyst.hardware.MotorType;

public class RobotContainer {

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // ── Elevator ──────────────────────────────────────────────
    private final LinearMechanism elevator = new LinearMechanism(
        LinearMechanism.Config.builder()
            .name("Elevator")
            .motor(1)                         // TalonFX CAN ID 1
            .follower(2, true)                // Follower on CAN ID 2, opposed
            .motorType(MotorType.KRAKEN_X60)
            .gearRatio(10.0)                  // 10:1 reduction
            .drumRadius(0.0254)               // 1-inch spool radius
            .stages(2)                        // 2-stage cascade
            .range(0.0, 1.2)                  // 0 to 1.2 meters travel
            .mass(5.0)                        // 5 kg carriage mass
            .pid(50, 0, 0.5)                  // Position PID gains
            .gravityGain(0.35)                // Gravity compensation voltage
            .motionMagic(2.0, 4.0, 20.0)      // Cruise vel, accel, jerk
            .currentLimit(40)                 // 40A supply current limit
            .maxTemperature(70)               // Safety cutoff at 70C
            .reverseLimitSwitch(0, true)      // DIO 0, auto-zero on contact
            .position("STOW", 0.0)
            .position("INTAKE", 0.15)
            .position("MID", 0.6)
            .position("HIGH", 1.1)
            .build()
    );

    // ── Intake ────────────────────────────────────────────────
    private final RollerMechanism intake = new RollerMechanism(
        RollerMechanism.Config.builder()
            .name("Intake")
            .motor(3)                         // TalonFX CAN ID 3
            .brakeMode(false)                 // Coast when stopped
            .intakeSpeed(0.8)                 // 80% power for intake
            .ejectSpeed(-0.6)                 // 60% power for eject
            .stallDetection(25, 0.2)          // 25A for 0.2s = game piece
            .beamBreak(1)                     // DIO 1 beam break sensor
            .build()
    );

    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
        // Elevator holds its position when no command is running
        elevator.setDefaultCommand(elevator.holdPosition());
    }

    private void configureBindings() {
        // ── Elevator presets ──
        operator.a().onTrue(elevator.goTo("STOW"));
        operator.b().onTrue(elevator.goTo("INTAKE"));
        operator.x().onTrue(elevator.goTo("MID"));
        operator.y().onTrue(elevator.goTo("HIGH"));

        // ── Manual elevator jog ──
        operator.leftBumper().whileTrue(
            elevator.jog(() -> -operator.getLeftY() * 4.0)
        );

        // ── Zero elevator at current position ──
        operator.start().onTrue(elevator.zero());

        // ── Intake controls ──
        operator.rightTrigger(0.3).whileTrue(intake.intake());
        operator.leftTrigger(0.3).whileTrue(intake.eject());
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
            // Score preloaded game piece
            elevator.goTo("HIGH"),
            Commands.waitSeconds(0.5),
            intake.eject(),
            Commands.waitSeconds(0.3),

            // Stow and prepare for teleop
            Commands.parallel(
                elevator.goTo("STOW"),
                intake.stopCommand()
            )
        );
    }
}
```

### Key Takeaways

1. **Mechanism config is self-documenting** — the builder reads like a spec sheet
2. **Commands are one-liners** — `elevator.goTo("HIGH")` just works
3. **Safety is built in** — temperature cutoff, current limits, and limit switch auto-zeroing happen automatically
4. **Autonomous is clean** — compose mechanism commands with `Commands.sequence()` and `Commands.parallel()`

---

## Intermediate Robot: Elevator + Arm + Intake + Shooter

A more complex robot with coordinated mechanisms using `SuperstructureCoordinator`.

### Mechanisms

```java
// ── Elevator ──
private final LinearMechanism elevator = new LinearMechanism(
    LinearMechanism.Config.builder()
        .name("Elevator")
        .motor(1).follower(2, true)
        .motorType(MotorType.KRAKEN_X60)
        .gearRatio(10.0).drumRadius(0.0254).stages(2)
        .range(0.0, 1.2).mass(5.0)
        .pid(50, 0, 0.5).gravityGain(0.35)
        .motionMagic(2.0, 4.0, 20.0)
        .currentLimit(40).maxTemperature(70)
        .position("STOW", 0.0)
        .position("INTAKE", 0.15)
        .position("SCORE_HIGH", 1.1)
        .build()
);

// ── Arm ──
private final RotationalMechanism arm = new RotationalMechanism(
    RotationalMechanism.Config.builder()
        .name("Arm")
        .motor(3)
        .motorType(MotorType.KRAKEN_X60)
        .gearRatio(50.0)
        .length(0.5).mass(3.0)
        .range(-10, 120)
        .useCosineGravity(true)
        .pid(80, 0, 1.0).gravityGain(0.4)
        .motionMagic(200, 400, 2000)
        .position("STOW", 0.0)
        .position("INTAKE", 15.0)
        .position("SCORE", 100.0)
        .build()
);

// ── Intake ──
private final RollerMechanism intake = new RollerMechanism(
    RollerMechanism.Config.builder()
        .name("Intake")
        .motor(5)
        .intakeSpeed(0.8).ejectSpeed(-0.6)
        .stallDetection(25, 0.2)
        .beamBreak(0)
        .build()
);

// ── Shooter ──
private final FlywheelMechanism shooter = new FlywheelMechanism(
    FlywheelMechanism.Config.builder()
        .name("Shooter")
        .motor(6).secondMotor(7)
        .motorType(MotorType.KRAKEN_X60)
        .gearRatio(1.5)
        .pid(0.3, 0, 0).feedforward(0.12, 0.11)
        .velocityTolerance(3.0)
        .build()
);
```

### SuperstructureCoordinator

```java
// ── Coordinate elevator + arm together ──
private final SuperstructureCoordinator superstructure =
    new SuperstructureCoordinator()
        .withLinear("elevator", elevator)
        .withRotational("arm", arm);

// In constructor:
superstructure.defineState("STOW")
    .setLinear("elevator", 0.0)
    .setRotational("arm", 0.0)
    .done();

superstructure.defineState("INTAKE")
    .setLinear("elevator", 0.15)
    .setRotational("arm", 15.0)
    .done();

superstructure.defineState("SCORE_HIGH")
    .setLinear("elevator", 1.1)
    .setRotational("arm", 100.0)
    .done();

// Custom transition: retract arm first, then raise elevator, then extend arm
superstructure.addTransitionRule("STOW", "SCORE_HIGH",
    (fromState, toState) -> arm.goTo("STOW")
        .andThen(elevator.goTo("SCORE_HIGH"))
        .andThen(arm.goTo("SCORE"))
);
```

### The same robot with `Superstructure` (v1.2.0+)

`SuperstructureCoordinator` is deprecated as of v1.2.0. The version above still compiles and still
runs, but it can only place the elevator and the arm — the intake and the shooter are left out of
the states entirely, because the coordinator has no way to hold a roller or a flywheel goal. Here
is the same four-mechanism robot written against
`frc.lib.catalyst.statemachine.robot.Superstructure`, which does.

Three things are different in kind, not just in spelling. States are enum constants rather than
strings, so a typo is a compile error instead of a rejected request at a regional. The transitions
you declare are the *only* transitions the robot will make — an edge you did not write is refused,
with a reason, and logged. And `current()` is only ever a state the robot was measured to have
reached, so a transition that times out leaves the machine where the robot actually is.

```java
import frc.lib.catalyst.statemachine.Handle;
import frc.lib.catalyst.statemachine.goals.FlywheelGoal;
import frc.lib.catalyst.statemachine.goals.LinearGoal;
import frc.lib.catalyst.statemachine.goals.RollerGoal;
import frc.lib.catalyst.statemachine.goals.RotationalGoal;
import frc.lib.catalyst.statemachine.mech.Mechanisms;
import frc.lib.catalyst.statemachine.robot.Superstructure;

public enum SuperState { STOW, INTAKE, SCORE_HIGH }

private final Superstructure<SuperState> superstructure;

public RobotContainer() {
    var b = Superstructure.builder(SuperState.class, "Superstructure");

    // bind() hands back a typed handle. Passing a RotationalGoal to the elevator
    // handle below would not compile.
    Handle<LinearGoal>     elev  = b.bind("elevator", Mechanisms.linear("elevator", elevator));
    Handle<RotationalGoal> pivot = b.bind("arm",      Mechanisms.rotational("arm", arm));
    Handle<RollerGoal>     roll  = b.bind("intake",   Mechanisms.roller("intake", intake));
    Handle<FlywheelGoal>   fly   = b.bind("shooter",  Mechanisms.flywheel("shooter", shooter));

    superstructure = b
        // The safe posture, stated once. Every state inherits it unless it says
        // otherwise, so the mechanism you forget in one state does not stay wherever
        // the previous state parked it.
        .defaults(s -> s
            .set(roll, RollerGoal.idle())
            .set(fly,  FlywheelGoal.idle()))

        .state(SuperState.STOW, s -> s
            .set(elev,  LinearGoal.meters(0.0))
            .set(pivot, RotationalGoal.degrees(0.0)))

        .state(SuperState.INTAKE, s -> s
            .set(elev,  LinearGoal.meters(0.15))
            .set(pivot, RotationalGoal.degrees(15.0))
            .set(roll,  RollerGoal.intakeUntilPiece(3.0)))

        .state(SuperState.SCORE_HIGH, s -> s
            .set(elev,  LinearGoal.meters(1.1))
            .set(pivot, RotationalGoal.degrees(100.0))
            .set(fly,   FlywheelGoal.rps(70.0))
            .entryGuard(intake::hasPiece, "no piece"))

        .hub(SuperState.STOW)                        // STOW connects to and from everything
        .allow(SuperState.INTAKE, SuperState.SCORE_HIGH)

        // The declarative replacement for addTransitionRule(...). Stage 1 starts only
        // once every gating mechanism in stage 0 reports at-goal, so the elevator is up
        // before the arm swings out and the arm never meets the chassis. Unlike a
        // hand-built command group, the machine can log what it is about to do.
        .edge(SuperState.STOW, SuperState.SCORE_HIGH, e -> e
            .stage(elev)
            .stage(pivot))

        .build();

    // Tell the machine where the robot is physically built. Without this it only
    // assumes a starting state; seeding makes the first transition plan from the truth.
    superstructure.engine().seed(SuperState.STOW);
}
```

### Command Bindings

```java
// ── Superstructure state transitions ──
operator.a().onTrue(superstructure.transitionTo("STOW"));
operator.b().onTrue(superstructure.transitionTo("INTAKE"));
operator.y().onTrue(superstructure.transitionTo("SCORE_HIGH"));

// ── Intake + shooter ──
operator.rightTrigger(0.3).whileTrue(intake.intake());
operator.leftTrigger(0.3).whileTrue(
    shooter.spinUp(70)
        .alongWith(Commands.waitUntil(shooter::atSpeed)
            .andThen(intake.eject()))
);
```

With `Superstructure`, the same bindings are typed, and there is somewhere useful to put the
failure cases:

```java
// The string argument is recorded in the transition history, so the log says who asked.
operator.a().onTrue(superstructure.goTo(SuperState.STOW,       "op.a"));
operator.b().onTrue(superstructure.goTo(SuperState.INTAKE,     "op.b"));
operator.y().onTrue(superstructure.goTo(SuperState.SCORE_HIGH, "op.y"));

// A refused request ends immediately, so the button visibly does nothing. Rumble on
// rejected() so the operator feels the refusal instead of pressing harder.
superstructure.rejected().onTrue(
    Commands.startEnd(
        () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0.5),
        () -> operator.getHID().setRumble(RumbleType.kBothRumble, 0.0)
    ).withTimeout(0.25)
);

// arrivedAt fires only on *proven* arrival — never when a transition merely timed out.
superstructure.arrivedAt(SuperState.SCORE_HIGH).onTrue(leds.solid(Color.kGreen));

// One drag onto Elastic or Shuffleboard gives a working pit display: state, phase,
// blocker, summary, progress.
superstructure.addToDashboard("Pit");
```

In autonomous, do not write `goTo(SCORE_HIGH).andThen(shoot())` — a self-detected fault cannot make
a command end "interrupted", so `shoot()` would run anyway, into the floor. Use `onlyIfSettled`,
which checks that the machine really is where it was asked to be:

```java
Commands.sequence(
    superstructure.goTo(SuperState.SCORE_HIGH, "auto"),
    superstructure.onlyIfSettled(SuperState.SCORE_HIGH, shootCommand())
);
```

### Shooter Distance Table

```java
// Lookup table: distance (meters) -> shooter RPM
InterpolatingTable shooterTable = new InterpolatingTable()
    .add(1.0, 50)    // 1m -> 50 RPS
    .add(2.0, 58)    // 2m -> 58 RPS
    .add(3.0, 65)    // 3m -> 65 RPS
    .add(5.0, 75);   // 5m -> 75 RPS

// Use with distance from vision
double rps = shooterTable.get(distanceToTarget);
shooter.spinUp(rps);
```

---

## Mechanism Lab: SimDashboard for every mechanism kind

Alongside the game cockpit, the example now also serves a generic **Mechanism Lab** at [localhost:5806](http://localhost:5806). It is driven by `MechanismShowcase`, which builds one of every Catalyst mechanism kind (linear, rotational, roller, flywheel, turret, claw, differential wrist, winch, and pneumatic) on CAN IDs 30 to 39 and registers each one with a `SimDashboard`. Every mechanism gets a live, fitting widget you can drive from the browser, and each one runs its own physics simulation. The pneumatic is the exception: a solenoid has no continuous position, so it shows forward / reverse / off state instead.

This runs in simulation only, side by side with the existing game cockpit on [localhost:5805](http://localhost:5805), so you can have both pages open at once.

`MechanismShowcase` is the template for wiring `SimDashboard` to your own robot. Nothing in it is specific to this year's game: register your mechanisms with `dash.add(...)`, chain `slider`, `command`, `button`, and `toggle` controls, call `dash.start()` once, and call `dash.update()` once per loop. The exact same calls work against your real robot's mechanisms, including a team's own `CatalystMechanism` subclass that overrides `describe()`.

```java
private final SimDashboard dash = new SimDashboard(5806).title("Catalyst Mechanism Lab");

public MechanismShowcase() {
    dash.add(elevator)
            .slider("Height (m)", 0.0, 0.60, v -> sched(elevator.goTo(v)))
            .command("Stow", () -> elevator.goTo("DOWN"))
            .command("Top", () -> elevator.goTo("UP"));

    dash.add(roller)
            .command("Intake", roller::intake)
            .command("Eject", roller::eject)
            .toggle("Game piece", roller::setSimHasPiece);

    // ... one entry per mechanism kind ...

    dash.start();
}

public void update() {
    dash.update();   // call once per loop in sim
}
```

---

## Test Project

For a full integration test project with JUnit tests covering every FrcCatalyst component, see:

[FrcCatalystTest on GitHub](https://github.com/TomAs-1226/FrcCatalystTest){: .btn .btn-primary }
