---
layout: default
title: Examples
nav_order: 6
has_children: true
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

## Test Project

For a full integration test project with JUnit tests covering every FrcCatalyst component, see:

[FrcCatalystTest on GitHub](https://github.com/TomAs-1226/FrcCatalystTest){: .btn .btn-primary }
