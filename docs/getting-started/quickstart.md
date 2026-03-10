---
layout: default
title: Quick Start
nav_order: 2
parent: Getting Started
---

# Quick Start Guide
{: .no_toc }

Build a complete elevator mechanism in under 5 minutes.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Step 1: Define Your Mechanism

In your `RobotContainer`, create an elevator with the builder:

```java
import frc.lib.catalyst.mechanisms.LinearMechanism;
import frc.lib.catalyst.hardware.MotorType;

public class RobotContainer {

    private final LinearMechanism elevator = new LinearMechanism(
        LinearMechanism.Config.builder()
            .name("Elevator")
            .motor(13)                    // TalonFX CAN ID
            .follower(14, true)           // Follower CAN ID, opposed direction
            .motorType(MotorType.KRAKEN_X60)
            .gearRatio(10.0)              // 10:1 reduction
            .drumRadius(0.0254)           // 1 inch spool
            .range(0.0, 1.2)             // 0 to 1.2 meters
            .mass(5.0)                    // 5 kg carriage
            .pid(50, 0, 0.5)             // Position PID
            .gravityGain(0.35)            // Gravity compensation
            .motionMagic(2.0, 4.0, 20.0) // Cruise vel, accel, jerk
            .currentLimit(40)
            .position("STOW", 0.0)
            .position("INTAKE", 0.3)
            .position("AMP", 0.8)
            .position("HIGH", 1.1)
            .build()
    );
}
```

That's it. You now have a fully functional elevator with Motion Magic control, gravity compensation, simulation support, telemetry, and safety features.

## Step 2: Set Up Commands

```java
private void configureBindings() {
    // Default command: hold current position
    elevator.setDefaultCommand(elevator.holdPosition());

    // Button bindings for named presets
    operatorController.a().onTrue(elevator.goTo("STOW"));
    operatorController.b().onTrue(elevator.goTo("INTAKE"));
    operatorController.x().onTrue(elevator.goTo("AMP"));
    operatorController.y().onTrue(elevator.goTo("HIGH"));

    // Manual jog with joystick
    operatorController.leftBumper().whileTrue(
        elevator.jog(() -> -operatorController.getLeftY() * 4.0)
    );

    // Zero the elevator
    operatorController.start().onTrue(elevator.zero());
}
```

## Step 3: Check the Dashboard

FrcCatalyst automatically publishes telemetry under `Catalyst/Elevator/`:

| Key | Description |
|-----|-------------|
| `PositionMeters` | Current position in meters |
| `VelocityMPS` | Current velocity |
| `SetpointMeters` | Target position |
| `CurrentAmps` | Motor current draw |
| `AtSetpoint` | Whether the mechanism is at its target |
| `State` | Current state name (e.g., "GoTo 1.10m") |

Open **Shuffleboard**, **AdvantageScope**, or **Elastic** to see these values live.

## Step 4: Estimate Your Gains

Not sure what PID gains to use? FrcCatalyst can estimate them from your mechanism's physical specs:

```java
var config = LinearMechanism.Config.builder()
    .motorType(MotorType.KRAKEN_X60)
    .gearRatio(10.0)
    .drumRadius(0.0254)
    .mass(5.0)
    .build();

double gravityFF = config.estimateGravityFF();  // Voltage to hold against gravity
double maxSpeed = config.estimateMaxSpeed();     // Max mechanism speed
System.out.println("Estimated kG: " + gravityFF);
System.out.println("Max speed: " + maxSpeed + " m/s");
```

## Step 5: Characterize (Optional)

For the best performance, use SysId characterization:

```java
CharacterizationHelper charHelper = new CharacterizationHelper(
    "Elevator",
    elevator,            // subsystem (extends SubsystemBase)
    elevator.getMotor()  // CatalystMotor for voltage control
);

// Bind to SmartDashboard for SysId
SmartDashboard.putData("Elevator QS Fwd", charHelper.quasistaticForward());
SmartDashboard.putData("Elevator QS Rev", charHelper.quasistaticReverse());
SmartDashboard.putData("Elevator Dyn Fwd", charHelper.dynamicForward());
SmartDashboard.putData("Elevator Dyn Rev", charHelper.dynamicReverse());
```

## What's Next?

- [Mechanisms Guide](../mechanisms/) — Learn about all mechanism types
- [Subsystems](../subsystems/) — Swerve drive, vision, LEDs
- [Utilities](../utilities/) — Math helpers, feedforward, alerts
- [Examples](../examples/) — Complete robot examples
- [Testing](../testing/) — How to test your code
